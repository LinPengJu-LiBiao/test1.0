/**
  * @file    car_control_app.c
  * @brief   STM32 接入层：CAN 帧/陀螺仪快照 → Robot 状态机，并执行 Watch 命令
  * @note    本文件只做胶水工作：帧过滤、命令分发、非阻塞电机配置。
  *          运动/航向/故障逻辑在 robot_control.c，协议编解码在 motor_protocol.c。
  */
#include "car_control_app.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <math.h>
#include <string.h>

Robot g_robot;
volatile RobotConfig g_robotSettings;
volatile CarControlRequest g_robotRequest;
volatile uint32_t g_robotCommandResult, g_robotCommandCount;
volatile uint32_t g_motorConfigFramesSubmitted;
volatile uint8_t g_lastDiscoveredSn[7];   /* 最近一帧 0x312 载荷，供 Watch 核对身份 */
volatile uint32_t g_discoveredSnCount;
/* 两只电机的 SN（2026-10-26 用户提供），按 0x313 载荷顺序 arr[0]..arr[6] 排列：
 *   左：19 0A 0A 07 FC D1 00        右：19 0A 09 07 06 D1 00
 * SN 只用于身份识别；绑定到哪一路由 RobotConfig.device[] 决定（左 03、右 02）。 */
const uint8_t g_leftMotorSn[7] = {0x19,0x0A,0x0A,0x07,0xFC,0xD1,0x00};
const uint8_t g_rightMotorSn[7] = {0x19,0x0A,0x09,0x07,0x06,0xD1,0x00};
/* 电机参数帧（0x316）默认值。Byte0..5 是驱动参数；Byte6 高四位为方向/自动反馈等
 * 配置位，低四位为马达号。本车令"马达号 = SN 绑定设备号"：左 03、右 02，且发送时
 * 由 Motor_Parameters() 按 device 重建低四位，避免两轮沿用同一个马达号。
 * 只在显式 command 4 时下发；装车方向由 forward_sign[] 单独映射。 */
volatile uint8_t g_motorParameters[2][7] = {
    {0xB4,0xBF,0xCF,0xAA,0x00,0x5F,0x03},
    {0xB4,0xBF,0xCF,0xAA,0x00,0x5F,0x02}
};
/* 非阻塞配置序号，每 20 ms 推进一步：0=空闲；1..2 双轮使能关；3..4 写参数；
 * 5..10 写轮径（每轮 3 帧）；11..12 双轮使能开。走完自增回 0，
 * 因此 g_motorConfigFramesSubmitted 正常应累计到 12。 */
static uint8_t config_phase;
static uint32_t config_tick;
/* 判定"车轮是否仍在动"的位移门限（mm）。与 robot_control.c 停止确认同口径：
 * 编码器静止抖动只有 1~2 个计数（约 0.03~0.07 mm），而手推车位移是毫米量级，
 * 取 0.2 mm 不会把抖动误判成运动，也不会漏掉真实推动。 */
#define GYRO_STILL_PERMITTED_MM 0.2
volatile uint8_t g_robotGyroStationary;      /* 1=两轮已连续静止 2 s */
volatile uint32_t g_robotGyroStationaryTick; /* 静止判定的时间戳，读取方须校验新鲜度 */
static volatile uint8_t gyro_motion_blocked; /* 1=运动命令/运动余波仍在生效期内 */
static uint8_t gyro_still_window;            /* 1=已取静止基线，可继续比较位移 */
static uint32_t gyro_still_since;            /* 静止窗口起点 */
static uint32_t gyro_last_motion_tick;       /* 最近一次判定"在动"的时间戳 */
static uint8_t gyro_motion_seen;             /* 上电后是否观察到过运动 */
static int32_t gyro_still_raw[2];            /* 静止基线：两轮原始位置计数 */
/* 校准保护：仍有未消费的命令，或车体处于运动生效期时返回 1。
 * 上层据此拒绝清零/重校准，避免运动中改写航向基准。 */
int CarControl_GyroCalibrationBlocked(void)
{
    return gyro_motion_blocked || g_robotRequest.command != CC_NONE;
}
/* 静止零偏更新的准入条件：未被保护逻辑否决，且静止判定足够新鲜（<=50 ms）。 */
int CarControl_GyroStationaryAllowed(uint32_t now)
{
    return !CarControl_GyroCalibrationBlocked() && g_robotGyroStationary &&
        (uint32_t)(now-g_robotGyroStationaryTick)<=50U;
}
/* 用两轮位置反馈维护"车体是否静止"。
 * 反馈必须新鲜（<=100 ms）；位移超过 GYRO_STILL_PERMITTED_MM 才算真的动了。
 * 旧实现直接比较原始计数是否严格相等，差 1 个计数（约 0.034 mm）就判在动；
 * 一旦命中就清零窗口、下一拍重新取基线，编码器静止抖动会让窗口永远重建，
 * gyro_motion_blocked 恒为 1、校准被永久否决。现改用位移门限，
 * 与 robot_control.c 停止确认的 0.2 mm 判据一致。 */
static void update_gyro_stationary(uint32_t now)
{
    unsigned i; int fresh=1, moved=0;
    int busy=CarControl_Busy();
    for(i=0;i<2;++i) {
        if(!g_robot.wheel[i].valid || (uint32_t)(now-g_robot.wheel[i].tick_ms)>100U) fresh=0;
        if(gyro_still_window &&
           fabs((double)g_robot.wheel[i].raw-(double)gyro_still_raw[i]) *
           (double)g_robot.config.position_mm_per_count[i] > GYRO_STILL_PERMITTED_MM) moved=1;
    }
    if(busy || moved) { gyro_last_motion_tick=now; gyro_motion_seen=1; }
    if(busy || !fresh) gyro_still_window=0;
    if(!busy && fresh && (moved || !gyro_still_window)) {
        gyro_still_since=now; gyro_still_window=1;
        for(i=0;i<2;++i) gyro_still_raw[i]=g_robot.wheel[i].raw;
    }
    taskENTER_CRITICAL();
    gyro_motion_blocked=(uint8_t)(busy || (gyro_motion_seen &&
        (uint32_t)(now-gyro_last_motion_tick)<2000U));
    g_robotGyroStationary=(uint8_t)(!busy && fresh && gyro_still_window &&
        (uint32_t)(now-gyro_still_since)>=2000U &&
        ((g_robot.state!=ROBOT_FAULT && g_robot.state!=ROBOT_STOPPED) || g_robot.stop_confirmed));
    g_robotGyroStationaryTick=now;
    taskEXIT_CRITICAL();
}
static int transport(void *context, const MotorFrame *f)
{
    (void)context;
    /* 0x413..0x416 是使能/加速度/速度/目标位置：发出即视为运动意图，
     * 立即撤销静止判定，避免运动期间误开放零偏更新。 */
    if(f->id>=0x413 && f->id<=0x416) {
        g_robotGyroStationary=0; gyro_still_window=0;
    }
    return CAN_SendFrameWait(f->id, f->data, f->len, 2U) == HAL_OK;
}
/* 上电初始化：只建立状态机与默认参数，不绑定、不使能、不启动运动。 */
void CarControl_Init(void)
{
    RobotConfig c = Robot_DefaultConfig();
    config_phase = 0; config_tick = 0; g_motorConfigFramesSubmitted = 0;
    g_robotGyroStationary=0; g_robotGyroStationaryTick=0;
    gyro_motion_blocked=0; gyro_still_window=0; gyro_motion_seen=0; gyro_last_motion_tick=0;
    g_robotSettings = c;
    Robot_Init(&g_robot, &c, transport, NULL);
    /* 课题二默认工况：5 m 单程、5 次完整往返。最后一次返回起点后
     * 直接结束，避免在终点再做一次无意义的 180° 掉头。 */
    g_robotRequest.amount = 5000.0f; g_robotRequest.speed = 600.0f;
    g_robotRequest.acceleration = 1600.0f; g_robotRequest.repetitions = 5;
    g_robotRequest.include_final_turn = 0; g_robotRequest.route_definition_confirmed = 0;
    g_robotRequest.command = CC_NONE;
}
/* 忙：状态机处于运动/停顿，或非阻塞配置尚未走完。 */
int CarControl_Busy(void) { return Robot_IsBusy(&g_robot) || config_phase != 0; }
/* CAN 接收分发。返回 1 表示该帧已被本层识别，无论内容是否被采用；
 * 上层据此区分合法帧与协议错误，命令回显不得计入错误统计。 */
int CarControl_OnFrame(uint16_t id, const uint8_t *data, uint8_t len, uint32_t now)
{
    MotorFrame f = {0}; unsigned i;
    if (len > 8) return 0;
    f.id = id; f.len = len; memcpy(f.data, data, len);
    if (id == 0x312 && len == 7) { /* SN 发现帧：记录载荷供 Watch 核对 */
        for (i=0; i<7; ++i) g_lastDiscoveredSn[i] = data[i];
        ++g_discoveredSnCount; return 1;
    }
    if (id == 0x409 || id == 0x41A || (id >= 0x100 && id <= 0x17F)) {
        Robot_OnFrame(&g_robot, &f, now); return 1;
    }
    /* 合法的非位置反馈与命令回显，不计为协议错误。 */
    return (id >= 0x380 && id <= 0x3FF) || (id >= 0x420 && id <= 0x4BF) ||
           id == 0x313 || id == 0x316 || id == 0x408 ||
           (id >= 0x413 && id <= 0x419) || id == 0x324 || id == 0x325;
}
/* 推进一拍非阻塞配置：每帧间隔 >=20 ms，给驱动器留出处理时间。
 * 任一步发送失败都按 TX_FAILED 停机，并中止剩余步骤。 */
static void configure_motors(uint32_t now)
{
    MotorFrame f; uint8_t parameters[7]; unsigned i, wheel;
    if (!config_phase || (uint32_t)(now-config_tick) < 20U) return;
    if (config_phase <= 2) f = Motor_Enable(g_robot.config.device[config_phase-1U], 0);
    else if (config_phase <= 4) {
        wheel = config_phase-3U;
        for (i=0; i<7; ++i) parameters[i] = g_motorParameters[wheel][i];
        f = Motor_Parameters(g_robot.config.device[wheel], parameters);
        g_motorParameters[wheel][6] = f.data[6]; /* 回写实际发送的 Byte6，Watch 可见最终马达号 */
    } else if (config_phase <= 10) {
        wheel = (config_phase-5U)/3U;
        f = Motor_WheelDiameter(g_robot.config.device[wheel],
            (uint16_t)(g_robot.config.wheel_diameter_mm*100.0f+0.5f));
    } else {
        f = Motor_Enable(g_robot.config.device[config_phase-11U], 1);
    }
    if (!transport(NULL, &f)) {
        config_phase = 0; g_robotCommandResult = ROBOT_TX_FAILED;
        Robot_Stop(&g_robot, ROBOT_TX_FAILED, now); return;
    }
    ++g_motorConfigFramesSubmitted; config_tick = now;
    if (++config_phase > 12U) config_phase = 0;
}
void CarControl_Poll(const RobotGyro *gyro, uint32_t now)
{
    CarControlRequest q; RobotResult result = ROBOT_OK; MotorFrame f;
    taskENTER_CRITICAL();
    q = g_robotRequest; g_robotRequest.command = CC_NONE;
    if(q.command!=CC_NONE) {
        gyro_motion_blocked=1; g_robotGyroStationary=0; gyro_still_window=0;
    }
    taskEXIT_CRITICAL();
    /* STOP 必须先于本拍状态推进处理：不能先发下一对运动目标再取消。 */
    if (q.command == CC_STOP) {
        ++g_robotCommandCount; config_phase = 0;
        Robot_Stop(&g_robot, ROBOT_OK, now); g_robotCommandResult = ROBOT_OK;
        Robot_Tick(&g_robot, now, gyro); update_gyro_stationary(now); return;
    }
    Robot_Tick(&g_robot, now, gyro);
    /* 任务完成后自动标定的 ppm 回写设置（分任务槽位）：之后的 command 1 不会
     * 再把标定值覆盖回旧值（自动标定关闭时不回写，保留手动设置）。 */
    if (g_robot.state == ROBOT_DONE && g_robot.config.yaw_auto_calibrate) {
        g_robotSettings.yaw_scale_ppm = g_robot.config.yaw_scale_ppm;
        g_robotSettings.yaw_scale_ppm_square = g_robot.config.yaw_scale_ppm_square;
    }
    if (q.command != CC_NONE) {
        ++g_robotCommandCount;
        if (CarControl_Busy()) result = ROBOT_BUSY;
        else switch (q.command) {
        case CC_APPLY_CONFIG: {
            RobotConfig c = g_robotSettings;
            if (g_robot.state != ROBOT_IDLE) { result = ROBOT_BUSY; break; }
            if (c.device[0] == 0 || c.device[0] > 15 || c.device[1] == 0 ||
                c.device[1] > 15 || c.device[0] == c.device[1] ||
                !isfinite(c.track_mm) || c.track_mm <= 0 ||
                !isfinite(c.wheel_diameter_mm) || c.wheel_diameter_mm < 0.01f ||
                c.wheel_diameter_mm > 655.35f) { result = ROBOT_BAD_CONFIG; break; }
            Robot_Init(&g_robot, &c, transport, NULL); break;
        }
        case CC_BIND_LEFT: case CC_BIND_RIGHT: {
            unsigned wheel = q.command == CC_BIND_LEFT ? 0U : 1U;
            if (g_robot.state != ROBOT_IDLE) { result = ROBOT_BUSY; break; }
            f = Motor_Bind(g_robot.config.device[wheel], wheel ? g_rightMotorSn : g_leftMotorSn);
            result = transport(NULL, &f) ? ROBOT_OK : ROBOT_TX_FAILED; break;
        }
        case CC_INITIALIZE_AND_ENABLE:
            if (g_robot.state != ROBOT_IDLE) { result = ROBOT_BUSY; break; }
            if (!g_robot.config.device[0] || g_robot.config.device[0] > 15 ||
                !g_robot.config.device[1] || g_robot.config.device[1] > 15 ||
                g_robot.config.device[0] == g_robot.config.device[1]) {
                result = ROBOT_BAD_CONFIG; break;
            }
            config_phase = 1; config_tick = now-20U; g_motorConfigFramesSubmitted = 0; break;
        case CC_ENABLE_MOTORS: {
            unsigned wheel;
            if (g_robot.state != ROBOT_IDLE) { result = ROBOT_BUSY; break; }
            if (g_robot.config.device[0] == 0 || g_robot.config.device[0] > 15 ||
                g_robot.config.device[1] == 0 || g_robot.config.device[1] > 15 ||
                g_robot.config.device[0] == g_robot.config.device[1]) {
                result = ROBOT_BAD_CONFIG; break;
            }
            /* 使能是取得位置反馈的前提，故本命令不要求位置、方向或 ACK 格式已确认。
             * 只发使能，不发速度/目标；返回 OK 仅表示 CAN 发送成功，不代表轮子已响应。 */
            for (wheel = 0; wheel < 2; ++wheel) {
                f = Motor_Enable(g_robot.config.device[wheel], 1);
                if (!transport(NULL, &f)) {
                    result = ROBOT_TX_FAILED;
                    Robot_Stop(&g_robot, result, now); break;
                }
            }
            break;
        }
        case CC_STRAIGHT: case CC_TURN: case CC_SQUARE: case CC_SHUTTLE:
            if ((q.command == CC_SQUARE || q.command == CC_SHUTTLE) &&
                q.route_definition_confirmed != 1) { result = ROBOT_BAD_ARGUMENT; break; }
            result = Robot_Start(&g_robot, (RobotJob)(q.command-CC_STRAIGHT), q.amount,
                        q.speed, q.acceleration, q.repetitions, q.include_final_turn, now); break;
        case CC_RESET: result = Robot_Reset(&g_robot); break;
        default: result = ROBOT_BAD_ARGUMENT; break;
        }
        g_robotCommandResult = (uint32_t)result;
    }
    configure_motors(now);
    update_gyro_stationary(now);
}

