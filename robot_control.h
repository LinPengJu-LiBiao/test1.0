#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "motor_protocol.h"

typedef enum {
    ROBOT_IDLE, ROBOT_SETUP, ROBOT_RUNNING, ROBOT_DWELL,
    ROBOT_DONE, ROBOT_STOPPED, ROBOT_FAULT
} RobotState;
typedef enum {
    ROBOT_OK, ROBOT_BAD_CONFIG, ROBOT_BUSY, ROBOT_BAD_ARGUMENT,
    ROBOT_GYRO_UNREADY, ROBOT_FEEDBACK_STALE, ROBOT_TX_FAILED,
    ROBOT_ACK_TIMEOUT, ROBOT_MOVE_TIMEOUT, ROBOT_STALLED,
    ROBOT_POSITION_JUMP, ROBOT_HEADING_ERROR, ROBOT_MOTOR_ERROR,
    ROBOT_CORRECTION_LIMIT, ROBOT_STOP_UNCONFIRMED
} RobotResult;
typedef enum { ROBOT_STRAIGHT, ROBOT_TURN, ROBOT_SQUARE, ROBOT_SHUTTLE } RobotJob;
typedef struct {
    float track_mm, wheel_diameter_mm;
    /* 每个位置计数对应的车轮实际行程（实测），与 CAN 速度单位无关。 */
    float position_mm_per_count[2];
    uint8_t device[2];                 /* 0=左03，1=右02；本车马达号=设备号，限定1..15 */
    int8_t forward_sign[2], gyro_ccw_sign; /* 实测后填写 +1/-1 */
    /* 陀螺仪积分标度校正，整数 ppm（百万分之一）：yaw_used = yaw_gyro ×
     * (1 + 对应任务的 ppm×1e-6)。分任务记忆：直线来回（任务四）与单段命令
     * 用 yaw_scale_ppm，方形（任务五）用 yaw_scale_ppm_square——陀螺仪
     * 顺时针/逆时针标度可能略有差异（实测方形在任务四标定下偏左 3 cm），
     * 两个任务各自标定、互不干扰，自动标定只更新当前任务对应的槽位。
     * 掉头方向锚定陀螺仪，标度误差 εg 让每次掉头物理上转 θ/(1+εg)：
     * 任务四 5 次 5 m 往返终点偏差 L ≈ 78.5 m × (ppm×1e-6 + εg)
     * （实测偏右 9 cm ↔ εg≈−1146 ppm → 填 +1146；偏右加大、偏左减小，
     * 1 cm ≈ 127 ppm）；任务五方形 ≈ 71 m × 同式（偏左 → 加大，1 cm ≈ 140 ppm）。
     * 整数便于 Live Watch 直接填写；由 yaw_auto_calibrate 按任务转向总账
     * 自动标定（比值 = 1/(1+εg)，与轮距轮径无关）。板级属性，换车不用重标。 */
    int32_t yaw_scale_ppm;
    /* 方形任务（任务五）专用槽位：初值与 yaw_scale_ppm 相同（本车 1080），
     * 由方形任务的自动标定独立收敛到本车顺时针直角的真实值。 */
    int32_t yaw_scale_ppm_square;
    /* 1=任务完成后按转向总账自动标定 yaw_scale_ppm（默认）：编码器总账 =
     * 指令角/(1+εg)、陀螺仪总账 = 指令角，比值即 1/(1+εg)。要求累计转向
     * ≥360° 且比值落在 [0.99,1.01]（超出说明有滑移/异常，放弃本次估计）。
     * 0=关闭，保持手动值。 */
    uint8_t yaw_auto_calibrate;
    uint8_t commissioned;              /* 两轮身份/方向/轮径/参数已核对 */
    uint8_t ack3_verified, ack_id_big_endian;
    uint8_t live_speed_verified;       /* 已确认运动期间速度指令能生效 */
    float max_speed_mm_s, max_accel_mm_s2;
    float heading_kp_mm_s_deg;      /* 直行差速修正 P 增益："100 mm/s 时每 1° 航向误差的 mm/s 差速"；实际差速 = kp×err×(v/100)，随速度归一，限幅 ±25% 速度 */
    float heading_tolerance_deg, position_tolerance_mm; /* 段末航向校正死区 / 位置到位容差 */
    float turn_speed_deg_s, turn_accel_deg_s2;
    float speed_cmd_ratio;           /* 速度/加速度指令尺度：raw = 车轮 mm/s × ratio × 10；默认 47/16 = 2.9375（指令是电机侧 0.1 mm 单位） */
    uint32_t feedback_timeout_ms, ack_timeout_ms, dwell_ms;
} RobotConfig;
typedef struct {
    float yaw_deg, rate_deg_s;
    uint32_t tick_ms, sequence;
    uint8_t valid, calibrated, degraded;
} RobotGyro;
typedef int (*RobotSend)(void *context, const MotorFrame *frame);
typedef struct {
    int32_t raw;
    uint32_t tick_ms, sequence, query_tick_ms;
    uint8_t valid, pending;
} RobotWheel;
/* CAN ID 既不是载荷错误码，也不是已验证的轮位地址。
 * 在驱动器错误载荷语义确认前，两种字节序都保留。 */
typedef struct {
    uint32_t tick_ms;
    uint16_t can_id, code_le, code_be;
    uint8_t valid, len, data[2];
} RobotMotorErrorReport;

typedef struct {
    RobotConfig config;
    RobotSend send;
    void *send_context;
    RobotState state;
    RobotResult error;
    RobotJob job;
    RobotWheel wheel[2];
    RobotGyro gyro;
    int32_t start_raw[2], target_raw[2], settle_raw[2];
    uint32_t evaluated_sequence[2], evaluated_gyro_sequence;
    uint32_t now_ms, start_ms, timeout_ms, ack_start_ms, last_query_ms;
    uint32_t last_control_ms, last_progress_ms, settle_ms, dwell_start_ms;
    uint32_t wheel_progress_ms[2];
    uint32_t stop_tick_ms, stop_sequence[2], stop_start_ms, stop_settle_ms;
    int32_t progress_raw[2], stop_raw[2];
    float speed_mm_s, acceleration_mm_s2, target_yaw_deg, base_yaw_deg;
    float amount, straight_speed, straight_accel;
    float remaining_mm[2], progress_mm[2];
    uint16_t expected_ack, step, total_steps;
    uint8_t ack_mask, phase, next_query, corrections, turning;
    uint8_t settling, stop_attempts, stop_confirmed, stop_settling, final_turn;
    uint32_t rejected_frames, accepted_frames, tx_failures;
    /* 速度缓存只在匹配ACK后更新；同一段内不重发相同的量化速度。 */
    uint32_t pending_speed_raw[2], acknowledged_speed_raw[2];
    uint8_t speed_ack_valid_mask;
    /* 故障快照在Stop清除ack_mask之前保存，Reset后仍保留供排查。 */
    uint16_t fault_ack_command;
    uint8_t fault_ack_mask, fault_phase, fault_state;
    uint32_t fault_tick_ms, ack_rx_count, ack_matched_count[2], last_ack_tick_ms;
    uint8_t last_ack_len, last_ack_data[8];
    uint32_t motor_error_rx_count;
    RobotMotorErrorReport last_motor_error, fault_motor_error;
    /* 0x102/0x103 电机状态广播：只计数留证，不参与故障判定。 */
    uint32_t motor_status_rx_count;
    uint16_t last_motor_status_id;
    uint8_t last_motor_status_data[2];
    /* 电机报错瞬间的两轮位置/目标快照：在停止/释放反馈改写它们之前保存。 */
    int32_t fault_motor_wheel_raw[2], fault_motor_target_raw[2];
    /* 段完成瞬间冻结的航向与位移，供修正或释放前留证。 */
    float segment_start_yaw_deg, segment_requested_deg;
    float completed_wheel_mm[2], completed_encoder_deg, completed_gyro_deg;
    float completed_heading_error_deg;
    /* 任务级转向累计（只统计转向段，含校正段；Robot_Start 清零）：
     * 陀螺仪与编码器两套独立角度参考在同一组转向上的总读数。
     * 比值 gyro/encoder ≈ (1+yaw_scale_ppm×1e-6)×(1+εg)/(1+εw)，用于在实车上分离
     * 陀螺仪标度误差与轮径标度误差（见 CALIBRATION_STEPS.md）。 */
    float mission_turn_gyro_deg, mission_turn_encoder_deg;
    uint16_t mission_turn_corrections;   /* 本次任务累计的掉头校正次数 */
    /* 任务完成瞬间的自动标定结果：last_ratio = encoder 总账/gyro 总账
     * （= 1/(1+εg) 的本次估计；未达到 ≥360°/被关闭时为 0），
     * applied = 1 表示本次已按比值更新 yaw_scale_ppm。 */
    float yaw_auto_last_ratio;
    uint8_t yaw_auto_applied;
} Robot;

RobotConfig Robot_DefaultConfig(void);
void Robot_Init(Robot *r, const RobotConfig *config, RobotSend send, void *context);
int Robot_IsBusy(const Robot *r);
void Robot_OnFrame(Robot *r, const MotorFrame *f, uint32_t now);
void Robot_Tick(Robot *r, uint32_t now, const RobotGyro *gyro);
/* 单线程接口：由 CAN_RX/小车任务调用；不可从其它任务直接改 Robot。 */
RobotResult Robot_Start(Robot *r, RobotJob job, float amount,
                       float speed, float acceleration, uint8_t repetitions,
                       uint8_t include_final_turn, uint32_t now);
void Robot_Stop(Robot *r, RobotResult reason, uint32_t now);
RobotResult Robot_Reset(Robot *r);
/* 几何函数：直行 mm；原地转向 deg（逆时针正），输出左右有符号 mm。 */
int Robot_WheelTravel(float track_mm, RobotJob job, float amount, float travel[2]);
#endif
