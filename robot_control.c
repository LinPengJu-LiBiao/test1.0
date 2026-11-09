#include "robot_control.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#define RAD_PER_DEG 0.01745329251994329577f
/* 电机→车轮减速比 16:47（带教提供，铭牌）：电机转 47/16 = 2.9375 圈，车轮转 1 圈。
 * 1000 rpm 电机 → 1000/2.9375 = 340.4 车轮 rpm → 1337 mm/s 车体速度。 */
#define ROBOT_GEAR_RATIO 2.9375f
/* 位置单位是"电机侧 0.1 mm"，车轮侧每单位 = 1/(10×减速比) = 0.03404 mm：
 *   驱动按配置轮径 D 换算，电机转 1 圈报 π·D·10 个单位；
 *   车轮转 1 圈 = 减速比 圈电机 = 减速比·π·D·10 个单位；
 *   于是车轮侧 mm/单位 = π·D /(减速比·π·D·10) = 1/(10·减速比)，与 D 无关。
 * 反推车轮一圈 = 235.619/0.03404 = 6921.3 个单位。
 * 2026-10-27 对齐同课题高精度参考实现：原先左右分别用 75π/6899 / 75π/6703
 * （手转一圈点数），相差 2.92%。同型号电机 + 同减速比不应有差异，单圈手测
 * 精度也不足以分辨 3%，而 6899 与理论 6921.3 只差 0.32%、6703 却差 3.15%，
 * 判定右轮那次手测不准。两轮统一用理论系数：等速直行不再被 2.92% 拉出
 * 几何航向误差（旧系数下 5 m 直行两轮走过的物理距离差 ≈ 146 mm）。 */
#define ROBOT_POSITION_MM_PER_COUNT (1.0f / (10.0f * ROBOT_GEAR_RATIO))
/* 速度/加速度命令也是电机侧 0.1 mm 标度：raw = 车轮 mm/s × speed_cmd_ratio × 10。
 * speed_cmd_ratio 是唯一换算出口，规划、限幅、故障判据全部留在真实车轮
 * mm/s 口径下，单位尺度问题只在这一个函数里（实测已确认 2.9375）。 */
static uint32_t value_raw(const RobotConfig *c, float wheel_value)
{
    float raw = wheel_value * c->speed_cmd_ratio * 10.0f + 0.5f;
    if (!(raw > 1.0f)) return 1U;           /* 也挡住 NaN：比较为假即归 1 */
    return raw > 4.0e9f ? 4000000000U : (uint32_t)raw;
}
/* 转向段的速度/加速度限制统一以轮速 (mm/s) 表达，和直行共用同一物理量纲：
 *   轮速 = 角度速度 × track/2 × π/180，track = 390 时 1 °/s = 3.4034 mm/s。
 * 上限锚在陀螺仪量程 XV7001BB_RANGE_DPS = ±100 °/s 上：100 °/s ↔ 340.3 mm/s。
 * 再快就会让陀螺仪饱和，积分少算角度，转完的航向基准直接作废。
 * 加速度取 300 °/s²（=1021 mm/s²），满速斜坡 0.30 s，90° 转向 1.15 s。
 * 改 track_mm 后同一对常量换算成不同角度值，物理轮速上限保持不变。 */
#define ROBOT_TURN_MAX_WHEEL_MM_S   341.0f
#define ROBOT_TURN_MAX_WHEEL_MM_S2  1022.0f
#define QUERY_PERIOD_MS 10U
#define CONTROL_PERIOD_MS 50U
#define SETTLE_MS 250U
/* 段末航向校正（直行与掉头共用，对齐同课题高精度参考实现）：每段结束检查
 * 目标航向残差，> 死区就追加低速原地校正段（30 mm/s ≈ 8.8°/s、10°/s²），
 * 预算 3 次；3 次还进不了死区或残差 >10° 判失控故障停机——保证每段出发
 * 方向都锚定任务轴线，不让残差带进下一段。 */
#define ROBOT_MAX_TURN_CORRECTIONS 3
#define ROBOT_MAX_HEADING_ERROR_DEG 10.0f
/* 0x102 / 0x103 是两只电机周期性发来的状态广播（实拍载荷 34 12，语义未确认），
 * 不是故障报告。旧实现把 0x100..0x17F 一律当 MOTOR_ERROR，运动中收到就进 FAULT，
 * 而 FAULT 只有显式 Reset 才能退出，于是本次运动之后的命令全部被 ROBOT_BUSY 拒绝。
 * 这些 ID 现在只计数、留最后载荷，不停机。真正需要停机的报告仍按原逻辑处理。
 * 与左右轮的对应关系未经验证，所以这里只列 ID，不声称哪一只。 */
static const uint16_t motor_status_ids[] = {0x102U, 0x103U};
static int is_motor_status_frame(uint16_t id)
{
    unsigned i;
    for (i = 0; i < sizeof(motor_status_ids) / sizeof(motor_status_ids[0]); ++i)
        if (id == motor_status_ids[i]) return 1;
    return 0;
}

static float clamp(float x, float low, float high)
{ return x < low ? low : (x > high ? high : x); }
static int sign_ok(int8_t s) { return s == 1 || s == -1; }
static int finite_positive(float v) { return isfinite(v) && v > 0.0f; }
/* 角度量纲 → 转向时轮子表面的线速度 (mm/s)。原地转向时两轮反向、速率为 ω×B/2。 */
static float turn_deg_to_mm_s(const RobotConfig *c, float deg_s)
{ return deg_s * c->track_mm * RAD_PER_DEG * 0.5f; }
static int send_frame(Robot *r, MotorFrame f)
{
    if (!r->send || !r->send(r->send_context, &f)) { ++r->tx_failures; return 0; }
    return 1;
}
static int fresh(const Robot *r, uint32_t now)
{
    return r->wheel[0].valid && r->wheel[1].valid &&
        (uint32_t)(now-r->wheel[0].tick_ms) <= r->config.feedback_timeout_ms &&
        (uint32_t)(now-r->wheel[1].tick_ms) <= r->config.feedback_timeout_ms;
}
static int gyro_ready(const Robot *r, uint32_t now)
{
    return r->gyro.valid && r->gyro.calibrated && !r->gyro.degraded &&
        isfinite(r->gyro.yaw_deg) && isfinite(r->gyro.rate_deg_s) &&
        (uint32_t)(now-r->gyro.tick_ms) <= 50U;
}
static float yaw(const Robot *r)
{
    int32_t ppm = (r->job == ROBOT_SQUARE) ? r->config.yaw_scale_ppm_square
                                           : r->config.yaw_scale_ppm;
    return r->gyro.yaw_deg * r->config.gyro_ccw_sign *
        (1.0f + (float)ppm * 1.0e-6f);
}
static int config_ok(const RobotConfig *c)
{
    return finite_positive(c->track_mm) && c->track_mm <= 2000.0f &&
        finite_positive(c->wheel_diameter_mm) && c->wheel_diameter_mm <= 655.35f &&
        isfinite(c->position_mm_per_count[0]) && c->position_mm_per_count[0] >= 0.0001f &&
        c->position_mm_per_count[0] <= 1.0f &&
        isfinite(c->position_mm_per_count[1]) && c->position_mm_per_count[1] >= 0.0001f &&
        c->position_mm_per_count[1] <= 1.0f &&
        c->device[0] > 0 && c->device[0] <= 15 && c->device[1] > 0 &&
        c->device[1] <= 15 && c->device[0] != c->device[1] &&
        sign_ok(c->forward_sign[0]) && sign_ok(c->forward_sign[1]) &&
        sign_ok(c->gyro_ccw_sign) &&
        c->yaw_scale_ppm >= -50000 && c->yaw_scale_ppm <= 50000 &&
        c->yaw_scale_ppm_square >= -50000 && c->yaw_scale_ppm_square <= 50000 &&
        c->yaw_auto_calibrate <= 1 &&
        c->commissioned == 1 && c->ack3_verified == 1 &&
        c->ack_id_big_endian <= 1 && c->live_speed_verified == 1 &&
        finite_positive(c->max_speed_mm_s) && c->max_speed_mm_s <= 2000.0f &&
        finite_positive(c->max_accel_mm_s2) && c->max_accel_mm_s2 <= 10000.0f &&
        finite_positive(c->heading_kp_mm_s_deg) && c->heading_kp_mm_s_deg <= 50.0f &&
        finite_positive(c->heading_tolerance_deg) && c->heading_tolerance_deg <= 2.0f &&
        finite_positive(c->position_tolerance_mm) && c->position_tolerance_mm <= 5.0f &&
        finite_positive(c->speed_cmd_ratio) && c->speed_cmd_ratio <= 16.0f &&
        finite_positive(c->turn_speed_deg_s) &&
        turn_deg_to_mm_s(c, c->turn_speed_deg_s) <= ROBOT_TURN_MAX_WHEEL_MM_S &&
        finite_positive(c->turn_accel_deg_s2) &&
        turn_deg_to_mm_s(c, c->turn_accel_deg_s2) <= ROBOT_TURN_MAX_WHEEL_MM_S2 &&
        c->feedback_timeout_ms >= 50 && c->feedback_timeout_ms <= 1000 &&
        c->ack_timeout_ms >= 20 && c->ack_timeout_ms <= 1000 && c->dwell_ms <= 60000;
}
RobotConfig Robot_DefaultConfig(void)
{
    RobotConfig c; memset(&c, 0, sizeof(c));
    c.track_mm = 390.0f; c.wheel_diameter_mm = 75.0f;
    /* 两轮共用同一位置尺度（对齐同课题高精度参考实现）：同型号电机、同减速比，
     * 编码器分辨率必须一致。手转一圈的实测值（左 6899 / 6703 右）精度不足以
     * 分辨 3% 的差异，且会让等速直行累积几何航向误差，故改用减速比推导值
     * 1/(10×47/16)（反推车轮一圈 = 6921.3 个计数，与手测 6899 差 0.32%）。
     * 装车方向差异仍由 forward_sign[] 单独表达，不体现在比例里。 */
    c.position_mm_per_count[0] = c.position_mm_per_count[1] = ROBOT_POSITION_MM_PER_COUNT;
    c.device[0] = 3; c.device[1] = 2;
    /* 2026-10-23 按用户实测确认的装车方向和协议配置固化。 */
    c.forward_sign[0] = 1; c.forward_sign[1] = -1;
    c.gyro_ccw_sign = -1;
    /* yaw_scale_ppm = 1080：本车陀螺仪板实车标定值（2026-10-27）。标度误差
     * εg 直接进 L：L ≈ 78.5 m × |yaw_scale_ppm×1e-6 + εg|（5 次 5 m 往返）。
     * 实测过程：偏右 9 cm → +1146 后偏右 3 cm → 1527 后偏左 3.5 cm，
     * 三点插值取 1080（1527 − 445）。这三点已在随机带内（死区 0.3° 的
     * 逐次随机量 σ≈2.4 cm 叠加在系统项上），之后不再逐次追调，以三组
     * 重复的均值定标。自动标定（yaw_auto_calibrate=1）每次任务完成后仍会
     * 按转向总账微调，换板/换车后的第一次任务也会自动重标。 */
    c.yaw_scale_ppm = 1080;
    /* 方形任务独立槽位：本车实车标定值 2060（2026-10-28）。实测过程：
     * 任务四标定（1080）下偏左 3→4.5 cm → +630 后（1710）偏左 1.6 cm、
     * 前 1.5 cm → 按实测灵敏度 1 cm ≈ 220 ppm 再 +350。陀螺仪顺时针/
     * 逆时针标度略有差异（MEMS 常见）；方形任务的自动标定会把本槽位
     * 继续收敛到真实值，不动任务四的系数。 */
    c.yaw_scale_ppm_square = 2060;
    /* 自动标定默认开启：每块陀螺仪板子的标度误差 εg 不同（参考实现的高精度
     * 部分是它的板子恰好准），任务完成后按转向总账自测 1/(1+εg) 并更新
     * **当前任务对应的槽位**——用户零测量，烧录后第一次任务即完成标定；
     * 实车表现异常时置 0 冻结手动值。 */
    c.yaw_auto_calibrate = 1;
    c.ack3_verified = 1; c.ack_id_big_endian = 0;
    c.live_speed_verified = 1; c.commissioned = 1;
    /* 物理上限（对齐参考实现）：1000 rpm ÷ 2.9375 × π×75 ÷ 60 = 1337 mm/s。 */
    c.max_speed_mm_s = 1337.0f; c.max_accel_mm_s2 = 3000.0f;
    /* 直行差速修正：速度归一化 P 控制。delta = kp×err×(v/100)，kp=4 的含义
     * 是"100 mm/s 时每 1° 航向误差产生 4 mm/s 差速"；差速随速度放大，横向
     * 偏移不随速度恶化（固定 mm/s 增益下 1337 mm/s 的横向偏移是 100 mm/s
     * 的 13 倍）。限幅 ±25% 速度，等价于误差超过 6.25° 才饱和。 */
    c.heading_kp_mm_s_deg = 4.0f;
    c.heading_tolerance_deg = 0.3f;
    c.position_tolerance_mm = 1.0f;
    /* 2026-10-26 转向提速到陀螺仪量程内：90 °/s（306 mm/s 轮速）、300 °/s²（1021 mm/s²）。
     * 留 10% 余量给斜坡过冲，避免角速度瞬时压到 ±100 °/s 让积分少算。 */
    c.turn_speed_deg_s = 90.0f; c.turn_accel_deg_s2 = 300.0f;
    /* 速度指令是电机侧 0.1 mm 标度（实车已确认）：发送出口乘 47/16。位置
     * 标定已含减速效果，不能再重复乘减速比。 */
    c.speed_cmd_ratio = ROBOT_GEAR_RATIO;
    /* 步间停顿从 1000 ms 降到 150 ms：正方形 8 步光停顿就占 7 s（36%）。
     * 电机位置环已在 SETTLE_MS 静止确认里把关，1000 ms 纯属浪费。 */
    c.feedback_timeout_ms = 300; c.ack_timeout_ms = 200; c.dwell_ms = 150;
    return c;
}
void Robot_Init(Robot *r, const RobotConfig *c, RobotSend send, void *context)
{
    memset(r, 0, sizeof(*r)); r->config = *c; r->send = send;
    r->send_context = context; r->state = ROBOT_IDLE;
}
int Robot_IsBusy(const Robot *r)
{ return r->state == ROBOT_SETUP || r->state == ROBOT_RUNNING || r->state == ROBOT_DWELL; }
int Robot_WheelTravel(float b, RobotJob kind, float amount, float out[2])
{
    if (!finite_positive(b) || !isfinite(amount) || !out) return 0;
    if (kind == ROBOT_STRAIGHT) { out[0] = amount; out[1] = amount; }
    else if (kind == ROBOT_TURN) {
        out[1] = b * 0.5f * amount * RAD_PER_DEG; out[0] = -out[1];
    } else return 0;
    return isfinite(out[0]) && isfinite(out[1]);
}
void Robot_Stop(Robot *r, RobotResult reason, uint32_t now)
{
    unsigned i;
    if (r->state == ROBOT_FAULT || r->state == ROBOT_STOPPED) return;
    if (reason != ROBOT_OK) {
        r->fault_ack_command = r->expected_ack; r->fault_ack_mask = r->ack_mask;
        r->fault_phase = r->phase; r->fault_state = (uint8_t)r->state;
        r->fault_tick_ms = now;
        r->fault_motor_error.valid = 0; /* A later non-motor fault must not reuse an old report. */
    }
    r->speed_ack_valid_mask = 0;
    r->state = reason == ROBOT_OK ? ROBOT_STOPPED : ROBOT_FAULT;
    r->error = reason; r->ack_mask = 0; r->stop_attempts = 1;
    r->stop_tick_ms = r->stop_start_ms = now; r->stop_confirmed = 0; r->stop_settling = 0;
    /* Release 是协议明确提供的退出驱动指令，不把它宣称为机械急停/制动。 */
    for (i = 0; i < 2; ++i) {
        (void)send_frame(r, Motor_Enable(r->config.device[i], 0));
        r->stop_sequence[i] = r->wheel[i].sequence; r->stop_raw[i] = r->wheel[i].raw;
    }
}
RobotResult Robot_Reset(Robot *r)
{
    if (Robot_IsBusy(r)) return ROBOT_BUSY;
    if ((r->state == ROBOT_FAULT || r->state == ROBOT_STOPPED) && !r->stop_confirmed)
        return ROBOT_STOP_UNCONFIRMED;
    r->state = ROBOT_IDLE; r->error = ROBOT_OK; r->ack_mask = 0; return ROBOT_OK;
}
static int target_from_travel(Robot *r, unsigned i, float mm)
{
    double raw = (double)r->wheel[i].raw +
        (double)mm / r->config.position_mm_per_count[i] * (double)r->config.forward_sign[i];
    if (!isfinite(raw) || raw > INT32_MAX || raw < INT32_MIN) return 0;
    r->start_raw[i] = r->wheel[i].raw;
    r->target_raw[i] = (int32_t)(raw < 0.0 ? ceil(raw - 0.5) : floor(raw + 0.5));
    r->settle_raw[i] = r->progress_raw[i] = r->wheel[i].raw;
    r->evaluated_sequence[i] = r->wheel[i].sequence;
    return 1;
}
static int begin_segment(Robot *r, int turning, float amount, float speed, float accel,
                         uint32_t now)
{
    float travel[2]; unsigned i;
    if (!Robot_WheelTravel(r->config.track_mm, turning ? ROBOT_TURN : ROBOT_STRAIGHT,
                          amount, travel)) return 0;
    for (i = 0; i < 2; ++i) if (!target_from_travel(r, i, travel[i])) return 0;
    r->wheel_progress_ms[0] = r->wheel_progress_ms[1] = now;
    r->turning = (uint8_t)turning; r->speed_mm_s = speed; r->acceleration_mm_s2 = accel;
    r->segment_start_yaw_deg = yaw(r);
    r->segment_requested_deg = turning ? amount : 0.0f;
    r->phase = 0; r->ack_mask = 0; r->speed_ack_valid_mask = 0;
    r->state = ROBOT_SETUP;
    r->start_ms = r->last_progress_ms = now;
    r->timeout_ms = (uint32_t)clamp(5000.0f + fabsf(travel[0])/speed * 4000.0f +
                                   speed/accel * 4000.0f, 5000.0f, 300000.0f);
    r->settling = 0; r->evaluated_gyro_sequence = r->gyro.sequence;
    return 1;
}
static int begin_step(Robot *r, uint32_t now)
{
    int turn = 0; float amount = r->amount, speed = r->straight_speed;
    float accel = r->straight_accel, k = r->config.track_mm * RAD_PER_DEG * 0.5f;
    r->corrections = 0;
    if (r->job == ROBOT_TURN) { turn = 1; r->target_yaw_deg = r->base_yaw_deg+r->amount; }
    else if (r->job == ROBOT_STRAIGHT) r->target_yaw_deg = r->base_yaw_deg;
    else {
        turn = (r->step & 1U) != 0;
        /* 路线每一步都用绝对航向目标（含直行步，对齐同课题高精度参考实现）：
         * 直行段目标 = 该步任务轴线方向，段末航向校正把车头重新锚定到轴线；
         * 掉头预转量 = 目标 − 当前航向，吸收直行段两轮速度差异/到位时单轮
         * 收尾转动攒下的偏移，不随来回累积。往返直行步目标依次 base、
         * base+180、base+180、base+360…；方形第 k 角 = base−90°×k。
         * 单段 command 6 转向仍是相对角度语义（base+amount）。 */
        r->target_yaw_deg = r->base_yaw_deg +
            (r->job == ROBOT_SQUARE ? -90.0f : 180.0f) * (float)((r->step+1U)/2U);
        if (turn) { speed = r->config.turn_speed_deg_s; accel = r->config.turn_accel_deg_s2; }
    }
    if (turn) { amount = r->target_yaw_deg-yaw(r); speed *= k; accel *= k; }
    if (speed > r->config.max_speed_mm_s || accel > r->config.max_accel_mm_s2) return 0;
    return begin_segment(r, turn, amount, speed, accel, now);
}
RobotResult Robot_Start(Robot *r, RobotJob job, float amount, float speed, float accel,
                       uint8_t repetitions, uint8_t final_turn, uint32_t now)
{
    if (Robot_IsBusy(r)) return ROBOT_BUSY;
    if (r->state == ROBOT_FAULT || r->state == ROBOT_STOPPED) return ROBOT_BUSY;
    if (!config_ok(&r->config)) return ROBOT_BAD_CONFIG;
    if (!isfinite(amount) || fabsf(amount) < 0.1f || !finite_positive(speed) ||
        !finite_positive(accel) || speed < 1.0f || accel < 1.0f ||
        (unsigned)job > (unsigned)ROBOT_SHUTTLE)
        return ROBOT_BAD_ARGUMENT;
    if ((job == ROBOT_TURN && (fabsf(amount) > 720.0f ||
            turn_deg_to_mm_s(&r->config, speed) > ROBOT_TURN_MAX_WHEEL_MM_S ||
            turn_deg_to_mm_s(&r->config, accel) > ROBOT_TURN_MAX_WHEEL_MM_S2)) ||
        (job != ROBOT_TURN && (fabsf(amount) > 10000.0f || speed > r->config.max_speed_mm_s ||
                               accel > r->config.max_accel_mm_s2))) return ROBOT_BAD_ARGUMENT;
    if ((job == ROBOT_SQUARE || job == ROBOT_SHUTTLE) &&
        (amount <= 0 || repetitions < 1 || repetitions > 20 || final_turn > 1))
        return ROBOT_BAD_ARGUMENT;
    if (!gyro_ready(r, now)) return ROBOT_GYRO_UNREADY;
    if (!fresh(r, now)) return ROBOT_FEEDBACK_STALE;
    r->job = job; r->amount = amount; r->straight_speed = speed; r->straight_accel = accel;
    r->base_yaw_deg = yaw(r); r->step = 0; r->final_turn = final_turn;
    r->mission_turn_gyro_deg = 0.0f; r->mission_turn_encoder_deg = 0.0f;
    r->mission_turn_corrections = 0;
    r->total_steps = job == ROBOT_SQUARE ? (uint16_t)(8U*repetitions) :
        (job == ROBOT_SHUTTLE ? (uint16_t)(4U*repetitions-(final_turn ? 0U : 1U)) : 1U);
    if (!begin_step(r, now)) { r->state = ROBOT_IDLE; return ROBOT_BAD_ARGUMENT; }
    r->error = ROBOT_OK; return ROBOT_OK;
}
void Robot_OnFrame(Robot *r, const MotorFrame *f, uint32_t now)
{
    unsigned i; int32_t position;
    if (f->id == 0x41A) {
        ++r->ack_rx_count; r->last_ack_tick_ms = now; r->last_ack_len = f->len;
        memset(r->last_ack_data, 0, sizeof(r->last_ack_data));
        memcpy(r->last_ack_data, f->data, f->len <= 8 ? f->len : 8);
    }
    if (f->id >= 0x100 && f->id <= 0x17F && f->len == 2) {
        RobotMotorErrorReport *report = &r->last_motor_error;
        if (is_motor_status_frame(f->id)) {
            ++r->motor_status_rx_count;
            r->last_motor_status_id = f->id;
            r->last_motor_status_data[0] = f->data[0];
            r->last_motor_status_data[1] = f->data[1];
            return;
        }
        ++r->motor_error_rx_count;
        report->valid = 1; report->tick_ms = now;
        report->can_id = f->id; report->len = f->len;
        report->data[0] = f->data[0]; report->data[1] = f->data[1];
        report->code_le = (uint16_t)((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));
        report->code_be = (uint16_t)(((uint16_t)f->data[0] << 8) | (uint16_t)f->data[1]);
        if (Robot_IsBusy(r)) {
            Robot_Stop(r, ROBOT_MOTOR_ERROR, now);
            r->fault_motor_error = *report;
            for (i = 0; i < 2; ++i) {
                r->fault_motor_wheel_raw[i] = r->wheel[i].raw;
                r->fault_motor_target_raw[i] = r->target_raw[i];
            }
        }
        /* 后续报告只刷新 last_motor_error，绝不覆盖第一次锁存的故障快照。 */
        return;
    }
    for (i = 0; i < 2; ++i) {
        RobotWheel *w = &r->wheel[i];
        if (w->pending && (uint32_t)(now-w->query_tick_ms) <= 100U &&
            Motor_Position(f, r->config.device[i], &position)) {
            if (w->valid && Robot_IsBusy(r)) {
                double change = fabs((double)position-(double)w->raw) *
                    r->config.position_mm_per_count[i];
                double allowed = (double)r->config.max_speed_mm_s *
                    (double)(uint32_t)(now-w->tick_ms) * 0.002 + 5.0;
                if (change > allowed) { Robot_Stop(r, ROBOT_POSITION_JUMP, now); return; }
            }
            w->raw = position; w->tick_ms = now; ++w->sequence;
            w->valid = 1; w->pending = 0; ++r->accepted_frames; return;
        }
        if ((r->ack_mask & (1U << i)) &&
            Motor_Ack(f, r->config.device[i], r->expected_ack, r->config.ack_id_big_endian)) {
            if (r->expected_ack == 0x415) {
                r->acknowledged_speed_raw[i] = r->pending_speed_raw[i];
                r->speed_ack_valid_mask |= (uint8_t)(1U << i);
            }
            ++r->ack_matched_count[i];
            r->ack_mask &= (uint8_t)~(1U << i); ++r->accepted_frames; return;
        }
    }
    if (f->id == 0x409 || f->id == 0x41A) ++r->rejected_frames;
}
static void send_pair(Robot *r, uint16_t id, uint32_t left, uint32_t right, uint32_t now)
{
    unsigned i; uint32_t values[2] = {left, right}; uint8_t mask = 3;
    if (id == 0x415) {
        for (i = 0; i < 2; ++i) {
            if ((r->speed_ack_valid_mask & (1U << i)) &&
                r->acknowledged_speed_raw[i] == values[i]) mask &= (uint8_t)~(1U << i);
        }
    }
    if (!mask) return;
    r->expected_ack = id; r->ack_mask = mask; r->ack_start_ms = now;
    for (i = 0; i < 2; ++i) {
        MotorFrame f;
        if (!(mask & (1U << i))) continue;
        if (id == 0x415) r->pending_speed_raw[i] = values[i];
        if (id == 0x413) f = Motor_Enable(r->config.device[i], 1);
        else if (id == 0x416) f = Motor_Target(r->config.device[i], r->target_raw[i]);
        else f = Motor_Value(id, r->config.device[i], values[i]);
        if (!send_frame(r, f)) { Robot_Stop(r, ROBOT_TX_FAILED, now); return; }
    }
}
static void poll_position(Robot *r, uint32_t now)
{
    unsigned i;
    for (i = 0; i < 2; ++i)
        if (r->wheel[i].pending && (uint32_t)(now-r->wheel[i].query_tick_ms) > 100U)
            r->wheel[i].pending = 0;
    if ((uint32_t)(now-r->last_query_ms) < QUERY_PERIOD_MS) return;
    i = r->next_query; r->next_query ^= 1U; r->last_query_ms = now;
    if (r->wheel[i].pending || r->config.device[i] == 0) return;
    if (!send_frame(r, Motor_QueryPosition(r->config.device[i]))) {
        if (Robot_IsBusy(r)) Robot_Stop(r, ROBOT_TX_FAILED, now);
        return;
    }
    r->wheel[i].pending = 1; r->wheel[i].query_tick_ms = now;
}
static void monitor_stop(Robot *r, uint32_t now)
{
    unsigned i; int still = 1;
    if (r->stop_attempts < 3 && (uint32_t)(now-r->stop_tick_ms) >= 100U) {
        for (i = 0; i < 2; ++i) (void)send_frame(r, Motor_Enable(r->config.device[i], 0));
        r->stop_tick_ms = now; ++r->stop_attempts;
    }
    if (!fresh(r, now) || !gyro_ready(r, now)) {
        r->stop_settling = 0; r->stop_confirmed = 0; return;
    }
    if (r->wheel[0].sequence == r->stop_sequence[0] ||
        r->wheel[1].sequence == r->stop_sequence[1]) return;
    for (i = 0; i < 2; ++i) {
        if (fabs((double)r->wheel[i].raw-r->stop_raw[i]) *
            r->config.position_mm_per_count[i] > 0.2) still = 0;
        r->stop_sequence[i] = r->wheel[i].sequence;
        if (!r->stop_settling) r->stop_raw[i] = r->wheel[i].raw;
    }
    if (fabsf(r->gyro.rate_deg_s) > 1.0f) still = 0;
    if (!still) { r->stop_settling = 0; r->stop_confirmed = 0; }
    else if (!r->stop_settling) { r->stop_settling = 1; r->stop_settle_ms = now; }
    else if ((uint32_t)(now-r->stop_settle_ms) >= SETTLE_MS) r->stop_confirmed = 1;
}
static void finish_or_correct(Robot *r, uint32_t now)
{
    float err = r->target_yaw_deg-yaw(r);
    unsigned i;
    for (i = 0; i < 2; ++i)
        r->completed_wheel_mm[i] = (float)((int64_t)r->wheel[i].raw-r->start_raw[i]) *
            r->config.position_mm_per_count[i] * r->config.forward_sign[i];
    r->completed_encoder_deg = (r->completed_wheel_mm[1]-r->completed_wheel_mm[0]) /
        (r->config.track_mm * RAD_PER_DEG);
    r->completed_gyro_deg = yaw(r)-r->segment_start_yaw_deg;
    r->completed_heading_error_deg = err;
    /* 任务级转向累计：把本段（含校正段）的陀螺仪/编码器读数并入任务总账。
     * 在进入校正分支之前累计——校正段走完会带着自己的 completed_* 再次
     * 回到这里，整组掉头（预转 + 全部校正）的读数因此自动完整。 */
    if (r->turning) {
        r->mission_turn_gyro_deg += r->completed_gyro_deg;
        r->mission_turn_encoder_deg += r->completed_encoder_deg;
    }
    /* 直行与掉头段统一做段末航向校正（对齐同课题高精度参考实现）：残差超
     * 死区就追加低速原地校正段（30 mm/s ≈ 8.8°/s、10°/s²），预算 3 次。
     * 收敛不了（3 次不进死区或残差 >10°）判失控故障停机——保证每段出发
     * 方向都锚定任务轴线，不让残差带进下一段；校正闭环把掉头锁死在陀螺仪
     * 说的目标上，陀螺仪标度误差 εg 仍需 yaw_scale_ppm 标定（板级属性）。 */
    if (fabsf(err) > r->config.heading_tolerance_deg) {
        float k = r->config.track_mm * RAD_PER_DEG * 0.5f;
        if (r->corrections >= ROBOT_MAX_TURN_CORRECTIONS ||
            fabsf(err) > ROBOT_MAX_HEADING_ERROR_DEG) {
            Robot_Stop(r, ROBOT_CORRECTION_LIMIT, now); return;
        }
        if (!begin_segment(r, 1, err, fminf(30.0f, r->config.max_speed_mm_s),
                           fminf(k * 10.0f, r->config.max_accel_mm_s2), now))
            Robot_Stop(r, ROBOT_BAD_ARGUMENT, now);
        else { ++r->corrections; ++r->mission_turn_corrections; }
        return;
    }
    ++r->step;
    if (r->step >= r->total_steps) {
        /* 任务完成后按转向总账自动标定 yaw_scale_ppm：校正闭环把掉头锁死在
         * 陀螺仪说的目标上（陀螺仪总账 = 指令角），编码器总账 = 指令角/
         * (1+εg)（执行容差级噪声，与轮距/轮径标度无关——编码器角度由指令
         * 计数换算，轮距在分子分母对消）。比值即 1/(1+εg)。累计转向 ≥360°
         * 且比值落在 [0.99,1.01] 才接受（否则可能有滑移/异常，保留原值）。 */
        r->yaw_auto_last_ratio = 0.0f; r->yaw_auto_applied = 0;
        /* 门限取绝对值：方形任务顺时针（负角度），总账是 −1800°，不能按
         * 正方向比较（否则方形任务永远进不了自动标定）。 */
        if (r->config.yaw_auto_calibrate &&
            fabsf(r->mission_turn_gyro_deg) >= 360.0f) {
            float ratio = r->mission_turn_encoder_deg / r->mission_turn_gyro_deg;
            r->yaw_auto_last_ratio = ratio;
            if (ratio > 0.99f && ratio < 1.01f) {
                int32_t current = (r->job == ROBOT_SQUARE) ?
                    r->config.yaw_scale_ppm_square : r->config.yaw_scale_ppm;
                long updated = (long)current + lroundf((ratio - 1.0f) * 1.0e6f);
                if (updated < -50000) updated = -50000;
                if (updated > 50000) updated = 50000;
                /* 分任务槽位：只更新当前任务对应的系数，另一个任务的标定不动。 */
                if (r->job == ROBOT_SQUARE)
                    r->config.yaw_scale_ppm_square = (int32_t)updated;
                else
                    r->config.yaw_scale_ppm = (int32_t)updated;
                r->yaw_auto_applied = 1;
            }
        }
        r->state = ROBOT_DONE; return;
    }
    r->state = ROBOT_DWELL; r->dwell_start_ms = now;
}
void Robot_Tick(Robot *r, uint32_t now, const RobotGyro *g)
{
    unsigned i; int reached = 1, still = 1; float err;
    r->now_ms = now; r->gyro = *g;
    poll_position(r, now);
    if (r->state == ROBOT_FAULT || r->state == ROBOT_STOPPED) { monitor_stop(r, now); return; }
    if (!Robot_IsBusy(r)) return;
    if (!gyro_ready(r, now)) { Robot_Stop(r, ROBOT_GYRO_UNREADY, now); return; }
    if (!fresh(r, now)) { Robot_Stop(r, ROBOT_FEEDBACK_STALE, now); return; }
    if (r->ack_mask) {
        if ((uint32_t)(now-r->ack_start_ms) >= r->config.ack_timeout_ms)
            Robot_Stop(r, ROBOT_ACK_TIMEOUT, now);
        return;
    }
    if (r->state == ROBOT_DWELL) {
        if ((uint32_t)(now-r->dwell_start_ms) >= r->config.dwell_ms && !begin_step(r, now))
            Robot_Stop(r, ROBOT_BAD_ARGUMENT, now);
        return;
    }
    if ((uint32_t)(now-r->start_ms) > r->timeout_ms) { Robot_Stop(r, ROBOT_MOVE_TIMEOUT, now); return; }
    if (r->state == ROBOT_SETUP) {
        uint32_t speed = value_raw(&r->config, r->speed_mm_s);
        uint32_t accel = value_raw(&r->config, r->acceleration_mm_s2);
        switch (r->phase++) {
        case 0: send_pair(r, 0x413, 0, 0, now); break;
        case 1: send_pair(r, 0x414, accel, accel, now); break;
        case 2: send_pair(r, 0x415, speed, speed, now); break;
        case 3:
            send_pair(r, 0x416, 0, 0, now); r->last_progress_ms = now;
            r->wheel_progress_ms[0] = r->wheel_progress_ms[1] = now; break;
        default: r->state = ROBOT_RUNNING; r->last_control_ms = now; break;
        }
        return;
    }
    for (i = 0; i < 2; ++i) {
        r->progress_mm[i] = (float)((int64_t)r->wheel[i].raw-r->start_raw[i]) *
            r->config.position_mm_per_count[i] * r->config.forward_sign[i];
        r->remaining_mm[i] = (float)((int64_t)r->target_raw[i]-r->wheel[i].raw) *
            r->config.position_mm_per_count[i];
        if (fabsf(r->remaining_mm[i]) > r->config.position_tolerance_mm) reached = 0;
        if (fabs((double)r->wheel[i].raw-r->progress_raw[i]) >= 1.0) {
            r->last_progress_ms = now; r->progress_raw[i] = r->wheel[i].raw;
            r->wheel_progress_ms[i] = now;
        }
        if (fabsf(r->remaining_mm[i]) > r->config.position_tolerance_mm &&
            (uint32_t)(now-r->wheel_progress_ms[i]) > 2000U) {
            Robot_Stop(r, ROBOT_STALLED, now); return;
        }
    }
    err = r->target_yaw_deg-yaw(r);
    if (!r->turning && fabsf(err) > ROBOT_MAX_HEADING_ERROR_DEG) {
        Robot_Stop(r, ROBOT_HEADING_ERROR, now); return;
    }
    if (!reached && (uint32_t)(now-r->last_progress_ms) > 2000U) {
        Robot_Stop(r, ROBOT_STALLED, now); return;
    }
    if (r->wheel[0].sequence != r->evaluated_sequence[0] &&
        r->wheel[1].sequence != r->evaluated_sequence[1] &&
        r->gyro.sequence != r->evaluated_gyro_sequence) {
        for (i = 0; i < 2; ++i) {
            if (fabs((double)r->wheel[i].raw-r->settle_raw[i]) *
                r->config.position_mm_per_count[i] > 0.2) still = 0;
            r->evaluated_sequence[i] = r->wheel[i].sequence;
            if (!r->settling) r->settle_raw[i] = r->wheel[i].raw;
        }
        r->evaluated_gyro_sequence = r->gyro.sequence;
        if (!reached || !still || fabsf(r->gyro.rate_deg_s) > 1.0f) r->settling = 0;
        else if (!r->settling) { r->settling = 1; r->settle_ms = now; }
        else if ((uint32_t)(now-r->settle_ms) >= SETTLE_MS) { finish_or_correct(r, now); return; }
    }
    /* 固定终点 + 运行中限幅调整左右速度（对齐同课题高精度参考实现）；
     * 每对速度命令须收到各自 ACK。正负运动统一到车体坐标，协议速度始终
     * 为非负数，方向来自目标位置。航向增益按速度归一：delta = kp×err×v/100，
     * kp 的含义是"100 mm/s 时每 1° 航向误差产生多少 mm/s 差速"。高速下若
     * 沿用固定 mm/s 增益，同样的初始航向误差会带来成比例的横向偏移
     * （τ 不变但走过的路更长），所以差速随速度放大；限幅 ±25% 速度，
     * 等价于误差超过 6.25° 才饱和。到位（reached）后不再更新速度，末段
     * 由电机位置环收尾；掉头段不做差速。值未变化时 send_pair 按已应答
     * 缓存自动跳过。 */
    if (!r->turning && !reached && (uint32_t)(now-r->last_control_ms) >= CONTROL_PERIOD_MS) {
        float direction = r->amount < 0.0f ? -1.0f : 1.0f;
        float delta = clamp(r->config.heading_kp_mm_s_deg*err*(r->speed_mm_s*0.01f),
                            -0.25f*r->speed_mm_s, 0.25f*r->speed_mm_s);
        float left = clamp(r->speed_mm_s-direction*delta, 1.0f, r->config.max_speed_mm_s);
        float right = clamp(r->speed_mm_s+direction*delta, 1.0f, r->config.max_speed_mm_s);
        send_pair(r, 0x415, value_raw(&r->config, left), value_raw(&r->config, right), now);
        r->last_control_ms = now;
    }
}
