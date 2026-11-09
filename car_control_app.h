#ifndef CAR_CONTROL_APP_H
#define CAR_CONTROL_APP_H
#include "robot_control.h"

/* Watch 命令码：调试器先写参数，最后写 command；本层在同一拍内一次性取走并清零。
 * 0..11 是已发布的调试接口，新增命令只能往后排，既有取值不得改动。 */
typedef enum {
    CC_NONE,                      /* 0  无待处理命令 */
    CC_APPLY_CONFIG,              /* 1  复制 g_robotSettings 到运行配置，仅 IDLE 允许 */
    CC_BIND_LEFT,                 /* 2  按 SN 绑定左电机，设备号取 device[0] */
    CC_BIND_RIGHT,                /* 3  按 SN 绑定右电机，设备号取 device[1] */
    CC_INITIALIZE_AND_ENABLE,     /* 4  12 帧非阻塞配置：使能关→参数→轮径→使能开 */
    CC_STRAIGHT,                  /* 5  直行 amount=mm、speed=mm/s、acceleration=mm/s² */
    CC_TURN,                      /* 6  原地转向 amount=deg、speed=deg/s、acceleration=deg/s² */
    CC_SQUARE,                    /* 7  方形路线 amount=边长mm、repetitions=圈数 */
    CC_SHUTTLE,                   /* 8  往返路线 amount=单程mm、repetitions=往返次数 */
    CC_STOP,                      /* 9  释放两轮并进入停止态，优先于当拍状态推进 */
    CC_RESET,                     /* 10 停止/故障后确认静止，再回到 IDLE */
    CC_ENABLE_MOTORS = 11,        /* 11 只使能两轮，用于首次取得位置反馈 */
    CC_WRITE_MOTOR_CONFIG = CC_INITIALIZE_AND_ENABLE /* 旧名称，命令值仍为 4 */
} CarControlCommand;
/* 命令参数表。单段运动只用 amount/speed/acceleration；路线类只用 amount 与
 * repetitions，直行参数取 speed/acceleration，转向参数取 RobotConfig.turn_*。
 * include_final_turn 仅对 SHUTTLE 有效：1=往返结束后补最后一次掉头。
 * 路线类命令必须把 route_definition_confirmed 置 1，防止误触发多步运动。 */
typedef struct {
    float amount, speed, acceleration;
    uint8_t repetitions, include_final_turn, route_definition_confirmed;
    uint32_t command;
} CarControlRequest;
extern volatile RobotConfig g_robotSettings;
extern volatile CarControlRequest g_robotRequest;
extern Robot g_robot;
extern volatile uint32_t g_robotCommandResult, g_robotCommandCount;
extern volatile uint32_t g_motorConfigFramesSubmitted;
extern volatile uint8_t g_lastDiscoveredSn[7];
extern volatile uint32_t g_discoveredSnCount;
extern const uint8_t g_leftMotorSn[7], g_rightMotorSn[7];
extern volatile uint8_t g_motorParameters[2][7];

void CarControl_Init(void);
int CarControl_OnFrame(uint16_t id, const uint8_t *data, uint8_t len, uint32_t now);
void CarControl_Poll(const RobotGyro *gyro, uint32_t now);
int CarControl_Busy(void);
int CarControl_GyroStationaryAllowed(uint32_t now);
int CarControl_GyroCalibrationBlocked(void);
extern volatile uint8_t g_robotGyroStationary;
extern volatile uint32_t g_robotGyroStationaryTick;
#endif
