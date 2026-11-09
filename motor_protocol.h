#ifndef ROBOT_MOTOR_PROTOCOL_H
#define ROBOT_MOTOR_PROTOCOL_H
#include <stdint.h>

/* 双电机 CAN 协议编解码，依据《电机 CAN 通信协议 V11》。
 * 所有多字节数值沿用课题一实测的小端布局；每只电机占一个设备号，共用主通道。
 * device 取 1..15，0 保留为"未配置"。 */
typedef struct { uint16_t id; uint8_t len; uint8_t data[8]; } MotorFrame;
/* 0x413：使能/释放。enable=1 → Data[0]=0x5A，enable=0 → Data[0]=0x4A；Data[4]=设备号。 */
MotorFrame Motor_Enable(uint8_t device, int enable);
/* 0x414 / 0x415：加速度 / 速度，数值单位为 0.1（mm/s² 或 mm/s）。 */
MotorFrame Motor_Value(uint16_t id, uint8_t device, uint32_t value);
/* 0x416：绝对位置目标（0.1 mm 计数），Data[5]=0x05；超时不重发。 */
MotorFrame Motor_Target(uint8_t device, int32_t absolute_position);
/* 0x408：查询当前位置，Data[0]=设备号、Data[1]=0x08。 */
MotorFrame Motor_QueryPosition(uint8_t device);
/* 0x313：按 7 字节 SN 绑定设备号；SN 按原顺序放在 Data[0..6]，Data[7]=设备号。 */
MotorFrame Motor_Bind(uint8_t device, const uint8_t sn[7]);
/* 0x60D：按设备类型广播发现，电机以 0x312 回 7 字节 SN。 */
MotorFrame Motor_Discover(uint8_t device_type);
/* 0x316：写 7 字节参数。发送时把 Data[6] 低四位强制改为 device（本车令马达号 =
 * 设备号），高四位方向/自动反馈等配置位原样保留；Data[7] 也是设备号。 */
MotorFrame Motor_Parameters(uint8_t device, const uint8_t parameters[7]);
/* 0x408 子命令 0x24：轮径，单位 0.01 mm。 */
MotorFrame Motor_WheelDiameter(uint8_t device, uint16_t diameter_001mm);
int32_t Motor_ReadI32(const uint8_t *p);
/* 解析 0x409 位置反馈：ID、DLC=8、Data[0]=设备号、Data[1]=0x08 全部匹配才算有效。 */
int Motor_Position(const MotorFrame *frame, uint8_t device, int32_t *position);
/* 解析 0x41A 命令回显。V11 的 ACK 表格自相矛盾：DLC 写 2，却另列 Data[2]=设备号。
 * 课题二双设备控制只接受已确认的 3 字节 ACK，字节序由实机配置决定。 */
int Motor_Ack(const MotorFrame *frame, uint8_t device, uint16_t command,
              uint8_t original_id_big_endian);
#endif
