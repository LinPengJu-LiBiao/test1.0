/**
  * @file    motor_protocol.c
  * @brief   双电机 CAN 协议编解码：只负责组帧/解帧，不持有运行状态
  * @note    多字节数值一律小端；设备号放在各帧约定的字节位置。
  */
#include "motor_protocol.h"
#include <string.h>

/* 构造指定 ID/DLC 的空帧，未用字节保持 0。 */
static MotorFrame frame(uint16_t id, uint8_t len)
{
    MotorFrame f = {0}; f.id = id; f.len = len; return f;
}
/* 按小端写入 32 位数值。 */
static void put32(uint8_t *p, uint32_t v)
{
    unsigned i; for (i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
/* 按小端读取有符号 32 位；负数用补码换算，不做实现定义的右移。 */
int32_t Motor_ReadI32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return u <= INT32_MAX ? (int32_t)u : -1 - (int32_t)(UINT32_MAX - u);
}
MotorFrame Motor_Enable(uint8_t device, int enable)
{
    MotorFrame f = frame(0x413, 5); f.data[0] = enable ? 0x5A : 0x4A;
    f.data[4] = device; return f;
}
MotorFrame Motor_Value(uint16_t id, uint8_t device, uint32_t value)
{
    MotorFrame f = frame(id, 5); put32(f.data, value); f.data[4] = device; return f;
}
MotorFrame Motor_Target(uint8_t device, int32_t position)
{
    MotorFrame f = Motor_Value(0x416, device, (uint32_t)position);
    f.len = 6; f.data[5] = 0x05; return f;
}
MotorFrame Motor_QueryPosition(uint8_t device)
{
    MotorFrame f = frame(0x408, 2); f.data[0] = device; f.data[1] = 0x08; return f;
}
MotorFrame Motor_Bind(uint8_t device, const uint8_t sn[7])
{
    MotorFrame f = frame(0x313, 8); memcpy(f.data, sn, 7); f.data[7] = device; return f;
}
MotorFrame Motor_Discover(uint8_t device_type)
{
    MotorFrame f = frame(0x60D, 2); f.data[0] = device_type; return f;
}
MotorFrame Motor_Parameters(uint8_t device, const uint8_t parameters[7])
{
    MotorFrame f = frame(0x316, 8); memcpy(f.data, parameters, 7);
    /* Byte6 低四位是马达号，Byte7 是设备号。本车令两者一致，避免两轮沿用
     * 厂家默认马达号 01；高四位方向/自动反馈等配置位原样保留。
     * 上层已把 device 限定在 1..15。 */
    f.data[6] = (uint8_t)((f.data[6] & 0xF0U) | (device & 0x0FU));
    f.data[7] = device; return f;
}
MotorFrame Motor_WheelDiameter(uint8_t device, uint16_t diameter)
{
    MotorFrame f = frame(0x408, 4); f.data[0] = device; f.data[1] = 0x24;
    f.data[2] = (uint8_t)diameter; f.data[3] = (uint8_t)(diameter >> 8); return f;
}
int Motor_Position(const MotorFrame *f, uint8_t device, int32_t *position)
{
    /* 只有"ID+DLC+设备号+子命令"四项全部匹配，才把 4 字节当作位置。 */
    if (f->id != 0x409 || f->len != 8 || f->data[0] != device || f->data[1] != 0x08)
        return 0;
    *position = Motor_ReadI32(f->data + 2); return 1;
}
int Motor_Ack(const MotorFrame *f, uint8_t device, uint16_t command, uint8_t be)
{
    uint16_t echoed;
    if (f->id != 0x41A || f->len != 3 || f->data[2] != device) return 0;
    echoed = be ? (uint16_t)(((uint16_t)f->data[0] << 8) | f->data[1]) :
                  (uint16_t)(((uint16_t)f->data[1] << 8) | f->data[0]);
    return echoed == command;
}
