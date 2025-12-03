#ifndef _SERVO_CONFIG_H_
#define _SERVO_CONFIG_H_

#include <stdint.h>

// 舵机指令协议定义
typedef enum {
    CMD_READ_FULL_STATE  = 0x05,    // 读取12个舵机完整状态
    CMD_READ_KEY_STATE   = 0x04,    // 读取5个关键舵机状态
    CMD_READ_IMU         = 0x03,    // 读取IMU数据
    CMD_SET_PARAMS       = 0x07,    // 设置参数
} rl_command_t;

// 舵机寄存器地址 (基于HLS协议)
typedef enum {
    HLS_PRESENT_POSITION_L = 56,
    HLS_MODE = 0x21,
    HLS_KP = 0x15,
    HLS_KD = 0x16,
} servo_register_t;

// 舵机ID配置
#define SERVO_BROADCAST_ID 0xFE

// 完整组舵机 (12个)
static const uint8_t FULL_SERVO_IDS[12] = {
    33, 32, 31,     // 组1
    43, 42, 41,     // 组2  
    23, 22, 21,     // 组3
    13, 12, 11      // 组4
};

// 关键舵机 (5个)
static const uint8_t KEY_SERVO_IDS[5] = {
    13, 12, 11,     // 关键组1
    23, 22          // 关键组2
};

// 数据包大小配置
#define FULL_SERVO_DATA_SIZE 168    // 12舵机 × 14字节
#define KEY_SERVO_DATA_SIZE  70     // 5舵机 × 14字节

#endif /* _SERVO_CONFIG_H_ */