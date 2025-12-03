#ifndef _SERVO_DRIVER_H_
#define _SERVO_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "servo_config.h"
#include "driver/uart.h"

#ifndef SERVO_UART_NUM
#define SERVO_UART_NUM UART_NUM_2
#endif

#ifndef SERVO_MAX_RESPONSE_SIZE
#define SERVO_MAX_RESPONSE_SIZE 64  // 定义最大响应大小
#endif

#ifndef SERVO_MAX_PACKET_SIZE  
#define SERVO_MAX_PACKET_SIZE 128   // 定义最大数据包大小
#endif

// 舵机状态枚举
typedef enum {
    SERVO_EEPROM_UNLOCKED = 0,  // 解锁状态（可修改）
    SERVO_EEPROM_LOCKED = 1,    // 锁定状态（只读）
    SERVO_EEPROM_UNKNOWN = 2    // 状态未知
} servo_eeprom_lock_state_t;

typedef enum {
    SERVO_MODE_POSITION = 0x00,    // 位置模式
    SERVO_MODE_VELOCITY = 0x01,    // 恒速模式
    SERVO_MODE_TORQUE = 0x02,      // 恒流模式
    SERVO_MODE_PWM = 0x03     // 开环模式
} servo_mode_t;

// 指令枚举
enum SERVO_INSTRUCTION {
    SERVO_PING = 0x01,          // 查询舵机是否存在
    SERVO_READ_DATA = 0x02,     // 读取寄存器数据
    SERVO_WRITE_DATA = 0x03,    // 写入数据（立即执行）
    SERVO_REG_WRITE = 0x04,     // 异步写入（准备指令）
    SERVO_ACTION = 0x05,        // 执行异步指令
    SERVO_SYNC_READ = 0x82,     // 同步读（批量控制）
    SERVO_SYNC_WRITE = 0x83,    // 同步写（批量控制）
    SERVO_CAL = 0x0B,           // 位置校准
    SERVO_REBOOT = 0x08,        // 舵机重启
    SERVO_RESET = 0x0A,         // 重置舵机圈数
};

// 错误码枚举
enum SERVO_ERROR {
    SERVO_VOLTAGE_ERROR = 0x01,     // 电压异常
    SERVO_ANGLE_ERROR = 0x02,       // 角度超限
    SERVO_TEMPERATURE_ERROR = 0x03, // 温度过高
    SERVO_RANGE_ERROR = 0x04,       // 参数范围错误
    SERVO_CHECKSUM_ERROR = 0x05,    // 校验和错误
    SERVO_LOCKED_ERROR = 0x06,      // 舵机锁死
    SERVO_PARSE_ERROR = 0x07,       // 数据解析错误
};

// 状态码枚举
enum SERVO_STATUS_ERROR {
    SERVO_STATUS_OK = 0x00,         // 通信正常
    SERVO_STATUS_NO_ANSWER = 0x01,  // 无响应（超时）
    SERVO_STATUS_SLAVE_ERROR = 0x02,// 从设备错误
    SERVO_STATUS_LENGTH_ERROR = 0x03,// 数据长度错误
    SERVO_STATUS_CRC_ERROR = 0x04,  // CRC校验错误
};

// 寄存器地址枚举
enum SERVO_REG {
    // EPROM配置寄存器
    SERVO_REG_ID = 0x05,                     // 舵机ID
    SERVO_REG_BAUDRATE = 0x06,               // 通信波特率
    SERVO_REG_MODE = 0x21,                   // 运行模式
    SERVO_REG_MIN_ANGLE_LIMIT_LOW = 0x09,    // 最小角度限制低字节
    SERVO_REG_MAX_ANGLE_LIMIT_LOW = 0x0B,    // 最大角度限制低字节
    SERVO_REG_TEMP_LIMIT = 0x0D,             // 温度限制
    SERVO_REG_VOLTAGE_LIMIT_LOW = 0x0F,      // 电压限制低字节
    SERVO_REG_VOLTAGE_LIMIT_HIGH = 0x0E,     // 电压限制高字节
    SERVO_REG_TORQUE_MAX = 0x10,             // 最大扭矩设置
    SERVO_REG_TORQUE_START_MIN = 0x18,       // 最小启动扭矩
    SERVO_REG_LOAD_SAVE = 0x13,              // 舵机卸载保护
    SERVO_REG_LED = 0x14,                    // LED控制
    SERVO_REG_EEPROM_LOCK = 0x37,            // 内存锁

    // SRAM控制寄存器
    SERVO_REG_TORQUE_ENABLE = 0x28,          // 扭矩开关
    SERVO_REG_ACCELERATION = 0x29,           // 加速度
    SERVO_REG_GOAL_POSITION_LOW = 0x2A,      // 目标位置低字节
    SERVO_REG_GOAL_SPEED_LOW = 0x2E,         // 目标速度低字节
    SERVO_REG_GOAL_TORQUE_LOW = 0x2C,        // 目标扭矩低字节

    // SRAM状态反馈寄存器
    SERVO_REG_CURRENT_POSITION_LOW = 0x38,   // 当前位置低字节
    SERVO_REG_CURRENT_SPEED_LOW = 0x3A,      // 当前速度低字节
    SERVO_REG_CURRENT_LOAD_LOW = 0x3C,       // 当前负载低字节
    SERVO_REG_CURRENT_VOLTAGE = 0x3E,        // 当前电压
    SERVO_REG_CURRENT_TEMP = 0x3F,           // 当前温度
    SERVO_REG_MOVING = 0x42,                 // 运动状态
};

// ============== 工具函数 ==============
/**
 * @brief 设置大小端模式
 * @param val 0:小端模式, 1:大端模式
 */
void servo_set_END(uint8_t val);

/**
 * @brief 获取当前大小端模式
 * @return 当前大小端模式
 */
uint8_t servo_get_END(void);

/**
 * @brief 将两个8位数据转换为16位数据（考虑大小端）
 * @param low 低字节
 * @param high 高字节
 * @return 转换后的16位数据
 */
uint16_t servo_Transform_8_To_16(uint8_t low, uint8_t high);

/**
 * @brief 调试打印数据包
 * @param label 标签
 * @param data 数据指针
 * @param len 数据长度
 */
void servo_debug_print_packet(const char* label, uint8_t* data, uint32_t len);

/**
 * @brief 计算校验和
 * @param data 数据指针
 * @param start_idx 起始索引
 * @param len 数据长度
 * @return 校验和
 */
uint8_t servo_calculate_checksum(uint8_t *data, uint8_t start_idx, uint8_t len);

/**
 * @brief 初始化舵机驱动
 * @return esp_err_t ESP错误码
 */
esp_err_t servo_driver_init(void);

/**
 * @brief 构建通用数据包（除同步读写外）
 * @param servo_id 舵机ID
 * @param instruction 指令
 * @param parameters 参数数组
 * @param param_count 参数数量
 * @param packet 输出数据包缓冲区
 * @return 数据包长度
 */
uint8_t servo_common_packet(uint8_t servo_id, uint8_t instruction, 
                           const uint8_t* parameters, uint8_t param_count, 
                           uint8_t* packet);

/**
 * @brief 构建同步读取数据包
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param addr 起始地址
 * @param num 读取字节数
 * @param packet 输出数据包缓冲区
 * @return 数据包长度
 */
uint8_t servo_sync_read_packet(const uint8_t* servo_ids, uint8_t id_count, 
                              uint8_t addr, uint8_t num, uint8_t* packet);

/**
 * @brief 构建同步写入数据包
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param addr 起始地址
 * @param data 数据数组
 * @param data_len_per_servo 每个舵机的数据长度
 * @param packet 输出数据包缓冲区
 * @return 数据包长度
 */
uint8_t servo_sync_write_packet(const uint8_t* servo_ids, uint8_t id_count, 
                               uint8_t addr, const uint8_t* data, 
                               uint8_t data_len_per_servo, uint8_t* packet);


/**
 * @brief 发送数据包
 * @param uart_num UART端口号
 * @param packet 数据包
 * @param packet_len 数据包长度
 * @param response 响应缓冲区
 * @param timeout_ms 超时时间(毫秒)
 * @param expect_response 是否期待响应
 * @return 发送/接收的字节数
 */
int servo_send_packet(uart_port_t uart_num, const uint8_t* packet, uint8_t packet_len, 
                     uint8_t* response, uint32_t timeout_ms, bool expect_response);

/**
 * @brief 解析响应数据
 * @param packet 响应数据包
 * @param length 数据包长度
 * @param expected_id 期望的舵机ID
 * @param data 输出数据缓冲区
 * @param data_len 期望的数据长度
 * @return 状态码
 */
static int servo_parse_response(uint8_t* packet, uint16_t length, 
                               uint8_t expected_id, uint8_t* data, uint8_t data_len);

static void servo_log_error(uint8_t id, uint8_t error_code);
// ============== 基础功能 ==============
/**
 * @brief 查询舵机状态
 * @param id 舵机ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_ping(uint8_t id, uint32_t timeout_ms);

/**
 * @brief 读取舵机数据
 * @param id 舵机ID
 * @param addr 起始地址
 * @param num 读取字节数
 * @param data 输出数据缓冲区
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_read_data(uint8_t id, uint8_t addr, uint8_t num, 
                   uint8_t* data, uint32_t timeout_ms);

/**
 * @brief 写入舵机数据
 * @param id 舵机ID
 * @param addr 起始地址
 * @param data 写入数据
 * @param data_len 数据长度
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_write_data(uint8_t id, uint8_t addr, const uint8_t* data, 
                    uint8_t data_len, uint32_t timeout_ms);

/**
 * @brief 安全写入EEPROM（自动处理锁状态）
 * @param id 舵机ID
 * @param addr 起始地址
 * @param data 写入数据
 * @param data_len 数据长度
 * @param require_unlock 是否需要解锁
 * @param restore_lock 是否恢复锁状态
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_safe_write_eeprom(uint8_t id, uint8_t addr, const uint8_t* data, 
                           uint8_t data_len, bool require_unlock, bool restore_lock,
                           uint32_t timeout_ms);

/**
 * @brief 检查EEPROM锁状态
 * @param id 舵机ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 锁状态
 */
int servo_check_eeprom_lock(uint8_t id, uint32_t timeout_ms);

/**
 * @brief 同步写入多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param addr 起始地址
 * @param data 数据数组
 * @param data_len_per_servo 每个舵机的数据长度
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_sync_write(const uint8_t* servo_ids, uint8_t id_count, 
                    uint8_t addr, const uint8_t* data, 
                    uint8_t data_len_per_servo, uint32_t timeout_ms);

/**
 * @brief 同步读取多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param addr 起始地址
 * @param num 读取字节数
 * @param responses 响应缓冲区数组
 * @param response_sizes 响应大小数组
 * @param timeout_ms 超时时间(毫秒)
 * @return 成功读取的舵机数量
 */
int servo_sync_read(const uint8_t* servo_ids, uint8_t id_count, 
                   uint8_t addr, uint8_t num,
                   uint8_t** responses, uint8_t* response_sizes, 
                   uint32_t timeout_ms);

/**
 * @brief 重置舵机圈数
 * @param id 舵机ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_reset(uint8_t id, uint32_t timeout_ms);

/**
 * @brief 批量重置多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param timeout_ms 超时时间(毫秒)
 * @return 成功重置的舵机数量
 */
int servo_batch_reset(const uint8_t* servo_ids, uint8_t id_count, uint32_t timeout_ms);

/**
 * @brief 校准舵机
 * @param id 舵机ID
 * @param position 目标位置(0xFFFF表示中心位置)
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_calibrate(uint8_t id, uint16_t position, uint32_t timeout_ms);

/**
 * @brief 校准舵机到中心位置
 * @param id 舵机ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_calibrate_center(uint8_t id, uint32_t timeout_ms);

/**
 * @brief 重启舵机
 * @param id 舵机ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_reboot(uint8_t id, uint32_t timeout_ms);

/**
 * @brief 批量重启多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param stagger_ms 重启间隔时间(毫秒)
 * @return 成功发送重启命令的舵机数量
 */
int servo_batch_reboot(const uint8_t* servo_ids, uint8_t id_count, uint16_t stagger_ms);

// ============== 功能函数 ==============
/**
 * @brief 设置舵机模式
 * @param id 舵机ID
 * @param mode 模式
 * @param timeout_ms 超时时间(毫秒)
 * @return 状态码
 */
int servo_set_mode(uint8_t id, servo_mode_t mode, uint32_t timeout_ms);

/**
 * @brief 初始化单个舵机
 * @param id 舵机ID
 * @return 状态码
 */
int servo_initialize(uint8_t id);

/**
 * @brief 初始化12个舵机
 * @param servo_ids 舵机ID数组(12个)
 * @param stagger_ms 间隔时间(毫秒)
 * @return 成功初始化的舵机数量
 */
int servo_initialize_12_servos(const uint8_t* servo_ids, uint16_t stagger_ms);

/**
 * @brief 控制单个舵机
 * @param id 舵机ID
 * @param position 目标位置
 * @param speed 目标速度
 * @param torque 目标力矩
 * @param timeout_ms 间隔时间(毫秒)
 * @return 返回报错值
 */
int servo_set_position_speed_torque(uint8_t id, uint16_t position, uint16_t speed, uint16_t torque, uint32_t timeout_ms);
/**
 * @brief 批量标定多个舵机到当前位置为零位
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param stagger_ms 操作间隔时间(毫秒)，避免同时操作产生大电流
 * @param timeout_ms 单次操作超时时间(毫秒)
 * @return 成功标定的舵机数量
 */
int servo_batch_calibrate_to_current_position(const uint8_t* servo_ids, 
                                            uint8_t id_count, 
                                            uint16_t stagger_ms,
                                            uint32_t timeout_ms,
                                            bool wait_for_user);

/**
 * @brief 批量卸载（关闭扭矩）多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param stagger_ms 操作间隔时间(毫秒)
 * @param timeout_ms 单次操作超时时间(毫秒)
 * @return 成功卸载的舵机数量
 */
int servo_batch_unload_torque(const uint8_t* servo_ids, 
                            uint8_t id_count, 
                            uint16_t stagger_ms,
                            uint32_t timeout_ms);

/**
 * @brief 批量开启扭矩多个舵机
 * @param servo_ids 舵机ID数组
 * @param id_count ID数量
 * @param stagger_ms 操作间隔时间(毫秒)
 * @param timeout_ms 单次操作超时时间(毫秒)
 * @return 成功开启的舵机数量
 */
int servo_batch_enable_torque(const uint8_t* servo_ids, 
                            uint8_t id_count, 
                            uint16_t stagger_ms,
                            uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif

#endif /* _SERVO_DRIVER_H_ */