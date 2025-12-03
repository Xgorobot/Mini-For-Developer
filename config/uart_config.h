#ifndef _UART_CONFIG_H_
#define _UART_CONFIG_H_

// UART引脚配置
#define UART1_TXD_PIN      (GPIO_NUM_5)   // 树莓派通信
#define UART1_RXD_PIN      (GPIO_NUM_4)
#define UART2_TXD_PIN      (GPIO_NUM_14)  // 舵机通信  
#define UART2_RXD_PIN      (GPIO_NUM_13)

// 其他硬件引脚
#define BLINK_LED_PIN      GPIO_NUM_22    // LED指示灯
#define I2C_MASTER_SCL_IO  GPIO_NUM_19    // I2C (IMU预留)
#define I2C_MASTER_SDA_IO  GPIO_NUM_18
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */
static icm20948_handle_t icm20948 = NULL;
// UART参数配置
#define UART_BAUDRATE_1      115200
#define UART_BAUDRATE_2      1000000
#define UART_BUFFER_SIZE   1024

// 任务配置
#define TX_TASK_PRIORITY   7      // 指令处理任务优先级
#define RX_TASK_PRIORITY   6      // 数据采集任务优先级
#define TASK_STACK_SIZE    2048

#endif /* _UART_CONFIG_H_ */