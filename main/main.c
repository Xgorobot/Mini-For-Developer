
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "icm20948.h"
#include "esp_task_wdt.h"
#include "servo_driver.h"
#include "uart_comm.h"
#include "uart_config.h"
#include "servo_config.h"
#include "LED.h"

#define PI_UART_NUM UART_NUM_1
icm20948_acce_value_t acce,acce1; //加速度
icm20948_gyro_value_t gyro,gyro1; //陀螺仪
/*
未完成的函数：
1.读取并发送数据给树莓派
2.处理树莓派发送过来的数据
3.IMU
4.LED设置
*/
void app_main(void)
{
  //初始化
  uart_comm_init(SERVO_UART_NUM,UART2_TXD_PIN,UART2_RXD_PIN,UART_BAUDRATE_2);
  uart_comm_init(PI_UART_NUM,UART1_TXD_PIN,UART1_RXD_PIN,UART_BAUDRATE_1);
  i2c_bus_init();
  led_init();
  servo_initialize_12_servos(FULL_SERVO_IDS,200);
  //测试
  servo_set_position_speed_torque(KEY_SERVO_IDS[0], 2048, 100, 500, 100);
  //舵机标定测试
  servo_batch_calibrate_to_current_position(FULL_SERVO_IDS, 12, 50, 200, true);
}
