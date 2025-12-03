#ifndef _UART_COMM_H_
#define _UART_COMM_H_

#include <driver/uart.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化UART通信
 * @param uart_num UART端口号
 * @param tx_pin 发送引脚
 * @param rx_pin 接收引脚  
 * @param baudrate 波特率
 * @return esp_err_t 初始化结果
 */
esp_err_t uart_comm_init(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baudrate);


#ifdef __cplusplus
}
#endif

#endif /* _UART_COMM_H_ */