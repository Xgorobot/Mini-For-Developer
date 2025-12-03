#include "uart_comm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>  
static const char *TAG = "UART_COMM";


esp_err_t uart_comm_init(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baudrate)
{
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // 配置UART参数
    esp_err_t ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d param config failed: 0x%x", uart_num, ret);
        return ret;
    }
    
    // 设置UART引脚
    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d set pin failed: 0x%x", uart_num, ret);
        return ret;
    }
    
    // 安装UART驱动程序
    const int uart_buffer_size = 1024;
    ret = uart_driver_install(uart_num, uart_buffer_size * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d driver install failed: 0x%x", uart_num, ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "UART%d initialized: TX=%d, RX=%d, Baud=%u", uart_num, tx_pin, rx_pin, baudrate);
    return ESP_OK;  
}
