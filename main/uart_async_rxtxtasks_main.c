/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "icm20948.h"
#include "freertos/FreeRTOSConfig.h"
#include "esp_task_wdt.h"

#define I2C_MASTER_SCL_IO  19        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  18        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */
static const char *TAG = "icm test";
static icm20948_handle_t icm20948 = NULL;
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN 14//(CONFIG_EXAMPLE_UART_TXD)
#define RXD_PIN 13//(CONFIG_EXAMPLE_UART_RXD)

#define TXD_PIN1 5
#define RXD_PIN1 4

#define GPIO_OUTPUT_IO_0    23
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))
uint8_t bBuf[20];
uint8_t wBuf[20];
uint8_t rBuf[20];
uint8_t* data;
uint8_t* data1;
bool flag_data=1;
icm20948_acce_value_t acce,acce1;
icm20948_gyro_value_t gyro,gyro1;
static esp_err_t i2c_bus_init(void)
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (ret != ESP_OK)
		return ret;

	return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;

	/*
	 * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
	 * if address pin pulled low (to GND)
	 */
	icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS_1);
	if (icm20948 == NULL) {
		ESP_LOGE(TAG, "ICM20948 create returned NULL!");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "ICM20948 creation successfull!");

	ret = icm20948_reset(icm20948);
	if (ret != ESP_OK)
		return ret;

	vTaskDelay(10 / portTICK_PERIOD_MS);

	ret = icm20948_wake_up(icm20948);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_set_bank(icm20948, 0);
	if (ret != ESP_OK)
		return ret;

	uint8_t device_id;
	ret = icm20948_get_deviceid(icm20948, &device_id);
	if (ret != ESP_OK)
		return ret;
	ESP_LOGI(TAG, "0x%02X", device_id);
	if (device_id != ICM20948_WHO_AM_I_VAL)
		return ESP_FAIL;

	ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = icm20948_set_acce_fs(icm20948, acce_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	return ret;
}
void init(void)
{
    gpio_config_t gpio_conf = {
        .intr_type      = GPIO_INTR_DISABLE,      //禁用中断
        .mode           = GPIO_MODE_OUTPUT,           //设置为输出模式
        .pin_bit_mask   = GPIO_OUTPUT_PIN_SEL,        //输出指定引脚
        .pull_down_en   = 0,                          //禁止下拉
        .pull_up_en     = 0,                          //禁止上拉
    };
    gpio_config(&gpio_conf);

    const uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits=UART_DATA_8_BITS,                //8位数据位
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,        //无硬件流控制
        .parity=UART_PARITY_DISABLE,                //无校验位
        .stop_bits=UART_STOP_BITS_1,                 //1位停止位
        .source_clk = UART_SCLK_DEFAULT,  // ESP32-S3支持APB时钟
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const uart_config_t uart_config1 = {
        .baud_rate = 115200,
        .data_bits=UART_DATA_8_BITS,                //8位数据位
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,        //无硬件流控制
        .parity=UART_PARITY_DISABLE,                //无校验位
        .stop_bits=UART_STOP_BITS_1,                 //1位停止位
        .source_clk = UART_SCLK_APB,  // ESP32-S3支持APB时钟
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config1));
    uart_set_pin(UART_NUM_1, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


}

int sendData(const char* logName, unsigned char* data)
{
    const int len = 20;
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData1(const char* logName, unsigned char* data)
{
    const int len = 15;
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData2(const char* logName, unsigned char* data, int length)
{
    const int txBytes = uart_write_bytes(UART_NUM_2, data, length);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
void init_wbuf()
{
    int n=1;
    int l=6;
    int i =0;
    uint8_t checkSum = 0x00;
// OX01 OX01 OXFF OX00 OX00 0X03 OXE8
    wBuf[0] = 0xff;
    wBuf[1] = 0xff;
    wBuf[2] = 0xfe;
    wBuf[3] = (l+1)*n+4;
    wBuf[4] = 0x83;
    wBuf[5] = 0x2a;
    wBuf[6] = 0x06;
    wBuf[7] = 11;
    wBuf[8] = 0x00;
    wBuf[9] = 0x00;
    wBuf[10] = 0x00;
    wBuf[11] = 0x00;
    wBuf[12] = 0x00;
    wBuf[13] = 0x01;
    for(i=2;i<14;i++)
	    checkSum = checkSum + wBuf[i];
	wBuf[14] = ~checkSum;
}
void init_rbuf()
{
    int i=0;
    uint8_t ID=0xfe;
	uint8_t checkSum = 0x00;
    rBuf[0] = 0xff;
	rBuf[1] = 0xff;
    rBuf[2] = 0x0b;//id
	rBuf[3] = 0x04;//length
	rBuf[4] = 0x02;//order
	rBuf[5] = 0x17;//addr
	rBuf[6] = 0x01;//1B
    for(i=2;i<7;i++)
	    checkSum = checkSum + rBuf[i];
	rBuf[7] = ~checkSum;
}
void init_buf()
{
    int i=0;
    uint8_t ID=0xfe;
	
	uint8_t checkSum = 0x00;
    uint8_t checkSum1 = 0x00;
    uint8_t checkSum2 = 0x00;
    uint8_t checkSum3 = 0x00;

    bBuf[0] = 0xff;
	bBuf[1] = 0xff;
    bBuf[2] = 0xfe;
	bBuf[3] = 12+4;
	bBuf[4] = 0x82;
	bBuf[5] = 0x38;
	bBuf[6] = 0x08;
    bBuf[7] = 21;
    bBuf[8] = 22;
    bBuf[9] = 23;
    bBuf[10] = 11;
    bBuf[11] = 12;
    bBuf[12] = 13;

    bBuf[13] = 31;
    bBuf[14] = 32;
    bBuf[15] = 33;
    bBuf[16] = 41;
    bBuf[17] = 42;
    bBuf[18] = 43;


    // bBuf[11] = 21;
    // bBuf[12] = 22;
    // bBuf[13] = 23;

    // bBuf[14] = 31;
    // bBuf[15] = 32;
    // bBuf[16] = 33;

    // bBuf[17] = 41;
    // bBuf[18] = 42;
    // bBuf[19] = 43;
    for(i=2;i<19;i++)
	    checkSum = checkSum + bBuf[i];
	bBuf[19] = ~checkSum;
}
static void tx_task(void *arg)//指令接收任务
{
    uint8_t* data8;
    int rxBytes;
    static const char *TX_TASK_TAG = "UART1_TASK";//1M
    data8 = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        rxBytes = uart_read_bytes(UART_NUM_1, data8, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(rxBytes > 0)
        {
            ESP_LOG_BUFFER_HEXDUMP(TX_TASK_TAG, data8, 10, ESP_LOG_INFO);
            // vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

static void rx_task(void *arg)//状态读取任务 指令下发任务
{
    esp_err_t ret1, ret2;
    
    static const char *RX_TASK_TAG = "UART2_TASK";//1M
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    data1 = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    static const char *TX_TASK_TAG = "TX_TASK";
    int gpio_io = 0;
    int rxBytes;
    
    
    while (1) {
        // sendData(TX_TASK_TAG, bBuf);// flag_data=1 data1,acce1,gyro1
        sendData2(TX_TASK_TAG,rBuf,8);
        if (flag_data)
        {
            rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
            ret1 = icm20948_get_acce(icm20948, &acce);
		    ret2 = icm20948_get_gyro(icm20948, &gyro);
        }
        else
        {
            rxBytes = uart_read_bytes(UART_NUM_2, data1, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
            ret1 = icm20948_get_acce(icm20948, &acce1);
		    ret2 = icm20948_get_gyro(icm20948, &gyro1);
        }
        if (rxBytes > 0 && ret1 == ESP_OK && ret2 == ESP_OK) {
            // vTaskDelay(5 / portTICK_PERIOD_MS);
            // data[rxBytes] = 0;
            // ESP_LOG_BUFFER_HEXDUMP(TX_TASK_TAG, bBuf, 20, ESP_LOG_INFO);
            if(flag_data)
            {
                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data+12+14*3, 14, ESP_LOG_INFO);//12+14*12
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);//data[12：] 当前位置2、当前速度2、当前负载2、当前电压1、当前温度1  低前 高后
            }
            else
            {
                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data1+12+14*3, 14, ESP_LOG_INFO);
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data1, rxBytes, ESP_LOG_INFO);
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data1);
            }
            flag_data =!flag_data;//flag_data true data 接收 返回 data1数据 
            rxBytes = 0;
            ret1=0;
            ret2=0;
            
            gpio_set_level(GPIO_OUTPUT_IO_0, gpio_io);
            gpio_io = ~gpio_io;
            // sendData1(TX_TASK_TAG, wBuf);
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    init_rbuf();
    ESP_LOGI(TAG, "Starting ICM test");
	esp_err_t ret = i2c_bus_init();
	ESP_LOGI(TAG, "I2C bus initialization: %s", esp_err_to_name(ret));
    ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_1000DPS);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ICM configuration failure");
		vTaskDelete(NULL);
	}
    init_buf();
    init_wbuf();
    xTaskCreate(rx_task, "uart_rx_task", CONFIG_EXAMPLE_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);//比较设置核心与否  imu数据 电机数据截取（解码在上位机）控制指令下发
    xTaskCreate(tx_task, "uart_tx_task", CONFIG_EXAMPLE_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);//上位机数据
}
//kp 20 kd 0 ki 0