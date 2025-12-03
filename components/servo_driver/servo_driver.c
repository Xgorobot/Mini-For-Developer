#include "servo_driver.h"
#include "uart_comm.h"
#include "esp_log.h"
#include "string.h"
#include <inttypes.h> 
#include "driver/uart.h"
#include "esp_timer.h"


static const char *TAG = "SERVO_DRIVER";

#define SERVO_BROADCAST_ID 0xFE
#define SERVO_DEBUG_ENABLED false  
#define SERVO_MAX_SYNC_IDS 15
#define SERVO_MAX_SYNC_DATA_LEN 64
#define SERVO_PACKET_HEADER 8   
#define SERVO_SYNC_PACKET_MAX_SIZE (SERVO_PACKET_HEADER + SERVO_MAX_SYNC_IDS)
#define SERVO_MAX_PARAM_SIZE 96

static uint8_t END = 0;  // 大小端标志

// 初始化舵机通讯
esp_err_t servo_driver_init(void)
{
    extern int UART2_TXD_PIN, UART2_RXD_PIN;
    extern int UART_BAUDRATE_2;
    
    return uart_comm_init(SERVO_UART_NUM, UART2_TXD_PIN, UART2_RXD_PIN, UART_BAUDRATE_2);
}


//=========工具函数==========

//大小端设置
void servo_set_END(uint8_t val){
    END = val;
}

uint8_t servo_get_END(void){
    return END;
}

uint16_t servo_Transform_8_To_16(uint8_t low, uint8_t high){
    return END == 0 ? (low | (high << 8)) : (high | (low << 8));
}

//调试打印
void servo_debug_print_packet(const char* label, uint8_t* data, uint32_t len) {
    printf("%s [%u bytes]: ", label, len);  // 改为 %u
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// 计算校验和
uint8_t servo_calculate_checksum(uint8_t *data, uint8_t start_idx, uint8_t len)
{
    uint32_t sum = 0;
    
    for (int i = start_idx; i < start_idx + len; i++) {
        sum += data[i];
    }
    
    return (uint8_t)(~sum & 0xFF);
}
//通用数据包（除同步读写）
uint8_t servo_common_packet(uint8_t servo_id, uint8_t instruction, const uint8_t* parameters, uint8_t param_count, uint8_t* packet)
{
    if (packet == NULL) {
        return 0;
    }

    // 数据包结构: [0xFF, 0xFF, ID, Length, Instruction, Parameters..., Checksum]
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = 2 + param_count;  
    packet[4] = instruction;
    
    for (int i = 0; i < param_count; i++) {
        packet[5 + i] = parameters[i];
    }
    
    packet[5 + param_count] = servo_calculate_checksum(packet, 2, 3 + param_count);
    
    uint8_t packet_len = 6 + param_count;

    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Built cmd: ID=0x%02X Inst=0x%02X Params=%d Len=%d", 
             servo_id, instruction, param_count, packet_len);
    #endif

    return packet_len;
}


//同步读取数据包
uint8_t servo_sync_read_packet(const uint8_t* servo_ids, uint8_t id_count, uint8_t addr, uint8_t num, uint8_t* packet)
{
    if (packet == NULL || servo_ids == NULL) {
        return 0;
    }

    if (id_count == 0 || id_count > SERVO_MAX_SYNC_IDS) {
        ESP_LOGE("SERVO", "Invalid servo count: %d", id_count);
        return 0;
    }

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_BROADCAST_ID;
    packet[3] = 4 + id_count;  // 长度字段
    packet[4] = SERVO_SYNC_READ;
    packet[5] = addr;  // 起始地址参数
    packet[6] = num;   // 读取字节数参数
    
    for (int i = 0; i < id_count; i++) {
        packet[7 + i] = servo_ids[i];
    }
    
    packet[7 + id_count] = servo_calculate_checksum(packet, 2, 5 + id_count);
    
    uint8_t packet_len = 8 + id_count;

    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Built sync read: %d servos Addr=0x%02X Bytes=%d Len=%d", 
             id_count, addr, num, packet_len);
    #endif

    return packet_len;
}


//同步写入数据包
uint8_t servo_sync_write_packet(const uint8_t* servo_ids, uint8_t id_count, uint8_t addr, const uint8_t* data, uint8_t data_len_per_servo, uint8_t* packet)
{
    if (packet == NULL || servo_ids == NULL || data == NULL) {
        return 0;
    }

    if (id_count == 0 || id_count > SERVO_MAX_SYNC_IDS) {
        ESP_LOGE("SERVO", "Invalid servo count: %d", id_count);
        return 0;
    }

    if (data_len_per_servo == 0 || data_len_per_servo > SERVO_MAX_SYNC_DATA_LEN) {
        ESP_LOGE("SERVO", "Invalid data length per servo: %d", data_len_per_servo);
        return 0;
    }

    uint8_t packet_length = 4 + id_count * (1 + data_len_per_servo); 

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_BROADCAST_ID;
    packet[3] = packet_length;
    packet[4] = SERVO_SYNC_WRITE;
    packet[5] = addr;
    packet[6] = data_len_per_servo;

    uint8_t packet_index = 7;
    for (int i = 0; i < id_count; i++) {

        packet[packet_index++] = servo_ids[i];
        for (int j = 0; j < data_len_per_servo; j++) {
            packet[packet_index++] = data[i * data_len_per_servo + j];
        }
    }
    
    packet[packet_index] = servo_calculate_checksum(packet, 2, packet_length + 1);
    
    uint8_t packet_len = packet_index + 1;

    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Built sync write: %d servos Addr=0x%02X DataLen=%d TotalLen=%d", 
             id_count, addr, data_len_per_servo, packet_len);
    #endif

    return packet_len;
}

// 发送数据包函数
int servo_send_packet(uart_port_t uart_num, const uint8_t* packet, uint8_t packet_len, 
                     uint8_t* response, uint32_t timeout_ms, bool expect_response)
{
    if (packet == NULL || packet_len == 0) {
        ESP_LOGE(TAG, "Invalid packet parameters");
        return -1;
    }
    
    uart_flush_input(uart_num);
    

    int tx_bytes = uart_write_bytes(uart_num, (const char*)packet, packet_len);
    if (tx_bytes < 0) {
        ESP_LOGE(TAG, "UART write error: %d", tx_bytes);
        return -1;
    }
    
    if (tx_bytes != packet_len) {
        ESP_LOGE(TAG, "Send failed: %d/%d bytes", tx_bytes, packet_len);
        return -1;
    }

    if (uart_wait_tx_done(uart_num, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
        ESP_LOGW(TAG, "Wait TX done timeout");
    }
    
    #if SERVO_DEBUG_ENABLED
    char hex_str[80] = {0};
    for (int i = 0; i < packet_len && i < 20; i++) {
        char temp[4];
        snprintf(temp, sizeof(temp), "%02X ", packet[i]);
        strcat(hex_str, temp);
    }
    if (packet_len > 20) {
        strcat(hex_str, "...");
    }
    ESP_LOGI(TAG, "Sent %d bytes: %s", packet_len, hex_str);
    #endif
    
    if (expect_response && response != NULL) {
        int rx_bytes = 0;
        uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < timeout_ms) {
            int len = 0;
            
            if (uart_get_buffered_data_len(uart_num, (size_t*)&len) == ESP_OK && len > 0) {
                int read_len = uart_read_bytes(uart_num, response + rx_bytes, 
                                              len, pdMS_TO_TICKS(10));
                
                if (read_len > 0) {
                    rx_bytes += read_len;
                    
                    if (rx_bytes >= 6) { 
                        uint8_t pkt_len = response[3];
                        if (rx_bytes >= pkt_len + 4) {  
                            break;
                        }
                    }
                }
            } else {
            
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        #if SERVO_DEBUG_ENABLED
        if (rx_bytes > 0) {
            char resp_str[60] = {0};
            for (int i = 0; i < rx_bytes && i < 15; i++) {
                char temp[4];
                snprintf(temp, sizeof(temp), "%02X ", response[i]);
                strcat(resp_str, temp);
            }
            if (rx_bytes > 15) {
                strcat(resp_str, "...");
            }
            ESP_LOGI(TAG, "Received %d bytes: %s", rx_bytes, resp_str);
        } else if (rx_bytes == 0) {
            ESP_LOGW(TAG, "No response received (timeout)");
        }
        #endif
        
        return rx_bytes;
    }
    
    return tx_bytes;
}

static int servo_parse_response(uint8_t* packet, uint16_t length, 
                               uint8_t expected_id, uint8_t* data, uint8_t data_len)
{
    if (packet == NULL || length < 6) {
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    // 检查包头
    if (packet[0] != 0xFF || packet[1] != 0xFF) {
        ESP_LOGE(TAG, "Invalid header: 0x%02X 0x%02X", packet[0], packet[1]);
        return SERVO_STATUS_CRC_ERROR;
    }
    
    // 检查ID
    uint8_t actual_id = packet[2];
    if (actual_id != expected_id) {
        ESP_LOGE(TAG, "ID mismatch: expected %d, got %d", expected_id, actual_id);
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    // 获取长度
    uint8_t pkt_len = packet[3];
    if (pkt_len + 4 > length) {
        ESP_LOGE(TAG, "Length error: packet len=%d, buffer len=%d", 
                 pkt_len + 4, length);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    // 检查校验和
    uint8_t checksum = servo_calculate_checksum(packet, 2, pkt_len + 1);
    if (packet[pkt_len + 3] != checksum) {
        ESP_LOGE(TAG, "Checksum error: calc=0x%02X, got=0x%02X", 
                 checksum, packet[pkt_len + 3]);
        return SERVO_STATUS_CRC_ERROR;
    }
    
    // 检查错误码
    uint8_t error = packet[4];
    if (error != 0x00) {
        servo_log_error(actual_id, error);
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    // 复制数据
    if (data != NULL && data_len > 0) {
        uint8_t data_start = 5;
        uint8_t data_available = pkt_len - 2; 
        
        uint8_t copy_len = (data_len < data_available) ? data_len : data_available;
        for (int i = 0; i < copy_len; i++) {
            data[i] = packet[data_start + i];
        }
        
        if (copy_len < data_len) {
            ESP_LOGW(TAG, "Less data than requested: wanted %d, got %d", 
                     data_len, copy_len);
        }
    }
    
    return SERVO_STATUS_OK;
}
//日志打印
static void servo_log_error(uint8_t id, uint8_t error_code)
{
    switch (error_code) {
        case SERVO_VOLTAGE_ERROR:
            ESP_LOGE(TAG, "Servo %d: Input voltage error", id);
            break;
        case SERVO_ANGLE_ERROR:
            ESP_LOGE(TAG, "Servo %d: Angle limit error", id);
            break;
        case SERVO_TEMPERATURE_ERROR:
            ESP_LOGE(TAG, "Servo %d: Over temperature error", id);
            break;
        case SERVO_RANGE_ERROR:
            ESP_LOGE(TAG, "Servo %d: Parameter range error", id);
            break;
        case SERVO_CHECKSUM_ERROR:
            ESP_LOGE(TAG, "Servo %d: Checksum error", id);
            break;
        case SERVO_LOCKED_ERROR:
            ESP_LOGE(TAG, "Servo %d: Overload error (locked)", id);
            break;
        case SERVO_PARSE_ERROR:
            ESP_LOGE(TAG, "Servo %d: Instruction error", id);
            break;
        default:
            ESP_LOGE(TAG, "Servo %d: Unknown error 0x%02X", id, error_code);
            break;
    }
}

//检查内存锁
int servo_check_eeprom_lock(uint8_t id, uint32_t timeout_ms)
{
    uint8_t lock_state = 0;
    int status = servo_read_data(id, SERVO_REG_EEPROM_LOCK, 1, &lock_state, timeout_ms);
    
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to read EEPROM lock state for servo %d", id);
        return SERVO_EEPROM_UNKNOWN;
    }
    
    return (lock_state == 0) ? SERVO_EEPROM_UNLOCKED : SERVO_EEPROM_LOCKED;
}


//===========基础功能=========
// 查询舵机状态
// 注意如果总线舵机的话禁止使用广播PING
int servo_ping(uint8_t id, uint32_t timeout_ms)
{
    uint8_t tx_packet[6]; 
    uint8_t rx_packet[6] = {0}; 
    
    int tx_len = servo_common_packet(id, SERVO_PING, NULL, 0, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build PING packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    bool expect_response = (id != SERVO_BROADCAST_ID);
    
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     rx_packet, timeout_ms, expect_response);
    
    if (rx_bytes < 0) {
        ESP_LOGE(TAG, "Send PING failed");
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGW(TAG, "Broadcast PING - no response expected");
        return SERVO_STATUS_OK;
    }
    
    return servo_parse_response(rx_packet, rx_bytes, id, NULL, 0);
}

// 读指令
int servo_read_data(uint8_t id, uint8_t addr, uint8_t num, uint8_t* data, uint32_t timeout_ms)
{
    if (num == 0 || data == NULL || num > SERVO_MAX_PARAM_SIZE) {
        ESP_LOGE(TAG, "Invalid parameters: num=%d, data=%p", num, data);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t tx_packet[8];
    uint8_t rx_packet[SERVO_MAX_RESPONSE_SIZE] = {0}; 
    uint8_t params[2] = {addr, num};
    
    int tx_len = servo_common_packet(id, SERVO_READ_DATA, params, 2, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build read packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    bool expect_response = (id != SERVO_BROADCAST_ID);
    
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, rx_packet, timeout_ms, expect_response);
    
    if (rx_bytes < 0) {
        ESP_LOGE(TAG, "Send packet failed");
        return SERVO_STATUS_SLAVE_ERROR;
    }

    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGW(TAG, "Broadcast read - no response expected");
        return SERVO_STATUS_OK;
    }
    
    uint8_t min_response_len = 6 + num; 
    if (rx_bytes < min_response_len) {
        ESP_LOGE(TAG, "Incomplete response: got %d, expected at least %d", 
                 rx_bytes, min_response_len);
        return SERVO_STATUS_NO_ANSWER;
    }
    
    return servo_parse_response(rx_packet, rx_bytes, id, data, num);
}
//写质量
int servo_write_data(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t data_len, uint32_t timeout_ms)
{
    if (data == NULL || data_len == 0 || data_len > SERVO_MAX_PARAM_SIZE) {
        ESP_LOGE(TAG, "Invalid write parameters: len=%d", data_len);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t tx_packet[7 + SERVO_MAX_PARAM_SIZE]; 
    uint8_t rx_packet[6] = {0}; 
    
    uint8_t params[1 + SERVO_MAX_PARAM_SIZE];
    params[0] = addr;
    for (int i = 0; i < data_len; i++) {
        params[1 + i] = data[i];
    }
    
    int tx_len = servo_common_packet(id, SERVO_WRITE_DATA, params, data_len + 1, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build write packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    bool expect_response = (id != SERVO_BROADCAST_ID);
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     rx_packet, timeout_ms, expect_response);
    
    if (rx_bytes < 0) {
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    if (id == SERVO_BROADCAST_ID) {
        return SERVO_STATUS_OK;
    }
    
    return servo_parse_response(rx_packet, rx_bytes, id, NULL, 0);
}
//带打开内存锁读取
int servo_safe_write_eeprom(uint8_t id, uint8_t addr, const uint8_t* data, 
                           uint8_t data_len, bool require_unlock, bool restore_lock,
                           uint32_t timeout_ms)
{
    if (data == NULL || data_len == 0) {
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    int original_lock_state = SERVO_EEPROM_UNKNOWN;
    
    if (require_unlock) {
        original_lock_state = servo_check_eeprom_lock(id, timeout_ms);
        
        if (original_lock_state == SERVO_EEPROM_LOCKED) {
            ESP_LOGI(TAG, "EEPROM is locked, unlocking servo %d...", id);
            uint8_t unlock_data[1] = {0};
            int status = servo_write_data(id, SERVO_REG_EEPROM_LOCK, unlock_data, 1, timeout_ms);
            if (status != SERVO_STATUS_OK) {
                ESP_LOGE(TAG, "Failed to unlock EEPROM for servo %d", id);
                return status;
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
            int new_state = servo_check_eeprom_lock(id, timeout_ms);
            if (new_state != SERVO_EEPROM_UNLOCKED) {
                ESP_LOGE(TAG, "EEPROM unlock verification failed for servo %d", id);
                return SERVO_STATUS_SLAVE_ERROR;
            }
            
            ESP_LOGI(TAG, "EEPROM unlocked successfully for servo %d", id);
        }
    }
    
    int status = servo_write_data(id, addr, data, data_len, timeout_ms);
    
    if (require_unlock && restore_lock && original_lock_state == SERVO_EEPROM_LOCKED) {
        if (status == SERVO_STATUS_OK) {
            ESP_LOGI(TAG, "Restoring EEPROM lock for servo %d...", id);
            uint8_t lock_data[1] = {1};
            servo_write_data(id, SERVO_REG_EEPROM_LOCK, lock_data, 1, timeout_ms);
        } else {
            ESP_LOGW(TAG, "Write failed, EEPROM remains unlocked for servo %d", id);
        }
    }
    return status;
}
//同步写
int servo_sync_write(const uint8_t* servo_ids, uint8_t id_count, uint8_t addr, const uint8_t* data, uint8_t data_len_per_servo, uint32_t timeout_ms)
{
    if (servo_ids == NULL || data == NULL || id_count == 0) {
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t tx_packet[SERVO_SYNC_PACKET_MAX_SIZE];

    int tx_len = servo_sync_write_packet(servo_ids, id_count, addr, data, data_len_per_servo, tx_packet);

    if (tx_len <= 0) {
        return SERVO_STATUS_LENGTH_ERROR;
    }

    int tx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     NULL, timeout_ms, false);
    
    if (tx_bytes != tx_len) {
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    return SERVO_STATUS_OK;
}
//同步读
int servo_sync_read(const uint8_t* servo_ids, uint8_t id_count, uint8_t addr, uint8_t num,
                    uint8_t** responses, uint8_t* response_sizes, uint32_t timeout_ms)
{
    if (servo_ids == NULL || responses == NULL || id_count == 0 || id_count > SERVO_MAX_SYNC_IDS) {
        ESP_LOGE(TAG, "Invalid sync read parameters: ids=%p, responses=%p, count=%d", 
                servo_ids, responses, id_count);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    if (num == 0 || num > SERVO_MAX_PARAM_SIZE) {
        ESP_LOGE(TAG, "Invalid read length: %d (max=%d)", num, SERVO_MAX_PARAM_SIZE);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    

    uint8_t tx_packet[SERVO_SYNC_PACKET_MAX_SIZE];
    int tx_len = servo_sync_read_packet(servo_ids, id_count, addr, num, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build sync read packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Sync reading %d servos, addr=0x%02X, bytes=%d", id_count, addr, num);
    #endif
    
    uint16_t expected_response_size = 0;
    uint8_t valid_response_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        if (servo_ids[i] != SERVO_BROADCAST_ID) {
            expected_response_size += 6 + num; 
            valid_response_count++;
        }
    }
    
    if (valid_response_count == 0) {
        ESP_LOGW(TAG, "No valid servo IDs for sync read (all broadcast?)");
        return 0;
    }
    
    uint8_t* rx_buffer = (uint8_t*)malloc(expected_response_size);
    if (rx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sync read buffer (%d bytes)", expected_response_size);
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     rx_buffer, timeout_ms, true);
    
    if (rx_bytes < 0) {
        free(rx_buffer);
        ESP_LOGE(TAG, "Failed to send sync read packet");
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    if (rx_bytes == 0) {
        free(rx_buffer);
        ESP_LOGW(TAG, "No response received for sync read");
        return 0;
    }
    
    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Received %d bytes total for sync read", rx_bytes);
    #endif
    
    int success_count = 0;
    uint16_t offset = 0;
    uint8_t processed_ids[SERVO_MAX_SYNC_IDS] = {0};
    
    while (offset < rx_bytes && offset + 6 <= rx_bytes) {
        if (rx_buffer[offset] != 0xFF || rx_buffer[offset + 1] != 0xFF) {
            offset++;
            continue;
        }
        
        uint8_t id = rx_buffer[offset + 2];
        uint8_t pkt_len = rx_buffer[offset + 3];
        uint8_t error = rx_buffer[offset + 4];
        
        if (offset + pkt_len + 4 > rx_bytes) {
            ESP_LOGW(TAG, "Incomplete packet for ID %d, skipping", id);
            break;
        }
        
        uint8_t checksum = servo_calculate_checksum(rx_buffer + offset, 2, pkt_len + 1);
        if (rx_buffer[offset + pkt_len + 3] != checksum) {
            ESP_LOGW(TAG, "Checksum error for ID %d, skipping", id);
            offset += 1; 
            continue;
        }
        
        int idx = -1;
        for (int i = 0; i < id_count; i++) {
            if (servo_ids[i] == id && !processed_ids[i]) {
                idx = i;
                break;
            }
        }
        
        if (idx == -1) {
            ESP_LOGW(TAG, "Unexpected response from ID %d, skipping", id);
            offset += pkt_len + 4;
            continue;
        }
        
        if (error != 0x00) {
            ESP_LOGW(TAG, "Servo %d returned error 0x%02X", id, error);
            servo_log_error(id, error);
            offset += pkt_len + 4;
            continue;
        }
        
        uint8_t data_start = 5;
        uint8_t data_available = pkt_len - 2; // 减去指令和错误码
        
        if (responses[idx] != NULL) {
            uint8_t copy_len = (num < data_available) ? num : data_available;
            for (int i = 0; i < copy_len; i++) {
                responses[idx][i] = rx_buffer[offset + data_start + i];
            }
            
            if (response_sizes != NULL) {
                response_sizes[idx] = copy_len;
            }
            
            success_count++;
            processed_ids[idx] = 1;
            
            #if SERVO_DEBUG_ENABLED
            ESP_LOGI(TAG, "Successfully read %d bytes from servo %d", copy_len, id);
            #endif
        }
        
        offset += pkt_len + 4;
    }
    
    free(rx_buffer);
    
    // 报告未响应的舵机
    for (int i = 0; i < id_count; i++) {
        if (servo_ids[i] != SERVO_BROADCAST_ID && !processed_ids[i]) {
            ESP_LOGW(TAG, "Servo %d did not respond to sync read", servo_ids[i]);
        }
    }
    
    return success_count;
}

//重置舵机圈数
int servo_reset(uint8_t id, uint32_t timeout_ms)
{
    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGE(TAG, "Cannot reset broadcast address");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t tx_packet[6]; 
    uint8_t rx_packet[6] = {0};  
    
    int tx_len = servo_common_packet(id, SERVO_RESET, NULL, 0, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build RESET packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    #if SERVO_DEBUG_ENABLED
    ESP_LOGI(TAG, "Resetting servo %d (clear turn count)...", id);
    servo_debug_print_packet("RESET TX", tx_packet, tx_len);
    #endif
    
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, rx_packet, timeout_ms, true);
    
    if (rx_bytes < 0) {
        ESP_LOGE(TAG, "Failed to send RESET packet");
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    int status = servo_parse_response(rx_packet, rx_bytes, id, NULL, 0);
    
    if (status == SERVO_STATUS_OK) {
        ESP_LOGI(TAG, "Servo %d reset successfully (turn count cleared)", id);
        //重置需要等待
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    return status;
}

//多个舵机reset
int servo_batch_reset(const uint8_t* servo_ids, uint8_t id_count, uint32_t timeout_ms)
{
    if (servo_ids == NULL || id_count == 0) {
        return 0;
    }
    
    ESP_LOGI(TAG, "Batch resetting %d servos (clear turn counts)...", id_count);
    
    int success_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        
        if (id == SERVO_BROADCAST_ID) {
            ESP_LOGW(TAG, "Skipping broadcast ID in batch reset");
            continue;
        }
        
        int status = servo_reset(id, timeout_ms);
        if (status == SERVO_STATUS_OK) {
            success_count++;
            ESP_LOGI(TAG, "Servo %d reset (%d/%d)", id, i+1, id_count);
        } else {
            ESP_LOGE(TAG, "Failed to reset servo %d (error: %d)", id, status);
        }

        if (i < id_count - 1) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    
    ESP_LOGI(TAG, "Batch reset complete: %d/%d succeeded", success_count, id_count);
    return success_count;
}

//舵机默认位姿设置
int servo_calibrate(uint8_t id, uint16_t position, uint32_t timeout_ms)
{
    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGE(TAG, "Cannot calibrate broadcast address");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t tx_packet[8];  
    uint8_t rx_packet[6] = {0};  
    
    int tx_len;
    bool is_center_calibration = (position == 0xFFFF);
    
    // 重要：校准前确保扭矩已关闭
    uint8_t torque_off[1] = {0};
    servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_off, 1, 50);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    if (is_center_calibration) {
        ESP_LOGI(TAG, "Calibrating servo %d to center position...", id);
        tx_len = servo_common_packet(id, SERVO_CAL, NULL, 0, tx_packet);
    } else {
        ESP_LOGI(TAG, "Calibrating servo %d to position %d...", id, position);
        //小端
        uint8_t params[2] = {position & 0xFF, (position >> 8) & 0xFF};
        tx_len = servo_common_packet(id, SERVO_CAL, params, 2, tx_packet);
    }
    
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build calibration packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    #if SERVO_DEBUG_ENABLED
    servo_debug_print_packet("CAL TX", tx_packet, tx_len);
    #endif
    
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     rx_packet, timeout_ms, true);
    
    if (rx_bytes < 0) {
        ESP_LOGE(TAG, "Failed to send calibration packet");
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    int status = servo_parse_response(rx_packet, rx_bytes, id, NULL, 0);
    
    if (status == SERVO_STATUS_OK) {
        if (is_center_calibration) {
            ESP_LOGI(TAG, "Servo %d calibrated to center position", id);
        } else {
            ESP_LOGI(TAG, "Servo %d calibrated to position %d", id, position);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Calibration complete - please verify physically");
        
        uint8_t pos_data[2];
        int read_status = servo_read_data(id, SERVO_REG_CURRENT_POSITION_LOW, 2, pos_data, 100);
        if (read_status == SERVO_STATUS_OK) {
            uint16_t current_pos = (pos_data[1] << 8) | pos_data[0];
            ESP_LOGI(TAG, "Current position after calibration: %d", current_pos);
        }
    }
    
    uint8_t torque_on[1] = {1};
    servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_on, 1, 50);
    
    return status;
}
//位姿设置到中间位置
int servo_calibrate_center(uint8_t id, uint32_t timeout_ms)
{
    return servo_calibrate(id, 0xFFFF, timeout_ms);
}

//重启时间约800ms
int servo_reboot(uint8_t id, uint32_t timeout_ms)
{
    uint8_t tx_packet[6]; 
    
    int tx_len = servo_common_packet(id, SERVO_REBOOT, NULL, 0, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build REBOOT packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    ESP_LOGI(TAG, "Rebooting servo %d...", id);
    
    // 重要：重启前关闭扭矩
    if (id != SERVO_BROADCAST_ID) {
        uint8_t torque_off[1] = {0};
        servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_off, 1, 50);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    #if SERVO_DEBUG_ENABLED
    servo_debug_print_packet("REBOOT TX", tx_packet, tx_len);
    #endif
    
    bool expect_response = false;  // 重启无响应
    int tx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     NULL, timeout_ms, expect_response);
    
    if (tx_bytes != tx_len) {
        ESP_LOGE(TAG, "Failed to send REBOOT packet: %d/%d bytes", tx_bytes, tx_len);
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    ESP_LOGI(TAG, "Reboot command sent to servo %d", id);
    

    if (id != SERVO_BROADCAST_ID) {
        uint32_t reboot_time = 1000;  
        ESP_LOGI(TAG, "Waiting %dms for servo %d to reboot...", reboot_time, id);
        vTaskDelay(pdMS_TO_TICKS(reboot_time));
        
        // 尝试ping舵机确认重启成功
        ESP_LOGI(TAG, "Verifying servo %d after reboot...", id);
        int ping_status = servo_ping(id, 200);
        if (ping_status == SERVO_STATUS_OK) {
            ESP_LOGI(TAG, "Servo %d rebooted successfully", id);
        } else {
            ESP_LOGW(TAG, "Servo %d may still be rebooting or failed", id);
        }
        
        return SERVO_STATUS_OK;  // 发送成功就返回OK，ping失败只是警告
    } else {
        ESP_LOGW(TAG, "Broadcast reboot sent - waiting for all servos to reboot...");
        vTaskDelay(pdMS_TO_TICKS(1500));  
        
        return SERVO_STATUS_OK;
    }
}
//批量重启
int servo_batch_reboot(const uint8_t* servo_ids, uint8_t id_count, uint16_t stagger_ms)
{
    if (servo_ids == NULL || id_count == 0) {
        return 0;
    }
    
    ESP_LOGI(TAG, "Batch rebooting %d servos (stagger=%dms)...", id_count, stagger_ms);
    
    int success_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        if (servo_ids[i] != SERVO_BROADCAST_ID) {
            uint8_t torque_off[1] = {0};
            servo_write_data(servo_ids[i], SERVO_REG_TORQUE_ENABLE, torque_off, 1, 50);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        
        if (id == SERVO_BROADCAST_ID) {
            ESP_LOGW(TAG, "Warning: Including broadcast ID in batch reboot");
        }
        
        ESP_LOGI(TAG, "Rebooting servo %d (%d/%d)...", id, i+1, id_count);
        
        int status = servo_reboot(id, 100);
        if (status == SERVO_STATUS_OK) {
            success_count++;
            ESP_LOGI(TAG, "Reboot command sent to servo %d", id);
        } else {
            ESP_LOGE(TAG, "Failed to send reboot to servo %d", id);
        }
        
        if (i < id_count - 1 && stagger_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
    
    uint32_t total_wait = 1500;  
    if (total_wait > 0) {
        ESP_LOGI(TAG, "Waiting %dms for all servos to reboot...", total_wait);
        vTaskDelay(pdMS_TO_TICKS(total_wait));
    }
    
    int responsive_count = 0;
    for (int i = 0; i < id_count; i++) {
        if (servo_ids[i] != SERVO_BROADCAST_ID) {
            if (servo_ping(servo_ids[i], 100) == SERVO_STATUS_OK) {
                responsive_count++;
            }
        }
    }
    
    ESP_LOGI(TAG, "Batch reboot complete: %d/%d servos responsive", 
            responsive_count, id_count);
    
    return success_count;
}

//=============功能函数============
//设置运动模式
int servo_set_mode(uint8_t id, servo_mode_t mode, uint32_t timeout_ms)
{
    uint8_t mode_data[1] = {(uint8_t)mode};
    
    return servo_safe_write_eeprom(id, SERVO_REG_MODE, 
                                   mode_data, 1, true, true, timeout_ms);
}

//初始化单个舵机

int servo_initialize(uint8_t id)
{
    ESP_LOGI(TAG, "初始化舵机 %d...", id);
    
    uint8_t angle_zero[2] = {0x00, 0x00};
    
    int status = servo_safe_write_eeprom(id, SERVO_REG_MIN_ANGLE_LIMIT_LOW, 
                                        angle_zero, 2, true, true, 200);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "写入最小角度限制失败");
        return status;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    status = servo_safe_write_eeprom(id, SERVO_REG_MAX_ANGLE_LIMIT_LOW, 
                                     angle_zero, 2, true, true, 200);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "写入最大角度限制失败");
        return status;
    }
    
    status = servo_set_mode(id,SERVO_MODE_POSITION,200);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "设置位置模式失败");
        return status;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    
    uint8_t verify_min[2], verify_max[2];
    servo_read_data(id, SERVO_REG_MIN_ANGLE_LIMIT_LOW, 2, verify_min, 100);
    servo_read_data(id, SERVO_REG_MAX_ANGLE_LIMIT_LOW, 2, verify_max, 100);
    
    uint16_t min_val = (verify_min[1] << 8) | verify_min[0];
    uint16_t max_val = (verify_max[1] << 8) | verify_max[0];
    
    if (min_val == 0 && max_val == 0) {
        ESP_LOGI(TAG, "验证通过");
    } else {
        ESP_LOGW(TAG, "验证失败: 最小=%u, 最大=%u", min_val, max_val);
    }
    
    return SERVO_STATUS_OK;
}
// 设置12个舵机初始化

int servo_initialize_12_servos(const uint8_t* servo_ids, uint16_t stagger_ms)
{
    if (servo_ids == NULL) {
        ESP_LOGE(TAG, "舵机ID数组为空");
        return 0;
    } 

    int success_count = 0;
    
    for (int i = 0; i < 12; i++) {
        uint8_t id = servo_ids[i];
        ESP_LOGI(TAG, "Ping 舵机 %d...", id);
        
        int ping_status = servo_ping(id, 100);
        if (ping_status == SERVO_STATUS_OK) {
            ESP_LOGI(TAG, "舵机 %d 在线", id);
        } else {
            ESP_LOGE(TAG, "舵机 %d 离线或错误: %d", id, ping_status);
        }
        
        if (i < 11) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    
    for (int i = 0; i < 12; i++) {
        uint8_t id = servo_ids[i];
        
        ESP_LOGI(TAG, "\n[%d/12] 初始化舵机 %d", i+1, id);
        
        int ping_status = servo_ping(id, 50);
        if (ping_status != SERVO_STATUS_OK) {
            ESP_LOGW(TAG, "舵机 %d 当前离线，跳过初始化", id);
            continue;
        }
        
        int status = servo_initialize(id);
        if (status == SERVO_STATUS_OK) {
            success_count++;
            ESP_LOGI(TAG, "舵机 %d 初始化成功", id);
        } else {
            ESP_LOGE(TAG, "舵机 %d 初始化失败 (错误: %d)", id, status);
        }
        
        if (i < 11 && stagger_ms > 0) {
            ESP_LOGI(TAG, "等待 %dms 后处理下一个舵机...", stagger_ms);
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
    
    return success_count;
}

int servo_calibrate_default_position(uint8_t id, uint32_t timeout_ms)
{
    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGE(TAG, "Cannot calibrate broadcast address");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t torque_off[1] = {0};
    int status = servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_off, 1, 50);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to disable torque for servo %d", id);
        return status;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    

    ESP_LOGI(TAG, "Calibrating servo %d default position...", id);
    uint8_t tx_packet[6]; 
    
    int tx_len = servo_common_packet(id, SERVO_CAL, NULL, 0, tx_packet);
    if (tx_len <= 0) {
        ESP_LOGE(TAG, "Failed to build calibration packet");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t rx_packet[6] = {0};
    int rx_bytes = servo_send_packet(SERVO_UART_NUM, tx_packet, tx_len, 
                                     rx_packet, timeout_ms, true);
    
    if (rx_bytes < 0) {
        ESP_LOGE(TAG, "Failed to send calibration packet");
        return SERVO_STATUS_SLAVE_ERROR;
    }
    
    status = servo_parse_response(rx_packet, rx_bytes, id, NULL, 0);
    
    if (status == SERVO_STATUS_OK) {
        ESP_LOGI(TAG, "Servo %d default position calibrated", id);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        uint8_t pos_data[2];
        int read_status = servo_read_data(id, SERVO_REG_CURRENT_POSITION_LOW, 2, pos_data, 100);
        if (read_status == SERVO_STATUS_OK) {
            uint16_t current_pos = (pos_data[1] << 8) | pos_data[0];
            ESP_LOGI(TAG, "Current position after calibration: %d (should be close to 0)", current_pos);
        }
    }
    
    uint8_t torque_on[1] = {1};
    servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_on, 1, 50);
    
    return status;
}

int servo_set_position_speed_torque(uint8_t id, uint16_t position, uint16_t speed, uint16_t torque, uint32_t timeout_ms)
{
    if (id == SERVO_BROADCAST_ID) {
        ESP_LOGE(TAG, "Cannot set position/speed/torque to broadcast address");
        return SERVO_STATUS_LENGTH_ERROR;
    }
    
    uint8_t current_mode;
    int status = servo_read_data(id, SERVO_REG_MODE, 1, &current_mode, timeout_ms);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to read mode from servo %d", id);
        return status;
    }
    
    if (current_mode != SERVO_MODE_POSITION) {
        ESP_LOGW(TAG, "Servo %d not in position mode (current: 0x%02X), switching...", id, current_mode);
        status = servo_set_mode(id, SERVO_MODE_POSITION, timeout_ms);
        if (status != SERVO_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to set position mode for servo %d", id);
            return status;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    uint8_t torque_enable[1] = {1};
    status = servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_enable, 1, timeout_ms);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to enable torque for servo %d", id);
        return status;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    

    uint8_t position_data[2];
    if (servo_get_END() == 0) { 
        position_data[0] = position & 0xFF;        
        position_data[1] = (position >> 8) & 0xFF; 
    } else { 
        position_data[0] = (position >> 8) & 0xFF; 
        position_data[1] = position & 0xFF;       
    }
    
   
    uint8_t speed_data[2];
    if (servo_get_END() == 0) { 
        speed_data[0] = speed & 0xFF;              
        speed_data[1] = (speed >> 8) & 0xFF;       
    } else { 
        speed_data[0] = (speed >> 8) & 0xFF;       
        speed_data[1] = speed & 0xFF;           
    }
    

    uint8_t torque_data[2];
    if (servo_get_END() == 0) { 
        torque_data[0] = torque & 0xFF;           
        torque_data[1] = (torque >> 8) & 0xFF;    
    } else {
        torque_data[0] = (torque >> 8) & 0xFF;     
        torque_data[1] = torque & 0xFF;           
    }
    
    // 写入位置
    status = servo_write_data(id, SERVO_REG_GOAL_POSITION_LOW, position_data, 2, timeout_ms);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set position for servo %d", id);
        return status;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 写入速度
    status = servo_write_data(id, SERVO_REG_GOAL_SPEED_LOW, speed_data, 2, timeout_ms);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set speed for servo %d", id);
        return status;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 写入力矩
    status = servo_write_data(id, SERVO_REG_GOAL_TORQUE_LOW, torque_data, 2, timeout_ms);
    if (status != SERVO_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set torque for servo %d", id);
        return status;
    }
    
    #if SERVO_DEBUG_ENABLED
    uint16_t actual_pos = servo_Transform_8_To_16(position_data[0], position_data[1]);
    uint16_t actual_speed = servo_Transform_8_To_16(speed_data[0], speed_data[1]);
    uint16_t actual_torque = servo_Transform_8_To_16(torque_data[0], torque_data[1]);
    ESP_LOGI(TAG, "Set servo %d: pos=%d(%d), speed=%d(%d), torque=%d(%d)", 
             id, position, actual_pos, speed, actual_speed, torque, actual_torque);
    #endif
    
    return SERVO_STATUS_OK;
}

int servo_batch_unload_torque(const uint8_t* servo_ids, 
                            uint8_t id_count, 
                            uint16_t stagger_ms,
                            uint32_t timeout_ms)
{
    if (servo_ids == NULL || id_count == 0) {
        ESP_LOGE(TAG, "无效的舵机ID数组");
        return 0;
    }  
    
    int success_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        
        if (id == SERVO_BROADCAST_ID) {
            ESP_LOGW(TAG, "跳过广播ID %d", id);
            continue;
        }
        
        int ping_status = servo_ping(id, 50);
        if (ping_status != SERVO_STATUS_OK) {
            ESP_LOGW(TAG, "舵机 %d 离线，跳过卸载", id);
            continue;
        }
        
        uint8_t torque_off[1] = {0};
        int status = servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_off, 1, timeout_ms);
        
        if (status == SERVO_STATUS_OK) {
            success_count++;
            ESP_LOGI(TAG, "舵机 %d 扭矩已关闭 (%d/%d)", id, i+1, id_count);
            
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t torque_state;
            int read_status = servo_read_data(id, SERVO_REG_TORQUE_ENABLE, 1, &torque_state, 50);
            if (read_status == SERVO_STATUS_OK && torque_state == 0) {
                ESP_LOGD(TAG, "舵机 %d 扭矩状态验证通过: 已关闭", id);
            } else {
                ESP_LOGW(TAG, "舵机 %d 扭矩状态验证失败", id);
            }
        } else {
            ESP_LOGE(TAG, "舵机 %d 扭矩关闭失败 (错误: %d)", id, status);
        }
        
        if (i < id_count - 1 && stagger_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
    
    ESP_LOGI(TAG, "扭矩卸载完成: %d/%d 个舵机成功", success_count, id_count);
    return success_count;
}

int servo_batch_enable_torque(const uint8_t* servo_ids, 
                            uint8_t id_count, 
                            uint16_t stagger_ms,
                            uint32_t timeout_ms)
{
    if (servo_ids == NULL || id_count == 0) {
        ESP_LOGE(TAG, "无效的舵机ID数组");
        return 0;
    }
    
    int success_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        
        if (id == SERVO_BROADCAST_ID) {
            ESP_LOGW(TAG, "跳过广播ID %d", id);
            continue;
        }
        
        int ping_status = servo_ping(id, 50);
        if (ping_status != SERVO_STATUS_OK) {
            ESP_LOGW(TAG, "舵机 %d 离线，跳过开启", id);
            continue;
        }
        
        uint8_t torque_on[1] = {1};
        int status = servo_write_data(id, SERVO_REG_TORQUE_ENABLE, torque_on, 1, timeout_ms);
        
        if (status == SERVO_STATUS_OK) {
            success_count++;
            ESP_LOGI(TAG, "舵机 %d 扭矩已开启 (%d/%d)", id, i+1, id_count);
            
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t torque_state;
            int read_status = servo_read_data(id, SERVO_REG_TORQUE_ENABLE, 1, &torque_state, 50);
            if (read_status == SERVO_STATUS_OK && torque_state == 1) {
                ESP_LOGD(TAG, "舵机 %d 扭矩状态验证通过: 已开启", id);
            } else {
                ESP_LOGW(TAG, "舵机 %d 扭矩状态验证失败", id);
            }
        } else {
            ESP_LOGE(TAG, "舵机 %d 扭矩开启失败 (错误: %d)", id, status);
        }
        
        if (i < id_count - 1 && stagger_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
    
    ESP_LOGI(TAG, "扭矩开启完成: %d/%d 个舵机成功", success_count, id_count);
    return success_count;
}

int servo_batch_calibrate_to_current_position(const uint8_t* servo_ids, 
                                            uint8_t id_count, 
                                            uint16_t stagger_ms,
                                            uint32_t timeout_ms,
                                            bool wait_for_user)
{
    if (servo_ids == NULL || id_count == 0) {
        ESP_LOGE(TAG, "无效的舵机ID数组");
        return 0;
    }
    

    int unload_count = servo_batch_unload_torque(servo_ids, id_count, stagger_ms, timeout_ms);
    if (unload_count == 0) {
        ESP_LOGE(TAG, "卸载所有舵机失败，停止标定流程");
        return 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (wait_for_user) {

        const uint32_t wait_time_ms = 1800000; // 等待180秒
        ESP_LOGI(TAG, "等待 %d 秒供人工摆放...", wait_time_ms / 1000);
        
        // 显示倒计时
        for (int i = wait_time_ms / 1000; i > 0; i--) {
            ESP_LOGI(TAG, "剩余等待时间: %d 秒", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        ESP_LOGI(TAG, "等待时间结束，开始标定...");
    } else {
        ESP_LOGI(TAG, "直接标定当前舵机位置...");
    }
    
    
    int success_count = 0;
    
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        
        if (id == SERVO_BROADCAST_ID) {
            ESP_LOGW(TAG, "跳过广播ID %d", id);
            continue;
        }
        
        ESP_LOGI(TAG, "[%d/%d] 标定舵机 %d", i+1, id_count, id);
        
        int ping_status = servo_ping(id, 50);
        if (ping_status != SERVO_STATUS_OK) {
            ESP_LOGW(TAG, "舵机 %d 离线，跳过标定", id);
            continue;
        }
        

        int cal_status = servo_calibrate_default_position(id, timeout_ms);
        
        if (cal_status == SERVO_STATUS_OK) {
            success_count++;
            
            vTaskDelay(pdMS_TO_TICKS(200)); 
            
            uint8_t pos_data[2];
            int read_status = servo_read_data(id, SERVO_REG_CURRENT_POSITION_LOW, 2, pos_data, 100);
            if (read_status == SERVO_STATUS_OK) {
                uint16_t current_pos = servo_Transform_8_To_16(pos_data[0], pos_data[1]);
                ESP_LOGI(TAG, "舵机 %d 标定后位置: %d", id, current_pos);
                
                if (abs(current_pos) < 50) { 
                    ESP_LOGI(TAG, "标定验证通过");
                } else {
                    ESP_LOGW(TAG, "标定后位置可能不正确");
                }
            }
        } else {
            ESP_LOGE(TAG, "舵机 %d 标定失败 (错误: %d)", id, cal_status);
        }
        
        if (i < id_count - 1 && stagger_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
    
    
    int enable_count = servo_batch_enable_torque(servo_ids, id_count, stagger_ms, timeout_ms);
    
    
    int online_count = 0;
    for (int i = 0; i < id_count; i++) {
        uint8_t id = servo_ids[i];
        if (id != SERVO_BROADCAST_ID && servo_ping(id, 50) == SERVO_STATUS_OK) {
            online_count++;
        }
    }
    
    return success_count;
}