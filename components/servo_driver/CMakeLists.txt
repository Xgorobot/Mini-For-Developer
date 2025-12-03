# 伺服驱动组件
idf_component_register(SRCS "servo_driver.c"
                    INCLUDE_DIRS "include" "../../config"
                    REQUIRES 
                        uart_comm
                    PRIV_REQUIRES 
                        driver
                        esp_timer
                    )