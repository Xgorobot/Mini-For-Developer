idf_component_register(SRCS "servo.c" "uart_async_rxtxtasks_main.c"
                    REQUIRES esp_driver_uart esp_driver_gpio driver icm20948
                    INCLUDE_DIRS ".")
