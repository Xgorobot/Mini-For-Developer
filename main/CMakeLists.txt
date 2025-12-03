idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES 
                        servo_driver 
                        uart_comm
                        icm20948
                        other)