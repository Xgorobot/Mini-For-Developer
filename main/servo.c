// #include "servo.h"
// #include "driver/uart.h"

// int sendData(unsigned char* data, unsigned int len)
// {
//     const int len = 8;
//     const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
//     return txBytes;
// }
// void read_angle(uint8_t ID)
// {
// 	uint8_t bBuf[8];
//     bBuf[0] = 0xff;
// 	bBuf[1] = 0xff;
// 	bBuf[2] = ID;
// 	bBuf[3] = 0x04;
// 	bBuf[4] = 0x02;
// 	bBuf[5] = 0x38;
// 	bBuf[6] = 0x02;
//     checkSum = ID + 0x04 + 0x02 + 0x38 + 0x02;
//     bBuf[7] = ~checkSum;

// }