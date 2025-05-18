#include "uart_module.h"
SoftwareSerial motorSerial(UART_RX_PIN, UART_TX_PIN);

void uart1_init()
{
    motorSerial.begin(115200);
    motorSerial.setTimeout(10);
}

void Send_Motor_ArrayU8(uint8_t *data, uint16_t len)
{
    motorSerial.write(data, len);
    //最开始方案是8266，所以采用软串口，所以就得用flush等串口数据处理完成
    motorSerial.flush();
}

int Send_Motor_U8(uint8_t data)
{
    return motorSerial.write(data);
}