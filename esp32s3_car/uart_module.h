#ifndef __UART_MODULE_H__
#define __UART_MODULE_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

// Arduino串口优化配置
#define SERIAL_RX_BUFFER_SIZE 256 // 增大默认缓冲区
#define SERIAL_TX_BUFFER_SIZE 256
// ESP8266的串口默认引脚(软件串口)
#define UART_TX_PIN 41
#define UART_RX_PIN 40

extern SoftwareSerial motorSerial;

// 函数原型
void uart1_init(void);
void Send_Motor_ArrayU8(uint8_t *data, uint16_t len);
int Send_Motor_U8(uint8_t data);

#endif