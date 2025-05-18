#include <Arduino.h>
#include "uart_module.h"
#ifndef __APP_MOTOR_USART_H_
#define __APP_MOTOR_USART_H_

#define RXBUFF_LEN 128 // 原256改为128以适应ESP8266内存

// 类型定义
typedef enum
{
    MOTOR_TYPE_NONE = 0x00,
    MOTOR_520,
    MOTOR_310,
    MOTOR_TT_Encoder,
    MOTOR_TT,
    Motor_TYPE_MAX
} motor_type_t;

// 全局变量声明
extern volatile uint8_t g_recv_flag; // 添加volatile保证中断安全
extern float g_Speed[4];
extern int Encoder_Offset[4];
extern int Encoder_Now[4];

// 函数原型
void send_motor_type(motor_type_t data);
void send_motor_deadzone(uint16_t data);
void send_pulse_line(uint16_t data);
void send_pulse_phase(uint16_t data);
void send_wheel_diameter(float data);
void send_upload_data(bool ALL, bool Ten, bool Speed);
void Contrl_Speed(int16_t s1, int16_t s2, int16_t s3, int16_t s4);
void Contrl_Pwm(int16_t p1, int16_t p2, int16_t p3, int16_t p4);

// 专用处理函数
void Deal_Control_Rxtemp(uint8_t c);
void Deal_data_real(void);

#endif