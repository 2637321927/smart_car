#pragma once

#include "main.hpp"
// #include "LQ_YOLO.hpp"
// #include "LQ_module_loader.hpp"

// GPIO输出功能演示
void GpioOutputDemo(uint8_t gpio);

// GPIO输入功能演示
void GpioInputDemo();

// 硬件PWM功能演示
void PwmHWDemo();

// 通用定时器PWM功能演示
void GtimPwmDemo();

// 高级定时器PWM功能演示
void AtimPwmDemo();

// 编码器功能演示
void EncoderDemo();

// 摄像头功能演示
void CameraDemo();

// ADC功能演示
void AdcFunDemo();

// TFT显示驱动演示
void TFT_Dri_Demo();

// I2C陀螺仪传感器演示
void Demo_I2C_Gyro();

// I2C VL53距离传感器演示
void Demo_I2C_VL53();

// 串口功能演示
void Uart_Demo();

// TCP通信演示
void TCP_Demo();

// UDP通信演示
void UDP_Demo();

// 获取时间功能演示
void GetTimeDemo();

// 秒级延时演示
void sleepDemo();

// 微秒级延时演示
void usleepDemo();

// 纳秒级延时演示
void nanosleepDemo();

// 高精度时钟延时演示
void clock_nanosleepDemo();

// 电机控制演示
void MotorDemo();

// 舵机控制演示
void ServoDemo();

// 信号模拟5ms中断演示
void setitimerDemo();

// 线程模拟5ms中断演示
void HibernateDemo();

// YOLO目标检测演示
void YOLO_Demo();

void CalculateFrameRate();

void GetCurrentMillisecond();

void printMicTimestamp();
