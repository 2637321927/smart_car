/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
@注意事项：TFT程序优先推荐使用硬件SPI部分, 也就是LQ_TFT18_dri部分, 使用前需加载对应驱动模块,
         双龙mini派中已提前添加, 未添加也可自行编译.
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "main.hpp"

// 蜂鸣器引脚初始化
// HWGpio beep(61, GPIO_Mode_Out);

int main()
{
    // beep.SetGpioValue(0);   // 关闭蜂鸣器

    while (1)
    {
        // // GPIO输出功能演示
        // GpioOutputDemo(88);

        // // GPIO输入功能演示
        // GpioInputDemo();

        // // 硬件PWM功能演示
        // PwmHWDemo();

        // // 通用定时器PWM功能演示
        // GtimPwmDemo();

        // // 高级定时器PWM功能演示
        // AtimPwmDemo();

        // // 编码器功能演示
        // EncoderDemo();

        // // 摄像头功能演示
        // CameraDemo();

        // // ADC功能演示
        // AdcFunDemo();

        // // TFT显示驱动演示
        // TFT_Dri_Demo();

        // // I2C陀螺仪传感器演示
        // Demo_I2C_Gyro();

        // // 串口功能演示
        // Uart_Demo();

        // // TCP通信演示
        // TCP_Demo();

        // // UDP通信演示
        // UDP_Demo();

        // // 获取时间功能演示
        // GetTimeDemo();

        // // 秒级延时演示
        // sleepDemo();

        // // 微秒级延时演示
        // usleepDemo();

        // // 纳秒级延时演示
        // nanosleepDemo();

        // // 高精度时钟延时演示
        // clock_nanosleepDemo();

        // // 电机控制演示
        // MotorDemo();

        // // 舵机控制演示
        // ServoDemo();

        // // 信号模拟5ms中断演示
        // setitimerDemo();

        // // 线程模拟5ms中断演示
        // HibernateDemo();

        // // YOLO目标检测演示
        // YOLO_Demo();
    }

    return 0;
}
