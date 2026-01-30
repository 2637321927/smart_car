#include "LQ_demo.hpp"


/********************************************************************************
 * @brief   GPIO输出演示函数
 * @param   gpio : GPIO引脚编号，用于指定要控制的GPIO引脚
 * @return  none.
 * @note    该函数演示了如何使用GPIO引脚进行输出控制，通过循环方式交替设置GPIO的高低电平，
 *          实现LED闪烁效果或类似的输出控制功能。
 ********************************************************************************/
void GpioOutputDemo(uint8_t gpio)
{
    HWGpio Gpio(gpio, GPIO_Mode_Out);
    while(1)
    {
        Gpio.SetGpioValue(0);
        printf("Set HWGpio value 0\n");
        sleep(1);
        Gpio.SetGpioValue(1);
        printf("Set HWGpio value 1\n");
        sleep(1);
    }
}
