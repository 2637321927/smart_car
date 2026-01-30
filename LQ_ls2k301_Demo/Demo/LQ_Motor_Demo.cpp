#include "LQ_demo.hpp"

#define Motor1_N 21
#define Motor2_N 22
#define Motor3_N 23
#define Motor4_N 24

/********************************************************************************
 * @brief   电机演示函数
 * @param   none.
 * @return  none.
 * @note    初始化4个电机（PWM控制），设置占空比为1000，启用电机，
 *          控制电机方向引脚交替高低电平，实现电机正反转循环控制
 * @note    电机配置：
 *          - 电机1：PWM引脚81，方向引脚21
 *          - 电机2：PWM引脚82，方向引脚22  
 *          - 电机3：PWM引脚83，方向引脚23
 *          - 电机4：PWM引脚76，方向引脚24
 ********************************************************************************/
void MotorDemo()
{
    AtimPwm Motor1(81, 1, LS_ATIM_INVERSED, 6400, 1000);
    AtimPwm Motor2(82, 2, LS_ATIM_INVERSED, 6400, 1000);
    AtimPwm Motor3(83, 3, LS_ATIM_INVERSED, 6400, 1000);
    AtimPwm Motor4(76, 4, LS_ATIM_INVERSED, 6400, 1000, 0b01);
    
    HWGpio Motor1_DIR(Motor1_N,GPIO_Mode_Out);
    HWGpio Motor2_DIR(Motor2_N,GPIO_Mode_Out);
    HWGpio Motor3_DIR(Motor3_N,GPIO_Mode_Out);
    HWGpio Motor4_DIR(Motor4_N,GPIO_Mode_Out);

    Motor1.SetDutyCycle(1000);
    Motor2.SetDutyCycle(1000);
    Motor3.SetDutyCycle(1000);
    Motor4.SetDutyCycle(1000);

    Motor1.Enable();
    Motor2.Enable();
    Motor3.Enable();
    Motor4.Enable();

    while(true)
    {
        Motor1_DIR.SetGpioValue(1);
        Motor2_DIR.SetGpioValue(1);
        Motor3_DIR.SetGpioValue(1);
        Motor4_DIR.SetGpioValue(1);
        sleep(2);
        Motor1_DIR.SetGpioValue(0);
        Motor2_DIR.SetGpioValue(0);
        Motor3_DIR.SetGpioValue(0);
        Motor4_DIR.SetGpioValue(0);
        sleep(2);
    }
}
