#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   ATIM PWM演示函数
 * @param   none.
 * @retval  none.
 * @note    该函数演示了如何使用ATIM通用定时器产生PWM信号。
 *          初始化4个PWM通道(TIM1_CH1, TIM1_CH2, TIM1_CH3, TIM1_CH4)，
 *          并周期性地改变占空比来展示PWM信号的变化效果。
 * @note    功能说明：
 *          - 配置4个PWM通道
 *          - 启用所有PWM通道
 *          - 循环改变PWM占空比
 *          - 每次改变后等待500ms并打印当前占空比值
 ********************************************************************************/
void AtimPwmDemo()
{
    AtimPwm TIM1CH1(81, 1, LS_GTIM_INVERSED, 2000000, 100000);
    AtimPwm TIM1CH2(82, 2, LS_GTIM_INVERSED, 2000000, 100000);
    AtimPwm TIM1CH3(83, 3, LS_GTIM_INVERSED, 2000000, 100000);
    AtimPwm TIM1CH4(76, 4, LS_GTIM_INVERSED, 2000000, 100000, 0b01);
    // 启用所有PWM通道
    TIM1CH1.Enable();
    TIM1CH2.Enable();
    TIM1CH3.Enable();
    TIM1CH4.Enable();
    while(1)
    {
        // 设置占空比为5%并打印
        TIM1CH1.SetDutyCycle(100000);
        TIM1CH2.SetDutyCycle(100000);
        TIM1CH3.SetDutyCycle(100000);
        TIM1CH4.SetDutyCycle(100000);
        printf("Atim PWM Set %d\n", 100000);
        usleep(500000);
        // 设置占空比为25%并打印
        TIM1CH1.SetDutyCycle(500000);
        TIM1CH2.SetDutyCycle(500000);
        TIM1CH3.SetDutyCycle(500000);
        TIM1CH4.SetDutyCycle(500000);
        printf("Atim PWM Set %d\n", 500000);
        usleep(500000);
        // 设置占空比为45%并打印
        TIM1CH1.SetDutyCycle(900000);
        TIM1CH2.SetDutyCycle(900000);
        TIM1CH3.SetDutyCycle(900000);
        TIM1CH4.SetDutyCycle(900000);
        printf("Atim PWM Set %d\n", 900000);
        usleep(500000);
    }
}
