#include "LQ_demo.hpp"


/********************************************************************************
 * @brief   GTIM PWM演示函数
 * @param   none.
 * @return  none.
 * @note    该函数演示了如何使用GTIM通用定时器产生PWM信号。
 *          初始化4个PWM通道(TIM2_CH1, TIM2_CH2, TIM2_CH3, TIM2_CH4)，
 *          并周期性地改变占空比来展示PWM信号的变化效果。
 * @note    功能说明：
 *          - 配置4个PWM通道
 *          - 启用所有PWM通道
 *          - 循环改变PWM占空比
 *          - 每次改变后等待500ms并打印当前占空比值
 ********************************************************************************/
void GtimPwmDemo()
{
    GtimPwm TIM2CH1(87, 1, LS_GTIM_INVERSED, 50, 1000);
    GtimPwm TIM2CH2(88, 2, LS_GTIM_INVERSED, 50, 1000);
    GtimPwm TIM2CH3(89, 3, LS_GTIM_INVERSED, 50, 1000);
    GtimPwm TIM2CH4(77, 4, LS_GTIM_INVERSED, 50, 1000, 0b01);
    // 启用所有PWM通道
    TIM2CH1.Enable();
    TIM2CH2.Enable();
    TIM2CH3.Enable();
    TIM2CH4.Enable();
    while(1)
    {
        // 设置占空比为5%并打印
        TIM2CH1.SetDutyCycle(500);
        TIM2CH2.SetDutyCycle(500);
        TIM2CH3.SetDutyCycle(500);
        TIM2CH4.SetDutyCycle(500);
        printf("Gtim PWM Set %d\n", 500);
        usleep(500000);
        // 设置占空比为25%并打印
        TIM2CH1.SetDutyCycle(2500);
        TIM2CH2.SetDutyCycle(2500);
        TIM2CH3.SetDutyCycle(2500);
        TIM2CH4.SetDutyCycle(2500);
        printf("Gtim PWM Set %d\n", 2500);
        usleep(500000);
        // 设置占空比为45%并打印
        TIM2CH1.SetDutyCycle(4500);
        TIM2CH2.SetDutyCycle(4500);
        TIM2CH3.SetDutyCycle(4500);
        TIM2CH4.SetDutyCycle(4500);
        printf("Gtim PWM Set %d\n", 4500);
        usleep(500000);
    }
}
