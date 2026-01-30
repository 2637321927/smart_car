#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   硬件PWM演示函数
 * @param   none.
 * @return  none.
 * @note    创建并控制4个PWM通道，分别使用不同的引脚和频率设置，
 *          演示PWM输出的硬件控制功能，包括占空比的动态调整。
 ********************************************************************************/
void PwmHWDemo()
{
    // 创建4个PWM对象，分别对应不同的引脚和频率
    HWPwm pwm0(64, LS_PWM0, LS_PWM_INVERSED, 50, 5);
    HWPwm pwm1(65, LS_PWM1, LS_PWM_INVERSED, 100, 10);
    HWPwm pwm2(66, LS_PWM2, LS_PWM_INVERSED, 1000, 100);
    HWPwm pwm3(67, LS_PWM3, LS_PWM_INVERSED, 10000, 5000);

    // 启用所有PWM通道
    pwm0.Enable();
    pwm1.Enable();
    pwm2.Enable();
    pwm3.Enable();
    while(1)
    {
        // 设置第一组占空比：5, 10, 100, 1000
        pwm0.SetDutyCycle(5);
        pwm1.SetDutyCycle(10);
        pwm2.SetDutyCycle(100);
        pwm3.SetDutyCycle(1000);
        printf("Set HW PWM 1000\n");
        sleep(1);
        // 设置第二组占空比：25, 50, 500, 5000
        pwm0.SetDutyCycle(25);
        pwm1.SetDutyCycle(50);
        pwm2.SetDutyCycle(500);
        pwm3.SetDutyCycle(5000);
        printf("Set HW PWM 5000\n");
        sleep(1);
        // 设置第三组占空比：45, 90, 900, 9000
        pwm0.SetDutyCycle(45);
        pwm1.SetDutyCycle(90);
        pwm2.SetDutyCycle(900);
        pwm3.SetDutyCycle(9000);
        printf("Set HW PWM 9000\n");
        sleep(1);
    }
}
