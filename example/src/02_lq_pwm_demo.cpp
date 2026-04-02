#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    PWM 输出测试, 使用引脚 81,82作为输出引脚, 读取当前信息并打印到终端.
 ********************************************************************************/
void lq_pwm_demo(void)
{
    ls_pwm pwm1(PWM0_PIN64, 82, 2000);//left_wheel
    ls_pwm pwm2(PWM1_PIN65, 81, 2000);//right_wheel

    while (1)
    {
        pwm1.pwm_set_duty(3000);
        pwm2.pwm_set_duty(3000);

        printf("pwm1->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm1.pwm_get_gpio(), pwm1.pwm_get_mux(), pwm1.pwm_get_channel(), pwm1.pwm_get_period(), pwm1.pwm_get_duty());
        printf("pwm2->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm2.pwm_get_gpio(), pwm2.pwm_get_mux(), pwm2.pwm_get_channel(), pwm2.pwm_get_period(), pwm2.pwm_get_duty());
        sleep(1);
        pwm1.pwm_set_duty(7000);
        pwm2.pwm_set_duty(7000);
        printf("pwm1->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm1.pwm_get_gpio(), pwm1.pwm_get_mux(), pwm1.pwm_get_channel(), pwm1.pwm_get_period(), pwm1.pwm_get_duty());
        printf("pwm2->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm2.pwm_get_gpio(), pwm2.pwm_get_mux(), pwm2.pwm_get_channel(), pwm2.pwm_get_period(), pwm2.pwm_get_duty());
        sleep(1);
    }
}
