#include "main.hpp"

int main()
{

    int count=0;
        // 默认极性的构造方式
    ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, 2000);
    // 自定义极性的构造方式
    ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, 2000, ATIM_PWM_POL_NORMAL);
    // 拷贝构造使用方法, 调用 pwm3 与调用 pwm1 同效
    ls_atim_pwm pwm3(pwm1);
    // 拷贝赋值使用方法, 调用 pwm4 与调用 pwm2 同效
    ls_atim_pwm pwm4 = pwm2;


    while (count<2)
    {
        pwm1.atim_pwm_set_duty(100);
        pwm2.atim_pwm_set_duty(100);
        sleep(1);
        pwm1.atim_pwm_set_duty(200);
        pwm2.atim_pwm_set_duty(1000);
        sleep(1);
        count++;
    }
     ls_gpio gpio(PIN_81, GPIO_MODE_OUT);
     ls_gpio gpio_2(PIN_82, GPIO_MODE_OUT);
       gpio.gpio_level_set(GPIO_LOW);
         gpio_2.gpio_level_set(GPIO_LOW);
    return 0;
}
