#include "main.hpp"

int main()
{
   // img_test();
//lq_udp_img_trans_demo();
//PID_control_test();
//lq_atim_pwm_demo();
/*
    uint16_t dis;

    lq_i2c_vl53l0x vl53l0x;
ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, 0);
ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, 0);

    while (1)
    {
        float dis=vl53l0x.get_vl53l0x_dis();
        PID_control_test(pwm1,pwm2,dis);
        printf("%f\n",dis);
        if(dis<90){
        pwm1.atim_pwm_set_duty(0);
        pwm2.atim_pwm_set_duty(0);//全部失能
            break;
        }
        usleep(100*100);
    }
        */
//img_test();
lq_encoder_pwm_demo();
    return 0;
}
