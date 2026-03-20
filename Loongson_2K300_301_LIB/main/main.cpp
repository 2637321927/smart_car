#include "main.hpp"

int main()
{
   // img_test();
//lq_udp_img_trans_demo();
//PID_control_test();
//lq_atim_pwm_demo();
    uint16_t dis;

    lq_i2c_vl53l0x vl53l0x;

    while (1)
    {
        float dis=vl53l0x.get_vl53l0x_dis();
        PID_control_test(dis);
        printf("%f\n",dis);
        if(dis<90){
            break;
        }
        usleep(100*100);
    }
    return 0;
}
