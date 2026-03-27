#ifndef __LQ_ALL_DEMO_H
#define __LQ_ALL_DEMO_H

#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"
extern ls_atim_pwm pwm1;
extern ls_atim_pwm pwm2;
extern ls_encoder_pwm enc1;
extern ls_encoder_pwm enc2;
void lq_gpio_output_demo(void);     // GPIO 输出模式测试
void lq_gpio_input_demo(void);      // GPIO 输入模式测试
void lq_pwm_demo(void);             // PWM 输出模式测试
void lq_gtim_pwm_demo(void);        // GTIM PWM 输出模式测试
void lq_atim_pwm_demo(void);        // ATIM PWM 输出模式测试
void lq_encoder_pwm_demo(void);     // 编码器 PWM 输出模式测试
void lq_canfd_demo(void);           // CANFD 测试
void lq_ncnn_demo(void);            // NCNN 测试
void lq_ncnn_photo_demo(void);      // NCNN 图像分类测试
void lq_ips20_demo(void);           // IPS屏幕测试
void lq_mpu6050_demo(void);         // MPU6050 测试
void lq_lsm6dsr_demo(void);         // LSM6DSR 测试
void lq_vl53l0x_demo(void);         // VL53L0X 测试
void lq_udp_img_trans_demo(void);   // UDP 图像传输测试
void lq_udp_wavefrom_demo(void);    // UDP 波形传输测试
void lq_icm42688_demo(void);        // ICM42688 测试
void img_test(void);
void PID_control_test(ls_atim_pwm& pwm1,ls_atim_pwm& pwm2,float error);
int calculate_diffrential(float error);
float img_return(void);
void lq_ncnn_photo_demo(cv::Mat& image,std::string& a);
void cut(void);
void close_circle_control(
    ls_atim_pwm& pwm1,
    ls_atim_pwm& pwm2,
    float& speed_of_motor1,
    float& speed_of_motor2,
    int target_speed_of_motor1_RPS,
    int target_speed_of_motor2_RPS);
    void input_speed(int&expected_speed_of_motor1_rps,int& expected_speed_of_motor2_rps);
    void test_enc_and_motor(int expected_speed_of_motor1_pwm,int expected_speed_of_motor2_pwm);
#endif
