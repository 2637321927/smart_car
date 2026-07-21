#ifndef __LQ_ALL_DEMO_H
#define __LQ_ALL_DEMO_H

#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"
typedef signed short       sint16;
//#include <mutex> 
extern bool have_target;
extern cv::Rect red_block_rect;
extern cv::Rect plate_rect;
extern int item_flag;
//extern std::mutex g_mutex;  // 全局锁
extern int marker_status;  
const int LCDH   = 60 ;  // 图像高度（行）
const int LCDW    =80 ;  // 图像宽度（列）
extern sint16 Road_Wide[LCDH]; 
extern ls_atim_pwm pwm1;
extern ls_atim_pwm pwm2;
extern ls_gpio polar_pwm1;
extern ls_gpio polar_pwm2;
extern ls_encoder_pwm enc1;
extern ls_encoder_pwm enc2;
 extern  lq_udp_client udp_client;
 extern  lq_udp_client udp_client_img;
  extern  lq_udp_client udp_client_img2;
 volatile extern int mid;
volatile extern float latest_error;
volatile extern float encoder_1;
volatile extern float encoder_2;
volatile extern float P1_motor;
 volatile extern float P2_motor;
  volatile extern float I1_motor;
 volatile extern float I2_motor;
volatile extern float set_speed_of_motor1_rps;
volatile extern float set_speed_of_motor2_rps;
volatile extern float pwm1_duty_rps;
volatile extern float pwm2_duty_rps;
volatile extern int current_pwm1;
volatile extern int current_pwm2;
volatile  extern int test_count ;
volatile extern float filtered_rps1;
volatile extern float filtered_rps2;
volatile extern float P;
volatile extern float I;
volatile extern float D;
volatile extern float dir_P;
volatile extern float dir_D;
volatile extern int spd_slow_ratio;
volatile extern float alpha_flit;   // 可调，0.7~0.85都可以先试
 volatile extern float encoder1_speed_avg ;
volatile extern float encoder2_speed_avg ;//demo for encoder ave
 extern lq_camera_ex cam;
 extern cv::Mat bgr_bird;
void start_camera(void); // start camera
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
int img_test(cv::Mat& frame);
void PID_control_test(float error);
void detectRedPlate(cv::Mat& frame);
float calculate_diffrential(float error,float expect_error);
float img_return(void);
void lq_ncnn_photo_demo(cv::Mat& image,std::string& a);
void cut(void);
void close_circle_control(
    float speed_of_motor1,  
    float speed_of_motor2,
    float target_speed_of_motor1_RPS,
    float target_speed_of_motor2_RPS);
    void input_speed(float&set_speed_of_motor1_rps,float& set_speed_of_motor2_rps);
    void test_enc_and_motor(int expected_speed_of_motor1_pwm,int expected_speed_of_motor2_pwm);
    void input_speed_rps();
    void  test_enc_and_motor_rps();
void vofa_receive(lq_udp_client &udp);



#endif
