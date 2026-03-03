#ifndef __LQ_ALL_DEMO_H
#define __LQ_ALL_DEMO_H

#include "lq_drv_inc.hpp"

void lq_gpio_output_demo(void);     // GPIO 输出模式测试

void lq_gpio_input_demo(void);      // GPIO 输入模式测试

void lq_pwm_demo(void);             // PWM 输出模式测试

void lq_gtim_pwm_demo(void);        // GTIM PWM 输出模式测试

void lq_atim_pwm_demo(void);        // ATIM PWM 输出模式测试

void lq_encoder_pwm_demo(void);     // 编码器 PWM 输出模式测试

void lq_canfd_demo(void);          // CANFD 测试

void lq_ncnn_demo(void);           // NCNN 测试

void lq_ncnn_photo_demo(void);     // NCNN 图像分类测试

void lq_ips20_demo(void);           //IPS屏幕测试


#endif
