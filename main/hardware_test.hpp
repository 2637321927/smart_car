#ifndef __HARDWARE_TEST_HPP
#define __HARDWARE_TEST_HPP

// 硬件测试模式默认关闭。只有停车且其它停车态控制均空闲时才能开启，
// 避免正常巡线、遥控、航向保持或绕行脚本与测试PWM争抢电机输出。
bool hardware_test_set_enabled(bool enabled);
bool hardware_test_is_enabled();

// 设置PWM1正向占空比，范围固定为0~5000。关闭模式下只保存调试值，
// 不访问PWM硬件；每次重新开启测试都会先把该值复位为0。
void hardware_test_set_pwm(int pwm);
int hardware_test_get_pwm();

// 仅由3ms速度定时器调用。返回true表示硬件测试已独占本周期电机输出：
// PWM1按设定值正向输出，PWM2始终保持0。
bool hardware_test_update();

#endif
