#include "LQ_demo.hpp"

#define Servo_0     80000
#define Servo_90    240000
#define Servo_180   400000

/********************************************************************************
 * @brief   舵机演示函数
 * @param   none.
 * @return  none.
 * @note    该函数演示舵机从0度到180度再到0度的往复运动。
 *          使用PWM信号控制舵机角度，通过调整占空比来实现角度变化。
 * @note    功能说明：
 *          - 初始化PWM输出，频率为50Hz，反转模式
 *          - 舵机角度范围：0度到180度
 *          - 每次角度变化间隔50ms，实现平滑运动
 *          - 程序无限循环执行往复运动
 ********************************************************************************/
void ServoDemo()
{
    GtimPwm Servo(88,2,LS_GTIM_INVERSED, 3200000, Servo_0);

    Servo.Enable();

    while (1)
    {
        // 从0度缓慢增加到180度
        for (int step = 0; step <= 100; step++)
        {
            uint32_t duty = Servo_0 + (Servo_180 - Servo_0) * step / 100;
            Servo.SetDutyCycle(duty);
            usleep(50000); // 50ms延迟，控制速度
        }

        // 从180度缓慢减少到0度
        for (int step = 100; step >= 0; step--)
        {
            uint32_t duty = Servo_0 + (Servo_180 - Servo_0) * step / 100;
            Servo.SetDutyCycle(duty);
            usleep(50000); // 50ms延迟，控制速度
        }
    }
}

