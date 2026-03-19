#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.
            
 ********************************************************************************/
void PID_control_test(void)
{
    // 初始化 GPIO 引脚 81 为输出模式
    ls_gpio motor_1(PIN_81, GPIO_MODE_OUT);
    ls_gpio motor_2(PIN_82, GPIO_MODE_OUT);
    int count=0;
    while (1)
    {
        
        std::cout << "GPIO output demo: setting GPIO 81 and 82HIGH" << std::endl;
        motor_1.gpio_level_set(GPIO_HIGH);
        motor_2.gpio_level_set(GPIO_HIGH);
        usleep(2);
        motor_1.gpio_level_set(GPIO_LOW);
        motor_2.gpio_level_set(GPIO_LOW);
        usleep(20);
        count++;
         if(count==1000)
        {
            break;
        }
    }
}
