#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   GPIO输入演示函数
 * @param   none.
 * @return  none.
 * @note    该函数演示了如何配置多个GPIO引脚为输入模式，并循环读取它们的电平状态。
 *          函数会持续监控GPIO44、GPIO45、GPIO80、GPIO20和GPIO17这5个引脚的电平变化，
 *          并以表格形式实时显示各引脚的当前状态（HIGH/LOW）和对应的时间戳。
 * @note    程序运行时会持续循环，直到用户按下Ctrl+C键退出。
 * @note    该函数使用100ms的间隔进行循环读取，确保不会过度占用CPU资源。
 ********************************************************************************/
void GpioInputDemo()
{
    HWGpio gpio44(44, GPIO_Mode_In);
    HWGpio gpio45(45, GPIO_Mode_In);
    HWGpio gpio80(80, GPIO_Mode_In);
    HWGpio gpio20(20, GPIO_Mode_In);
    HWGpio gpio17(17, GPIO_Mode_In);

    std::cout << "开始循环读取GPIO引脚电平..." << std::endl;
    std::cout << "按 Ctrl+C 退出程序" << std::endl << std::endl;

    // 打印表头
    std::cout << "引脚\\时间\tGPIO44\tGPIO45\tGPIO80\tGPIO20\tGPIO17" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;

    // 循环读取引脚电平
    while (true) {
        // 读取各引脚电平值
        bool val44 = gpio44.GetGpioValue();
        bool val45 = gpio45.GetGpioValue();
        bool val80 = gpio80.GetGpioValue();
        bool val20 = gpio20.GetGpioValue();
        bool val17  = gpio17.GetGpioValue();

        // 打印当前时间戳和各引脚状态
        std::cout << "Time: " << time(nullptr) << "\t";
        std::cout << (val44 ? "HIGH" : "LOW") << "\t";
        std::cout << (val45 ? "HIGH" : "LOW") << "\t";
        std::cout << (val80 ? "HIGH" : "LOW") << "\t";
        std::cout << (!val20 ? "HIGH" : "LOW") << "\t";
        std::cout << (!val17  ? "HIGH" : "LOW") << std::endl;

        // 延时100ms后再次读取
        usleep(100000);  // 100ms = 100,000微秒
    }
}
