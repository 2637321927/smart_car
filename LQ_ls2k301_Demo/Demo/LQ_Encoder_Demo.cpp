#include "LQ_demo.hpp"


/********************************************************************************
 * @brief   编码器演示函数
 * @param   none.
 * @return  none.
 * @note    创建四个LS_PwmEncoder编码器实例，分别对应不同的GPIO引脚，
 *          初始化后持续循环读取并显示编码器值。
 ********************************************************************************/
void EncoderDemo()
{
    // 创建四个编码器实例，分别对应不同的GPIO引脚
    LS_PwmEncoder Encoder1(0, 72); //64
    LS_PwmEncoder Encoder2(1, 73); //65
    LS_PwmEncoder Encoder3(2, 74); //66
    LS_PwmEncoder Encoder4(3, 75); //67

    // 初始化所有编码器
    Encoder1.Init();
    Encoder2.Init();
    Encoder3.Init();
    Encoder4.Init();
    // 持续循环读取并显示编码器值
    while(1)
    {
        cout << "Encoder1=" << Encoder1.Update() 
        << ",Encoder2=" << Encoder2.Update() 
        << ",Encoder3=" << Encoder3.Update() 
        << ",Encoder4=" << Encoder4.Update() 
        << endl;
        usleep(50000);
    }
}
