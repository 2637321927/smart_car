#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   UART通信演示函数
 * @param   none.
 * @return  none.
 * @note    该函数演示了UART通信的基本操作，包括：
 *          - 初始化UART1，波特率115200，8位数据位，1位停止位，无校验位
 *          - 循环读取UART数据并打印接收到的字节数和内容
 *          - 将接收到的数据回写到UART
 *          - 每次操作间隔1秒
 * @note    错误处理：
 *          - 读取数据失败时打印错误信息并退出函数
 *          - 写入数据失败时打印错误信息并退出函数
 ********************************************************************************/
void Uart_Demo()
{
    LS_UART uart(UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_NONE);
    
    while(1) {
        char buf[256];
        int ret = uart.ReadData(buf, sizeof(buf));
        if (ret < 0) {
            printf("Reading error!\n");
            return;
        }
        buf[ret] = '\0';
        printf("Receive %d bytes : %s\n", ret, buf);
        ret = uart.WriteData(buf, ret);
        if (ret < 0) {
            printf("Writeing error!\n");
            return;
        }
        sleep(1);
    }
    return;
}
