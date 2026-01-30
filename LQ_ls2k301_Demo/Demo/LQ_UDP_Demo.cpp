#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   UDP通信演示函数
 * @param   none.
 * @return  none.
 * @note    该函数创建一个UDP客户端，连接到指定的IP地址和端口，
 *          实现了从标准输入读取数据并通过UDP发送，
 *          然后接收并打印返回数据的循环通信过程。
 * @note    功能说明：
 *          - 初始化UDP客户端连接到本地9999端口
 *          - 循环执行以下操作：
 *              1. 清空接收缓冲区
 *              2. 从标准输入读取用户数据
 *              3. 通过UDP发送数据
 *              4. 接收UDP返回数据
 *              5. 打印接收到的数据
 ********************************************************************************/
void UDP_Demo()
{
    UDP_Client cli("127.0.0.1", 9999);

    char buf[255] = {0};
    
    while(1)
    {
        memset(buf, 0, sizeof(buf));   
        fgets(buf, sizeof(buf), stdin);
        cli.UDP_Send(buf, strlen(buf));
        cli.UDP_Recv(buf, sizeof(buf));
        printf("cli Recv : %s\n", buf);
    }
}