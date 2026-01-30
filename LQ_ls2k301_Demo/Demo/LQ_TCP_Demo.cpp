#include "LQ_demo.hpp"


/********************************************************************************
 * @brief   TCP客户端演示函数
 * @param   none.
 * @return  none.
 * @note    创建TCP客户端连接到指定服务器，实现双向通信：
 *          1. 首先接收服务器发送的欢迎消息
 *          2. 进入循环，允许用户从键盘输入消息并发送给服务器
 *          3. 接收服务器返回的响应消息并显示
 *          4. 重复步骤2-3，直到程序被手动终止
 * @note    该函数使用 127.0.0.1:9999 作为默认服务器地址和端口
 * @note    使用缓冲区大小为 255 字节的字符数组进行数据收发
 ********************************************************************************/
void TCP_Demo()
{
    TCP_Client cli("127.0.0.1", 9999);

    char buf[255] = {0};
    cli.TCP_Recv(buf, sizeof(buf));
    printf("%s\n", buf);

    while (1)
    {
        memset(buf, 0, sizeof(buf));   
        fgets(buf, sizeof(buf), stdin); 
        cli.TCP_Send(buf, strlen(buf));
        cli.TCP_Recv(buf, sizeof(buf)); 
        printf("cli Recv:%s\n", buf);   
    }
}
