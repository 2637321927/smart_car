#include "lq_all_demo.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>

// 解析VOFA+发来的指令
void parse_vofa_cmd(const char *str)
{
    if (!str) return;
    printf("receive\n");
    // 格式：kp=2.5  kd=0.8  th=170  speed=400
    if (strstr(str, "cnm")) {
        //track_kp = atof(str + 3);
        printf("nmsl");
    }
}

// 接收VOFA数据 函数
void vofa_receive(lq_udp_client &udp)
{
    char recv_buf[128];  // 接收缓冲区
    
    // 非阻塞接收
    ssize_t recv_len = udp.udp_recv(recv_buf, sizeof(recv_buf) - 1);

    if (recv_len > 0)
    {
        recv_buf[recv_len] = '\0';  // 必须加结束符
        printf("接收VOFA: %s\n", recv_buf);
        parse_vofa_cmd(recv_buf);   // 解析指令
    }
}


