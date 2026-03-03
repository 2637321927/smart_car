/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：新增线程接收模式
@注意事项：使用前需确保CAN接口已正确配置
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <thread>
#include <atomic>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>

using namespace std;

#define CANFD_MAX_DATA_LEN  64  // CANFD最大数据长度

// CAN接口名称宏定义
#define CAN0    "can0"
#define CAN1    "can1"

// 接收模式枚举
typedef enum
{
    CANFD_MODE_BLOCKING = 0,    // 阻塞模式
    CANFD_MODE_ASYNC    = 1,    // 异步信号模式（会中断主线程sleep）
    CANFD_MODE_THREAD   = 2,    // 独立线程模式（推荐，不影响主线程）
} CANFD_RxMode_e;

// CAN数据帧结构体
typedef struct
{
    uint32_t can_id;            // CAN ID
    uint8_t  len;               // 数据长度
    uint8_t  data[CANFD_MAX_DATA_LEN];  // 数据
} CANFD_Frame_t;

// 接收回调函数类型
typedef function<void(const CANFD_Frame_t &frame)> CANFD_RxCallback_t;

class LS_CANFD
{
public:
    /*!
     * @brief   CANFD无参构造函数
     */
    LS_CANFD();

    /*!
     * @brief   CANFD带参构造函数
     * @param   ifname   : CAN接口名称，如 "can0", "can1"
     * @param   rx_mode  : 接收模式
     * @param   callback : 接收回调函数
     */
    LS_CANFD(const string &ifname, CANFD_RxMode_e rx_mode = CANFD_MODE_THREAD, 
             CANFD_RxCallback_t callback = nullptr);

    /*!
     * @brief   初始化CANFD
     * @param   ifname   : CAN接口名称
     * @param   rx_mode  : 接收模式
     * @param   callback : 接收回调函数
     * @return  true:成功 false:失败
     */
    bool Init(const string &ifname, CANFD_RxMode_e rx_mode = CANFD_MODE_THREAD,
              CANFD_RxCallback_t callback = nullptr);

    /*!
     * @brief   发送CANFD数据
     * @param   can_id : CAN ID
     * @param   data   : 数据指针
     * @param   len    : 数据长度（最大64字节）
     * @return  发送字节数，-1表示失败
     */
    int Write(uint32_t can_id, const uint8_t *data, uint8_t len);

    /*!
     * @brief   发送CANFD帧
     * @param   frame : CANFD帧结构体
     * @return  发送字节数，-1表示失败
     */
    int Write(const CANFD_Frame_t &frame);

    /*!
     * @brief   阻塞接收CANFD数据
     * @param   frame : 接收帧结构体
     * @param   timeout_ms : 超时时间(毫秒)，-1表示无限等待
     * @return  接收字节数，-1表示失败，0表示超时
     */
    int Read(CANFD_Frame_t &frame, int timeout_ms = -1);

    /*!
     * @brief   设置接收回调函数
     * @param   callback : 回调函数
     */
    void SetRxCallback(CANFD_RxCallback_t callback);

    /*!
     * @brief   获取Socket描述符
     * @return  socket描述符
     */
    int GetSocket() const { return m_socket; }

    /*!
     * @brief   CANFD析构函数
     */
    ~LS_CANFD();

private:
    /*!
     * @brief   设置异步信号模式
     * @return  true:成功 false:失败
     */
    bool SetupAsyncMode();

    /*!
     * @brief   启动接收线程
     */
    void StartRxThread();

    /*!
     * @brief   接收线程函数
     */
    void RxThreadFunc();

    /*!
     * @brief   SIGIO信号处理函数
     */
    static void SignalHandler(int signo);

    int                 m_socket;       // socket描述符
    string              m_ifname;       // 接口名称
    CANFD_RxMode_e      m_rx_mode;      // 接收模式
    CANFD_RxCallback_t  m_rx_callback;  // 接收回调函数
    bool                m_initialized;  // 初始化标志

    // 线程相关
    thread              m_rx_thread;    // 接收线程
    atomic<bool>        m_running;      // 线程运行标志

    static LS_CANFD*    s_instance;     // 单例指针（用于信号处理）
};
