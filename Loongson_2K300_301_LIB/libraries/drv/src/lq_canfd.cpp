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
#include "lq_canfd.hpp"

// 静态成员初始化
LS_CANFD* LS_CANFD::s_instance = nullptr;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LS_CANFD::LS_CANFD()
 * @功能说明：CANFD类的无参构造函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：LS_CANFD can;
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LS_CANFD::LS_CANFD()
    : m_socket(-1)
    , m_rx_mode(CANFD_MODE_THREAD)
    , m_rx_callback(nullptr)
    , m_initialized(false)
    , m_running(false)
{
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LS_CANFD::LS_CANFD(const string &ifname, CANFD_RxMode_e rx_mode, CANFD_RxCallback_t callback)
 * @功能说明：CANFD类的带参构造函数
 * @参数说明：ifname   : CAN接口名称，如 "can0", "can1"
 * @参数说明：rx_mode  : 接收模式
 * @参数说明：callback : 接收回调函数
 * @函数返回：无
 * @调用方法：LS_CANFD can(CAN1, CANFD_MODE_THREAD, callback);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LS_CANFD::LS_CANFD(const string &ifname, CANFD_RxMode_e rx_mode, CANFD_RxCallback_t callback)
    : m_socket(-1)
    , m_rx_mode(rx_mode)
    , m_rx_callback(callback)
    , m_initialized(false)
    , m_running(false)
{
    Init(ifname, rx_mode, callback);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：bool LS_CANFD::Init(const string &ifname, CANFD_RxMode_e rx_mode, CANFD_RxCallback_t callback)
 * @功能说明：初始化CANFD
 * @参数说明：ifname   : CAN接口名称
 * @参数说明：rx_mode  : 接收模式
 * @参数说明：callback : 接收回调函数
 * @函数返回：true:成功 false:失败
 * @调用方法：can.Init(CAN1, CANFD_MODE_THREAD, callback);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
bool LS_CANFD::Init(const string &ifname, CANFD_RxMode_e rx_mode, CANFD_RxCallback_t callback)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    char cmd[256];

    m_ifname = ifname;
    m_rx_mode = rx_mode;
    m_rx_callback = callback;

    // 0. 先关闭CAN接口（如果已打开）
    snprintf(cmd, sizeof(cmd), "ip link set %s down", ifname.c_str());
    system(cmd);

    // 1. 配置并启动CAN接口 (CANFD模式: 500kbps 常规 + 2Mbps 数据速率)
    snprintf(cmd, sizeof(cmd),
             "ip link set %s up type can bitrate 500000 dbitrate 2000000 fd on",
             ifname.c_str());
    printf("执行: %s\n", cmd);
    int ret = system(cmd);
    if (ret != 0) {
        printf("警告: CAN接口配置命令执行失败，尝试继续...\n");
    }

    // 等待接口启动
    usleep(100000);  // 100ms

    // 2. 创建socket
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket < 0) {
        perror("CANFD socket");
        return false;
    }

    // 3. 启用CANFD模式
    int enable_fd = 1;
    if (setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        perror("CANFD setsockopt");
        close(m_socket);
        m_socket = -1;
        return false;
    }

    // 4. 绑定CAN接口
    strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("CANFD ioctl");
        close(m_socket);
        m_socket = -1;
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("CANFD bind");
        close(m_socket);
        m_socket = -1;
        return false;
    }

    // 5. 根据模式设置接收方式
    if (m_rx_mode == CANFD_MODE_ASYNC) {
        // 异步信号模式
        if (!SetupAsyncMode()) {
            close(m_socket);
            m_socket = -1;
            return false;
        }
    } else if (m_rx_mode == CANFD_MODE_THREAD) {
        // 独立线程模式（推荐）
        StartRxThread();
    } else {
        // 阻塞模式设置非阻塞标志（用于poll超时）
        int flags = fcntl(m_socket, F_GETFL, 0);
        fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
    }

    m_initialized = true;
    
    const char* mode_str = "阻塞";
    if (m_rx_mode == CANFD_MODE_ASYNC) mode_str = "异步信号";
    else if (m_rx_mode == CANFD_MODE_THREAD) mode_str = "独立线程";
    
    printf("CANFD初始化成功 (%s), 模式: %s\n", m_ifname.c_str(), mode_str);

    return true;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：bool LS_CANFD::SetupAsyncMode()
 * @功能说明：设置异步信号接收模式
 * @参数说明：无
 * @函数返回：true:成功 false:失败
 * @调用方法：内部调用
 * @备注说明：会中断主线程的sleep
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
bool LS_CANFD::SetupAsyncMode()
{
    // 设置信号处理函数
    signal(SIGIO, SignalHandler);

    // 设置socket所有者
    if (fcntl(m_socket, F_SETOWN, getpid()) < 0) {
        perror("CANFD fcntl F_SETOWN");
        return false;
    }

    // 启用异步I/O
    int flags = fcntl(m_socket, F_GETFL);
    if (fcntl(m_socket, F_SETFL, flags | O_ASYNC | O_NONBLOCK) < 0) {
        perror("CANFD fcntl F_SETFL");
        return false;
    }

    // 保存实例指针
    s_instance = this;

    return true;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LS_CANFD::StartRxThread()
 * @功能说明：启动接收线程
 * @参数说明：无
 * @函数返回：无
 * @调用方法：内部调用
 * @备注说明：独立线程接收，不影响主线程
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LS_CANFD::StartRxThread()
{
    m_running = true;
    m_rx_thread = thread(&LS_CANFD::RxThreadFunc, this);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LS_CANFD::RxThreadFunc()
 * @功能说明：接收线程函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：内部调用
 * @备注说明：独立线程中运行，可安全调用回调
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LS_CANFD::RxThreadFunc()
{
    struct pollfd pfd;
    pfd.fd = m_socket;
    pfd.events = POLLIN;

    while (m_running) {
        // 等待数据，100ms超时检查一次运行状态
        int ret = poll(&pfd, 1, 100);
        
        if (!m_running) {
            break;
        }

        if (ret > 0 && (pfd.revents & POLLIN)) {
            struct canfd_frame frame;
            int nbytes = read(m_socket, &frame, sizeof(frame));
            
            if (nbytes > 0 && m_rx_callback != nullptr) {
                CANFD_Frame_t rx_frame;
                rx_frame.can_id = frame.can_id;
                rx_frame.len = frame.len;
                memcpy(rx_frame.data, frame.data, frame.len);
                
                // 在独立线程中调用回调，可安全使用printf
                m_rx_callback(rx_frame);
            }
        }
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LS_CANFD::SignalHandler(int signo)
 * @功能说明：SIGIO信号处理函数
 * @参数说明：signo : 信号编号
 * @函数返回：无
 * @调用方法：系统自动调用
 * @备注说明：有数据到来时自动触发
 * @注意事项：信号处理函数中不能使用printf等非async-signal-safe函数
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LS_CANFD::SignalHandler(int signo)
{
    if (s_instance == nullptr) {
        return;
    }

    struct canfd_frame frame;
    int nbytes;

    // 读取所有可用数据
    while (1) {
        nbytes = read(s_instance->m_socket, &frame, sizeof(frame));
        if (nbytes <= 0) {
            break;
        }

        // 如果有回调函数，构造帧并调用
        if (s_instance->m_rx_callback != nullptr) {
            // 构造帧结构体
            CANFD_Frame_t rx_frame;
            rx_frame.can_id = frame.can_id;
            rx_frame.len = frame.len;
            memcpy(rx_frame.data, frame.data, frame.len);

            // 注意：回调函数中不要使用printf，会导致阻塞！
            s_instance->m_rx_callback(rx_frame);
        }
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：int LS_CANFD::Write(uint32_t can_id, const uint8_t *data, uint8_t len)
 * @功能说明：发送CANFD数据
 * @参数说明：can_id : CAN ID
 * @参数说明：data   : 数据指针
 * @参数说明：len    : 数据长度（最大64字节）
 * @函数返回：发送字节数，-1表示失败
 * @调用方法：can.Write(0x123, data, 8);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LS_CANFD::Write(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (!m_initialized || m_socket < 0) {
        return -1;
    }

    if (len > CANFD_MAX_DATA_LEN) {
        len = CANFD_MAX_DATA_LEN;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    memcpy(frame.data, data, len);

    return write(m_socket, &frame, sizeof(frame));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：int LS_CANFD::Write(const CANFD_Frame_t &frame)
 * @功能说明：发送CANFD帧
 * @参数说明：frame : CANFD帧结构体
 * @函数返回：发送字节数，-1表示失败
 * @调用方法：can.Write(frame);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LS_CANFD::Write(const CANFD_Frame_t &frame)
{
    return Write(frame.can_id, frame.data, frame.len);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：int LS_CANFD::Read(CANFD_Frame_t &frame, int timeout_ms)
 * @功能说明：阻塞接收CANFD数据
 * @参数说明：frame      : 接收帧结构体
 * @参数说明：timeout_ms : 超时时间(毫秒)，-1表示无限等待
 * @函数返回：接收字节数，-1表示失败，0表示超时
 * @调用方法：can.Read(frame, 1000);
 * @备注说明：适用于阻塞模式
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LS_CANFD::Read(CANFD_Frame_t &frame, int timeout_ms)
{
    if (!m_initialized || m_socket < 0) {
        return -1;
    }

    struct pollfd pfd;
    pfd.fd = m_socket;
    pfd.events = POLLIN;

    // 等待数据
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        perror("CANFD poll");
        return -1;
    }
    if (ret == 0) {
        return 0;  // 超时
    }

    // 读取数据
    struct canfd_frame can_frame;
    int nbytes = read(m_socket, &can_frame, sizeof(can_frame));
    if (nbytes <= 0) {
        return -1;
    }

    // 填充帧结构体
    frame.can_id = can_frame.can_id;
    frame.len = can_frame.len;
    memcpy(frame.data, can_frame.data, can_frame.len);

    return nbytes;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LS_CANFD::SetRxCallback(CANFD_RxCallback_t callback)
 * @功能说明：设置接收回调函数
 * @参数说明：callback : 回调函数
 * @函数返回：无
 * @调用方法：can.SetRxCallback([](const CANFD_Frame_t &frame){...});
 * @备注说明：异步模式或线程模式下使用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LS_CANFD::SetRxCallback(CANFD_RxCallback_t callback)
{
    m_rx_callback = callback;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LS_CANFD::~LS_CANFD()
 * @功能说明：CANFD类的析构函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：创建的对象生命周期结束后系统自动调用
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LS_CANFD::~LS_CANFD()
{
    // 停止接收线程
    m_running = false;
    if (m_rx_thread.joinable()) {
        m_rx_thread.join();
    }

    if (m_socket >= 0) {
        close(m_socket);
        m_socket = -1;
    }
    if (s_instance == this) {
        s_instance = nullptr;
    }
}
