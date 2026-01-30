#include "LQ_demo.hpp"

#include <signal.h>
#include <sys/time.h>

#include <thread>

using namespace std;

/********************************************************************************
 * @brief   定时器中断处理函数
 * @param   none.
 * @return  none.
 ********************************************************************************/
void timer_handler(int signum)
{
    static uint64_t val = 0;
    printf("Timer signal received! %ld\n", val++);
}

/********************************************************************************
 * @brief   setitimerDemo函数演示
 * @param   none.
 * @return  none.
 * @note    该函数演示了使用setitimer设置定时器的基本用法。函数设置一个实时定时器，
 *          初始延迟2秒后触发，之后每5毫秒触发一次定时器中断。定时器触发时调用
 *          timer_handler函数处理定时器事件.
 * @note    定时器配置：
 *          - 初始延迟：2秒
 *          - 周期间隔：5毫秒
 *          - 定时器类型：ITIMER_REAL（实时定时器）
 * @note    注意：该函数会进入无限循环，需要手动终止程序才能退出。
 ********************************************************************************/
void setitimerDemo()
{
    signal(SIGALRM, timer_handler);
    struct itimerval timer;
    timer.it_value.tv_sec = 2;          
    timer.it_value.tv_usec = 0;         
    timer.it_interval.tv_sec = 0;    
    timer.it_interval.tv_usec = 5000; 

    if (setitimer(ITIMER_REAL, &timer, NULL) == -1)
    {
        perror("setitimer");
        return;
    }
    printf("Waiting for timer...\n");
    while(1)
    {
        printf("setitimerDemo!\n");
        sleep(1);
    }
}

/********************************************************************************
 * @brief   线程处理函数
 * @param   none.
 * @return  none.
 ********************************************************************************/
void timer_thread()
{
    static uint64_t val = 0;
    while(true)
    {

        printf("timer_thread! %ld\n", val++);

        this_thread::sleep_for(chrono::milliseconds(5));
    }
}

/********************************************************************************
 * @brief   HibernateDemo函数演示
 * @param   none.
 * @return  none.
 * @note    该函数演示了使用C++11线程库创建后台定时器线程的基本用法。函数创建一个
 *          后台线程运行timer_thread函数，该线程每5毫秒输出一次计数器值。主线程
 *          则每秒输出一次HibernateDemo信息，形成双线程并发执行的效果。
 * @note    线程配置：
 *          - 主线程：每秒输出HibernateDemo信息
 *          - 后台线程：每5毫秒输出计数器值
 * @note    注意：该函数会进入无限循环，需要手动终止程序才能退出。
 ********************************************************************************/
void HibernateDemo()
{
    thread t(timer_thread);

    while(1)
    {
        printf("HibernateDemo!\n");
        sleep(1);
    }
}
