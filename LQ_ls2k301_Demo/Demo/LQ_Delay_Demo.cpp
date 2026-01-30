#include "LQ_demo.hpp"


/********************************************************************************
 * @brief   sleep函数演示
 * @param   none.
 * @return  none.
 * @note    该函数演示了使用sleep函数进行简单的延时操作
 *          每隔1秒打印一次信息，形成无限循环的延时演示
 ********************************************************************************/
void sleepDemo()
{
    while (1)
    {
        printf("sleep function delay 1s\n");
        sleep(1);
    }
}


/********************************************************************************
 * @brief   usleep函数演示
 * @param   none.
 * @return  none.
 * @note    该函数演示了usleep函数的使用方法，通过循环打印信息并使用usleep实现0.5秒的延迟。
 *          函数会持续运行，每隔0.5秒输出一次提示信息。
 ********************************************************************************/
void usleepDemo()
{
    while (1)
    {
        printf("usleep function delay 0.5s\n");
        usleep(500000); 
    }
}


/********************************************************************************
 * @brief   nanosleep函数演示
 * @param   none.
 * @return  none.
 * @note    该函数演示了使用nanosleep函数进行高精度延时操作
 *          通过timespec结构体设置延时时间，实现纳秒级精度的延时控制
 *          函数会持续运行，每隔0.5秒输出一次提示信息
 ********************************************************************************/
void nanosleepDemo()
{
    struct timespec req;   
    struct timespec rem;
    req.tv_sec = 0; 
    req.tv_nsec = 500000000;   
    while(1)
    {
        printf("nanosleep function delay 0.5s\n");
        nanosleep(&req, &rem);
    }
}


/**
 * clock_nanosleep函数演示
 * 
 * 该函数演示了使用clock_nanosleep函数进行高精度延时操作
 * 通过timespec结构体设置延时时间，使用CLOCK_MONOTONIC时钟实现纳秒级精度的延时控制
 * 函数会持续运行，每隔0.5秒输出一次提示信息
 */
void clock_nanosleepDemo()
{
    struct timespec req;    
    struct timespec rem;    
    req.tv_sec = 0;             
    req.tv_nsec = 500000000;    
    while(1)
    {
        printf("clock_nanosleep function delay 0.5s\n");
        clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &rem);
    }
}
