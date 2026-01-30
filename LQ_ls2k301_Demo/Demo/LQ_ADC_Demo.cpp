#include "LQ_demo.hpp"

#define ADC_0 "in_voltage0_raw"
#define ADC_1 "in_voltage1_raw"
#define ADC_2 "in_voltage2_raw"
#define ADC_3 "in_voltage3_raw"
#define ADC_4 "in_voltage4_raw"
#define ADC_5 "in_voltage5_raw"
#define ADC_6 "in_voltage6_raw"
#define ADC_7 "in_voltage7_raw"

#define ADC_SCALE "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

/********************************************************************************
 * @brief   ADC功能演示函数
 * @param   none.
 * @retval  none.
 * @note    该函数演示了从ADC设备读取数据的过程，包括：
 *          - 读取ADC的scale值用于单位转换
 *          - 循环读取ADC原始数据
 *          - 将原始数据转换为实际电压值并显示
 * @note    函数会持续运行，每秒读取一次ADC数据并输出结果，
 *          直到出现错误或手动退出。
 ********************************************************************************/
void AdcFunDemo()
{
    int fd_scale = open(ADC_SCALE, O_RDONLY);
    if(fd_scale < 0)
    {
        perror("open1");
        return;
    }
    char scale[10] = {0};
    if (read(fd_scale, scale, sizeof(scale)) != sizeof(scale))
    {
        perror("read1");
        return;
    }
    close(fd_scale);

    double scaleNum = strtod(scale, nullptr);
    printf("scaleNum = %f\n", scaleNum);
    string str = "/sys/bus/iio/devices/iio:device0/" + string(ADC_0);
    char num[4] = {0};
    double Num = 0;
    while (1)
    {
        int fd_adc = open(str.c_str(), O_RDONLY);
        if (fd_adc < 0)
        {
            perror("open2");
            return;
        }
        if (read(fd_adc, num, sizeof(num)) <= 0)
        {
            perror("read2");
        }
        close(fd_adc);
        Num = strtod(num, nullptr);
        printf("Num = %f, ADC = %f\n", Num, scaleNum*Num/1000);
        memset(num, 0, sizeof(num));
        sleep(1);
    }
}
