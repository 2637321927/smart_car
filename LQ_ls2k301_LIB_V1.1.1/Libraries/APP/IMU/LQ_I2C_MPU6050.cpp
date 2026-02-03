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

@修改日期：2025-03-25
@修改内容：
@注意事项：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "LQ_I2C_MPU6050.hpp"

/*!
 * @brief   1.如果想要使用该文件中的MPU6050相关函数，需要先将对应模块加载到内核中
 * @brief   2.需要加载的模块：lq_i2c_dev.ko 和 lq_i2c_drv.ko
 * @brief   3.安装模块：insmod lq_i2c_dev.ko
 * @brief   4.卸载模块：rmmod lq_i2c_dev.ko
 * @brief   5.查看当前模块：lsmod
 */

/*!
 * @brief   构造函数
 * @param   i2c_path : I2C 驱动生成的设备文件路径
 * @return  无
 * @date    2025/3/20
 */
I2C_MPU6050::I2C_MPU6050(const std::string &i2c_path)
{
    LS_I2C_DEVS &i2c_dev = GetInstance(i2c_path);
}

/*!
 * @brief   获取陀螺仪 ID
 * @param   无
 * @return  返回陀螺仪 ID
 * @date    2025/11/20
 */
uint8_t I2C_MPU6050::I2C_MPU6050_Get_ID(void)
{
    uint8_t id = 0;
    ioctl(this->I2C_fd, I2C_GET_MPU6050_ID, &id);
    return id;
}

/*!
 * @brief   获取温度值
 * @param   无
 * @return  温度值
 * @date    2025/11/20
 */
float I2C_MPU6050::I2C_MPU6050_Get_Tem(void)
{
    int16_t tem = 0;
    ioctl(this->I2C_fd, I2C_GET_MPU6050_TEM, &tem);
    return (float)(tem / 100.0);
}

/*!
 * @brief    获取角速度值
 * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return   成功返回 0，失败返回 -1
 * @date     2025/11/20
 */
int8_t I2C_MPU6050::I2C_MPU6050_Get_Ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_MPU6050_ANG, data) != 0)
        return -1;
    *gx = data[0];
    *gy = data[1];
    *gz = data[2];
    return 0;
}

/*!
 * @brief    获取加速度值
 * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @return   成功返回 0，失败返回 -1
 * @date     2025/11/20
 */
int8_t I2C_MPU6050::I2C_MPU6050_Get_Acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t data[3] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_MPU6050_ACC, data) != 0)
        return -1;
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    return 0;
}

/*!
 * @brief    获取陀螺仪加速度值、角速度值
 * @param    ax,ay,az : 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @param    gx,gy,gz : 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return   成功返回 0
 * @date     2025/11/20
 */
int8_t I2C_MPU6050::I2C_MPU6050_Get_RawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->I2C_fd, I2C_GET_MPU6050_GYRO, data) != 0)
        return -1;
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    *gx = data[3];
    *gy = data[4];
    *gz = data[5];
    return 0;
}
