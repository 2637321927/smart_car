#include "LQ_I2C_ALL_DRV.h"
#include "LQ_MPU6050_drv.h"
#include "LQ_I2C_RDWR_drv.h"

/********************************************************************************
 * @brief   设置陀螺仪测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±250dps    1 --> ±500dps
 *                2 --> ±1000dps   3 --> ±2000dps
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Set_Gyro_Fsr(struct ls_i2c_dev *dev, uint8_t fsr)
{
    return i2c_write_reg(dev, MPU_GYRO_CFG_REG, fsr << 3);
}

/********************************************************************************
 * @brief   设置加速度计测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 0 --> ±2g    1 --> ±4g
 *                2 --> ±8g    3 --> ±16g
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Set_Accel_Fsr(struct ls_i2c_dev *dev, uint8_t fsr)
{
    return i2c_write_reg(dev, MPU_ACCEL_CFG_REG, fsr << 3);
}

/********************************************************************************
 * @brief   设置数字低通滤波
 * @param   dev : 自定义 I2C 相关结构体
 * @param   lpf : 数字低通滤波频率(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Set_LPF(struct ls_i2c_dev *dev, uint16_t lpf)
{
    uint8_t dat = 0;
    if (lpf >= 188)
        dat = 1;
    else if (lpf >= 98)
        dat = 2;
    else if (lpf >= 42)
        dat = 3;
    else if (lpf >= 20)
        dat = 4;
    else if (lpf >= 10)
        dat = 5;
    else
        dat = 6;
    return i2c_write_reg(dev, MPU_CFG_REG, dat); // 设置数字低通滤波器
}

/********************************************************************************
 * @brief   设置采样率
 * @param   dev : 自定义 I2C 相关结构体
 * @param   rate: 4 ~ 1000(Hz)
 * @return  为 1 表示设置成功，小于等于 0 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Set_Rate(struct ls_i2c_dev *dev, uint16_t rate)
{
    uint8_t dat;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    dat = 1000 / rate - 1;
    i2c_write_reg(dev, MPU_SAMPLE_RATE_REG, dat); // 设置数字低通滤波器
    return MPU6050_Set_LPF(dev, rate / 2);            // 自动设置LPF为采样率的一半
}

/********************************************************************************
 * @brief    获取温度值
 * @param    dev : 自定义 I2C 相关结构体
 * @return   温度值(扩大了100倍)
 * @date     2019/6/12
 ********************************************************************************/
int16_t MPU6050_Get_Temperature(struct ls_i2c_dev *dev)
{
    uint8_t buf[2];
    int16_t raw;
    int32_t temp;
    const int32_t SCALE = 10000;        // 放大倍数
    const int32_t DIVISOR = 33387;      // 333.87 * 100
    const int32_t OFFSET = 21 * SCALE;  // 偏移量放大

    i2c_read_regs(dev, MPU_TEMP_OUTH_REG, buf, 2);
    raw = (((uint16_t)buf[0] << 8) | buf[1]);
    temp = OFFSET + (raw * SCALE) / DIVISOR;
    
    return (int16_t)(temp / 100);
}

/********************************************************************************
 * @brief   获取陀螺仪值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Get_Gyroscope(struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, MPU_GYRO_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/********************************************************************************
 * @brief   获取加速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Get_Accelerometer(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, MPU_ACCEL_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/********************************************************************************
 * @brief   获取 加速度值 角速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 陀螺仪 x,y,z 轴的加速度值原始读数(带符号)
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的角速度值原始读数(带符号)
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
uint8_t MPU6050_Get_Raw_data(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14], res;
    res = i2c_read_regs(dev, MPU_ACCEL_XOUTH_REG, buf, 14);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
        *gx = ((uint16_t)buf[8] << 8) | buf[9];
        *gy = ((uint16_t)buf[10] << 8) | buf[11];
        *gz = ((uint16_t)buf[12] << 8) | buf[13];
    }
    return res;
}

/********************************************************************************
 * @brief   初始化MPU6050
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/3/20
 ********************************************************************************/
uint8_t I2C_MPU6050_Init(struct ls_i2c_dev *dev, ls_all_mod *mod)
{
    u8 res;
    if (mod->I2C_MPU6050 != 1)      // 器件 ID 正确
    {
        printk("Gyro_ID not MPU6050 0x%x\n", res);
        return -1;
    }
    res = 0;
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    Delay_Ms(100);                                      // 延时100ms
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    res += MPU6050_Set_Gyro_Fsr(dev, 3);                    // 陀螺仪传感器,±2000dps
    res += MPU6050_Set_Accel_Fsr(dev, 1);                   // 加速度传感器,±4g
    res += MPU6050_Set_Rate(dev, 1000);                     // 设置采样率1000Hz
    res += i2c_write_reg(dev, MPU_CFG_REG, 0x02);       // 设置数字低通滤波器   98hz
    res += i2c_write_reg(dev, MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    res += i2c_write_reg(dev, MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    res += i2c_write_reg(dev, MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
    res += i2c_write_reg(dev, MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
    printk("MPU6050 Init success\n");
    return 0;
}
