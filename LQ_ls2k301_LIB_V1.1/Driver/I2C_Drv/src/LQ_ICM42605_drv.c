#include "LQ_ICM42605_drv.h"
#include "LQ_I2C_RDWR_drv.h"
#include "LQ_util_drv.h"

/********************************************************************************
 * @brief   初始化ICM42605
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/11/22
 ********************************************************************************/
uint8_t I2C_ICM42605_Init(struct ls_i2c_dev *dev, ls_all_mod *mod)
{
    u8 res;
    if (mod->I2C_ICM42605 != 1)
    {
        printk("Gyro_ID not ICM42605 0x%x\n", mod->ID);
        return -1;
    }

    i2c_write_reg(dev, device_config_reg, bit_soft_reset_chip_config);
    Delay_Ms(10);

    i2c_write_reg(dev, reg_bank_sel, 0x01);
    i2c_write_reg(dev, intf_config4, 0x03);
 
    i2c_write_reg(dev, reg_bank_sel, 0x00);
    i2c_write_reg(dev, fifo_config_reg, 0x40);

    res = i2c_read_reg_byte(dev, int_source0_reg);
    i2c_write_reg(dev, int_source0_reg, 0x00);
    i2c_write_reg(dev, fifo_config2_reg, 0x00);
    i2c_write_reg(dev, fifo_config3_reg, 0x02);

    i2c_write_reg(dev, int_source0_reg, (unsigned char)res);
    i2c_write_reg(dev, fifo_config1_reg, 0x63);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    i2c_write_reg(dev, int_config_reg, 0x36);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = (i2c_read_reg_byte(dev, int_source0_reg) | bit_int_fifo_ths_int1_en);
    i2c_write_reg(dev, int_source0_reg, (unsigned char)res);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = ((i2c_read_reg_byte(dev, accel_config0_reg) & 0x1F) | (bit_accel_ui_fs_sel_8g));
    i2c_write_reg(dev, accel_config0_reg, (unsigned char)res);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = ((i2c_read_reg_byte(dev, accel_config0_reg) & 0xF0) | bit_accel_odr_50hz);
    i2c_write_reg(dev, accel_config0_reg, (unsigned char)res);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = ((i2c_read_reg_byte(dev, gyro_config0_reg) & 0x1F) | (bit_gyro_ui_fs_sel_1000dps));
    i2c_write_reg(dev, gyro_config0_reg, (unsigned char)res);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = ((i2c_read_reg_byte(dev, gyro_config0_reg) & 0xF0) | bit_gyro_odr_50hz);
    i2c_write_reg(dev, gyro_config0_reg, (unsigned char)res);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = i2c_read_reg_byte(dev, pwr_mgmt0_reg) | (bit_accel_mode_ln);
    i2c_write_reg(dev, pwr_mgmt0_reg, (unsigned char)res);
    Delay_Ms(1);

    i2c_write_reg(dev, reg_bank_sel, 0x00);
    res = i2c_read_reg_byte(dev, pwr_mgmt0_reg) | (bit_gyro_mode_ln);
    i2c_write_reg(dev, pwr_mgmt0_reg, (unsigned char)res);
    Delay_Ms(1);

    printk("ICM42605 Init success\n");
    return 0;
}

/********************************************************************************
 * @brief   获取温度数据
 * @param   dev : 自定义 I2C 相关结构体
 * @return  返回温度值（摄氏度）
 * @note    注意返回的温度值是乘以100的，如果要得到摄氏度，需要除以100
 * @date    2025/11/22
 ********************************************************************************/
int16_t I2C_ICM42605_Get_Temperature(struct ls_i2c_dev *dev)
{
    uint8_t buf[2];
    int16_t raw_temp;
    int32_t temp_calc;
    // 手册公式：温度=(raw_temp/132.48) + 25
    const int32_t SCALE = 10000;
    const int32_t DIVISOR = 13248;
    const int32_t OFFSET = 25 * SCALE;

    i2c_read_regs(dev, ICM42605_TEMP_OUTH_REG, buf, 2);

    raw_temp = (((int16_t)buf[0]) << 8) | buf[1];
    temp_calc = (raw_temp * SCALE) / DIVISOR;
    temp_calc += OFFSET;

    return (int16_t)(temp_calc / 100);
}

/********************************************************************************
 * @brief   获取陀螺仪数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   gx : 陀螺仪X轴数据
 * @param   gy : 陀螺仪Y轴数据
 * @param   gz : 陀螺仪Z轴数据
 * @return  返回获取字节数
 * @date    2025/11/24
 ********************************************************************************/
uint8_t I2C_ICM42605_Get_Gyroscope(struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, ICM42605_GYRO_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *gx = ((int16_t)buf[0] << 8) | buf[1];
        *gy = ((int16_t)buf[2] << 8) | buf[3];
        *gz = ((int16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/********************************************************************************
 * @brief   获取加速度数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   ax : 加速度X轴数据
 * @param   ay : 加速度Y轴数据
 * @param   az : 加速度Z轴数据
 * @return  返回获取字节数
 * @date    2025/11/24
 ********************************************************************************/
uint8_t I2C_ICM42605_Get_Accelerometer(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6], res;
    res = i2c_read_regs(dev, ICM42605_ACCEL_XOUTH_REG, buf, 6);
    if (res == 0)
    {
        *ax = ((int16_t)buf[0] << 8) | buf[1];
        *ay = ((int16_t)buf[2] << 8) | buf[3];
        *az = ((int16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/********************************************************************************
 * @brief   获取原始数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   ax : 加速度X轴数据
 * @param   ay : 加速度Y轴数据
 * @param   az : 加速度Z轴数据
 * @param   gx : 陀螺仪X轴数据
 * @param   gy : 陀螺仪Y轴数据
 * @param   gz : 陀螺仪Z轴数据
 ********************************************************************************/
uint8_t I2C_ICM42605_Get_Raw_data(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[12], res;
    res = i2c_read_regs(dev, ICM42605_ACCEL_XOUTH_REG, buf, 12);
    if (res == 0)
    {
        *ax = ((int16_t)buf[0] << 8) | buf[1];
        *ay = ((int16_t)buf[2] << 8) | buf[3];
        *az = ((int16_t)buf[4] << 8) | buf[5];
        *gx = ((int16_t)buf[6] << 8) | buf[7];
        *gy = ((int16_t)buf[8] << 8) | buf[9];
        *gz = ((int16_t)buf[10] << 8) | buf[11];
    }
    return res;
}
