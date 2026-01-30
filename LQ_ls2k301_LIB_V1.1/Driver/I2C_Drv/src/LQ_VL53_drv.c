#include "LQ_I2C_ALL_DRV.h"
#include "LQ_VL53_drv.h"
#include "LQ_I2C_RDWR_drv.h"

const unsigned char VL53_STAR = 0x02; // 0x02 连续测量模式    0x01 单次测量模式

/********************************************************************************
 * @brief   VL53L0X写一个寄存器
 * @param   dev: 自定义设备相关结构体
 * @param   reg: 寄存器地址
 * @param   val: 寄存器值
 * @retval  0: 成功; 其他：失败
 * @date    2025/11/17
 ********************************************************************************/
int vl53l0x_write_reg(struct ls_i2c_dev *dev, uint8_t reg, uint8_t val)
{
    int ret;
    ret = i2c_write_reg(dev, reg, val);
    if (ret < 0)
    {
        printk(KERN_INFO "VL53L0X write reg 0x%02x failed! ret=%d\n", reg, ret);
        return ret;
    }
    return 0;
}

/********************************************************************************
 * @brief   VL53L0X读一个寄存器
 * @param   dev: 自定义设备相关结构体
 * @param   reg: 寄存器地址
 * @param   len: 寄存器长度
 * @param   val: 寄存器值
 * @retval  0: 成功; 其他：失败
 * @date    2025/11/17
 ********************************************************************************/
int vl53l0x_read_reg(struct ls_i2c_dev *dev, uint8_t reg, uint8_t len, uint8_t *val)
{
    int ret;
    // 合法性检查
    if (!val || len == 0)
    {
        printk(KERN_INFO "VL53L0x read invalid param! val=NULL or len=0\n");
        return -EINVAL;
    }
    ret = i2c_read_regs(dev, reg, val, len);
    if (ret < 0)
    {
        printk(KERN_INFO "VL53L0X read reg 0x%02x failed! ret=%d\n", reg, ret);
        return ret;
    }
    return 0;
}

/********************************************************************************
 * @brief   VL53L0X初始化
 * @param   dev: 自定义设备相关结构体
 * @retval  0: 成功; 其他：失败
 * @date    2025/11/17
 ********************************************************************************/
int vl53l0x_init(struct ls_i2c_dev *dev)
{
    int ret;
    ret = vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, VL53_STAR);
    if (ret != 0)
    {
        printk(KERN_INFO "VL53L0X init failed! ret=%d\n", ret);
        return ret;
    }
    msleep(50);
    printk(KERN_INFO "VL53L0X Init success (mode: %s)\n", 
           VL53_STAR == 0x01 ? "single measurement" : "continuous measurement");
    return 0;
}

/********************************************************************************
 * @brief   VL53L0X读取距离
 * @param   dev: 自定义设备相关结构体
 * @param   distance: 距离值
 * @retval  0: 成功; 其他：失败
 * @date    2025/11/17
 ********************************************************************************/
int vl53l0x_read_distance(struct ls_i2c_dev *dev, uint16_t *distance)
{
    int ret;
    uint8_t dis_buf[2] = {0};
    uint16_t dis = 0;
    static uint16_t last_dis = 0;   // 静态变量保存上一次有效距离(过滤抖动)

    // 检查合法性
    if (!distance)
    {
        printk(KERN_INFO "VL53L0X read distance invalid param! distance=NULL\n");
        return -EINVAL;
    }
    // 读取2字节距离数据
    ret = vl53l0x_read_reg(dev, VL53_REG_DIS, 2, dis_buf);
    if (ret != 0)
    {
        printk(KERN_INFO "VL53L0X read distance failed! ret=%d\n", ret);
        return ret;
    }
    // 数据转换
    dis = (dis_buf[0] << 8) | dis_buf[1];
    // 异常值过滤
    if (dis > MAX_DISTABCE)
    {
        dis = 0;
    } 
    if (dis == INVALID_DISTANCE)
    {
        dis = last_dis;
    }
    last_dis = dis;
    *distance = dis;
    return 0;
}
