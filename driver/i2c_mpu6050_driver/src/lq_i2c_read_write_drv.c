#include "lq_i2c_read_write_drv.h"
#include <linux/ktime.h>

#define MPU6050_I2C_WARN_US          (10000LL)
#define MPU6050_I2C_LOG_INTERVAL_NS  (1000000000LL)

static void mpu6050_log_slow_i2c_transfer(const char *op, u8 reg, int len,
                                          int ret, s64 start_ns, s64 end_ns)
{
    static s64 last_log_ns;
    s64 used_us = (end_ns - start_ns) / 1000;

    if (used_us < MPU6050_I2C_WARN_US) {
        return;
    }
    if (last_log_ns != 0 && end_ns - last_log_ns < MPU6050_I2C_LOG_INTERVAL_NS) {
        return;
    }

    last_log_ns = end_ns;
    printk(KERN_WARNING "%s: slow i2c_transfer op=%s reg=0x%02x len=%d ret=%d used=%lld us\n",
           DEVICE_NAME, op, reg, len, ret, (long long)used_us);
}

/********************************************************************************
 * @brief   从寄存器读取数据
 * @param   dev : 自定义 I2C 相关结构体 
 * @param   reg : 寄存器地址
 * @param   val : 读取数据缓冲区
 * @param   len : 缓冲区长度
 * @return  成功返回 0，失败返回错误码
 * @date    2025/3/20
 ********************************************************************************/
int i2c_read_regs(struct ls_i2c_dev *dev, u8 reg, void *val, int len)
{
    int ret;
    // 用于描述 I2C 消息的基本数据结构
    struct i2c_msg msg[2];
    // 表示一个连接到 I2C 总线上的客户端设备
    struct i2c_client *cli = (struct i2c_client*)dev->client;
    s64 start_ns;
    s64 end_ns;
    // 配置第一个消息(写消息)
    msg[0].addr = cli->addr;// cli 指向的 I2C 设备的地址
    msg[0].flags = 0;       // 消息标志位，表示一个写操作
    msg[0].buf = &reg;      // 要读取的寄存器地址
    msg[0].len = 1;         // 消息的数据长度 1 字节，因为只需要发送一个寄存器地址
    // 配置第二个消息(读消息)
    msg[1].addr = cli->addr;// cli 指向的 I2C 设备的地址
    msg[1].flags = I2C_M_RD;// 消息标志位，表示一个读操作
    msg[1].buf = val;       // 消息缓冲区指向 val
    msg[1].len = len;       // 消息长度为 len，即要从寄存器读取的字节数
    // 在 I2C 总线上进行数据传输
    start_ns = ktime_get_ns();
    ret = i2c_transfer(cli->adapter, msg, 2);
    end_ns = ktime_get_ns();
    mpu6050_log_slow_i2c_transfer("read", reg, len, ret, start_ns, end_ns);
    if (ret == 2) {
        ret = 0;
    } else {
        printk("i2c read error. ret = %d, reg = %06x, len = %d\r\n", ret, reg, len);
        // Preserve kernel errors such as -ETIMEDOUT(-110). User space needs
        // to know the read really failed instead of treating zeroed data as new.
        if (ret >= 0) {
            ret = -EREMOTEIO;
        }
    }
    return ret;
}

/********************************************************************************
 * @brief   使用 I2C 向寄存器读取一个字节
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @return  返回获取到的数据
 * @date    2025/3/20
 ********************************************************************************/
u8 i2c_read_reg_byte(struct ls_i2c_dev *dev, u8 reg)
{
    // 定义一个变量存储读取的数据
    u8 data = 0;
    // 调用上面的读取函数，将自定义结构体、要读取的寄存器地址、存储缓冲区、缓冲区长度传入函数中
    i2c_read_regs(dev, reg, &data, 1);
    // 返回数据
    return data;
}

/********************************************************************************
 * @brief   向寄存器写数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入数据缓冲区
 * @param   len : 缓冲区长度
 * @return  返回 i2c_transfer 函数返回值
 * @date    2025/3/20
 ********************************************************************************/
s32 i2c_write_regs(struct ls_i2c_dev *dev, u8 reg, u8 *buf, u8 len)
{
    // 存储要发送的完整数据，包括寄存器地址和要写入的数据
    u8 b[256];
    // 用于描述 I2C 消息的基本数据结构
    struct i2c_msg msg;
    // 表示一个连接到 I2C 总线上的客户设备
    struct i2c_client *cli = (struct i2c_client*)dev->client;

    b[0] = reg;             // 将寄存器地址 reg 存储在数组 b 的第一个元素中
    memcpy(&b[1], buf, len);// 将 buf 中长度为 len 的数据存储到 b 数组中，从第二个元素开始存储

    msg.addr = cli->addr;   // cli 指向的 I2C 设备的地址
    msg.flags = 0;          // 消息标志位设备为 0，表示一个写操作

    msg.buf = b;            // 将消息的数据缓冲区指针指向数组 b，即要发送的完整数据
    msg.len = len + 1;      // 设置消息的数据长度为 len + 1，因为除了要写入的数据len字节，好包含一个字节的寄存器地址

    // 执行 I2C 数据传输的函数
    return i2c_transfer(cli->adapter, &msg, 1);
}

/********************************************************************************
 * @brief   向寄存器写入一个字节数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   reg : 寄存器地址
 * @param   buf : 写入的数据
 * @return  返回 i2c_transfer 函数返回值
 * @date    2025/3/20
 ********************************************************************************/
s32 i2c_write_reg(struct ls_i2c_dev *dev, u8 reg, u8 buf)
{
    return i2c_write_regs(dev, reg, &buf, 1);
}
