#ifndef __LQ_DRV_COMMON_H__
#define __LQ_DRV_COMMON_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* 设备名称, 请勿修改 */
#define DEVICE_ID_NAME          ( "loongson,IIC_Eqm" )

/* 定义循环检测设备的时间间隔 (毫秒), 感觉间隔时间不合适, 可适当修改 */
#define DETECT_INTERVAL_MS      ( 500 ) // 0.5秒

/* I2C设备最大数量, 请勿修改 */
#define MAX_I2C_DEVICES         ( 10 )

/* I2C最大和最小合法地址, 请勿修改 */
#define I2C_MIN_VALID_ADDR      ( 0x08 )
#define I2C_MAX_VALID_ADDR      ( 0x77 )

/* 相关陀螺仪的ID, 请勿修改 */
#define MPU6050_ID              ( 0x68 )
#define ICM42605_ID             ( 0x42 )

/* 各模块的地址, 请勿修改 */
#define MPU6050_SLAVE_ADDR      ( 0x68 )    // MPU6050 设备地址
#define ICM42605_SLAVE_ADDR     ( 0x69 )    // ICM42605 设备地址
#define VL53L0X_SLAVE_ADDR      ( 0x29 )    // VL53L0X 设备地址

/* MPU6050 相关幻数号, 请勿修改 */
#define I2C_MPU6050_MAGIC       ( 'm' )                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_MPU6050_ID      ( _IO(I2C_MPU6050_MAGIC, 1) )   // 获取 MPU6050 ID
#define I2C_GET_MPU6050_TEM     ( _IO(I2C_MPU6050_MAGIC, 2) )   // 获取 MPU6050 温度
#define I2C_GET_MPU6050_ANG     ( _IO(I2C_MPU6050_MAGIC, 3) )   // 获取 MPU6050 角度值
#define I2C_GET_MPU6050_ACC     ( _IO(I2C_MPU6050_MAGIC, 4) )   // 获取 MPU6050 加速度
#define I2C_GET_MPU6050_GYRO    ( _IO(I2C_MPU6050_MAGIC, 5) )   // 获取 MPU6050 角度和加速度值

/* ICM42605 相关幻数号, 请勿修改 */
#define I2C_ICM42605_MAGIC      ( 'i' )                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_ICM42605_ID     ( _IO(I2C_ICM42605_MAGIC, 1) )  // 获取 MPU6050 ID
#define I2C_GET_ICM42605_TEM    ( _IO(I2C_ICM42605_MAGIC, 2) )  // 获取 MPU6050 温度
#define I2C_GET_ICM42605_ANG    ( _IO(I2C_ICM42605_MAGIC, 3) )  // 获取 MPU6050 角度值
#define I2C_GET_ICM42605_ACC    ( _IO(I2C_ICM42605_MAGIC, 4) )  // 获取 MPU6050 加速度
#define I2C_GET_ICM42605_GYRO   ( _IO(I2C_ICM42605_MAGIC, 5) )  // 获取 MPU6050 角度和加速度值

/* VL53L0X 相关幻数号, 请勿修改 */
#define I2C_VL53L0X_MAGIC       ( 'v' )                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_VL53L0X_DIS     ( _IO(I2C_VL53L0X_MAGIC, 1) )   // 获取 VL53L0X 距离值

/**************************** 存储当前所支持的所有I2C设备地址列表 ****************************/
/*************************** 可选择使用的设备, 不使用直接注释即可 ****************************/
static const unsigned short All_Addr[] = {
    MPU6050_SLAVE_ADDR,     // 陀螺仪MPU6050传感器地址
    // ICM42605_SLAVE_ADDR,    // 陀螺仪ICM42605传感器地址
    // VL53L0X_SLAVE_ADDR,     // 距离传感器VL53L0X地址
    I2C_CLIENT_END
};

/* 自定义结构体, 存储了一些 iic 设备所需要的成员变量, 请勿修改 */
struct ls_i2c_dev {
    struct i2c_client  *client;     // 表示连接到 iic 总线上的客户端设备，包含了设备的各种信息和操作方式
    struct cdev         cdev;       // 表示字符设备的核心结构体，包含字符设备的各种属性和操作方法
    struct class       *class;      // 表示设备类的核心结构体
    struct device      *device;     // 表示系统中的一个具体设备
    struct i2c_adapter *adapter;    // 表示连接到 iic 总线上的适配器，包含了适配器的各种信息和操作方式
    dev_t               dev_id;     // 表示该设备的设备号-
};

/* 自定义结构体, 存储了可支持的陀螺仪型号, 请勿修改 */
typedef struct {
    uint8_t ID;             // 存储设备地址
    uint8_t I2C_MPU6050;    // MPU6050，默认为 0，成功匹配则为 1
    uint8_t I2C_ICM42605;   // ICM42605，默认为 0，成功匹配则为 1
    uint8_t I2C_VL53;       // VL53，默认为 0，成功匹配则为 1
    uint8_t I2C_errorid;    // 不支持的设备，默认为 0，若检测到的地址不支持则为 1
} ls_all_mod;

/* 自定义结构体, 存储定时器相关信息, 请勿修改 */
struct ls_cycle_data
{
    struct work_struct work;            // 定时器工作队列结构体
    struct timer_list cycle_detection;  // 定时器结构体
    struct ls_i2c_dev *Dev;             // 设备结构体指针
    struct workqueue_struct *wq;       // 工作队列结构体指针
};


#endif