#ifndef __LQ_UTIL_H__
#define __LQ_UTIL_H__

#include "LQ_drv_common.h"
#include "LQ_List_drv.h"
#include "LQ_MPU6050_drv.h"
#include "LQ_ICM42605_drv.h"
#include "LQ_VL53_drv.h"

/* 定义一个函数指针类型，指向一个函数，返回值是int，参数是struct ls_i2c_dev *dev, LQ_List *list类型的 */
typedef int (*DeviceInitFunc)(struct ls_i2c_dev *dev, LQ_List *list);

int     scan_all_i2c_dev_addrs  (struct ls_i2c_dev *dev, LQ_List *list);                            /* 扫描所有可能的 I2C 设备地址 */
int     switch_i2c_target_addr  (struct ls_i2c_dev *dev, uint8_t target_addr, LQ_List *list);       /* 切换目标 I2C 设备地址 */
int     device_inspection       (struct ls_i2c_dev *dev, LQ_List *list, DeviceInitFunc deviceInit); /* 检查当前连接设备 */
uint8_t Obt_Gyro_Dev_ID         (struct ls_i2c_dev *dev, ls_all_mod *mod);                          /* 读取陀螺仪的设备ID */

void    Delay_Ms                (uint16_t ms);                                                      /* 内核毫秒级延时函数 */

#endif
