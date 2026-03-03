#ifndef __LQ_IIC_ALL_DRV_H__
#define __LQ_IIC_ALL_DRV_H__

#include "LQ_drv_common.h"
#include "LQ_I2C_RDWR_drv.h"
#include "LQ_util_drv.h"
#include "LQ_List_drv.h"

/************************************* 各模块初始化函数 *************************************/

int  device_init(struct ls_i2c_dev *dev, LQ_List *list);        /* 初始化设备 */
void cycle_detection_timer_callback(struct timer_list *timer);  /* 循环检测设备定时器回调函数 */

/************************************ 上层接口的函数声明 ************************************/

int     i2c_open    (struct inode *inode, struct file *f);                              /* 上层 open 函数相关实现 */
int     i2c_release (struct inode *inode, struct file *f);                              /* 上层 close 函数相关实现 */
ssize_t i2c_read    (struct file *f, char __user *buf, size_t cnt, loff_t *off);        /* 上层 read 函数相关实现 */
ssize_t i2c_write   (struct file *f, const char __user *buf, size_t cnt, loff_t *off);  /* 上层 write 函数相关实现 */
long    i2c_ioctl   (struct file *f, unsigned int cmd, unsigned long arg);              /* 上层 ioctl 函数相关实现 */

/********************************** 设备驱动的相关函数声明 **********************************/

int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);   /* 设备驱动的探测函数 */
int i2c_remove(struct i2c_client *c);                                       /* 设备驱动的移除函数 */

#endif
