#include "LQ_util_drv.h"
#include "LQ_I2C_RDWR_drv.h"

/********************************************************************************
 * @brief   扫描所有可能的 I2C 设备地址
 * @param   dev : 自定义 I2C 相关结构体
 * @return  成功返回 0，失败返回错误码
 * @date    2025/11/17
 ********************************************************************************/
int scan_all_i2c_dev_addrs(struct ls_i2c_dev *dev, LQ_List *list)
{
    int i;
    int ret, count;
    uint8_t orig_addr;  // 保留原来的地址
    LQ_Node *node;
    struct i2c_client *cli = (struct i2c_client *)dev->client;
    
    // 合法性检查
    if (!dev || !cli || !cli->adapter) {
        printk(KERN_ERR "ls_i2c_dev or i2c_client is NULL!\n");
        return -EINVAL;
    }
    if (list == NULL) {
        printk(KERN_ERR "LQ_List is NULL\n");
        return -EINVAL;
    }
    // 保存原来的地址
    orig_addr = cli->addr;
    count = lq_list_count(list);

    // 遍历所有可能的 I2C 设备地址
    for (i = 0; i < (sizeof(All_Addr)/sizeof(unsigned short)-1); i++)
    {
        if (count >= MAX_I2C_DEVICES)
        {
            printk(KERN_WARNING "I2C device count exceeds MAX(%d)\n", MAX_I2C_DEVICES);
            break;
        }
        // 临时将设备地址设置为当前遍历的 addr
        cli->addr = (uint8_t)All_Addr[i];
        // 检测当前地址是否为合法的 I2C 设备
        if (cli->addr < I2C_MIN_VALID_ADDR || cli->addr > I2C_MAX_VALID_ADDR)
        {
            continue;
        }
        
        // 发送探测请求，看是否会回应ACK
        ret = i2c_write_reg(dev, 0x00, 0x00);
        // 判断是否探测成功
        if (ret == 1)
        {
            lq_list_add(list, (uint8_t)cli->addr);
        }
    }
    // 打印所有探测到的设备地址
    node = list->head;
    count = lq_list_count(list);
    if (count > 0)
    {
        for (i = 0; i < count; i++)
        {
            // printk(KERN_INFO "Detected I2C device: addr=0x%02x\n", node->data);
            node = node->next;
        }
        // printk(KERN_INFO "Total I2C devices detected: %d\n", count);
    }
    else
    {
        printk(KERN_WARNING "No I2C devices detected.\n");
    }

    // 恢复原来的地址
    cli->addr = orig_addr;

    return 0;

}

/********************************************************************************
 * @brief   切换目标 I2C 设备地址
 * @param   dev : 自定义 I2C 相关结构体
 * @param   target_addr : 目标设备地址
 * @return  成功返回 0，失败返回错误码
 * @date    2025/11/17
 ********************************************************************************/
int switch_i2c_target_addr(struct ls_i2c_dev *dev, uint8_t target_addr, LQ_List *list)
{
    int i, count, is_dev_exist = 0;
    LQ_Node *curr_node;
    // 合法性检查
    if (target_addr < 0x08 || target_addr > 0x77)
    {
        printk(KERN_ERR "Invalid I2C device address: %d\n", target_addr);
        return -EINVAL;
    }
    count = lq_list_count(list);
    curr_node = list->head;
    // 检查目标地址是否在扫描到的设备列表中
    for (i = 0; i < count; i++)
    {
        if (curr_node->data == target_addr)
        {
            is_dev_exist = 1;
            break;
        }
        curr_node = curr_node->next;
    }
    if (!is_dev_exist)
    {
        printk(KERN_ERR "switch_i2c _target_addr: Target device 0x%02x not found on I2C bus!\n", target_addr);
        return -EINVAL;
    }
    // 切换目标地址
    dev->client->addr = target_addr;
    return 0;
}

/********************************************************************************
 * @brief   检查当前连接设备
 * @param   dev : 自定义 I2C 相关结构体
 * @param   list : 存储所有探测到的设备地址的链表
 * @return  成功返回 0，失败返回错误码
 * @date    2025/11/17
 ********************************************************************************/
int device_inspection(struct ls_i2c_dev *dev, LQ_List *list, DeviceInitFunc deviceInit)
{
    int ret;
    LQ_List new_list, *result_list;
    // 参数检查
    if (unlikely(dev == NULL || list == NULL))
    {
        printk(KERN_ERR "device_inspection: dev or list is NULL\n");
        return -EINVAL;
    }
    lq_list_init(&new_list);
    // 重新扫描所有设备
    ret = scan_all_i2c_dev_addrs(dev, &new_list);
    if (ret != 0)
    {
        printk(KERN_ERR "Failed to scan I2C devices\n");
        lq_list_clear(&new_list);
        return ret;
    }
    // 比较新旧链表是否相同
    if (lq_list_compare(list, &new_list) == 1)
    {
        lq_list_clear(&new_list);
        return 0;
    }
    // 不同则查看是否有新增的设备
    result_list = lq_list_difference(list, &new_list);
    // 检查是否正常分配内存
    if (result_list != NULL)
    {
        // 检测是否有新增的设备
        if (result_list->head != NULL)
        {
            deviceInit(dev, result_list);
        }
        lq_list_clear(result_list);
        kfree(result_list);
    }
    // 更新链表，并清除旧链表
    lq_list_swap(list, &new_list);
    lq_list_clear(&new_list);
    return 0;
}

/********************************************************************************
 * @brief   读取陀螺仪的设备ID
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  陀螺仪的设备ID
 * @date    2025/3/20
 ********************************************************************************/
uint8_t Obt_Gyro_Dev_ID(struct ls_i2c_dev *dev, ls_all_mod *mod)
{
    mod->ID = i2c_read_reg_byte(dev, WHO_AM_I); //获取陀螺仪设备 ID
    switch (mod->ID)
    {
        // case 0x12:IIC_ICM20602 = 1;break;
        // case 0x71:IIC_MPU9250  = 1;break;
        // case 0x98:IIC_ICM20689 = 1;break;
        case 0x42:mod->I2C_ICM42605 = 1;break;
        case 0x68:mod->I2C_MPU6050  = 1;break;
        default:  mod->I2C_errorid  = 1;
    }
    return mod->ID;
}

/********************************************************************************
 * @brief   内核毫秒级延时函数
 * @param   ms : 毫秒值
 * @return  无
 * @date    2025/3/20
 ********************************************************************************/
void Delay_Ms(uint16_t ms)
{
    mdelay(ms);
}
