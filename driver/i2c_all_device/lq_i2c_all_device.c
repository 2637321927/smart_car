#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/errno.h>

// 新增模块参数，可加载时传参修改总线号
static int bus_num = 5;
module_param(bus_num, int, 0644);
MODULE_PARM_DESC(bus_num, "Select I2C bus, 4 or 5");

// 定义设备信息结构体
struct i2c_dev_info {
    const char    *name;    // 设备名称
    unsigned short addr;    // 设备地址
};

// 定义设备列表 - 每个设备对应一个地址
static const struct i2c_dev_info I2C_DEVS[] = {
    {"ls,lq_i2c_mpu6050" , 0x68},   // MPU6050
    {"ls,lq_i2c_lsm6dsr" , 0x6b},   // LSM6DSR
    {"ls,lq_i2c_vl53l0x" , 0x29},   // VL53L0X
    {"ls,lq_i2c_icm42688", 0x69},   // ICM42688
    { NULL, 0 } // 结束标志
};

// 保存多个注册成功的I2C client
static struct i2c_client *clients[8] = {NULL};
static int client_count = 0;

static int __init dev_init(void)
{
    struct i2c_adapter *adapter = NULL;
    struct i2c_board_info i2c_board_info;
    int i = 0;
    // 使用传入的bus_num参数获取适配器
    adapter = i2c_get_adapter(bus_num);
    if (!adapter)
    {
        printk("LQ_I2C_ALL_DEV: get i2c adapter %d failed!\n", bus_num);
        return -ENODEV;
    }
    printk("LQ_I2C_ALL_DEV init bus %d, start probe multi devices\n", bus_num);
    // 遍历所有设备名称
    while (I2C_DEVS[i].name != NULL && client_count < ARRAY_SIZE(clients)) {
        unsigned short addr_list[2];    // 单个设备的地址列表
        memset(&i2c_board_info, 0, sizeof(struct i2c_board_info));
        strlcpy(i2c_board_info.type, I2C_DEVS[i].name, I2C_NAME_SIZE);
        addr_list[0] = I2C_DEVS[i].addr;
        addr_list[1] = I2C_CLIENT_END;
        clients[client_count] = i2c_new_probed_device(
            adapter,
            &i2c_board_info,
            addr_list,
            NULL
        );
        if (clients[client_count]) {
            printk(KERN_INFO "LQ_I2C_ALL_DEV: register device %s success, addr=0x%02x!\n", I2C_DEVS[i].name, clients[client_count]->addr);
            client_count++;
        } else {
            printk(KERN_WARNING "LQ_I2C_ALL_DEV: probe device %s failed!\n", I2C_DEVS[i].name);
        }
        i++;
    }
    i2c_put_adapter(adapter);

    // ========== 修改点：无论探测到几个设备，模块都正常加载 ==========
    printk("LQ_I2C_ALL_DEV: scan finish, detected %d valid i2c devices on bus %d\n", client_count, bus_num);
    return 0;
}

static void __exit dev_exit(void)
{
    int i;
    for (i = 0; i < client_count; i++) {
        printk(KERN_INFO "LQ_I2C_ALL_DEV: unregister device %s, addr=0x%02x!\n", clients[i]->name, clients[i]->addr);
        i2c_unregister_device(clients[i]);
        clients[i] = NULL;
    }
    client_count = 0;
    printk("LQ_I2C_ALL_DEV exit function, all devices unregistered\n");
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_AUTHOR("LQ_012 <chiusir@163.com>");
MODULE_DESCRIPTION("支持多驱动匹配的I2C设备端模块");
MODULE_VERSION("2.1");
MODULE_LICENSE("GPL");