#include "LQ_I2C_ALL_DRV.h"

#define DEVICE_CNT      ( 1 )           // 设备数量
#define DEVICE_NAME     ( "ls_iic" )    // 设备文件名称

/*************************************** 全局变量定义 **************************************/

struct i2c_client *main_client;     // 创建一个全局的 iic 客户端设备结构体
static ls_all_mod mod = {0};        // 各模块结构体
LQ_List i2c_dev_addrs_list;         // I2C 设备地址存储链表
struct ls_cycle_data all_timer;     // 定义循环检测设备所需定时器结构

/***************************** 文件操作指针集 -- 提供给上层接口 *****************************/
static const struct file_operations i2c_ops = {
    .owner = THIS_MODULE,       // 该文件操作集合属于当前的内核模块
    .open = i2c_open,           // 指向一个处理文件打开操作的函数
    .read = i2c_read,           // 指向一个处理文件读取操作的函数
    .write = i2c_write,         // 指向一个处理文件写入操作的函数
    .unlocked_ioctl = i2c_ioctl,// 指向一个处理文件自定义操作的函数
    .release = i2c_release,     // 指向一个处理文件关闭操作的函数
};

/********************************** 设备与驱动的匹配结构体 **********************************/
static const struct i2c_device_id i2c_dev_id[] = {
    { DEVICE_ID_NAME, 0 },
    {  }
};

/********************************* I2C设备驱动的核心结构体 **********************************/
static struct i2c_driver i2c_drv = {
    .driver = {
        .name = "i2c_gyro_drv", // 驱动的名称
        .owner = THIS_MODULE,   // 代表当前内核模块
    },
    .probe = i2c_probe,         // 探测函数
    .remove = i2c_remove,       // 移除函数
    .id_table = i2c_dev_id      // 数组，包含了iic设备的ID信息
};

/************************************* 各模块初始化函数 *************************************/

/********************************************************************************
 * @brief   初始化设备
 * @param   dev : 自定义 I2C 相关结构体
 * @return  成功返回 0，失败返回错误码
 * @date    2025/11/21
 ********************************************************************************/
int device_init(struct ls_i2c_dev *dev, LQ_List *list)
{
    LQ_Node *node;
    // 参数校验
    if (dev == NULL || list == NULL)
    {
        printk(KERN_ERR "device_init: dev or list is NULL\n");
        return -1;
    }
    node = list->head;
    if (node != NULL)
    {
        // 初始化MPU6050设备
        if ((lq_list_contains(list, MPU6050_SLAVE_ADDR)) && (switch_i2c_target_addr(dev, MPU6050_SLAVE_ADDR, list) == 0))
        {
            // 获取陀螺仪 ID
            if (Obt_Gyro_Dev_ID(dev, &mod) == MPU6050_ID)
            {
                if (I2C_MPU6050_Init(dev, &mod) != 0)
                    return -1;
                printk("MPU6050 Init success, addr=0x%x\n", mod.ID);
            }
        }

        // 初始化ICM42605设备
        if ((lq_list_contains(list, ICM42605_SLAVE_ADDR)) && (switch_i2c_target_addr(dev, ICM42605_SLAVE_ADDR, list) == 0))
        {
            if (Obt_Gyro_Dev_ID(dev, &mod) == ICM42605_ID)
            {
                // dev->client->addr = 0x68;
                if (I2C_ICM42605_Init(dev, &mod) != 0)
                    return -1;
                printk("ICM42605 Init success, addr=0x%x\n", mod.ID);
            }
        }

        // 初始化VL53L0X设备
        if ((lq_list_contains(list, VL53L0X_SLAVE_ADDR)) && (switch_i2c_target_addr(dev, VL53L0X_SLAVE_ADDR, list) == 0))
        {
            // 初始化VL53L0X
            if (vl53l0x_init(dev) != 0)
                return -1;
            printk("VL53L0X Init success, addr=0x%x\n", VL53L0X_SLAVE_ADDR);
        }
        /************************ 新增初始化部分按上面格式写在下面 ************************/
    }
    return 0;
}

/********************************************************************************
 * @brief   循环检测设备定时器回调函数
 * @param   timer 定时器结构
 * @return  void
 * @note    该函数将在定时器超时后被调用，用于执行循环检测设备的逻辑。
 * @date    2025/11/21
 ********************************************************************************/
void cycle_detection_timer_callback(struct timer_list *timer)
{
    // 从timer指针反响解析出整个自定义结构体
    struct ls_cycle_data *data = container_of(timer, struct ls_cycle_data, cycle_detection);
    // 提交工作队列任务
    schedule_work(&data->work);
}

/********************************************************************************
 * @brief   循环检测设备工作函数
 * @param   work : 工作结构体指针
 * @return  void
 * @note    该函数将执行循环检测设备的逻辑，并重新设置定时器以实现周期性检测。
 * @date    2025/11/21
 ********************************************************************************/
void cycle_work_handler(struct work_struct *work)
{
    // 从work指针反响解析出整个自定义结构体
    struct ls_cycle_data *data = container_of(work, struct ls_cycle_data, work);

    // 循环检测设备逻辑代码
    device_inspection(data->Dev, &i2c_dev_addrs_list, device_init);

    // 重置定时器，实现下一次循环
    mod_timer(&all_timer.cycle_detection, jiffies + msecs_to_jiffies(DETECT_INTERVAL_MS));
}

/************************************ 上层接口的函数实现 ************************************/

/********************************************************************************
 * @brief   上层 open 函数相关实现
 * @param   inode: 指向 inode 结构体的指针，包括文件系统中文件或目录的元数据结构
 * @param   f    : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @return  成功返回 0，失败返回错误码
 * @date    2025/3/20
 ********************************************************************************/
int i2c_open(struct inode *inode, struct file *f)
{
    // 从 file 结构体中获取对应的字符设备结构体 cdev，cdev 结构体代表了字符设备在内核中的抽象表现
    struct cdev *cdev = f->f_path.dentry->d_inode->i_cdev;
    // 使用 container_of 宏根据 cdev 结构体的地址找到自定义结构体的地址
    struct ls_i2c_dev *dev = container_of(cdev, struct ls_i2c_dev, cdev);

    // 初始化设备
    if (device_init(dev, &i2c_dev_addrs_list) != 0)
        return -1;
    
    return 0;
}

/********************************************************************************
 * @brief   上层 ioctl 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   cmd : 控制命令码，用于指定要执行的操作类型
 * @param   arg : 与命令码相关的参数，可传递额外的数据
 * @return  0 表示操作成功，负数表示操作失败
 * @date    2025/3/20
 ********************************************************************************/
long i2c_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    int ret;                // 返回值，用于表示操作是否成功
    uint8_t ID = 0;         // 设备ID，用于存放获取到的设备ID值
    int16_t data[6] = {0};  // 存放各种数据的数组
    // 从 file 结构体中获取对应的字符设备结构体 cdev，cdev 结构体代表了字符设备在内核中的抽象表现
    struct cdev *cdev = f->f_path.dentry->d_inode->i_cdev;
    // 使用 container_of 宏根据 cdev 结构体的地址找到自定义结构体的地址
    struct ls_i2c_dev *dev = container_of(cdev, struct ls_i2c_dev, cdev);
    
    // 判断属于哪个设备，修改地址
    switch (_IOC_TYPE(cmd))
    {
        case I2C_MPU6050_MAGIC: // 判断是否为MPU6050设备命令
            ret = switch_i2c_target_addr(dev, MPU6050_SLAVE_ADDR, &i2c_dev_addrs_list);
            if (ret != 0) return ret;
            break;
        case I2C_ICM42605_MAGIC:// 判断是否为ICM42605设备命令
            ret = switch_i2c_target_addr(dev, ICM42605_SLAVE_ADDR, &i2c_dev_addrs_list);
            if (ret != 0) return ret;
            break;
        case I2C_VL53L0X_MAGIC: // 判断是否为VL53L0X设备命令
            ret = switch_i2c_target_addr(dev, VL53L0X_SLAVE_ADDR, &i2c_dev_addrs_list);
            if (ret != 0) return ret;
            break;
        default:
            return -EINVAL;
    }

    // 根据命令码执行相应的操作
    switch (cmd)
    {
        /**************************************** MPU6050 ****************************************/
        case I2C_GET_MPU6050_ID:    // 获取MPU6050的ID
            ID = Obt_Gyro_Dev_ID(dev, &mod);
            if (copy_to_user((uint8_t*)arg, &ID, sizeof(ID)))
                return -EFAULT;
            break;
        case I2C_GET_MPU6050_TEM:  // 获取MPU6050的温度
            data[0] = MPU6050_Get_Temperature(dev);
            if (copy_to_user((int16_t*)arg, &data[0], sizeof(int16_t)))
                return -EFAULT;
            break;
        case I2C_GET_MPU6050_ANG:  // 获取MPU6050的角速度值
            MPU6050_Get_Gyroscope(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 3))
                return -EFAULT;
            break;
        case I2C_GET_MPU6050_ACC:  // 获取MPU6050的加速度值
            MPU6050_Get_Accelerometer(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 3))
                return -EFAULT;
            break;
        case I2C_GET_MPU6050_GYRO:  // 获取MPU6050陀螺仪的角速度值
            MPU6050_Get_Raw_data(dev, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 6))
                return -EFAULT;
            break;
        /**************************************** VL53L0X ****************************************/
        case I2C_GET_VL53L0X_DIS:   // 获取VL53L0X的距离值
            vl53l0x_read_distance(dev, &data[0]);
            if (copy_to_user((int16_t*)arg, &data[0], sizeof(int16_t)))
                return -EFAULT;
            break;
        /*************************************** ICM42605 ****************************************/
        case I2C_GET_ICM42605_ID:   // 获取MPU6050的ID
            ID = Obt_Gyro_Dev_ID(dev, &mod);
            if (copy_to_user((uint8_t*)arg, &ID, sizeof(ID)))
                return -EFAULT;
            break;
        case I2C_GET_ICM42605_TEM:  // 获取MPU6050的温度
            data[0] = I2C_ICM42605_Get_Temperature(dev);
            if (copy_to_user((int16_t*)arg, &data[0], sizeof(int16_t)))
                return -EFAULT;
            break;
        case I2C_GET_ICM42605_ANG:  // 获取MPU6050的角速度值
            I2C_ICM42605_Get_Gyroscope(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 3))
                return -EFAULT;
            break;
        case I2C_GET_ICM42605_ACC:  // 获取MPU6050的加速度值
            I2C_ICM42605_Get_Accelerometer(dev, &data[0], &data[1], &data[2]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 3))
                return -EFAULT;
            break;
        case I2C_GET_ICM42605_GYRO: // 获取MPU6050陀螺仪的角速度值
            I2C_ICM42605_Get_Raw_data(dev, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
            if (copy_to_user((int16_t*)arg, data, sizeof(data[0]) * 6))
                return -EFAULT;
            break;
        default:
            return -EFAULT;
    }
    return 0;
}

/********************************************************************************
 * @brief   上层 read 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   buf : 指向用户空间缓冲区的指针
 * @param   cnt : 表示用户期望从 I2C 设备读取的字节数
 * @param   off : 指向文件偏移量的指针
 * @return  大于零表示成功复制到用户空间的字节数；等于零表示已到文件末尾；失败返回错误码
 * @date    2025/3/20
 ********************************************************************************/
ssize_t i2c_read(struct file *f, char __user *buf, size_t cnt, loff_t *off)
{
    return 0;
}

/********************************************************************************
 * @brief   上层 write 函数相关实现
 * @param   f   : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @param   buf : 指向用户空间缓冲区的指针
 * @param   cnt : 表示用户期望从 I2C 设备写入的字节数
 * @param   off : 指向文件偏移量的指针
 * @return  大于零表示成功写入到用户空间的字节数；失败返回错误码
 * @date    2025/3/20
 ********************************************************************************/
ssize_t i2c_write(struct file *f, const char __user *buf, size_t cnt, loff_t *off)
{
    return 0;
}

/********************************************************************************
 * @brief   上层 close 函数相关实现
 * @param   inode: 指向 inode 结构体的指针，包括文件系统中文件或目录的元数据结构
 * @param   f    : 指向 file 结构体的指针，代表打开了一个打开的文件描述符
 * @return  0 表示释放成功，负数表示释放操作失败
 * @date    2025/3/20
 ********************************************************************************/
int i2c_release(struct inode *inode, struct file *f)
{
    return 0;
}

/*********************************** 设备驱动相关核心函数 ***********************************/

/********************************************************************************
 * @brief   设备驱动的探测函数
 * @param   client: 指向 I2C 客户端结构体，包含设备的地址、适配器等信息
 * @param   id    : 指向 I2C 设备 ID 结构体，包含设备的标识符，用于判断设备是否与驱动程序匹配
 * @return  0 表示设备探测和初始化成功，负数表示操作失败
 * @date    2025/3/20
 ********************************************************************************/
int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;                    // 存储各种操作的返回值
    struct ls_i2c_dev *ls_i2c;  // 创建一个自定义结构体
    struct ls_cycle_data *data; // 创建一个定时器结构体，用于周期性检测设备状态
    main_client = client;       // 将传入的client参数保存到全局变量main_client中
    data = &all_timer;

    // devm_kzalloc 是一个内存分配函数，会给 client->dev 分配内存，并初始化为 0
    ls_i2c = devm_kzalloc(&client->dev, sizeof(*ls_i2c), GFP_KERNEL);
    if (!ls_i2c)
        return -ENOMEM;

    // 获取字符设备的设备编号
    ret = alloc_chrdev_region(&ls_i2c->dev_id, 0, DEVICE_CNT, DEVICE_NAME);
    if (ret < 0) {
        pr_err("%s Couldn't alloc_chrdev_regin, ret = %d\r\n", DEVICE_NAME, ret);
        return -ENOMEM;
    }
    // 将字符设备的所有者设备为当前模块
    ls_i2c->cdev.owner = THIS_MODULE;
    // 初始化字符设备结构体
    cdev_init(&ls_i2c->cdev, &i2c_ops);
    // 将字符设备添加到内核的字符设备管理系统中
    ret = cdev_add(&ls_i2c->cdev, ls_i2c->dev_id, DEVICE_CNT);
    if (ret < 0)
        goto del_unregister;
    // 创建一个设备类，设备类用于在/sys/class目录下创建对应的目录，方便用户空间对设备进行管理
    ls_i2c->class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(ls_i2c->class))
        goto del_cdev;
    // 用于在 /dev 目录下创建一个设备节点，用户空间程序可以通过该节点与设备进行交互
    ls_i2c->device = device_create(ls_i2c->class, NULL, ls_i2c->dev_id, NULL, DEVICE_NAME);
    if (IS_ERR(ls_i2c->device))
        goto destroy_class;
    // 将 I2C 客户端指针保存到自定义结构体中
    ls_i2c->client = client;
    // 用于将自定义结构体与 I2C 客户端关联起来，方便在其他函数中通过 i2c_get_clientdata 获取设备的私有数据
    i2c_set_clientdata(client, ls_i2c);
    // 保存 I2C 适配器指针
    ls_i2c->adapter = client->adapter;
    // 初始化地址存放链表
    lq_list_init(&i2c_dev_addrs_list);
    // 扫描 I2C 总线上的所有设备地址，并将结果保存到自定义结构体中
    ret = scan_all_i2c_dev_addrs(ls_i2c, &i2c_dev_addrs_list);
    if (ret != 0) {
        printk(KERN_ERR "Scan I2C devices failed! ret=%d\n", ret);
        return ret;
    }
    // 初始化设备，包括设置设备状态、读取初始数据等操作
    device_init(ls_i2c, &i2c_dev_addrs_list);
    // 开启定时器
    {
        data->wq = create_workqueue("cycle_workqueue");
        if (!data->wq) {
            printk(KERN_ERR "Failed to create workqueue\n");
            return -ENOMEM;
        }
        // 初始化工作
        INIT_WORK(&data->work, cycle_work_handler);
        // 初始化定时器
        data->Dev = ls_i2c;                                                             // 将自定义结构体指针赋值给定时器的设备成员
        timer_setup(&data->cycle_detection, cycle_detection_timer_callback, 0);         // 初始化定时器，设置回调函数为 cycle_detection_timer_callback
        data->cycle_detection.expires = jiffies + msecs_to_jiffies(DETECT_INTERVAL_MS); // 设置定时器的超时时间，使其在设定的毫秒数后触发
        add_timer(&data->cycle_detection);                                              // 添加定时器，使其在设定的时间后执行回调函数
    }
    // 打印调试信息，表示设备探测和初始化成功
    printk("LQ_I2C_ALL_DRV module probe function\r\n");
    return 0;
destroy_class:
    // 注销之前创建的设备节点
    device_destroy(ls_i2c->class, ls_i2c->dev_id);
del_cdev:
    // 从内核的字符设备管理系统中删除指定的字符设备
    cdev_del(&ls_i2c->cdev);
del_unregister:
    // 用于注销之前分配的字符设备编号
    unregister_chrdev_region(ls_i2c->dev_id, DEVICE_CNT);
    return -EIO;
}

/********************************************************************************
 * @brief   设备驱动的移除函数
 * @param   c : 指向 I2C 客户端结构体，包含设备的地址、适配器等信息
 * @return  0 表示设备移除操作成功，负数表示操作失败
 * @date    2025/3/20
 ********************************************************************************/
int i2c_remove(struct i2c_client *c)
{
    struct ls_cycle_data *data;
    // 获取与 i2c_client 结构体关联的设备私有数据
    struct ls_i2c_dev *ls_i2c = i2c_get_clientdata(c);
    // 从内核的字符设备管理系统中删除指定的字符设备
    cdev_del(&ls_i2c->cdev);
    // 用于注销之前分配的字符设备编号
    unregister_chrdev_region(ls_i2c->dev_id, DEVICE_CNT);
    // 注销之前创建的设备节点
    device_destroy(ls_i2c->class, ls_i2c->dev_id);
    // 用于销毁之前创建的设备类
    class_destroy(ls_i2c->class);

    data = &all_timer;
    // 停止定时器，避免继续提交工作
    del_timer(&data->cycle_detection);
    // 等待工作队列无工作，销毁工作队列，释放相关资源
    flush_workqueue(data->wq);
    destroy_workqueue(data->wq);
   
    printk("LQ_I2C_ALL_DRV module remove function\r\n");
    return 0;
}

/********************************************************************************
 * @brief   设备驱动的加载函数
 * @date    2025/3/20
 ********************************************************************************/
static int __init i2c_drv_init(void)
{
    // IIC 设备驱动注册入口函数
    i2c_add_driver(&i2c_drv);
    return 0;
}

/********************************************************************************
 * @brief   设备驱动的卸载函数
 * @date    2025/3/20
 ********************************************************************************/
static void __exit i2c_drv_exit(void)
{
    // IIC 设备驱动注销入口函数
    i2c_del_driver(&i2c_drv);
}

/************************************ 内核模块相关宏函数 ************************************/
module_init(i2c_drv_init);                  // 指定加载函数
module_exit(i2c_drv_exit);                  // 指定卸载函数
MODULE_AUTHOR("LQ_012 <chiusir@163.com>");  // 作者以及邮箱
MODULE_DESCRIPTION("一些I2C设备的驱动端模块"); // 模块简单介绍
MODULE_VERSION("2.0");                      // 版本号
MODULE_LICENSE("GPL");                      // 许可证声明
