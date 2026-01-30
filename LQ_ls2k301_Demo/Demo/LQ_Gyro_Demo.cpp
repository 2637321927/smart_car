#include "LQ_demo.hpp"

// 定义要加载的模块
unordered_map<string, string> modules = {
    {"lq_i2c_drv", "lq_i2c_drv.ko"},
    {"lq_i2c_dev", "lq_i2c_dev.ko"},
    {"TFT18_dri", "TFT18_dri.ko"},
    {"TFT18_dev", "TFT18_dev.ko"}
};
    
// 创建模块加载器
LQModuleLoader loader(modules);

/********************************************************************************
 * @brief   MPU6050 陀螺仪测试
 * @param   none.
 * @retval  none.
 * @note    测试 MPU6050 陀螺仪模块，读取温度、角度和加速度数据.
 ********************************************************************************/
void Demo_I2C_Gyro()
{
    int16_t data[6] = {0};

    // 卸载所有模块（如果已加载）
    cout << "\nUnloading modules if already loaded..." << endl;
    loader.unloadAllModules();

    // 加载所有模块
    cout << "\nLoading all modules..." << endl;
    if (loader.loadAllModules()) {
        cout << "All modules loaded successfully!" << endl;
    } else {
        cerr << "Failed to load all modules!" << endl;
    }
    I2C_MPU6050 mpu6050(LS_IIC_PATH);
    while(1)
    {
        printf("ID: 0x%x\n", mpu6050.I2C_MPU6050_Get_ID());
        printf("Tem: %03.2f\n", mpu6050.I2C_MPU6050_Get_Tem());
        mpu6050.I2C_MPU6050_Get_Ang(&data[0], &data[1], &data[2]);
        printf("Ang: %d, %d, %d\n", data[0], data[1], data[2]);
        mpu6050.I2C_MPU6050_Get_Acc(&data[0], &data[1], &data[2]);
        printf("Acc: %d, %d, %d\n", data[0], data[1], data[2]);
        mpu6050.I2C_MPU6050_Get_RawData(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
        printf("Gyro: %d, %d, %d, %d, %d, %d\n", data[0], data[1], data[2], data[3], data[4], data[5]);

        usleep(100000);
    }
}

/********************************************************************************
 * @brief   VL53 测试
 * @param   none.
 * @retval  none.
 * @note    测试 VL53 模块，读取距离数据.
 ********************************************************************************/
void Demo_I2C_VL53()
{
    int16_t data[6] = {0};

    // 卸载所有模块（如果已加载）
    cout << "\nUnloading modules if already loaded..." << endl;
    loader.unloadAllModules();

    // 加载所有模块
    cout << "\nLoading all modules..." << endl;
    if (loader.loadAllModules()) {
        cout << "All modules loaded successfully!" << endl;
    } else {
        cerr << "Failed to load all modules!" << endl;
    }
    I2C_VL53L0X vl53l0x(LS_IIC_PATH);

    while(1)
    {
        printf("Dis: %d\n", vl53l0x.I2C_VL53L0X_Get_Dis());
        usleep(100000);
    }
}
