#ifndef __LQ_ICM42605_H__
#define __LQ_ICM42605_H__

#include "LQ_drv_common.h"

#define ICM42605_TEMP_OUTH_REG          0X1D // 温度值高8位寄存器
#define ICM42605_TEMP_OUTL_REG          0X1E // 温度值低8位寄存器

#define ICM42605_ACCEL_XOUTH_REG        0X1F // 加速度值,X轴高8位寄存器
#define ICM42605_ACCEL_XOUTL_REG        0X20 // 加速度值,X轴低8位寄存器
#define ICM42605_ACCEL_YOUTH_REG        0X21 // 加速度值,Y轴高8位寄存器
#define ICM42605_ACCEL_YOUTL_REG        0X22 // 加速度值,Y轴低8位寄存器
#define ICM42605_ACCEL_ZOUTH_REG        0X23 // 加速度值,Z轴高8位寄存器
#define ICM42605_ACCEL_ZOUTL_REG        0X24 // 加速度值,Z轴低8位寄存器

#define ICM42605_GYRO_XOUTH_REG         0X25 // 陀螺仪值,X轴高8位寄存器
#define ICM42605_GYRO_XOUTL_REG         0X26 // 陀螺仪值,X轴低8位寄存器
#define ICM42605_GYRO_YOUTH_REG         0X27 // 陀螺仪值,Y轴高8位寄存器
#define ICM42605_GYRO_YOUTL_REG         0X28 // 陀螺仪值,Y轴低8位寄存器
#define ICM42605_GYRO_ZOUTH_REG         0X29 // 陀螺仪值,Z轴高8位寄存器
#define ICM42605_GYRO_ZOUTL_REG         0X2A // 陀螺仪值,Z轴低8位寄存器

#define reg_bank_sel                    0x76
#define device_config_reg               0x11
#define bit_spi_mode                    0x10
#define bit_soft_reset_chip_config      0x01

#define intf_config4                    0x7A
#define pwr_mgmt0_reg                   0x4E
// #define bit_temp_dis                    0x20
#define bit_idle                        0x10
#define bit_gyro_mode_mask              ((0x03) << 2)
#define bit_gyro_mode_off               ((0x00) << 2)
#define bit_gyro_mode_standby           ((0x01) << 2)
#define bit_gyro_mode_lp                ((0x02)<<2)
#define bit_gyro_mode_ln                ((0x03) << 2)
#define bit_accel_mode_mask             ((0x03) << 0)
#define bit_accel_mode_off              0x00
#define bit_accel_mode_lp               0x02
#define bit_accel_mode_ln               0x03

#define gyro_config0_reg                0x4F
#define bit_gyro_ui_fs_sel_shift        5
#define bit_gyro_ui_fs_sel_2000dps      ((0x00) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_1000dps      ((0x01) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_500dps       ((0x02) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_250dps       ((0x03) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_125dps       ((0x04) << bit_gyro_ui_fs_sel_shift)
#define bit_gyro_ui_fs_sel_mask         ((0x07) << bit_gyro_ui_fs_sel_shift)

#define bit_gyro_odr_200hz              ((0x07) << 0)
#define bit_gyro_odr_100hz              ((0x08) << 0)
#define bit_gyro_odr_50hz               ((0x09) << 0)
#define bit_gyro_odr_25hz               ((0x0A) << 0)
#define bit_gyro_odr_12_5hz             ((0x0B) << 0)
#define bit_gyro_odr_nonflame_mask      ((0x0F) << 0)

#define accel_config0_reg               0x50
#define bit_accel_ui_fs_sel_shift       5
#define bit_accel_ui_fs_sel_16g         ((0x00) << bit_accel_ui_fs_sel_shift)
#define bit_accel_ui_fs_sel_8g          ((0x01) << bit_accel_ui_fs_sel_shift)
#define bit_accel_ui_fs_sel_4g          ((0x02) << bit_accel_ui_fs_sel_shift)
#define bit_accel_ui_fs_sel_2g          ((0x03) << bit_accel_ui_fs_sel_shift)
#define bit_accel_ui_fs_sel_mask        ((0x07) << bit_accel_ui_fs_sel_shift)

#define bit_accel_odr_200hz             ((0x07) << 0)
#define bit_accel_odr_100hz             ((0x08) << 0)
#define bit_accel_odr_50hz              ((0x09) << 0)
#define bit_accel_odr_25hz              ((0x0A) << 0)
#define bit_accel_odr_12_5hz            ((0x0B) << 0)
#define bit_accel_odr_nonflame_mask     ((0x0F) << 0)

#define int_source0_reg                 0x65
#define bit_int_ui_fsync_int1_en        0x40
#define bit_int_pll_rdy_int1_en         0x20
#define bit_int_reset_done_int1_en      0x10
#define bit_int_ui_drdy_int1_en         0x08
#define bit_int_fifo_ths_int1_en        0x04 // FIFO threshold interrupt
#define bit_int_fifo_full_int1_en       0x02
#define bit_int_ui_agc_rdy_int1_en      0x01

#define sensor_selftest_reg             0x6B
#define bit_accel_st_result             0x08
#define bit_gyro_st_result              0x04
#define bit_accel_st_status             0x02
#define bit_gyro_st_status              0x01

#define int_config_reg                  0x14
#define bit_int2_mode                   0x20
#define bit_int2_drive_circuit          0x10
#define bit_int2_polarity               0x08
#define bit_int1_mode                   0x04
#define bit_int1_drive_circuit          0x02
#define bit_int1_polarity               0x01

#define fifo_config_reg                 0x16
#define bit_fifo_mode_ctrl_mask         ((0x03) << 6)
#define bit_fifo_mode_ctrl_bypass       ((0x00) << 6)
#define bit_fifo_mode_ctrl_stream       ((0x01) << 6)
#define bit_fifo_mode_ctrl_snapshot     ((0x02) << 6)

#define tmst_config_reg                 0x54
#define bit_fifo_ram_iso_ena            0x40
#define bit_en_dreg_fifo_d2a            0x20
#define bit_tmst_to_regs_en             0x10
#define bit_tmst_resol                  0x08
#define bit_tmst_delta_en               0x04
#define bit_tmst_fsync_en               0x02
#define bit_tmst_en                     0x01

#define fifo_config2_reg                0x60
#define fifo_config3_reg                0x61

#define fsync_config_reg                0x62
#define bit_fsync_ui_sel_mask           ((0x07) << 4)
#define bit_fsync_ui_sel_tag_temp       ((0x01) << 4)
#define bit_fsync_ui_flag_clear_sel     0x02

#define fifo_config1_reg                0x5F
#define bit_fifo_resume_partial         0x40
#define bit_fifo_wm_gt_th               0x20
#define bit_fifo_hires_en               0x10
#define bit_fifo_tmst_fsync_en          0x08
#define bit_fifo_temp_en                0x04
#define bit_fifo_gyro_en                0x02
#define bit_fifo_accel_en               0x01

#define int_config0_reg                 0x63
#define int_config1_reg                 0x64
#define bit_int_asy_rst_disable         0x10

#define fifo_byte_count_h_res           0x2E
#define fifo_byte_count_l_res           0x2F

#define fifo_accel_en                   0x40
#define fifo_gyro_en                    0x20

#define fifo_data_port                  0x30

/************************************* 陀螺仪相关函数 **************************************/

uint8_t I2C_ICM42605_Init               (struct ls_i2c_dev *dev, ls_all_mod *mod);                                                              /* 初始化ICM42605 */

int16_t I2C_ICM42605_Get_Temperature    (struct ls_i2c_dev *dev);                                                                               /* 获取温度数据 */
uint8_t I2C_ICM42605_Get_Gyroscope      (struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz);                                        /* 获取陀螺仪数据 */
uint8_t I2C_ICM42605_Get_Accelerometer  (struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az);                                        /* 获取加速度数据 */
uint8_t I2C_ICM42605_Get_Raw_data       (struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz); /* 获取原始数据 */

#endif
