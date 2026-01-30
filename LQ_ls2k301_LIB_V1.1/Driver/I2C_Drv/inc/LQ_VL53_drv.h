#ifndef __LQ_VL53_DIR_H__
#define __LQ_VL53_DIR_H__

#include "LQ_drv_common.h"

#define MAX_DISTABCE                                ( 80000 )
#define INVALID_DISTANCE                            ( 20 )

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         ( 0xc0 )
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      ( 0xc2 )
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   ( 0x50 )
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD ( 0x70 )
#define VL53L0X_REG_SYSRANGE_START                  ( 0x00 )
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         ( 0x13 )
#define VL53L0X_REG_RESULT_RANGE_STATUS             ( 0x14 )
#define VL53_REG_DIS                                ( 0x1E )
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS        ( 0x8a )
#define VL53ADDR                                    ( 0x29 )    // 0x52   默认地址
#define VL53NEWADDR                                 ( 0x30 )    // VL53新地址



int vl53l0x_write_reg       (struct ls_i2c_dev *dev, uint8_t reg, uint8_t val);                 /* VL53L0X写一个寄存器 */
int vl53l0x_read_reg        (struct ls_i2c_dev *dev, uint8_t reg, uint8_t len, uint8_t *val);   /* VL53L0X读一个寄存器 */

int vl53l0x_init            (struct ls_i2c_dev *dev);                                           /* VL53L0X初始化 */
int vl53l0x_read_distance   (struct ls_i2c_dev *dev, uint16_t *distance);                       /* VL53L0X读取距离 */

#endif
