/**

Global configuration for MPU9250/MPU6050 driver and hal layer

*/



#ifndef _MPU_CONFIG_H_
#define _MPU_CONFIG_H_



//FOR: inv_mpu.c and inv_mpu_dmp_moiton_driver.c

#include "i2c.h"   
#include "app_config.h"
#include "log.h"

#define mpu_config_i2c_write   Sensors_I2C_WriteRegister_swap //Sensors_I2C_WriteRegister
#define mpu_config_i2c_read    Sensors_I2C_ReadRegister_swap  
#define mpu_config_delay_ms    Delay
#define mpu_config_get_ms      stm32l_get_clock_ms
#define mpu_config_log_i       MPL_LOGI
#define mpu_config_log_e       MPL_LOGE
#define mpu_config_min(a,b)    ((a < b) ? a : b)

    
    
//FOR: PACKET.C    
//#include "stm32l1xx.h" //->let's check what for is it
#include "uart.h"

#define mpu_config_fputchar fputchar
    
#endif  /* #ifndef _MPU_CONFIG_H_ */