

#ifndef _MPU_INT_H_
#define _MPU_INT_H_

#include "stdint.h"

#define MPU_INT_DEFAULT_MPU_HZ  (100)
#define MPU_INT_TEMP_READ_MS    (500)
#define MPU_INT_COMPASS_READ_MS (100)



/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};


/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif




uint16_t mpu_int_sensor_init(void);
uint16_t mpu_int_library_init(void);
void mpu_int_sensor_and_library_setup(void);
uint8_t mpu_int_dmp_setup(void);
uint8_t mpu_int_checkNewCompassReadingTimer(uint32_t timestamp);
uint8_t mpu_int_checkNewTemperatureReadingTimer(uint32_t timestamp);
void mpu_int_check_timers_flags(uint8_t * new_temp_flag, uint8_t * new_compass_flag);

#endif  /* #ifndef _MPU_CONFIG_H_ */