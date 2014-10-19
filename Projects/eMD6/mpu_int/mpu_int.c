
#include "mpu_int.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "app_config.h"


static uint8_t accel_range;
static uint16_t compass_range;
static uint16_t gyro_sampling_rate;
static uint16_t gyro_range;
static uint16_t local_flags_of_dmp;



#ifdef COMPASS_ENABLED
    unsigned short compass_range;
#endif



static void mpu_int_sensor_setup(void);
static void mpu_int_library_setup(void);


uint16_t mpu_int_sensor_init(void)
{
  struct int_param_s int_param;
  uint16_t ret = 0u;
  
  ret |= (uint16_t) mpu_init(&int_param);
  
 
  return ret;
}


uint16_t mpu_int_library_init(void)
{

  uint16_t result = 0u;

  result |= inv_init_mpl();

  inv_enable_quaternion();
  inv_enable_9x_sensor_fusion();
    
  /* The MPL expects compass data at a constant rate (matching the rate
  * passed to inv_set_compass_sample_rate). If this is an issue for your
  * application, call this function, and the MPL will depend on the
  * timestamps passed to inv_build_compass instead.
  *
  * inv_9x_fusion_use_timestamps(1);
  */

   /* Update gyro biases when not in motion.
   * WARNING: These algorithms are mutually exclusive.
   */
  inv_enable_fast_nomot();
  /* inv_enable_motion_no_motion(); */
  /* inv_set_no_motion_time(1000); */

  /* Update gyro biases when temperature changes. */
  inv_enable_gyro_tc();

  /* This algorithm updates the accel biases when in motion. A more accurate
  * bias measurement can be made when running the self-test (see case 't' in
  * handle_input), but this algorithm can be enabled if the self-test can't
  * be executed in your application.
  *
  * inv_enable_in_use_auto_calibration();
  */

  #ifdef COMPASS_ENABLED
  /* Compass calibration algorithms. */
  inv_enable_vector_compass_cal();
  inv_enable_magnetic_disturbance();
  #endif
  /* If you need to estimate your heading before the compass is calibrated,
   * enable this algorithm. It becomes useless after a good figure-eight is
   * detected, so we'll just leave it out to save memory.
   * inv_enable_heading_from_gyro();
   */

  /* Allows use of the MPL APIs in read_from_mpl. */
  inv_enable_eMPL_outputs();

  result |= inv_start_mpl();

  
    
  return result;

}

void mpu_int_sensor_and_library_setup(void)
{
  mpu_int_sensor_setup();
  mpu_int_library_setup();
  
  
}

uint8_t mpu_int_dmp_setup(void)
{
  uint8_t ret = 0u;
   /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
   /*
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    */
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    local_flags_of_dmp = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(local_flags_of_dmp);
    dmp_set_fifo_rate(MPU_INT_DEFAULT_MPU_HZ);
    
    
    ret = mpu_set_dmp_state(1);
    

    
    return ret;
    
}

static void mpu_int_library_setup(void)
{
  
  /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_sampling_rate);
    inv_set_accel_sample_rate(1000000L / gyro_sampling_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(MPU_INT_COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_range<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_range<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_range<<15);
#endif
    
    
}

static void mpu_int_sensor_setup(void)
{
  
           /* Get/set hardware configuration. Start gyro. */
        /* Wake up all sensors. */
    #ifdef COMPASS_ENABLED
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    #else
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    #endif
         /* Push both gyro and accel data into the FIFO. */
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu_set_sample_rate(MPU_INT_DEFAULT_MPU_HZ);
    #ifdef COMPASS_ENABLED
        /* The compass sampling rate can be less than the gyro/accel sampling rate.
         * Use this function for proper power management.
         */
        mpu_set_compass_sample_rate(1000 / MPU_INT_COMPASS_READ_MS);
    #endif
        
        //mpu_set_accel_fsr(16);
        
        
        /* Read back configuration in case it was set improperly. */
        mpu_get_sample_rate(&gyro_sampling_rate);
        mpu_get_gyro_fsr(&gyro_range);
        mpu_get_accel_fsr(&accel_range);
    #ifdef COMPASS_ENABLED
        mpu_get_compass_fsr(&compass_range);
    #endif
   
}


    
/* We're not using a data ready interrupt for the compass, so we'll
 * make our compass reads timer-based instead.
 */
uint8_t mpu_int_checkNewCompassReadingTimer(void)
{
  static uint32_t next_compass_ms = 0u;
  unsigned long timestamp;
  uint8_t ret = 0xFFu;
   
  app_config_get_clock_ms(&timestamp);     
    
  if( timestamp > next_compass_ms)
  {
    ret = 1u;
    next_compass_ms = timestamp + MPU_INT_COMPASS_READ_MS;
  }
  else
  {
    ret = 0u;
  }
  
  return ret;
}


/* Temperature data doesn't need to be read with every gyro sample.
 * Let's make them timer-based like the compass reads.
 */
uint8_t mpu_int_checkNewTemperatureReadingTimer(void)
{
  static uint32_t next_temp_ms = 0u;
  unsigned long timestamp;
  uint8_t ret = 0xFFu;
  
  
  app_config_get_clock_ms(&timestamp);    
    
    
  if(timestamp > next_temp_ms)
  {
    ret = 1u;
    next_temp_ms = timestamp + MPU_INT_TEMP_READ_MS;
  }
  else
  {
    ret = 0u;
  }
  
  return ret;
}


  