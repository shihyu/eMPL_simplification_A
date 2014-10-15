
/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "stdio.h"
#include "discover_board.h"

#include "uart.h"
#include "i2c.h"
#include "gpio.h"
#include "main.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"


/* Private typedef -----------------------------------------------------------*/

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */

#define DEFAULT_MPU_HZ  (100)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};


unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

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


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
void MPU_hardware_init(unsigned char * accel_range,  
                       unsigned short * compass_range,
                       unsigned short * gyro_sampling_rate,
                       unsigned short * gyro_range);

void data_builder_init(unsigned char accel_range,  
                       unsigned short compass_range,
                       unsigned short gyro_sampling_rate,
                       unsigned short gyro_range);

inv_error_t MPL_libraries_init(void);
void hal_variable_init(void);
void motion_processor_init(void);

void my_read_from_mpl(void);
void  RCC_Configuration(void);
void  Init_GPIOs (void);
void Delay(uint32_t nTime);
void platform_init(void);
int stm32l_get_clock_ms(unsigned long *count);




/*******************************************************************************/


                                  
int main(void)
{ 
    inv_error_t result;
    unsigned char accel_range,  new_temp = 0;
    unsigned short gyro_sampling_rate, gyro_range;
    unsigned long timestamp;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_range;
#endif
    
    
    platform_init();
    
    MPL_LOGE("Init -> \n");  
     
    result = mpu_init(&int_param);
    if (result) 
    {
        MPL_LOGE("Could not initialize hardware.\n");
    }
    
    result = MPL_libraries_init();
    if (result) 
    {
        MPL_LOGE("Could not initialize libraries.\n");
    }
    
    
    MPU_hardware_init(&accel_range,  
                       &compass_range,
                       &gyro_sampling_rate,
                       &gyro_range);
    
    
    data_builder_init(accel_range,  
                       compass_range,
                       gyro_sampling_rate,
                       gyro_range);
    
    hal_variable_init();
    
     /* Compass reads are handled by scheduler. */
    stm32l_get_clock_ms(&timestamp);
    
    motion_processor_init();
      
      MPL_LOGE(" -> OK\n"); 
      
      
    while(1)
    {
      unsigned long sensor_timestamp;
      int new_data = 0;
      

      /* parsing serial input deleted */
    
    stm32l_get_clock_ms(&timestamp);     /* Compass reads are handled by scheduler. */

#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif
        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

      if (hal.sensors || hal.new_gyro) 
      {
          if (hal.new_gyro && hal.dmp_on) 
          {
              short gyro[3], accel_short[3], sensors;
              unsigned char more;
              long accel[3], quat[4], temperature;
              /* This function gets new data from the FIFO when the DMP is in
               * use. The FIFO can contain any combination of gyro, accel,
               * quaternion, and gesture data. The sensors parameter tells the
               * caller which data fields were actually populated with new data.
               * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
               * the FIFO isn't being filled with accel data.
               * The driver parses the gesture data to determine if a gesture
               * event has occurred; on an event, the application will be notified
               * via a callback (assuming that a callback function was properly
               * registered). The more parameter is non-zero if there are
               * leftover packets in the FIFO.
               */
              dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
             
             
             
              if (!more)
                  hal.new_gyro = 0;
              
              /* go back here */
              
              if (sensors & INV_XYZ_GYRO) {
                  /* Push the new data to the MPL. */
                  inv_build_gyro(gyro, sensor_timestamp);
                  new_data = 1;
                  if (new_temp) {
                      new_temp = 0;
                      /* Temperature only used for gyro temp comp. */
                      mpu_get_temperature(&temperature, &sensor_timestamp);
                      inv_build_temp(temperature, sensor_timestamp);
                  }
              }
              if (sensors & INV_XYZ_ACCEL) {
                  accel[0] = (long)accel_short[0];
                  accel[1] = (long)accel_short[1];
                  accel[2] = (long)accel_short[2];
                  inv_build_accel(accel, 0, sensor_timestamp);
                  new_data = 1;
              }
          
          
              if (sensors & INV_WXYZ_QUAT) {
                  inv_build_quat(quat, 0, sensor_timestamp);
                  new_data = 1;
              }
          }
  #ifdef COMPASS_ENABLED
          if (new_compass) {
              short compass_short[3];
              long compass[3];
              new_compass = 0;
              /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
               * magnetometer registers are copied to special gyro registers.
               */
              if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                  compass[0] = (long)compass_short[0];
                  compass[1] = (long)compass_short[1];
                  compass[2] = (long)compass_short[2];
                  /* NOTE: If using a third-party compass calibration library,
                   * pass in the compass data in uT * 2^16 and set the second
                   * parameter to INV_CALIBRATED | acc, where acc is the
                   * accuracy from 0 to 3.
                   */
                  inv_build_compass(compass, 0, sensor_timestamp);
              }
              new_data = 1;
          }
  #endif
          if (new_data) {
              inv_execute_on_data();
              /* This function reads bias-compensated sensor data and sensor
               * fusion outputs from the MPL. The outputs are formatted as seen
               * in eMPL_outputs.c. This function only needs to be called at the
               * rate requested by the host.
               */
              my_read_from_mpl();
          }
          
          
      }
  
    }

}

/*---------------------------------------------------------------------------*/
void my_read_from_mpl(void)
{
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
        eMPL_send_quat(data);
    }
}
       

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}


void motion_processor_init(void)
{
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
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
}

inv_error_t MPL_libraries_init(void)
{
  inv_error_t result = 0;
  
    result = inv_init_mpl();
    if (result) {
        MPL_LOGE("Could not initialize MPL.\n");
    }
    
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

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            MPL_LOGE("Not authorized.\n");
        }
    }
    if (result) {
        MPL_LOGE("Could not start the MPL.\n");
    }
    
    return result;
}

void MPU_hardware_init(unsigned char * accel_range,  
                       unsigned short * compass_range,
                       unsigned short * gyro_sampling_rate,
                       unsigned short * gyro_range)
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
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(gyro_sampling_rate);
    mpu_get_gyro_fsr(gyro_range);
    mpu_get_accel_fsr(accel_range);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(compass_range);
#endif
   
    
}

void data_builder_init(unsigned char accel_range,  
                       unsigned short compass_range,
                       unsigned short gyro_sampling_rate,
                       unsigned short gyro_range)
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
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
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

void hal_variable_init(void)
{
  

    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;
    
}
    
    
/**
 * Configure the hardware of the Discovery board to link with the MPU
 */
void platform_init(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
   /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_md.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 

  /* Configure Clocks for Application need */
  RCC_Configuration();

  /* Set internal voltage regulator to 1.8V */
  PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

  /* Wait Until the Voltage Regulator is ready */
  while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;
  
  /* Configure SysTick IRQ and SysTick Timer to generate interrupts every 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000); 

  /* Init I/O ports */
  Init_GPIOs();  //Initialize the I2C, UART, Intterupts, and the Green and Blue LEDs
}
		

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{  
  
  /* Enable HSI Clock */
  RCC_HSICmd(ENABLE);
  
  /*!< Wait till HSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  
  RCC_MSIRangeConfig(RCC_MSIRange_6);

  RCC_HSEConfig(RCC_HSE_OFF);  
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    while(1);
  }
 
  /* Enable  comparator clock LCD and PWR mngt */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);
    
  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);

}


void  Init_GPIOs (void)
{
  //GPIO_InitTypeDef GPIO_InitStructure;
  
    /* Enable GPIOs clock */ 	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | 
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);
  
  
  //Configure I2C
  I2C_Config();
  
  //Configure Interrupts
  GPIO_Config();
  
  //Configure UART
  USART_Config();
  
/* Configure the GPIO_LED pins  LD3 & LD4*/
  /*GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);	
  GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);*/

}  



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
    
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
