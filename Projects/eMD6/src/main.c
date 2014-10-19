
/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "stdio.h"
#include "discover_board.h"

#include "uart.h"
#include "i2c.h"
#include "exti.h"
#include "main.h"
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
     
    result = mpu_int_sensor_init();
    if (result) 
    {
        MPL_LOGE("Could not initialize hardware.\n");
    }
    
    result = mpu_int_library_init();
    if (result) 
    {
        MPL_LOGE("Could not initialize libraries.\n");
    }
    
    
    mpu_int_sensor_and_library_setup();
    
    hal_variable_init();
    
     hal.dmp_on = (0u == mpu_int_dmp_setup())? 1u : 0u;
    
      
      MPL_LOGE(" -> OK\n"); 
      
      
    while(1)
    {
      unsigned long sensor_timestamp;
      int new_data = 0;
      

      /* parsing serial input deleted */
    
    app_config_get_clock_ms(&timestamp);     /* Compass reads are handled by scheduler. */

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

  //Configure I2C
  I2C_Config();
  
  //Configure Interrupts
  EXTI_INT_Config();
  
  //Configure UART
  USART_Config();

}  

