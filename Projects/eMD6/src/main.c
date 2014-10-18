
/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "stdio.h"
#include "discover_board.h"

#include "uart.h"
#include "i2c.h"
#include "exti.h"
#include "main.h"

#include "mpu_int.h"
#include "log.h"
#include "app_config.h"


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "packet.h"

/* Private typedef -----------------------------------------------------------*/

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

#define PRINT_ACCEL     1


unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void my_read_from_mpl(void);
void RCC_Configuration(void);
void hardware_init(void);
void platform_init(void);


/* Global flags --------------------------------------------------------------*/

static uint8_t flags_dmp_on = 0u;
static uint8_t flags_sensors_on = 0u;
static volatile uint8_t flags_new_data = 0u;


    
/*******************************************************************************/


                                  
int main(void)
{ 
    platform_init();
    
    if (0u != mpu_int_sensor_init()) 
    {
        MPL_LOGE("Could not initialize sensor.\n");
    }
        
    if (0u != mpu_int_library_init()) 
    {
        MPL_LOGE("Could not initialize libraries.\n");
    }
    
    mpu_int_sensor_and_library_setup();
    
      
    
     /* Initialize global flags */
#ifdef COMPASS_ENABLED
    flags_sensors_on = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    flags_sensors_on = ACCEL_ON | GYRO_ON;
#endif
    
  
    if (0u == mpu_int_dmp_setup()) 
    {
      flags_dmp_on = 1u;
    }
    else
    {
      flags_dmp_on = 0u;
      MPL_LOGE("Could not initialize DMP.\n");
    }
    
    
    
          
    while(1)
    {
      uint8_t new_data_flag = 0u;
      uint8_t new_temp_flag = 0u;
      uint8_t new_compass_flag = 0u;   
      
      unsigned long sensor_timestamp = 0u;


      mpu_int_check_timers_flags(&new_temp_flag,&new_compass_flag);

          
      if (flags_sensors_on || flags_new_data) 
      {
          if (flags_new_data && flags_dmp_on) 
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
                  flags_new_data = 0;
              
              /* go back here */
              
              if (sensors & INV_XYZ_GYRO) {
                  /* Push the new data to the MPL. */
                  inv_build_gyro(gyro, sensor_timestamp);
                  new_data_flag = 1;
                  if (new_temp_flag) {
                      new_temp_flag = 0;
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
                  new_data_flag = 1;
              }
          
          
              if (sensors & INV_WXYZ_QUAT) {
                  inv_build_quat(quat, 0, sensor_timestamp);
                  new_data_flag = 1;
              }
          }
  #ifdef COMPASS_ENABLED
          if (new_compass_flag) {
              short compass_short[3];
              long compass[3];
              new_compass_flag = 0;
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
              new_data_flag = 1;
          }
  #endif
          
          
          if (new_data_flag) {
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


/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    flags_new_data = 1;
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
    
    if (PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    
    
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
  hardware_init();  //Initialize the I2C, UART, Intterupts, and the Green and Blue LEDs
}


void  hardware_init (void)
{
  //Configure I2C
  I2C_Config();
  
  //Configure Interrupts
  EXTI_INT_Config();
  
  //Configure UART
  USART_Config();
}  




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
