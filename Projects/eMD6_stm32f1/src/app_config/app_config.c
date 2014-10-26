#include "app_config.h"
#include "stdint.h"
  
  
static volatile uint32_t TimingDelay;
volatile uint32_t hal_timestamp = 0;
/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }

}


void TimeStamp_Increment(void)
{
  hal_timestamp++;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
  
}



uint8_t app_config_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = hal_timestamp;
    return 0;
}


