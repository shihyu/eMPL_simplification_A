
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "discover_board.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void TimingDelay_Decrement(void);
void TimeStamp_Increment(void);
void Delay(__IO uint32_t nTime);
int stm32l_get_clock_ms(unsigned long *count);

#endif /* APP_CONFIG_H */
