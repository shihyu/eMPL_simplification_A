/*
 * hardware.h
 *
 *  Created on: May 15, 2014
 *      Author: jerzy
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "stdint.h"

/* ----- LED1 definitions --------------------------------------------------- */
#define hardLED1_PORT      GPIOA
#define hardLED1_PIN       GPIO_Pin_2
#define hardLED1_RCC_APB2  RCC_APB2Periph_GPIOA

void vLed1Initialise(void);
void vLed1Set(const uint8_t value);
void vLed1Toggle(void);


/* ----- LED2 definitions --------------------------------------------------- */
#define hardLED2_PORT      GPIOA
#define hardLED2_PIN       GPIO_Pin_3
#define hardLED2_RCC_APB2  RCC_APB2Periph_GPIOA

void vLed2Initialise(void);
void vLed2Set(const uint8_t value);
void vLed2Toggle(void);







#endif /* HARDWARE_H_ */
