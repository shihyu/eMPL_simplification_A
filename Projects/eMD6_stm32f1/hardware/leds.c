/*
 * leds.c
 *
 *  Created on: May 15, 2014
 *      Author: jerzy
 */
#include "leds.h"
#include "stm32f10x_gpio.h"


#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()


/**
 * Initialization of any led GPIO
 */
void vLedInitialise(GPIO_TypeDef* const hardLED_PORT,
                        const uint16_t hardLED_PIN,
                        const uint32_t hardLED_RCC_APB2 )
{

    /* GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(hardLED_RCC_APB2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure pin in output push/pull mode */
    GPIO_InitStructure.GPIO_Pin = hardLED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(hardLED_PORT, &GPIO_InitStructure);
}
/**
 * turns on the LED if value is diffrent than 0
 */
void vLedSet(GPIO_TypeDef* const hardLED_PORT,
               const uint16_t hardLED_PIN,
               const uint8_t value)
{

    if(0u != value){
        taskENTER_CRITICAL();
        /* Turn on led by setting the pin low */
        GPIO_ResetBits(hardLED_PORT, hardLED_PIN);
        taskEXIT_CRITICAL();
    }
    else{
        taskENTER_CRITICAL();
        /* Turn off led by setting the pin high */
        GPIO_SetBits(hardLED_PORT, hardLED_PIN);
        taskEXIT_CRITICAL();
    }

}
/*
 * Toggles selected LED on selected port
 */
void vLedToggle(GPIO_TypeDef* const hardLED_PORT,
                  const uint16_t hardLED_PIN )
{
    taskENTER_CRITICAL();

    hardLED_PORT->ODR ^= hardLED_PIN;

    taskEXIT_CRITICAL();
}


/******************************************************************************/

void vLed1Initialise(void)
{
    vLedInitialise(hardLED1_PORT,hardLED1_PIN,hardLED1_RCC_APB2);
}

void vLed1Set(const uint8_t value)
{
    vLedSet(hardLED1_PORT, hardLED1_PIN, value);
}

void vLed1Toggle(void)
{
    vLedToggle(hardLED1_PORT,hardLED1_PIN);
}



/******************************************************************************/

void vLed2Initialise(void)
{
    vLedInitialise(hardLED2_PORT,hardLED2_PIN,hardLED2_RCC_APB2);
}

void vLed2Set(const uint8_t value)
{
    vLedSet(hardLED2_PORT, hardLED2_PIN, value);
}

void vLed2Toggle(void)
{
    vLedToggle(hardLED2_PORT,hardLED2_PIN);
}


