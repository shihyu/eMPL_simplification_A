/*
 * uart.c
 *
 *  Created on: May 15, 2014
 *      Author: jerzy
 */

#include "uart.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdlib.h>
#include <stdio.h>

#define USARTx                           USART1
#define hardUSART1_DEFAULT_BAUDRATE    (uint32_t) 115200u
#define hardUSART1_Mode                (USART_Mode_Rx | USART_Mode_Tx)
#define hardUSART1_PortClock           RCC_APB2Periph_GPIOA
#define hardUSART1_PeriphClock         RCC_APB2Periph_USART1
#define hardUSART1_Port                GPIOA
#define hardUSART1_TX_Pin              GPIO_Pin_9
#define hardUSART1_RX_Pin              GPIO_Pin_10



void USART_Config(){

    /* structure with data for USART configuration */
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* RCC configuration */
    RCC_APB2PeriphClockCmd(hardUSART1_PortClock, ENABLE);
    RCC_APB2PeriphClockCmd(hardUSART1_PeriphClock, ENABLE);


    /* GPIO configuration */
    /* Configuring USART1_Tx as 'alternate function push-pull' */
    GPIO_InitStructure.GPIO_Pin = hardUSART1_TX_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(hardUSART1_Port, &GPIO_InitStructure);

    /* Configuring USART1_Rx as 'input floating' */
    GPIO_InitStructure.GPIO_Pin = hardUSART1_RX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(hardUSART1_Port, &GPIO_InitStructure);


    /* Place the vector table into FLASH */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    /* Enabling interrupt from USART1 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xf - 1;// System works with 1 degree lower priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART parameters */
    USART_InitStructure.USART_BaudRate = hardUSART1_DEFAULT_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = hardUSART1_Mode;

    /* Configuring and enabling USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enabling interrupt from USART1 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USARTx, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

int fputchar(int ch )
{
  //* Place your implementation of fputc here 
  //* e.g. write a character to the USART 
  USART_SendData(USARTx, (uint8_t) ch);

  //* Loop until the end of transmission 
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}