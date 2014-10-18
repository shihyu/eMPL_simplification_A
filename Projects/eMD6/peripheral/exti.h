/*********************************************************************
File    : exti.h
Purpose : 
**********************************************************************/

#ifndef __EXTI_H__
#define __EXTI_H__

/************************** Includes ***********************************/
/****************************** Defines *******************************/
#define ENABLE_INV_INTERRUPTS  EXTI_INT_EnableInvInterrupt()
#define DISABLE_INV_INTERRUPTS EXTI_INT_DisableInvInterrupt()


/***************************Globals *******************************************/
/***************************** Prototypes *****************************/
void EXTI_INT_Config(void);
void EXTI_INT_EnableInvInterrupt(void);
void EXTI_INT_DisableInvInterrupt(void);
void EXTI_INT_ExitFromInterupt(void);
#endif // __EXTI_H__

