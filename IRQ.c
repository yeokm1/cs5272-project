/******************************************************************************/
/* IRQ.C: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <91x_lib.h>
#include <RTL.h>


short Potentiometer,SlideSensor;


unsigned char AD_in_progress;           /* AD conversion in progress flag     */

__irq void ADC_IRQ_Handler (void) {     /* AD converter interrupt routine     */

 
	Potentiometer = ADC->DR0 & 0x03FF;    /* AD value for global usage (10 bit) */	
	

	
	SlideSensor = ADC->DR1 & 0x03FF;          /* AD value for global usage (10 bit) */	//dr4.1
	
	
	
  ADC->CR &= 0xFFFE;                    /* Clear STR bit (Start Conversion)   */
  ADC->CR &= 0x7FFF;                    /* Clear End of Conversion flag       */

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;

  AD_in_progress = 0;                   /* Clear flag, as AD conv finished    */
}
