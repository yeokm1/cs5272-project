/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    DASHBOARD_TEMPLATE.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code has been modified from the example Blinky project of the
				RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"
#include <stdlib.h>
#include <cstdio>

#include <91x_lib.h>
#include <RTL.h>
#include <stdlib.h> 

#define LCD_COL 16

os_mbx_declare (mailbox_slidesensor, 20); 
os_mbx_declare (mailbox_potsensor, 20); 

unsigned char A0,A1; //A0 - Button 3.5, A1 - Button 3.6
unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7
unsigned char AD_in_progress;           /* AD conversion in progress flag     */

int slideValue;
int potValue;

OS_TID id_task_adc_send;
OS_TID id_task_adc_recv;

void printMessage(char * buff){
		LCD_cls();
		LCD_puts((unsigned char *) buff);
}


void printNumber(int number){
	  char buff[20];
		sprintf(buff, "%d", number);
		printMessage(buff);
}


void initMailBoxes(){
	os_mbx_init (&mailbox_slidesensor, sizeof (mailbox_slidesensor)); 	
}





__irq void ADC_IRQ_Handler (void) {     /* AD converter interrupt routine     */

	int potValue = ADC->DR0 & 0x03FF;    /* AD value for global usage (10 bit) */	
	int slideValue = ADC->DR1 & 0x03FF;          /* AD value for global usage (10 bit) */	//dr4.1
	
	int * slidePointer = malloc(sizeof(int));
	int * potPointer = malloc(sizeof(int));
	
	
	* slidePointer = slideValue;
	* potPointer = potValue;
	

	os_mbx_send (&mailbox_slidesensor, slidePointer, 0xFFFF);  
	os_mbx_send (&mailbox_potsensor, potPointer, 0xFFFF);  

	
  ADC->CR &= 0xFFFE;                    /* Clear STR bit (Start Conversion)   */
  ADC->CR &= 0x7FFF;                    /* Clear End of Conversion flag       */

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;

  AD_in_progress = 0;                   /* Clear flag, as AD conv finished    */
}


//Function to read input
void read_buttons()
{
	//BUTTON_3_5:
	A0 = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	A1 = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released

}


//Function to write to led
void write_led()
{
  unsigned char mask = 0;

  mask  = (B0<<0);
  mask |= (B1<<1);
  mask |= (B2<<2);
  mask |= (B3<<3);
  mask |= (B4<<4);
  mask |= (B5<<5);
  mask |= (B6<<6);
  mask |= (B7<<7);

  GPIO7->DR[0x3FC] = mask;
}

/*----------------------------------------------------------------------------
 *        Task 1 'ADC_Con': ADC Conversion
 *---------------------------------------------------------------------------*/
__task void ADC_Con(void){
  // timing
	const unsigned int period = 100;

	os_itv_set(period);	
	for(;;){ 
		os_itv_wait();
		/* Do actions below */
		if (!AD_in_progress){             						    /* If conversion not in progress      */
        AD_in_progress = 1;                 /* Flag that AD conversion is started */
        ADC->CR |= 0x0423;                  /* Set STR bit (Start Conversion)     */
		}
	//Now the interrupt will be called when conversion ends
			

					
	}
}	 // End ADC_Con(void)

void printSpeed(){

	char buff[LCD_COL];
	int speedValue = slideValue / 2;
	
	sprintf(buff, "Speed: %03dkm/h", speedValue);
	
	printMessage(buff);


}


__task void ADC_Recv(void){
		void * slideMsg;
		void * potMsg;

	
		while(1){
			
			os_mbx_wait (&mailbox_slidesensor, &slideMsg, 0xffff); 
	
			slideValue =  * ((int * ) slideMsg);
			free(slideMsg);
			
		  os_mbx_wait (&mailbox_potsensor, &potMsg, 0xffff); 
			
			potValue =  * ((int * ) potMsg);
			
			free(potMsg);
			
			printSpeed();
			


		}
		
	
	
	
	
	

}



/*----------------------------------------------------------------------------
 *        Task 0 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

  unsigned int n = 0;
  		  
	/* Set up Potentiometer and Light sensor*/                                                              
  
  SCU->GPIOIN[4]  |= 0x07;                /* P4.0, P4.1 and P4.2 input  - mode 0             */
  SCU->GPIOOUT[4] &= 0xFFC0;              /* P4.0 output - mode 0             */

  /* To look up search for GPIO4_DIR in the peripherals manual 				  */
  GPIO4->DDR      &= 0xF8;                /* P4.0 and P4.1 direction - input  */
  SCU->GPIOANA    |= 0x0007;              /* P4.0 analog mode ON              */
	
	/* To understand the ADC configuration register consult the manual 				  */
	ADC->CR         |= 0x0042;              /* Set POR bit                      */
  for (n = 0; n < 100000; n ++);          /* Wait > 1 ms  (at 96 MHz)         */

  ADC->CR         &= 0xFFB7;              /* Clear STB bit                    */
  for (n = 0; n < 1500; n ++);            /* Wait > 15 us (at 96 MHz)         */

  ADC->CR         |= 0x0423;              /* Enable end of conversion interupt*/
  ADC->CCR         = 0x003F;              /* AD Conversion for Channels 0,1,2, No WDG on Ch 0    */

  /* Configure and enable IRQ for A/D Converter (ADC)                         */
  VIC0->VAiR[15]  = (unsigned int)ADC_IRQ_Handler; /* Setup ADC IRQ Hndl addr */
  VIC0->VCiR[15] |= 0x20;                 /* Enable the vector interrupt      */
  VIC0->VCiR[15] |= 15;                   /* Specify the interrupt number     */
  VIC0->INTER    |= (1<<15);              /* Enable ADC interrupt             */

  /* Configuring LED                     */
  SCU->GPIOOUT[7]  = 0x5555;
  GPIO7->DDR       = 0xFF;
  GPIO7->DR[0x3FC] = 0x00;

  /* LCD Setup                           */
  GPIO8->DDR       = 0xFF;
  GPIO9->DDR       = 0x07;

  /* Port 3 setup for button 3.5 and 3.6 */
  SCU->GPIOIN[3]  |= 0x60;
  SCU->GPIOOUT[3] &= 0xC3FF;
  GPIO3->DDR      &= 0x9F;

  LCD_init(); //Initialize LCD
  LCD_cur_off(); //Remove LCD cursor
  LCD_cls(); //Clearing LCD screen

	initMailBoxes();

  id_task_adc_send = os_tsk_create(ADC_Con,1);
	id_task_adc_recv = os_tsk_create(ADC_Recv,2);
  
  os_tsk_delete_self ();
}


/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  os_sys_init (init);                    /* Initialize RTX and start init    */
  return 0;
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
