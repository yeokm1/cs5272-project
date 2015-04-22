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
#include <math.h>

#define LCD_COL 16
#define BRIGHTNESS_TOTAL 20
#define TOTAL_BRIGHTNESS_SETTINGS 5

//1992 Toyota Camry
const	int CAR_MASS = 1300;
const float CAR_DRAG_COEFFICIENT = 0.31; //Cd
const float CAR_FRONT_AREA = 0.703; //m2


const	float MASS_DENSITY_AIR = 6;
		




const int POT_CAT[TOTAL_BRIGHTNESS_SETTINGS] = {204, 408, 612, 816, 1023};
const int BRIGHTNESS[TOTAL_BRIGHTNESS_SETTINGS] = {20, 10 , 6 , 3, 0};


int brightnessPositionHeadlight = 0;
int brightnessPositionInterior = 0;

unsigned char headlightCurrentlyOn = 0;

unsigned char engineButtonPrevState = 0;
unsigned char doorButtonPrevState = 0;

unsigned char engineCurrentlyOn = 0;
unsigned char doorCurrentlyOpen = 0;

unsigned char engineChangingState = 0;

os_mbx_declare (mailbox_slidesensor, 40); 
os_mbx_declare (mailbox_potsensor, 40); 
os_mbx_declare (mailbox_engineButton, 40);
os_mbx_declare (mailbox_customsensor, 40);
os_mbx_declare (mailbox_doorButton, 40);

unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7
unsigned char AD_in_progress;           /* AD conversion in progress flag     */

int slideValue;
int potValue;
int customSensorValue;

float currentSpeed;

int openToClose = 0;

OS_TID id_task_get_adc_and_buttons;
OS_TID id_task_adc_recv;
OS_TID id_task_headlight;
OS_TID id_task_lcd;
OS_TID id_task_engine;
OS_TID id_task_door;
OS_TID id_task_speed;
OS_TID id_task_interior;
OS_TID id_task_alarm;


OS_MUT mutex_lcd;


void printMessageWithoutMutex(char * buff){
			LCD_cls();
			LCD_puts((unsigned char *) buff);
}


OS_RESULT printMessage(char * buff, U16 timeout, int releaseWhenDone){
	  OS_RESULT acquireResult = os_mut_wait (&mutex_lcd, timeout);
		
		if(acquireResult == OS_R_OK || acquireResult == OS_R_MUT){
			
			printMessageWithoutMutex(buff);
			
			if(releaseWhenDone){
				os_mut_release (&mutex_lcd);
			}
			
		}
		
		return acquireResult;	
}



void initMailBoxes(){
	os_mbx_init (&mailbox_slidesensor, sizeof (mailbox_slidesensor));
	os_mbx_init (&mailbox_potsensor, sizeof (mailbox_potsensor)); 
	os_mbx_init (&mailbox_customsensor, sizeof (mailbox_customsensor));
	os_mbx_init (&mailbox_engineButton, sizeof (mailbox_engineButton));
	os_mbx_init (&mailbox_doorButton, sizeof (mailbox_doorButton)); 	
}





__irq void ADC_IRQ_Handler (void) {     /* AD converter interrupt routine     */

	int potValueRaw = ADC->DR0 & 0x03FF;    /* AD value for global usage (10 bit) */	
	int slideValueRaw = ADC->DR1 & 0x03FF;          /* AD value for global usage (10 bit) */	//dr4.1
	int customSensorValueRaw = ADC->DR2 & 0x03FF;   
	
	int * slidePointer = malloc(sizeof(int));
	int * potPointer = malloc(sizeof(int));
	int * customSensorPointer = malloc(sizeof(int));
	
	* slidePointer = slideValueRaw;
	* potPointer = potValueRaw;
	* customSensorPointer = customSensorValueRaw;
	

	os_mbx_send (&mailbox_slidesensor, slidePointer, 0xFFFF);  
	os_mbx_send (&mailbox_potsensor, potPointer, 0xFFFF);
	os_mbx_send (&mailbox_customsensor, customSensorPointer, 0xFFFF);    

	
  ADC->CR &= 0xFFFE;                    /* Clear STR bit (Start Conversion)   */
  ADC->CR &= 0x7FFF;                    /* Clear End of Conversion flag       */

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;

  AD_in_progress = 0;                   /* Clear flag, as AD conv finished    */
}


//Function to read input
void read_and_process_buttons(){
	 //A0 - Button 3.5, A1 - Button 3.6
	int button_door_now;
	int button_engine_now;
	
	//BUTTON_3_5:
	button_door_now = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	button_engine_now = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released

	if(currentSpeed < 1){
		//Debounce the engine button press
		if(button_engine_now && !engineButtonPrevState){
		
			
			if(!engineChangingState){
					engineChangingState = 1;
					os_mbx_send (&mailbox_engineButton, NULL, 0xFFFF); 	
			}			
		} 
		engineButtonPrevState = button_engine_now;
	}
	
	
	//Debounce the door button press
	if(button_door_now && !doorButtonPrevState){
			os_mbx_send (&mailbox_doorButton, NULL, 0xFFFF); 	
	} 
	doorButtonPrevState = button_door_now;
	
	
	
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


void changeHeadLightsState(int state){
	
		if(headlightCurrentlyOn != state){
			headlightCurrentlyOn = state;
			B0 = state;
			B1 = state;
			B2 = state;
			
			
			write_led();
		}
}

int isCurrentSpeedEffectivelyZero(){
	if(-1 < currentSpeed && currentSpeed < 1){
		return 1;
	} else {
		return 0;
	}

}

__task void headlightBrightness(){

	while(1){ 
		
		if(engineCurrentlyOn){
				int onDelay = BRIGHTNESS[brightnessPositionHeadlight];
				int offDelay = BRIGHTNESS_TOTAL - onDelay;
		
				changeHeadLightsState(0);
				os_dly_wait (offDelay); 
				changeHeadLightsState(1);
				os_dly_wait (onDelay);
		} else {
				changeHeadLightsState(0);
				
				//Delay to prevent excessive CPU use 
				os_dly_wait (100); 
		}
				

	}

}

void start_adc(){
			//This code starts the ADC
		if (!AD_in_progress){             						    /* If conversion not in progress      */
        AD_in_progress = 1;                 /* Flag that AD conversion is started */
        ADC->CR |= 0x0423;                  /* Set STR bit (Start Conversion)     */
		}
	//Now the interrupt will be called when conversion ends
	
}


/*----------------------------------------------------------------------------
 *        Task 1 'ADC_Con': ADC Conversion
 *---------------------------------------------------------------------------*/
__task void GET_INPUTS(void){
  // timing
	const unsigned int period = 100;

	os_itv_set(period);	
	for(;;){ 
		os_itv_wait();
		
		start_adc();
		read_and_process_buttons();
			
	}
}

void processPotValue(){
	int i;
	for(i = 0; i < TOTAL_BRIGHTNESS_SETTINGS; i++){
		if(potValue <= POT_CAT[i]){
			brightnessPositionHeadlight = i;
			break;
		}
		
	}

}


__task void ADC_Recv(void){
		void * slideMsg;
		void * potMsg;
		void * customSensorMsg;

	
		while(1){
			
			os_mbx_wait (&mailbox_slidesensor, &slideMsg, 0xffff); 
			slideValue =  * ((int * ) slideMsg);
			free(slideMsg);
			
		  os_mbx_wait (&mailbox_potsensor, &potMsg, 0xffff); 
			potValue =  * ((int * ) potMsg);
			free(potMsg);
			
			
			
			os_mbx_wait (&mailbox_customsensor, &customSensorMsg, 0xffff); 
			customSensorValue =  * ((int * ) customSensorMsg);
			free(customSensorMsg);
			
			processPotValue();
		
		}
		

}

__task void printLCD(void){

  // timing
	const unsigned int period = 100;
	char buff[LCD_COL];

	os_itv_set(period);	

	while(1){
		
		os_itv_wait();
				
		if(engineCurrentlyOn){
			sprintf(buff, "S:%03.0fkmh        D:%d, Amb:%d", currentSpeed, doorCurrentlyOpen, potValue);
			
		} else {
			sprintf(buff, "Engine Off      D:%d, Amb:%d", doorCurrentlyOpen, potValue);
			
		}
		
		printMessage(buff, 0, TRUE);
	
	
	}

}

void printEngineStoppedMessage(){
	printMessageWithoutMutex("Engine Stop");
}


__task void alarmTask(void){
	
	
	while(1){
		if(!engineCurrentlyOn || !doorCurrentlyOpen){
			break;
		}
		
		B6 = 1;
		B7 = 1;
			
		write_led();
		os_dly_wait (200); 

		
		B6 = 0;
		B7 = 0;

			
		write_led();
		
				
		os_dly_wait (200); 
		
		
	}
	
	
	os_tsk_delete_self();
}

void startAlarmTaskIfNeeded(){
	
	if(doorCurrentlyOpen || engineCurrentlyOn){
		os_tsk_create(alarmTask, 9);
	}
	
}

__task void engineChangerTask(void){

	
	void * engineButtonMessage;
	
	while(1){
		os_mbx_wait (&mailbox_engineButton, &engineButtonMessage, 0xffff);
		
		free(engineButtonMessage);
		
		if(engineCurrentlyOn){
			
			
			if(currentSpeed < 1){
				printMessage("Stopping Engine",0xFFFE, FALSE);
			
				os_dly_wait (1000); 
	
		
				engineCurrentlyOn = 0;
			
				engineChangingState = 0;
				os_mut_release (&mutex_lcd);
			}
			
			
		} else {
			
			
			if(slideValue == 0){
		
				printMessage("Starting Engine", 0xFFFE, FALSE);
			
				os_dly_wait (1000); 
			
				engineCurrentlyOn = 1;
			
				engineChangingState = 0;
				os_mut_release (&mutex_lcd);
			
				startAlarmTaskIfNeeded();
			} else {
				printMessage("Acc pressed,    release to start", 0xFFFE, FALSE);
			
				os_dly_wait (2000); 

				engineChangingState = 0;
				os_mut_release (&mutex_lcd);
			
			}
		}
		
	}


}

__task void doorTask(void){
	void * doorButtonMessage;
	

	while(1){
		os_mbx_wait (&mailbox_doorButton, &doorButtonMessage, 0xffff);
		free(doorButtonMessage);
		
		if(doorCurrentlyOpen){
			doorCurrentlyOpen = 0;
		} else {
			if(isCurrentSpeedEffectivelyZero()){
				doorCurrentlyOpen = 1;
				startAlarmTaskIfNeeded();
			}
		
		}
	
	
	}
}

__task void speedTask(){
	
		float forwardOrBrakeForce;
		float dragForce;
		float resultantForce;
		float acceleration;
		float accIn100ms;
	
		
		os_itv_set(100);	
		
	
		while(1){
		
			os_itv_wait();
			
			if(!doorCurrentlyOpen && engineCurrentlyOn){
				

				
				if(customSensorValue == 0) {
						forwardOrBrakeForce = slideValue * 40;
				} else {
						forwardOrBrakeForce = customSensorValue * 40 * -1;
				}
				

				dragForce = 0.5 * MASS_DENSITY_AIR * pow(currentSpeed, 2) * CAR_FRONT_AREA * CAR_DRAG_COEFFICIENT;
				resultantForce = forwardOrBrakeForce - dragForce;

				
				acceleration = resultantForce / CAR_MASS;
				
				accIn100ms = acceleration / 10;
				
				
				//To allow continued slowing if deceleration is too low.
				if(-0.1 < accIn100ms && accIn100ms < 0){
					accIn100ms = -0.1;
				}
				
				//Don't go into reverse speed if speed is 0
				if(accIn100ms < 0 && currentSpeed <= 0){
					continue;
				}
				
				currentSpeed += accIn100ms;
				
				//Prevent showing of negative values
				if(currentSpeed < 0){
					currentSpeed = 0;
				}
				
			}

		}

	}

void changeInteriorLights(char state){
		B3 = state;
		B4 = state;
		B5 = state;
		write_led();
}

void doorJustClosed(){
	int lightsOnCountDown;
	
	int ticksAtEachBrightness = 0;
	
	openToClose = 0;
	changeInteriorLights(1);
	
	//10 seconds. We have a short delay so the doorCurrentlyOpen and engineCurrentlyOn can react faster
	for(lightsOnCountDown = 0; lightsOnCountDown < 100; lightsOnCountDown++){
		if(doorCurrentlyOpen){
			return;
		}
		
		if(engineCurrentlyOn){
				changeInteriorLights(0);
				return;
		}
		os_dly_wait(100);
	}

	brightnessPositionInterior = 0;
	while(1){ 
		
		int onDelay = BRIGHTNESS[brightnessPositionInterior];
		int offDelay = BRIGHTNESS_TOTAL - onDelay;
		
		changeInteriorLights(1);
		os_dly_wait (onDelay);

		changeInteriorLights(0);
		os_dly_wait (offDelay);
		
		ticksAtEachBrightness++;
		
		
		
		if(doorCurrentlyOpen){
			return;
		}
		
		if(ticksAtEachBrightness > 50){
		  brightnessPositionInterior++;	
			
			if(brightnessPositionInterior >= TOTAL_BRIGHTNESS_SETTINGS){
				return;
			}
			ticksAtEachBrightness = 0;
		}			
		
		if(engineCurrentlyOn){
				changeInteriorLights(0);
				return;
		}

		
	}
}



__task void interiorTask(){
	
	
	while(1){
		int doorOpenCountDown;
		os_dly_wait(100);
		if(!engineCurrentlyOn){
		
			if(openToClose && !doorCurrentlyOpen){
				doorJustClosed();
			} else if(doorCurrentlyOpen && potValue < 512 && openToClose == 0){
				changeInteriorLights(1);

			
				for(doorOpenCountDown = 0; doorOpenCountDown < 20; doorOpenCountDown++){
					if(!doorCurrentlyOpen || engineCurrentlyOn){
						break;
					}
					os_dly_wait(1000);
				}
			
				if(engineCurrentlyOn){
					changeInteriorLights(0);
					continue;
				}
				if(doorCurrentlyOpen){
					changeInteriorLights(0);
					openToClose = 1;
				} else {
					doorJustClosed();
				}
				
		
			}
		
		}
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

	printEngineStoppedMessage();

	initMailBoxes();
	
	id_task_headlight = os_tsk_create(headlightBrightness,1);
  id_task_get_adc_and_buttons = os_tsk_create(GET_INPUTS,2);
	id_task_adc_recv = os_tsk_create(ADC_Recv,3);
	id_task_speed = os_tsk_create(speedTask, 4);
	
	
	id_task_interior = os_tsk_create(interiorTask, 5);
	id_task_engine = os_tsk_create(engineChangerTask,6);
	id_task_door = os_tsk_create(doorTask, 7);
	id_task_lcd = os_tsk_create(printLCD, 8);
  
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
