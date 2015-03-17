// CS5272 Group 5

#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"

unsigned char A0,A1; //A0 - Button 3.5, A1 - Button 3.6
unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7
unsigned int i, counter;
unsigned char SENSOR = 0;
//Function to read input
void read_input()
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

//Function to print an unsigned char value on the LCD Display
void print_uns_char (unsigned char value)
{	 
	int flag = 0,i;
	char c[4];
	do {
	   int rem = value%10;
	   value = value/10;
	   c[flag] = rem+48;
	   flag++;
	}while(value>0);
	for(i=flag-1;i>=0;i--)
		LCD_putc(c[i]);
}


OS_TID Water_Dispenser_controller_id; // Declare variable t_Lighting to store the task id
enum SENSOR_States { TAP_OFF, TAP_ON} SENSOR_State;

int TickFct_Sensor(int state) {
   /*VARIABLES MUST BE DECLARED STATIC*/
/*e.g., static int x = 0;*/
/*Define User Variables and Functions For this State Machine Here.*/
   switch(state) { // Transitions
      case -1:
					state = TAP_OFF;				 
      break;
			case TAP_OFF:
				if (A0) {
					B1=1;
          state = TAP_ON;
					counter =0;
        }
				else 
					state = TAP_OFF;
			break;
      case TAP_ON:
			{
				if (A0) {		 
          state = TAP_ON;
					counter=0;
				}
				else if (counter<50) {//Wait for (50*200ms=)10 seconds
					state=TAP_ON;
					counter++;
				}
				else if (counter==50) {
					B1=0;
					state = TAP_OFF;
				}
			}
			break;
      default:
         state = -1;
      } // Transitions

   switch(state) { // State actions
		case TAP_OFF:
		{
			LCD_on(); // Turn on LCD
			LCD_puts("STATUS          ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts("DISPENSER OFF   ");
			LCD_cur_off ();
		}
		break;
    case TAP_ON:
		{
			LCD_cls();
			LCD_puts("STATUS          ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts("WATER RUNNING         ");
			LCD_cur_off ();
		}
    break;
		default: // ADD default behaviour below
    break;
   } // State actions
   SENSOR_State = state;
   return state;
}

__task void TASK_SENSOR(void) {
  int state = -1;
  const unsigned int taskperiod = 200; // Copy task period value in milliseconds here
  os_itv_set(taskperiod);
	while(1){
		read_input();
		state = TickFct_Sensor(state);
		write_led();
		os_itv_wait();
   } // while (1)
}


/*----------------------------------------------------------------------------
 *        Task 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

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

  //Launch the task in the following manner
  //taskname_id = os_tsk_create (State_Machine_Name, 0);
	counter=0;
  Water_Dispenser_controller_id = os_tsk_create(TASK_SENSOR,0);
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

