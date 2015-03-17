#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"

unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7
OS_TID task_id_main;


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

__task void task_func_main(void) {
  int state = -1;
  const unsigned int taskperiod = 200; // Copy task period value in milliseconds here
  os_itv_set(taskperiod);
	while(1){
		write_led();
		LCD_cls();
		LCD_puts("group5");
		os_itv_wait();
   } // while (1)
}




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
	
	task_id_main = os_tsk_create(task_func_main,0);

  os_tsk_delete_self ();
}


int main (void) {

  os_sys_init (init);                    /* Initialize RTX and start init    */
  return 0;
}