/*----------------------------------------------------------------------------
 *      RL-ARM - Library
 *----------------------------------------------------------------------------
 *      Name:    LCD.C
 *      Purpose: LCD module 2x16 driver for ST7066 controller
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"

/* LCD IO definitions */
#define LCD_DATA GPIO8->DR[0xFF<<2]     /* Data bits D0 = P8.0 .. DB7 = P8.7  */
#define LCD_E    GPIO9->DR[0x01<<2]     /* Enable control                     */
#define LCD_RW   GPIO9->DR[0x02<<2]     /* Read/Write control                 */
#define LCD_RS   GPIO9->DR[0x04<<2]     /* Data/Instruction control           */
#define LCD_CTRL GPIO9->DR[0x07<<2]     /* All 3 control lines (E/RW/RS)      */

#define E_ENA    0x01
#define E_DIS    0x00
#define RW_READ  0x02
#define RW_WRITE 0x00
#define RS_DATA  0x04
#define RS_INST  0x00

/* Local variables */
static U32 lcd_ptr;

/* 8 user defined characters to be loaded into CGRAM (used for bargraph) */
static const U8 UserFont[8][8] = {
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 },
  { 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 },
  { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C },
  { 0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E },
  { 0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F },
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
};

/* Local Function Prototypes */
static void delay (U32 cnt);
static void lcd_write (U32 c);
static U32  lcd_rd_stat (void);
static void lcd_wr_cmd (U32 c);
static void lcd_wr_data (U32 d);
static void lcd_wait_busy (void);

/*----------------------------------------------------------------------------
 * LCD Driver Interface Functions
 *---------------------------------------------------------------------------*/


/*--------------------------- delay -----------------------------------------*/

static void delay (U32 cnt) {
  /* Delay in while loop cycles. */

  while (cnt--);
}


/*--------------------------- lcd_write -------------------------------------*/

static void lcd_write (U32 c) {
  /* Write data/command to LCD controller. */

  GPIO8->DDR = 0xFF;
  LCD_RW     = RW_WRITE;
  LCD_DATA   = c;
  LCD_E      = E_ENA;
  delay (10);
  LCD_E      = E_DIS;
  delay (10);
}


/*--------------------------- lcd_rd_stat -----------------------------------*/

static U32 lcd_rd_stat (void) {
  /* Read status of LCD controller (ST7066) */
  U32 stat;

  GPIO8->DDR = 0x00;
  LCD_CTRL   = RW_READ;
  delay (10);
  LCD_E      = E_ENA;
  delay (10);
  stat       = LCD_DATA;
  LCD_E      = E_DIS;
  return (stat);
}


/*--------------------------- lcd_wait_busy ---------------------------------*/

static void lcd_wait_busy (void) {
  /* Wait until LCD controller (ST7066) is busy. */
  U32 stat;

  do {
    stat = lcd_rd_stat ();
  } while (stat & 0x80);                /* Wait for busy flag                */
}


/*--------------------------- lcd_wr_cmd ------------------------------------*/

static void lcd_wr_cmd (U32 c) {
  /* Write command to LCD controller. */

  lcd_wait_busy ();
  LCD_RS = RS_INST;
  lcd_write (c);
}


/*--------------------------- lcd_wr_data -----------------------------------*/

static void lcd_wr_data (U32 d) {
  /* Write data to LCD controller. */

  lcd_wait_busy ();
  LCD_RS = RS_DATA;
  lcd_write (d);
}


/*--------------------------- LCD_init --------------------------------------*/

void LCD_init (void) {
  /* Initialize the ST7066 LCD controller to 4-bit mode. */ 

  GPIO8->DDR  = 0xFF;
  GPIO9->DDR |= 0x07;
  LCD_CTRL    = RS_INST;

  lcd_write (0x38);                     /* Select 8-bit interface            */
  delay (100000);
  lcd_write (0x38);
  delay (10000);
  lcd_write (0x38);

  lcd_write (0x38);                     /* 2 lines, 5x8 character matrix     */
  lcd_wr_cmd (0x0e);                    /* Display ctrl:Disp/Curs/Blnk=ON    */
  lcd_wr_cmd (0x06);                    /* Entry mode: Move right, no shift  */

  LCD_load ((U8 *)&UserFont, sizeof (UserFont));
  LCD_cls ();
  	LCD_puts ("Poten Value = ");
}


/*--------------------------- LCD_load --------------------------------------*/

void LCD_load (U8 *fp, U32 cnt) {
  /* Load user-specific characters into CGRAM */
  U32 i;

  lcd_wr_cmd (0x40);                    /* Set CGRAM address counter to 0    */
  for (i = 0; i < cnt; i++, fp++)  {
    lcd_wr_data (*fp);
  }
}

/*--------------------------- LCD_gotoxy ------------------------------------*/

void LCD_gotoxy (U32 x, U32 y) {
  /* Set cursor position on LCD display. Left corner: 1,1, right: 16,2 */
  U32 c;

  c = --x;
  if (--y) {
    c |= 0x40;
  }
  lcd_wr_cmd (c | 0x80);
  lcd_ptr = y*16 + x;
}


/*--------------------------- LCD_cls ---------------------------------------*/

void LCD_cls (void) {
  /* Clear LCD display, move cursor to home position. */

  lcd_wr_cmd (0x01);
  LCD_gotoxy (1,1);
}


/*--------------------------- LCD_cur_off------------------------------------*/

void LCD_cur_off (void) {
  /* Switch off LCD cursor. */

  lcd_wr_cmd (0x0c);
}


/*--------------------------- LCD_on ------ ---------------------------------*/

void LCD_on (void) {
  /* Switch on LCD and enable cursor. */

  lcd_wr_cmd (0x0e);
}


/*--------------------------- LCD_putc --------------------------------------*/

void LCD_putc (U8 c) { 
  /* Print a character to LCD at current cursor position. */

  if (lcd_ptr == 16) {
    lcd_wr_cmd (0xc0);
  }
  lcd_wr_data (c);
  lcd_ptr++;
}


/*--------------------------- LCD_puts --------------------------------------*/

void LCD_puts (U8 *sp) {
  /* Print a string to LCD display. */

  while (*sp) {
    LCD_putc (*sp++);
  }
}


/*--------------------------- LCD_bargraph ----------------------------------*/

void LCD_bargraph (U32 val, U32 size) {
  /* Print a bargraph to LCD display.  */
  /* - val:  value 0..100 %            */
  /* - size: size of bargraph 1..16    */
  U32 i;

  val = val * size / 20;                /* Display matrix 5 x 8 pixels       */
  for (i = 0; i < size; i++) {
    if (val > 5) {
      LCD_putc (5);
      val -= 5;
    }
    else {
      LCD_putc (val);
      break;
    }
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
