/*! \file lcd.c \brief Character LCD driver for HD44780/SED1278 displays. */
//*****************************************************************************
//
// File Name	: 'lcd.c'
// Title		: Character LCD driver for HD44780/SED1278 displays
//					(usable in mem-mapped, or I/O mode)
// Author		: Pascal Stang
// Created		: 11/22/2000
// Revised		: 4/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include "lcd.h"


GPIO_InitTypeDef GPIO_LCD;
/*************************************************************/
/********************** LOCAL FUNCTIONS **********************/
/*************************************************************/
static void lcduDelay(unsigned int time_us)
{
  register unsigned int j;

  for(j = 0; j < 60; j++)
  { asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 5
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 10
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 15
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 20
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 25
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 30
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 35
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 40
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 45
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 50
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 55
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 60
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 65
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 70
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 75
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 80
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 85
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 90
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 95
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 100
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 105
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 110
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 115
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 120
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 125
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 130
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 135
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 140
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 145
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 150
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 155
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");// 160
    asm("NOP");                                            // 161
  }
}

static void lcdmDelay(const uint32_t mCount)
{
  __IO uint32_t mindex = 0; 
  for(mindex = (16100 * mCount); mindex != 0; mindex--);
}

void lcdInitHW(void)
{
  GPIO_InitTypeDef GPIO_LCD;
    // initialize I/O ports
  __HAL_RCC_GPIOB_CLK_ENABLE();                          //stm32f4xx_hal.rcc.h
  GPIO_LCD.Pin = LCD_CTRL_RS | LCD_CTRL_RW | LCD_CTRL_E;
  GPIO_LCD.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_LCD.Pull = GPIO_NOPULL;
  GPIO_LCD.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CTRL_PORT, &GPIO_LCD);
  
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_RS | LCD_CTRL_RW | LCD_CTRL_E, GPIO_PIN_RESET);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_LCD.Pin = LCD_DATA_BUS;
  HAL_GPIO_Init(LCD_DATA_PORT, &GPIO_LCD);

/*  StdPeriph_Driver
// initialize I/O ports
  RCC_AHB1PeriphClockCmd(LCD_CTRL_PORT_CLK, ENABLE);
  GPIO_LCD.GPIO_Pin = LCD_CTRL_RS | LCD_CTRL_RW | LCD_CTRL_E;
  GPIO_LCD.Mode = GPIO_Mode_OUT;
  GPIO_LCD.OType = GPIO_OType_PP;
  GPIO_LCD.Speed = GPIO_Speed_100MHz;
  GPIO_LCD.PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_CTRL_PORT, &GPIO_LCD);

  GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_RS | LCD_CTRL_RW |LCD_CTRL_E );

  RCC_AHB1PeriphClockCmd(LCD_DATA_PORT_CLK, ENABLE);
 
  GPIO_LCD.Pin = LCD_DATA_BUS;
  GPIO_Init(LCD_DATA_PORT, &GPIO_LCD);
*/
}

void lcdControlWrite(uint8_t data) 
{
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_RS | LCD_CTRL_RW, GPIO_PIN_RESET);   // set RS to "control", set R/W to "write"

#ifdef LCD_DATA_4BIT
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET);           // set "E" line	                             
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DATA_PORT, data&LCD_DATA_BUS, GPIO_PIN_SET);// output data, high 4 bits        
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line             
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET); // set "E" line                         
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DATA_PORT, (data<<4)&LCD_DATA_BUS, GPIO_PIN_SET);// output data, low 4 bits                   
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line                          
  #else

  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET);// set "E" line	                             
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DATA_PORT, data, GPIO_PIN_SET); // output data, 8bits
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line                         
  #endif
    
/* StdPeriph_Driver 
  GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_RS | LCD_CTRL_RW);// set RS to "control", set R/W to "write"

  #ifdef LCD_DATA_4BIT
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, data&LCD_DATA_BUS);                      // output data, high 4 bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
    lcduDelay(1);
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, (data<<4)&LCD_DATA_BUS);                   // output data, low 4 bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
  #else
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, data);                                   // output data, 8bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
  #endif
*/
  lcdmDelay(2);                                                          // wait until LCD not busy
}

void lcdDataWrite(uint8_t data) 
{
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_RS, GPIO_PIN_SET);// set RS to "data"
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_RW, GPIO_PIN_RESET);// set R/W to "write"                           

  #ifdef LCD_DATA_4BIT
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET);     // set "E" line
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);    
  HAL_GPIO_WritePin(LCD_DATA_PORT, data&LCD_DATA_BUS, GPIO_PIN_SET);// output data, high 4 bits                    
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line                             
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET); // set "E" line	                            
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DATA_PORT, (data<<4)&LCD_DATA_BUS, GPIO_PIN_SET);// output data, low 4 bits                 
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line                         
  #else
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_SET);// set "E" line	                             
  HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_BUS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DATA_PORT, data, GPIO_PIN_SET); // output data, 8bits
  lcduDelay(1);
  HAL_GPIO_WritePin(LCD_CTRL_PORT, LCD_CTRL_E, GPIO_PIN_RESET);// clear "E" line
                           
  #endif
    
/* StdPeriph_Driver
  GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_RS);                             // set RS to "data"
  GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_RW);                           // set R/W to "write"

  #ifdef LCD_DATA_4BIT
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, data&LCD_DATA_BUS);                      // output data, high 4 bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
    lcduDelay(1);
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, (data<<4)&LCD_DATA_BUS);                   // output data, low 4 bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
  #else
    GPIO_SetBits(LCD_CTRL_PORT, LCD_CTRL_E);	                             // set "E" line
    GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_BUS);
    GPIO_SetBits(LCD_DATA_PORT, data);                                   // output data, 8bits
    lcduDelay(1);
    GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_E);                           // clear "E" line
  #endif
*/
  lcdmDelay(2);                                                          // wait until LCD not busy
}

/*************************************************************/
/********************* PUBLIC FUNCTIONS **********************/
/*************************************************************/

void lcdInit()
{
  // 하드웨어 초기화 
  lcdInitHW(); 
  // LCD function set
  lcdControlWrite(LCD_FUNCTION_DEFAULT);
  // clear LCD
  lcdControlWrite(1<<LCD_CLR);
  lcdmDelay(100);	// wait 100ms
  // set entry mode
  lcdControlWrite(1<<LCD_ENTRY_MODE | 1<<LCD_ENTRY_INC);
  // set display to on
  //lcdControlWrite(1<<LCD_ON_CTRL | 1<<LCD_ON_DISPLAY | 1<<LCD_ON_BLINK);
  lcdControlWrite(1<<LCD_ON_CTRL | 1<<LCD_ON_DISPLAY );
  // move cursor to home
  lcdControlWrite(1<<LCD_HOME);
  // set data address to 0
  lcdControlWrite(1<<LCD_DDRAM | 0x00);
}

void lcdHome(void)
{
  // move cursor to home
  lcdControlWrite(1<<LCD_HOME);
}

void lcdClear(void)
{
  // clear LCD
  lcdControlWrite(1<<LCD_CLR);
}

void lcdGotoXY(uint8_t x, uint8_t y)
{
  uint8_t DDRAMAddr;

  // remap lines into proper order
  switch(y)
  {
    case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
    case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
    case 2: DDRAMAddr = LCD_LINE2_DDRAMADDR+x; break;
    case 3: DDRAMAddr = LCD_LINE3_DDRAMADDR+x; break;
    default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
  }

  // set data address
  lcdControlWrite(1<<LCD_DDRAM | DDRAMAddr);
}

void lcdPrintData(char* data, uint8_t nBytes)
{
  uint8_t i;

  // check to make sure we have a good pointer
  if (!data) return;

  // print data
  for(i=0; i<nBytes; i++)
  {
    lcdDataWrite(data[i]);
  }
}

void lcdPrint(char* data)
{
  uint8_t i;

  // check to make sure we have a good pointer
  if (!data) return;

  // print data
  for(i=0; data[i]!='\0'; i++)
  {
    lcdDataWrite(data[i]);
  }
}
