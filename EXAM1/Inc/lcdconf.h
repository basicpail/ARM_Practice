/*! \file lcdconf.h \brief Character LCD driver configuration. */
//*****************************************************************************
//
// File Name	: 'lcdconf.h'
// Title		: Character LCD driver for HD44780/SED1278 displays
//					(usable in mem-mapped, or I/O mode)
// Author		: Pascal Stang - Copyright (C) 2000-2002
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

#ifndef LCDCONF_H
#define LCDCONF_H

#include "stm32f4xx.h"

// Define type of interface used to access the LCD
// LCD_PORT_INTERFACE:
//		This mode allows you to connect the control and data lines of the LCD
//		directly to the I/O port pins (no interfacing hardware is needed),
//		but it generally runs slower than the LCD_MEMORY_INTERFACE.
//		Depending on your needs, when using the LCD_PORT_INTERFACE, the LCD may
//		be accessed in 8-bit or 4-bit mode.  In 8-bit mode, one whole I/O port
//		(pins 0-7) is required for the LCD data lines, but transfers are faster.
//		In 4-bit mode, only I/O port pins 4-7 are needed for data lines, but LCD
//		access is slower.  In either mode, three additional port pins are
//		required for the LCD interface control lines (RS, R/W, and E).

// Enable one of the following interfaces to your LCD

#ifndef LCD_CTRL_PORT
	// port and pins you will use for control lines
	#define LCD_CTRL_PORT_CLK       RCC_AHB1Periph_GPIOB
	#define LCD_CTRL_PORT           GPIOB
	#define LCD_CTRL_RW             GPIO_PIN_0
	#define LCD_CTRL_RS	    	GPIO_PIN_1
	#define LCD_CTRL_E      	GPIO_PIN_2
#endif
#ifndef LCD_DATA_POUT
	// port you will use for data lines
	#define LCD_DATA_PORT_CLK       RCC_AHB1Periph_GPIOC
	#define LCD_DATA_PORT           GPIOC
	// access mode you will use (default is 8bit unless 4bit is selected)
	#define LCD_DATA_4BIT
	#ifdef LCD_DATA_4BIT
	#define LCD_DATA_BUS            0x00F0
	#else
	#define LCD_DATA_BUS            0x00FF
	#endif
#endif


// LCD display geometry
// change these definitions to adapt settings
#define LCD_LINES				2	// visible lines
#define LCD_LINE_LENGTH			16	// line length (in characters)
// cursor position to DDRAM mapping
#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40
#define LCD_LINE2_DDRAMADDR		0x14
#define LCD_LINE3_DDRAMADDR		0x54


#endif
