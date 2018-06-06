/*
 * PowerSTEP01.h
 *
 * Created: 6/06/2018 13:16:55
 *  Author: Vincent Agemans & Robbie Smedts
 *
 *
 *
 */ 
#ifndef PowerSTEP01_H_
#define PowerSTEP01_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#if defined(__AVR_ATmega2560__)

#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_MOSI	DDB2
#define DD_SCK	DDB1
#define DD_CS0	DDB0
#define CS0		PORTB0
#define LEDinit DDRB |= (1<<DDB7)

#elif defined(__AVR_ATmega328P__)

#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_MOSI	DDB3
#define MOSI	PORTB3
#define DD_MISO	DDB3
#define DD_SCK	DDB5
#define SCK		PORTB5
#define DD_CS0	DDB2
#define CS0		PORTB2

#elif defined(__AVR_ATmega32U4__)

#define DDR_SPI DDRB
#define PORT_SPI PORTB
/************************************************************************/
/* SPI Pin definitions for non-Arduino IDE                              */
/************************************************************************/
#ifndef Arduino_h
	#define DD_MOSI	DDB2
	#define MOSI	PORTB2
	#define DD_MISO	DDB3
	#define MISO	PORTB3
	#define DD_SCK	DDB1
	#define SCK		PORTB1
#endif // Arduino_h
/************************************************************************/
/* PowerSTEP01 motordriver Pin definitions                             */
/************************************************************************/
#define DD_CS0		DDB0
#define CS0			PORTB0
#define DDR_Flag	DDRC
#define DD_Flag		DDC6
#define PORT_Flag	PORTC
#define Flag		PORTC6
#define DDR_Reset	DDRE
#define DD_Reset	DDE6
#define PORT_Reset	PORTE
#define Reset		PORTE6
/************************************************************************/
/* Relaisdriver Pin definitions                                       */
/************************************************************************/
#define DD_CS1	DDB4
#define CS1		PORTB4
/************************************************************************/
/* FRAM Pin definitions                                                 */
/************************************************************************/
#define DD_CS2		DDB5
#define CS2			PORTB5
#define DD_HOLD		DDB6
#define HOLD		PORTB6

#endif

#define PIN_HIGH(x, y)	x |= (1<<y)
#define PIN_LOW(x, y)	x &= ~(1<<y)
#define N_ELEMENTS(x)	(sizeof(x)/sizeof(x[0]))

#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>
#include <FRAM.h>
#include <PowerSTEP01_Constants.h>
#include <PowerSTEP01functions.h>

/************************************************************************/
/* function prototypes                                                  */
/************************************************************************/
void IOinit(void);

/************************************************************************/
/* Function(s)                                                          */
/************************************************************************/
void IOinit(void)
{
	DDR_SPI |= (1<<DD_CS0) | (1<<DD_CS1) | (1<<DD_CS2) | (1<<DD_HOLD);
	PORT_SPI |= (1<<CS0) | (1<<CS1) | (1<<CS2) | (1<<HOLD);
	
	DDR_Flag &= ~(1<<DD_Flag);
	PORT_Flag |= (1<<Flag);
	
	DDR_Reset |= (1<<DD_Reset);
	PORT_Reset &= ~(1<<Reset); 
}
#endif /*PowerSTEP01*/
