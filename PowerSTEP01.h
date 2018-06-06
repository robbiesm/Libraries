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
#define DD_SC0	DDB0
#define SC0		PORTB0

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
	#ifdef Arduino_h
	#define DDR_SPI DDRB
	#define PORT_SPI PORTB
	#define DD_MOSI	DDB2
	#define MOSI	PORTB2
	#define DD_MISO	DDB3
	#define MISO	PORTB3
	#define DD_SCK	DDB1
	#define SCK		PORTB1
	#endif // Arduino_h
#define DD_CS0	DDB0
#define CS0		PORTB0
#define DD_CS1	DDB4
#define CS1		PORTB4
#define DD_CS2	DDB5
#define CS2		PORTB5

#endif

#endif /*PowerSTEP01*/
