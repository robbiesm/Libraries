/*
 * powerSTEP01.h
 *
 * Created: 23/03/2018 9:21:44
 *  Author: Robbie
 */ 


#ifndef POWERSTEP01_H_
#define POWERSTEP01_H_

#include <avr/io.h>
#include "../SPI/SPI.h"
#include "powerSTEP01_Constants.h"
#include <util/delay.h>

#if defined(__AVR_ATmega328P__)
#define DD_reset	DDB0
#define reset		PORTB0
#define CSMotor CS0
#define	CSM_High (PORTB |= (1<<CSMotor))
#define	CSM_Low (PORTB &= ~(1<<CSMotor))
#elif defined(__AVR_ATmega32U4__)
#define CSMotor CS0
#define	CSM_High (PORTB |= (1<<CSMotor))
#define	CSM_Low (PORTB &= ~(1<<CSMotor)) 
#endif

uint8_t SPIXfer(uint8_t data);
uint8_t getStatus(void);

void motorControl_Init(void);
void powerSTEP01_GPIO_Init(void);
void releaseReset(void);

void configStepMode(uint8_t stepMode);

void setSlewRate(uint8_t slewRate);
void setRunKval(uint8_t kval);
void setHoldKval(uint8_t kval);
void setAccKval(uint8_t kval);
void setDecKval(uint8_t kval);
void setOCD_TH(uint8_t OCD);
void setOCShutdown(uint8_t OCShutdown);
void setPWMFreq(uint8_t divisor, uint8_t multiplier);
void setVoltageComp(int vsCompMode);
void setSwitchMode(int switchMode);
void setOscMode(int oscillatorMode);

uint8_t getRunKval(void);
uint8_t getHoldKval(void);
uint8_t getAccKval(void);
uint8_t getDecKval(void);
uint8_t getOCD_TH(void);

void setParam(uint8_t deviceId, uint8_t param, uint32_t data);
long getParam(uint8_t param);
long paramHandler(uint8_t param, uint32_t value);

unsigned long maxSpdCalc (unsigned long stepsPerSec);

/************************************************************************/
/* Parameter options*/                                                                     
/************************************************************************/

// Slew rate options, GATECFG1 7:5 = Igate, GATECFG1 4:0 = Tcc,
// see datasheet tables 11, 34, 35
#define SR_114V_us              0x0040 | 0x0018  // 8mA | 3125ns = 114V/us
#define SR_220V_us              0x0060 | 0x000C  // 16mA | 1625ns = 220V/us
#define SR_400V_us              0x0080 | 0x0007  // 24mA | 1000ns = 400V/us
#define SR_520V_us              0x00A0 | 0x0006  // 32mA | 875ns = 520V/us
#define SR_790V_us              0x00C0 | 0x0003  // 64mA | 500ns = 790V/us
#define SR_980V_us              0x00D0 | 0x0002  // 96mA | 275ns = 980V/us

// Overcurrent bridge shutdown options
#define OC_SD_DISABLE           0x0000  // Bridges do NOT shutdown on OC detect
#define OC_SD_ENABLE            0x0080  // Bridges shutdown on OC detect

// Voltage compensation settings. See p 34 of datasheet.
#define VS_COMP_DISABLE         0x0000  // Disable motor voltage compensation.
#define VS_COMP_ENABLE          0x0020  // Enable motor voltage compensation.

// External switch input functionality.
#define SW_HARD_STOP            0x0000 // Default; hard stop motor on switch.
#define SW_USER                 0x0010 // Tie to the GoUntil and ReleaseSW
//  commands to provide jog function.
//  See page 25 of datasheet.

#endif /* POWERSTEP01_H_ */