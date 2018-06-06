/*
 * powerSTEP01.h
 *
 * Created: 23/03/2018 9:21:44
 *  Author: Robbie
 */ 


#ifndef POWERSTEP01_H_
#define POWERSTEP01_H_

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#define CSMotor		CS0

#include "powerSTEP01_Constants.h"

class PowerSTEP01
{
public:
/************************************************************************/
/* Public functions                                                                     */
/************************************************************************/	
	PowerSTEP01(uint8_t motorNumber);
	void motorControl_Init(uint8_t motor);
	void releaseReset(void);
	uint8_t busyCheck(uint8_t motor);
	uint16_t getStatus(uint8_t motor);
	void setParam(uint8_t motor, uint8_t param, uint32_t data);
	long getParam(uint8_t motor, uint8_t param);
	void flagHandler(void);
	void motorsResetPos(void);
	void setStepMode(uint8_t motor, uint8_t stepMode);
	void setMode(uint8_t motor, uint8_t Mode);
	void setSlewRate(uint8_t motor, uint8_t slewRate);
	void setRunKval(uint8_t motor, uint8_t kval);
	uint8_t getRunKval(uint8_t motor);
	void setHoldKval(uint8_t motor, uint8_t kval);
	uint8_t getHoldKval(uint8_t motor);
	void setAccKval(uint8_t motor, uint8_t kval);
	uint8_t getAccKval(uint8_t motor);
	void setDecKval(uint8_t motor, uint8_t kval);
	uint8_t getDecKval(uint8_t motor);
	void setOCD_TH(uint8_t motor, uint8_t OCD);
	uint8_t getOCD_TH(uint8_t motor);
	void setOCShutdown(uint8_t motor, uint8_t OCShutdown);
	void setPWMFreq(uint8_t motor, uint8_t divisor, uint8_t multiplier);
	void setVoltageComp(uint8_t motor, int vsCompMode);
	void setSwitchMode(uint8_t motor, int switchMode);
	void setOscMode(uint8_t motor, int oscillatorMode);
	void setMaxSpeed(uint8_t motor, unsigned long stepsPerSecond);
	void setAcc(uint8_t motor, unsigned long stepsPerSecondPerSecond);
	void setDec(uint8_t motor, unsigned long stepsPerSecondPerSecond);
	void setSTALL_TH(uint8_t motor, uint8_t STALL);
	void run(uint8_t motor, uint8_t dir, unsigned long stepsPerSec);
	void move(uint8_t motor, uint8_t dir, unsigned long numSteps);
	void softStop(uint8_t motor);
	void hardStop(uint8_t motor);
	void softHiZ(uint8_t motor);
	void hardHiZ(uint8_t motor);
	/*Functions to access all motors at once*/
	void runAll(uint8_t dir, unsigned long stepsPerSec);
	void moveAll(uint8_t dir, unsigned long numSteps);
	void softStopAll(void);
	void hardStopAll(void);
	void softHiZAll(void);
	void hardHiZAll(void);
//protected:
private:
/************************************************************************/
/* Private variables                                                                     */
/************************************************************************/
	static volatile uint8_t _numberOfDevices;
/************************************************************************/
/* Private Functions                                                                     */
/************************************************************************/
	static uint8_t SPIXfer(uint8_t data);
	static uint8_t SPIXferMotors(uint8_t motor, uint8_t data);
	static void SPIXferMotorsAll(uint8_t data1, uint8_t data2, uint8_t data3);
	long paramHandler(uint8_t motor, uint8_t param, uint32_t value);
	
	/*Support functions for converting user units to PowerSTEP units*/
	unsigned long maxSpdCalc (unsigned long stepsPerSec);
	unsigned long spdCalc(unsigned long stepsPerSec);
	unsigned long accCalc(unsigned long stepsPerSecPerSec);
	unsigned long decCalc(unsigned long stepsPerSecPerSec);
};

#endif /* POWERSTEP01_H_ */