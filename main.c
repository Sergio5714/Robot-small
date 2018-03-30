#include "stm32f4xx.h"
#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"
#include "Collision_avoidance.h"


extern RobotStatus Robot;
extern I2C_Module_With_State_Typedef I2CModule;


extern uint32_t timeMilliseconds;
extern uint32_t timeOfLastI2CResetMillis;


uint32_t numberOfReceivedPackages;
uint32_t numberOfChecksumErrors;
uint32_t numberOfSmallLengthErrors;

uint8_t values;

uint8_t range;
uint8_t rangeRaw;


int main(void)
{		
   	boardInitAll();
	initManipulators();
	Robot.forwardKinCalcStatusFlag = 0x01;
	
	//initAllRangefinders();
	//rangeFinderInitContiniousInterruptMode(RANGEFINDER_DEFAULT_ADDR);
	//startContiniousMeasurements(RANGEFINDER_DEFAULT_ADDR);

	while (1)
	{
		switch(getPackage())
		{
			case SMALL_LENGTH:
				numberOfSmallLengthErrors++;
				break;
			case WRONG_CHECKSUM:
				numberOfChecksumErrors++;
				break;
			case SUCCESFULL_PACKAGE:
				numberOfReceivedPackages++;
			default:
				break;
		};
		checkCommandAndExecute();
	}
}
