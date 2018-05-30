#include "Interrupts.h"
extern RobotStatus Robot;
extern I2C_Module_With_State_Typedef I2CModule;

extern Pid_Regulator_Struct_Typedef pidRegulator;
extern Chosen_Motor_Typedef chosenShooter;

// Local time of Robot's operation in ms
uint32_t timeInOneTenthOfMillisecond = 0x00;

// Interrupt handler for motor control
void TIM6_DAC_IRQHandler(void)
{
	if (MOTOR_CONTROL_TIM_MODULE->SR & TIM_SR_UIF)
	{
		timClearStatusRegisterFlag(MOTOR_CONTROL_TIM_MODULE, TIM_SR_UIF);
		
		// Disable interrupt of servo checkers and collision avoidance to prevent interference
		__NVIC_DisableIRQ(SERVO_CHECKER_IRQN);
		__NVIC_DisableIRQ(COLL_AVOID_IRQN);
		
		// Read data from Encoders (Encoders -> wheelsSpeed (+ wheelsCoord) -> robotSpeedCs1 (+ robotCoordCs1) )
		readEnc();
		
		// Calculate global speed and coordinate (robotSpeedCs1 -> robotSpeedCsGlobal (+ robotCoordCsGlobal))
		calcGlobSpeedAndCoord();
		
		// Update robot status
		updateRobotStatus();
		
		// Odometry Movement
		if (Robot.odometryMovingStatusFlag)
		{
			// Check if we reached target position or not
			checkIfPositionIsReached();
			
			// Calculate speed for current moment
			speedRecalculation();
		}
		else
		{
			// TBD
		}
		
		// Collision avoidance
		if (Robot.collisionAvoidanceStatusFlag)
		{
			// Check collision avoidance (correct robotTargetSpeedCs1 if it is necessary)
			checkCollisionAvoidance();
		}
		// Calculation of forward kinematics
		if (Robot.forwardKinCalcStatusFlag)
		{
			// Calculate Forward kinematics ( robotTargetSpeedCs1 -> robotTargetMotorSpeedCs1)
			calcForwardKin();
			
			// Set speeds for motors (robotTargetMotorSpeedCs1 -> PWM)
			setMotorSpeeds();
		}
		
		// Calculate speed of shooter motor
		shooterReadEnc();
		
		// Calculate pid
		pidCalc(&pidRegulator);
		
		// Set duty cycle
		shooterSetDutyCycle(chosenShooter, pidRegulator.output);
		
		// Enable interrupt of servo checkers and collision avoidance back
		__NVIC_EnableIRQ(SERVO_CHECKER_IRQN);
		__NVIC_EnableIRQ(COLL_AVOID_IRQN);
	}
	return;
}

// Interrupt handler for software PWM for shooter motors
void TIM8_BRK_TIM12_IRQHandler(void)
{
	if (SHOOTER_MOTOR_PWM_TIM_MODULE->SR & TIM_SR_UIF)
	{
		timClearStatusRegisterFlag(SHOOTER_MOTOR_PWM_TIM_MODULE, TIM_SR_UIF);
		// Set high level
		gpioPinSetLevel(SHOOTER_MOTOR_CH1_PWM_PORT, SHOOTER_MOTOR_CH1_PWM_PIN, GPIO_LEVEL_HIGH);
		gpioPinSetLevel(SHOOTER_MOTOR_CH2_PWM_PORT, SHOOTER_MOTOR_CH2_PWM_PIN, GPIO_LEVEL_HIGH);
	}
	if (SHOOTER_MOTOR_PWM_TIM_MODULE->SR & TIM_SR_CC1IF)
	{
		timClearStatusRegisterFlag(SHOOTER_MOTOR_PWM_TIM_MODULE, TIM_SR_CC1IF);
		// Set low level on ch1
		gpioPinSetLevel(SHOOTER_MOTOR_CH1_PWM_PORT, SHOOTER_MOTOR_CH1_PWM_PIN, GPIO_LEVEL_LOW);
	}
	if (SHOOTER_MOTOR_PWM_TIM_MODULE->SR & TIM_SR_CC2IF)
	{
		timClearStatusRegisterFlag(SHOOTER_MOTOR_PWM_TIM_MODULE, TIM_SR_CC2IF);
		// Set low level on ch2
		gpioPinSetLevel(SHOOTER_MOTOR_CH2_PWM_PORT, SHOOTER_MOTOR_CH2_PWM_PIN, GPIO_LEVEL_LOW);
	}
	return;
}

// Interrupt handler for I2C errors
void I2C2_ER_IRQHandler()
{
	// Ardbitration lost error
	if (READ_BIT(I2C_MODULE->SR1, I2C_SR1_ARLO))
	{
		// Clear bit
		CLEAR_BIT(I2C_MODULE->SR1, I2C_SR1_ARLO);
		I2CModule.status = I2C_ARBITRATION_LOST_ERROR;
		return;
	}	
	// Acknowledge error
	if (READ_BIT(I2C_MODULE->SR1, I2C_SR1_AF))
	{
		// Clear bit
		CLEAR_BIT(I2C_MODULE->SR1, I2C_SR1_AF);
		I2CModule.status = I2C_ACKNOWLEDGE_ERROR;
		return;
	}
	// Bus error(misplaced stop or start condition)
	if (READ_BIT(I2C_MODULE->SR1, I2C_SR1_BERR))
	{
		// Clear bit
		CLEAR_BIT(I2C_MODULE->SR1, I2C_SR1_BERR);
		I2CModule.status = I2C_BUS_ERROR;
		return;
	}
	return;
}

// Interrupt for Local time timer (that counts in ms)
void TIM7_IRQHandler()
{
	if (LOCAL_TIME_TIM_MODULE->SR & TIM_SR_UIF)
	{
		timClearStatusRegisterFlag(LOCAL_TIME_TIM_MODULE, TIM_SR_UIF);
		
		// Increase absolute time by one tenth of a millisecond
		timeInOneTenthOfMillisecond ++;
	}
	 return;
}

// Interrupt handler for external startup interrupt
//void EXTI1_IRQHandler(void)
//{
//	// if startup switch is source
//	if(EXTI->PR & (0x01 << EXTI_STARTUP_PIN)) 
//	{  
//		// Clear Interrupt flag
//		EXTI->PR |= 0x01 << EXTI_STARTUP_PIN;
//		
//		// Change status of startup flag
//		Robot.startupInterruptStatusFlag = 0x01;
//	}
//}

// Interrupt handler for collision avoidance
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if (COLL_AVOID_TIM_MODULE->SR & TIM_SR_UIF)
	{
		timClearStatusRegisterFlag(COLL_AVOID_TIM_MODULE, TIM_SR_UIF);
		if (getLocalTime() > RANGE_FINDERS_FIRST_READ_DELAY_TENTH_OF_MS)
		{
			// Acquire rangefinder's measurements
			readRangesGlobally();
			postprocessDataForCalibration();
			// Check if initialization is needed
			checkRangeFindersReinitFlags();
		}
		timEnable(COLL_AVOID_TIM_MODULE);
	}
	 return;
}

//--------------------------------------------- Some funtions for local time calculations --------------------------------------//
uint32_t getLocalTime()
{
	return timeInOneTenthOfMillisecond;
}
uint32_t getTimeDifference(uint32_t startTime)
{
	uint32_t diff;
	// turn off IRQN
	__NVIC_DisableIRQ(LOCAL_TIME_IRQN);
	if (timeInOneTenthOfMillisecond >= startTime)
	{
		diff = (timeInOneTenthOfMillisecond - startTime);
	}
	else
	{
		diff =(0xFFFF - startTime) + timeInOneTenthOfMillisecond;

	}
	// turn IRQN back on
	__NVIC_EnableIRQ(LOCAL_TIME_IRQN);
	return diff;
}
uint8_t checkTimeout(uint32_t startTime, uint32_t timeout)
{
	uint32_t diff = getTimeDifference(startTime);
	if(diff >= timeout)
	{
		// Timeout is exceeded
		return 0x01;
	}
	else
	{
		// Timeout is not exceeded
		return 0x00;	
	}
}
void delayInTenthOfMs(uint16_t delay)
{
	uint32_t startTime = getLocalTime();
	while(!checkTimeout(startTime, delay))
	{
		
	}
	return;
}
