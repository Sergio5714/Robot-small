#include "Interrupts.h"
extern RobotStatus Robot;
extern I2C_Module_With_State_Typedef I2CModule;
extern Pid_Regulator_Struct_Typedef pidRegulator;
extern Chosen_Motor_Typedef chosenShooter;

// Local time of Robot's operation in ms
uint32_t timeMilliseconds = 0x00;

// Interrupt handler for motor control
void TIM6_DAC_IRQHandler(void)
{
	if (MOTOR_CONTROL_TIM_MODULE->SR & TIM_SR_UIF)
	{
		timClearStatusRegisterFlag(MOTOR_CONTROL_TIM_MODULE, TIM_SR_UIF);
		
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
			// If there is no odometry movement make decceleraton for all speeds 
//			if (Robot.movingStatusFlag)
//			{
//				// TBD
//			}
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
		shooterSetdutyCycle(chosenShooter, pidRegulator.output);
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
		// Set low level on ch1
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
		
		// Increase absolute time in milliseconds
		timeMilliseconds ++;
	}
	 return;
}

// Interrupt handler for external startup interrupt
void EXTI1_IRQHandler(void)
{
	// if startup switch is source
	if(EXTI->PR & (0x01 << EXTI_STARTUP_PIN)) 
	{  
		// Clear Interrupt flag
		EXTI->PR |= 0x01 << EXTI_STARTUP_PIN;
		
		// Change status of startup flag
		Robot.startupInterruptStatusFlag = 0x01;
	}
}

//--------------------------------------------- Some funtions for local time calculations --------------------------------------//
uint32_t getLocalTime()
{
	return timeMilliseconds;
}
uint8_t checkTimeout(uint32_t startTime, uint32_t timeout)
{
	if (timeMilliseconds >= startTime)
	{
		if((timeMilliseconds - startTime) >= timeout)
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
	else
	{
		if((timeMilliseconds + 0xFFFF - startTime) >= timeout)
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
}
void delayMs(uint16_t delay)
{
	uint32_t startTime = getLocalTime();
	while(!checkTimeout(startTime, delay))
	{
		
	}
	return;
}
