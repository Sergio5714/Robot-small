#include "Shooter_motors.h"
float shooterMotorSpeed;
Chosen_Motor_Typedef chosenShooter;

Pid_Regulator_Struct_Typedef pidRegulator;

void shooterChooseEnc(Chosen_Encoder_Typedef encoderNumber)
{
	switch(encoderNumber)
	{
		case CHOSEN_ENCODER_FIRST:
		{
			// Turn off second encoder
			gpioPinSetLevel(SHOOTER_ENCODER_SW_SECOND_PORT, SHOOTER_ENCODER_SW_SECOND_PIN, GPIO_LEVEL_LOW);
			// Turn on first encoder
			gpioPinSetLevel(SHOOTER_ENCODER_SW_FIRST_PORT, SHOOTER_ENCODER_SW_FIRST_PIN, GPIO_LEVEL_HIGH);
			break;
		}
		case CHOSEN_ENCODER_SECOND:
		{
			// Turn off first encoder
			gpioPinSetLevel(SHOOTER_ENCODER_SW_FIRST_PORT, SHOOTER_ENCODER_SW_FIRST_PIN, GPIO_LEVEL_LOW);
			// Turn on second encoder
			gpioPinSetLevel(SHOOTER_ENCODER_SW_SECOND_PORT, SHOOTER_ENCODER_SW_SECOND_PIN, GPIO_LEVEL_HIGH);
			break;
		}
	}
	return;
}

void shooterReadEnc()
{
	// Buffer for encoders' ticks
	int16_t encTicksBuf;
	
	// Get data from encoder
	encTicksBuf = *SHOOTER_ENCODER_CNT - ENCODER_CNT_INITIAL_VALUE;
	*SHOOTER_ENCODER_CNT = ENCODER_CNT_INITIAL_VALUE;
	
	// Calculate speeds
	shooterMotorSpeed = encTicksBuf * SHOOTER_TICKS_TO_SPEED_COEF;
	
	// Upload value to pid struct
	pidRegulator.current = shooterMotorSpeed;
	return;
}

void shooterSetDutyCycle(Chosen_Motor_Typedef motorNumber, float dutyCycle)
{
	timPwmChangeDutyCycle(SHOOTER_MOTOR_PWM_TIM_MODULE, motorNumber + 0x01, dutyCycle);
	return;
}
// Init pid regulator
void pidInit(void)
{
	pidRegulator.pk = 0.04f;
	pidRegulator.dk = 0.02f;
	pidRegulator.ik = 0.02f;
	pidRegulator.maxOutput = 0.9;
	pidRegulator.minOutput = 0.0;
	pidRegulator.maxSumError = 60;
	pidRegulator.pidErrorEnd = 10;
	return;
}
// Calculate PID regulator
void pidCalc(Pid_Regulator_Struct_Typedef* pidRegulator)
{
	float error, difError;
	
	// Calculate error, differential error and sum error
	error = pidRegulator->target - pidRegulator->current;
	difError = error - pidRegulator->prevError;
	pidRegulator->prevError = error;
	pidRegulator->sumError += error;

	// Check sum error for saturation
	if (pidRegulator->sumError > pidRegulator->maxSumError)
		pidRegulator->sumError = pidRegulator->maxSumError;
	if (pidRegulator->sumError < -pidRegulator->maxSumError)
		pidRegulator->sumError = -pidRegulator->maxSumError;
	
	// If regulation is on
	if (pidRegulator->pidOn)
	{
		// Calculate output value
		pidRegulator->output = ((float)(pidRegulator->pk * error)+(pidRegulator->ik * pidRegulator->sumError)+(pidRegulator->dk * difError));

		// Check if value exceeds maximum value
		if (pidRegulator->output > pidRegulator->maxOutput)
		{
		  pidRegulator->output = pidRegulator->maxOutput;
		}
		// Check if value exceeds minimum value
		if (pidRegulator->output < pidRegulator->minOutput)
		{
		  pidRegulator->output = 0;
		}

		// If we reached accuracy of regulation that we need
		if ((pidRegulator->output <= pidRegulator->pidOutputEnd) 
			&&(pidRegulator->output >= -pidRegulator->pidOutputEnd) 
			&&(error <= pidRegulator->pidErrorEnd) && (error >= -pidRegulator->pidErrorEnd))
		{
		  pidRegulator->pidFinish = 0x01;
		}
		else
		{
		  pidRegulator->pidFinish = 0x00;
		}
	}
	else
	{
		pidRegulator->output = 0x00;
		pidRegulator->pidFinish = 0x00;
	}
	return;
}
