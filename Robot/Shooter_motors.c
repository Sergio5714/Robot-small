#include "Shooter_motors.h"
float shooterMotorSpeed;
Chosen_Motor_Typedef chosenShooter;

Pid_Regulator_Struct_Typedef pidRegulator;

uint32_t regulationStartTime;

// Time of last odometry data acquisition (tenth of millisec)
uint32_t timeofLastShooterMotorDataAcquisition;
// Last time interval (sec)
float lastTimeIntervalShooterMotorSec;

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
	
	// Calculate time interval between current and last data acquisition
	lastTimeIntervalShooterMotorSec = (float)(getTimeDifference(timeofLastShooterMotorDataAcquisition)) / 10000;
	timeofLastShooterMotorDataAcquisition = getLocalTime();
	
	if (lastTimeIntervalShooterMotorSec == 0.0f)
	{
		return;
	}
	// Calculate speeds
	shooterMotorSpeed = encTicksBuf * SHOOTER_TICKS_TO_RAD_COEF/ lastTimeIntervalShooterMotorSec;
	
	// Upload value to pid struct
	pidRegulator.current = shooterMotorSpeed;
	return;
}

void shooterSetDutyCycle(Chosen_Motor_Typedef motorNumber, float dutyCycle)
{
	// Check speed
	if (fabs(shooterMotorSpeed - pidRegulator.target) > SHOOTER_MOTOR_SPEED_EPS)
	{
		if (checkTimeout(regulationStartTime, SHOOTER_MOTOR_RESP_TIMEOUT_TENTH_OF_MILLIS))
		{
			// Turn off motor and regulation
			timPwmChangeDutyCycle(SHOOTER_MOTOR_PWM_TIM_MODULE, motorNumber + 0x01, 0.0f);
			pidRegulator.target = 0.0f;
			pidRegulator.pidOn = 0x00;
			pidRegulator.sumError = 0.0f;
			return;
		}
	}
	else
	{
		// Everythink is OK
		regulationStartTime = getLocalTime();
	}
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
