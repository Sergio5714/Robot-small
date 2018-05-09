#include "Commands.h"
extern Command_Struct inputCommand;
extern RobotStatus Robot;

void checkCommandAndExecute()
{
	if (inputCommand.status == 0x00)
	{
		// No command
		return;
	}
	switch (inputCommand.command)
	{
		case ECHO:
		{
			if (inputCommand.numberOfreceivedParams != 0x04)
				break;
			if (!((inputCommand.params[0] == 'E') && (inputCommand.params[1] == 'C') 
				&& (inputCommand.params[2] == 'H') && (inputCommand.params[3] == 'O')))
				break;
			uint8_t* answer = (uint8_t*)&"ECHO";
			sendAnswer(inputCommand.command, answer, 0x04);
			break;
		}	
		case SET_PWM:
		{
			if (inputCommand.numberOfreceivedParams != 0x05)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			// Set pwm
			timPwmChangeDutyCycle(motorPwmCh[motorNumber].timModule,motorPwmCh[motorNumber].channel, *(__packed float*)(inputCommand.params +1));
			break;
		}
		case SET_DIR_BIT:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_HIGH);
			break;
		}
		case CLEAR_DIR_BIT:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Get motor Number and shift it
			uint8_t  motorNumber = inputCommand.params[0];
			motorNumber = motorNumber - 1;
			
			gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_LOW);
			break;
		}
		case SET_ALL_MOTOR_SPEEDS:
		{
			// Check if there are at least one motor
			if (inputCommand.numberOfreceivedParams < 0x05)
				break;
			uint8_t numberOfMotors = inputCommand.params[0];
			if (inputCommand.numberOfreceivedParams != (0x01 + 0x04*numberOfMotors))
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			uint8_t motorNumber;
			for (motorNumber = 0x01; motorNumber <= numberOfMotors; motorNumber++)
			{
				// Set speed for all motors
				// Use __packed to avoid problem with data alignment
				setMotorSpeed(motorNumber, *(__packed float*)(inputCommand.params + 0x01 + 0x04*(motorNumber - 0x01)));
			}
			
			break;
		}
		case GET_ALL_WHEELS_SPEEDS:
		{
			// Check if number of wheels was received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t numberOfMotors = inputCommand.params[0];
			// Check if number of wheels exceeded real number or not
			if (numberOfMotors > ROBOT_NUMBER_OF_MOTORS)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float speedBuf[numberOfMotors];
			uint8_t i;
			for (i = 0x00; i < numberOfMotors; i++ )
			{
				speedBuf[i] = wheelsSpeed[i];
			}
			// Send Answer
			sendAnswer(inputCommand.command, (__packed uint8_t*)speedBuf, 0x04*numberOfMotors);
			break;
		}
		case SET_SPEED_ROBOT_CS1:
		{
			// Check if 12 bytes (= 3 float) was received
			if (inputCommand.numberOfreceivedParams != 0x0C)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			// Copy target speeds
			robotTargetSpeedCs1[0] = *(__packed float*)(inputCommand.params);
			robotTargetSpeedCs1[1] = *(__packed float*)(inputCommand.params + 0x04);
			robotTargetSpeedCs1[2] = *(__packed float*)(inputCommand.params + 0x08);
			break;
		}
		case GET_SPEED_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float speedBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				speedBuf[i] = robotSpeedCs1[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)speedBuf, 0x0C);
			break;
		}
		case GET_COORD_ROBOT_CS1:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float coordBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				coordBuf[i] = robotCoordCs1[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)coordBuf, 0x0C);
			break;
		}
		case SET_ANGLE_DNMX:
		{
			if (inputCommand.numberOfreceivedParams != 0x03)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			
			// Set angle for particular servo motor
			setServoAngle(inputCommand.params[0], *(uint16_t*)(inputCommand.params + 1));
			break;
		}
		case GET_ANGLE_DNMX:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			float answerFloat;
			// Get angle from particular servo motor
			getServoAngle(inputCommand.params[0], &answerFloat);
			// send answer
			sendAnswer(inputCommand.command, (uint8_t*)&answerFloat, 0x04);
			break;
		}
		case SET_COORD_ROBOT_CS_GLOBAL:
		{
			// Check if 12 bytes (= 3 float) was received
			if (inputCommand.numberOfreceivedParams != 0x0C)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			// Copy target speeds
			robotCoordCsGlobal[0] = *(__packed float*)(inputCommand.params);
			robotCoordCsGlobal[1] = *(__packed float*)(inputCommand.params + 0x04);
			robotCoordCsGlobal[2] = *(__packed float*)(inputCommand.params + 0x08);
			break;
		}
		case GET_COORD_ROBOT_CS_GLOBAL:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// Save current coord to send data correctly (process can be interrupted and values can be changed)
			float coordBuf[3];
			uint8_t i;
			for (i = 0x00; i < 0x03; i++ )
			{
				coordBuf[i] = robotCoordCsGlobal[i];
			}
			// Send Answer of 4 float (12 bytes)
			sendAnswer(inputCommand.command, (__packed uint8_t*)coordBuf, 0x0C);
			break;
		}
		case TURN_FORW_KIN_ON_OFF:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			if (inputCommand.params[0] > 0x00)
			{
				Robot.forwardKinCalcStatusFlag = 0x01;
			}
			else
			{
				Robot.forwardKinCalcStatusFlag = 0x00;
			}
			break;
		}
		case GET_ODOMETRY_MOVEMENT_STATUS:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			uint8_t buf = Robot.odometryMovingStatusFlag;
			sendAnswer(inputCommand.command, (uint8_t*)&buf, 0x01);
			break;
		}
		case GET_STARTUP_STATUS:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// If startup flag has not been requested before check pin
			if (!Robot.startupStatusFlag)
			{
				// Read level on startup pin
				GPIO_Level_TypeDef level;
				level = gpioPinReadInput(EXTI_STARTUP_PORT, EXTI_STARTUP_PIN);
				if (level == GPIO_LEVEL_LOW)
				{
					// Match started
					timeOfStart = getLocalTime();
					Robot.startupStatusFlag = 0x01;
				}
			}
			sendAnswer(inputCommand.command, (uint8_t*)&Robot.startupStatusFlag, 0x01);
			break;
		}
		case FORCED_START:
		{
			// Check if there is no parameters
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			// If startup flag has not been requested before
			if (!Robot.startupStatusFlag)
			{
				timeOfStart = getLocalTime();
				Robot.startupStatusFlag = 0x01;
			}
			sendAnswer(inputCommand.command, (uint8_t*)&Robot.startupStatusFlag, 0x01);
			break;
		}
		case MAKE_FUNNY_ACTION:
		{
			// Check if no parameters  is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t mode = inputCommand.params[0];
			switch (mode)
			{
				case 0x00:
				{
					setSorterHighLevelCommand(CLOSE_ALL_FUNNY_ACTIONS, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x01:
				{
					setSorterHighLevelCommand(OPEN_LATCH, 0x00, &sorterManipulators[0]);
					break;
				}
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case ODOMETRY_MOVEMENT:
		{
			// Check if 6 float numbers or 24 bytes are received
			if (inputCommand.numberOfreceivedParams != 0x18)
				break;
			uint8_t i;
			float distance[3];
			float speed[3];
			float acceleration[3];
			for (i = 0x00; i < 0x03; i++)
			{
				distance[i] = *(__packed float*)(inputCommand.params + 0x04*i);
				speed[i] = *(__packed float*)(inputCommand.params + 0x04*i + 0x0C);
				acceleration[i] = accelerationMax[i];
			}
			// If distance is extremly small apply multiple factor for acceleration x and y
			if (distance[0] <= ODOMETRY_MOVEMENT_SMALL_DIST_THRES)
			{
				acceleration[0] = ODOMETRY_MOVEMENT_SMALL_DIST_ACCEL_FACTOR * acceleration[0];
			}
			if (distance[1] <= ODOMETRY_MOVEMENT_SMALL_DIST_THRES)
			{
				acceleration[1] = ODOMETRY_MOVEMENT_SMALL_DIST_ACCEL_FACTOR * acceleration[1];
			}
			// If odometry movement flag is empty
			if (!Robot.odometryMovingStatusFlag)
			{
				startMovementRobotCs1(&distance[0], &speed[0], &acceleration[0]);
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case GET_SORTER_STATUS:
		{
			// Check if no params have been received
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			uint8_t buf = sorterManipulators[0].subtasksExecutorStatusFlag; 
			// Send status of manipulator
			sendAnswer(inputCommand.command, (uint8_t*)&buf, 0x01);
			break;
		}
		case MOVE_LATCH_OF_SORTER:
		{
			// Check if mode is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t mode = inputCommand.params[0];
			if (mode)
			{
				setSorterHighLevelCommand(OPEN_LATCH, 0x00, &sorterManipulators[0]);
			}
			else
			{
				setSorterHighLevelCommand(CLOSE_LATCH, 0x00, &sorterManipulators[0]);
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case MOVE_HEAP_GRIPPER:
		{
			// Check if mode is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t mode = inputCommand.params[0];
			switch(mode)
			{
				case 0x00:
				{
					setSorterHighLevelCommand(CLOSE_HEAP_GRIPPER, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x01:
				{
					setSorterHighLevelCommand(FIX_HEAP_GRIPPER, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x02:
				{
					setSorterHighLevelCommand(OPEN_HEAP_GRIPPER, 0x00, &sorterManipulators[0]);
					break;
				}
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case MOVE_BOTTOM_SORTER:
		{
			// Check if number of position is received
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			uint8_t numberOfPos = inputCommand.params[0];
			switch(numberOfPos)
			{
				case 0x00:
				{
					setSorterHighLevelCommand(BOTTOM_SORT_BALL_TO_LEFT, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x01:
				{
					setSorterHighLevelCommand(BOTTOM_SORT_BALL_TO_RIGHT, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x02:
				{
					setSorterHighLevelCommand(BOTTOM_SORT_GO_TO_INTERM, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x03:
				{
					setSorterHighLevelCommand(BOTTOM_SORT_RELEASE_BALL_TO_LEFT, 0x00, &sorterManipulators[0]);
					break;
				}
				case 0x04:
				{
					setSorterHighLevelCommand(BOTTOM_SORT_RELEASE_BALL_TO_RIGHT, 0x00, &sorterManipulators[0]);
					break;	
				}
				default:
				break;
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case TURN_ON_AND_OFF_MOTORS:
		{
			// Check if side(left(0) or right(1), mode on(1) or off (0)) is received
			if (inputCommand.numberOfreceivedParams != 0x02)
				break;
			// Choose motor
			if (inputCommand.params[0] == 0x00)
			{
				shooterChooseEnc(CHOSEN_ENCODER_FIRST);
				chosenShooter = CHOSEN_MOTOR_FIRST;
			}
			else if (inputCommand.params[0] == 0x01)
			{
				shooterChooseEnc(CHOSEN_ENCODER_SECOND);
				chosenShooter = CHOSEN_MOTOR_SECOND;
			}
			// Remember current time
			regulationStartTime = getLocalTime();
			// Choose mode
			switch(inputCommand.params[1])
			{
				case 0x00:
				{
					// turn off motor
					pidRegulator.target = 0.00;
					// Turn off PID
					pidRegulator.pidOn = 0x00;
					break;
				}
				case 0x01:
				{
					// Set target speed
					switch(chosenShooter)
					{
						case CHOSEN_MOTOR_FIRST:
						{
							pidRegulator.target = SHOOTER_MOTOR_SPEED_TO_SHOOT_FIRST;
							break;
						}
						case CHOSEN_MOTOR_SECOND:
						{
							pidRegulator.target = SHOOTER_MOTOR_SPEED_TO_SHOOT_SECOND;
							break;
						}
					}
					// Turn on PID
					pidRegulator.pidOn = 0x01;
					break;
				}
				case 0x02:
				{
					// Set target speed
					switch(chosenShooter)
					{
						case CHOSEN_MOTOR_FIRST:
						{
							pidRegulator.target = SHOOTER_MOTOR_SPEED_TO_SHOOT_DIRTY_FIRST;
							break;
						}
						case CHOSEN_MOTOR_SECOND:
						{
							pidRegulator.target = SHOOTER_MOTOR_SPEED_TO_SHOOT_DIRTY_SECOND;
							break;
						}
					}
					// Turn on PID
					pidRegulator.pidOn = 0x01;
					break;
				}
			}
			// Send answer
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
		case GET_SHOOTER_MOTOR_SPEED:
		{
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			float buf  = shooterMotorSpeed;
			// send answer
			sendAnswer(inputCommand.command, (uint8_t*)&buf, 0x04);
			break;
		}
		case GET_DATA_AND_STATUS_FROM_RF:
		{
			// Check if no params are received
			if (inputCommand.numberOfreceivedParams != 0x00)
				break;
			uint8_t i;
			uint8_t buf[NUMBER_OF_RANGE_FINDERS * 2];
			// Copy data from rangeFinders for collision avoidance
			for (i = 0x00; i < RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR + 0x01; i++)
			{
				buf[i] = rangeFinders.rangeValues[i];
			}
			// Copy status bytes
			for (i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
			{
				buf[i + NUMBER_OF_RANGE_FINDERS] = rangeFinders.errorFlags[i];
			}
			// Send answer
			sendAnswer(inputCommand.command, buf, NUMBER_OF_RANGE_FINDERS * 2);
			break;
		}
		case TURN_COLL_AVOID_ON_OFF:
		{
			if (inputCommand.numberOfreceivedParams != 0x01)
				break;
			if (inputCommand.params[0] > 0x00)
			{
				Robot.collisionAvoidanceStatusFlag = 0x01;
			}
			else
			{
				Robot.collisionAvoidanceStatusFlag = 0x00;
			}
			uint8_t* answer = (uint8_t*)&"OK";
			sendAnswer(inputCommand.command, answer, 0x02);
			break;
		}
	}

	// Command is already executed
	inputCommand.status = 0x00;
	inputCommand.command = 0x00;
	return;
}
