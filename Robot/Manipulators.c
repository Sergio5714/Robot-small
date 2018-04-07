#include "Manipulators.h"
#include "Interrupts.h"

// For cube manipulators
Cube_Manipulator_Typedef cubeManipulators[NUMBER_OF_MANIPULATORS];
Servo_Checker_Typedef servoCheckerCubeManip[NUMBER_OF_MANIPULATORS];

// For ball sorters
Sorter_Manipulator_Typedef sorterManipulators[NUMBER_OF_SORTERS];
Servo_Checker_Typedef servoCheckerSorterManip[NUMBER_OF_SORTERS];

// FOR CUBE MANIPULATORS
// Task sequence for taking a cube
Manipulator_Subtasks_Typedef takeCubeTaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR,
                                                  SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR,
                                                  SUBTASK_TERMINATOR};
// Task sequence for taking last cube
Manipulator_Subtasks_Typedef takeLastCubeTaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR,
                                                      SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR_INTERM,
                                                      SUBTASK_TERMINATOR};
// Task sequence for unloading tower
Manipulator_Subtasks_Typedef unloadTowerTaskSeq[] = {SUBTASK_LOWER_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR, 
                                                     SUBTASK_LIFT_MANIPULATOR, SUBTASK_TERMINATOR};
// Task sequence for lifting to intermediate position
Manipulator_Subtasks_Typedef liftToIntermPosTaskSeq[] = {SUBTASK_LIFT_MANIPULATOR_INTERM, SUBTASK_TERMINATOR};

// Task sequence to open the door
Manipulator_Subtasks_Typedef openDoorTaskSeq[] = {SUBTASK_OPEN_DOOR, SUBTASK_TERMINATOR};

// Task sequence to close the door
Manipulator_Subtasks_Typedef closeDoorTaskSeq[] = {SUBTASK_CLOSE_DOOR, SUBTASK_TERMINATOR};

// Task sequence to release magic cube
Manipulator_Subtasks_Typedef releaseMagicCubeTaskSeq[] = {SUBTASK_PUSH_MAGIC_CUBE, SUBTASK_RETURN_MAGIC_CUBE, SUBTASK_TERMINATOR};

Manipulator_Subtasks_Typedef taskTerminator = SUBTASK_TERMINATOR;

// FOR SORTERS MANIPULATORS
Sorter_Subtasks_Typedef      topSorterSortBadBallTaskSeq[]  = {SUBTASK_TOP_SORTER_SORT_BAD_BALL, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      topSorterSortGoodBallTaskSeq[] = {SUBTASK_TOP_SORTER_SORT_GOOD_BALL, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      topSorterGoToIntermTaskSeq[] = {SUBTASK_TOP_SORTER_GO_TO_INTERM_POS, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterSortToRightTaskSeq[] = {SUBTASK_BOTTOM_SORTER_SORT_TO_RIGHT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterSortToLeftTaskSeq[] = {SUBTASK_BOTTOM_SORTER_SORT_TO_LEFT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterGoToIntermTaskSeq[] = {SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterReleaseToLeftTaskSeq[] = {SUBTASK_BOTTOM_SORTER_RELEASE_LEFT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterReleaseToRightTaskSeq[] = {SUBTASK_BOTTOM_SORTER_RELEASE_RIGHT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      openLatchTaskSeq[] = {SUBTASK_OPEN_LATCH, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      closeLatchTaskSeq[] = {SUBTASK_CLOSE_LATCH, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      sorterTaskTerminator = SUBTASK_SORTER_TERMINATOR;

// Timer interrupt handler for servoChecker
void TIM8_UP_TIM13_IRQHandler(void)
{

	if (SERVO_CHECKER_TIM_MODULE->SR & TIM_SR_UIF)
	{	
		timClearStatusRegisterFlag(SERVO_CHECKER_TIM_MODULE, TIM_SR_UIF);
		// Check manipulators
		execManipTasks(0x00, &cubeManipulators[0]);
		checkPosServo(&servoCheckerCubeManip[0]);
		execSorterTasks(0x00, &sorterManipulators[0]);
		checkPosServo(&servoCheckerSorterManip[0]);
	}
	return;
}

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void)
{
	cubeManipulators[0].gripper.id = MANIP_LEFT_SERVO_GRIPPER_ID;
	cubeManipulators[0].gripper.closedAngle = MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[0].gripper.openedAngle = MANIP_LEFT_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[0].slider.id = MANIP_LEFT_SERVO_SLIDER_ID;
	cubeManipulators[0].slider.botPos = MANIP_LEFT_SERVO_SLIDER_BOT_POS;
	cubeManipulators[0].slider.topPos = MANIP_LEFT_SERVO_SLIDER_TOP_POS;
	cubeManipulators[0].slider.intermPos = MANIP_LEFT_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[0].door.id = MANIP_LEFT_SERVO_DOOR_ID;
	cubeManipulators[0].door.closedAngle = MANIP_LEFT_SERVO_DOOR_CLOSED_POS;
	cubeManipulators[0].door.openedAngle  = MANIP_LEFT_SERVO_DOOR_OPENED_POS;
	cubeManipulators[0].magicCube.id = MANIP_LEFT_SERVO_MAGIC_CUBE_ID;
	cubeManipulators[0].magicCube.initialPosition = MANIP_LEFT_SERVO_MAGIC_CUBE_INITIAL_POS;
	cubeManipulators[0].magicCube.finalPosition = MANIP_LEFT_SERVO_MAGIC_CUBE_FINAL_POS;
	cubeManipulators[0].tasksSequencePtr = &taskTerminator;
	
	sorterManipulators[0].topSorter.id = SORTER_SERVO_TOP_ID;
	sorterManipulators[0].topSorter.rightPos = SORTER_SERVO_TOP_RIGHT_POS;
	sorterManipulators[0].topSorter.leftPos = SORTER_SERVO_TOP_LEFT_POS;
	sorterManipulators[0].topSorter.intermPos = SORTER_SERVO_TOP_INTERM_POS;
	sorterManipulators[0].bottomSorter.id = SORTER_SERVO_BOTTOM_ID;
	sorterManipulators[0].bottomSorter.rightPos = SORTER_SERVO_BOTTOM_RIGHT_POS;
	sorterManipulators[0].bottomSorter.leftPos = SORTER_SERVO_BOTTOM_LEFT_POS;
	sorterManipulators[0].bottomSorter.intermPos = SORTER_SERVO_BOTTOM_INTERM_POS;
	sorterManipulators[0].bottomSorter.releaseLeftPos = SORTER_SERVO_BOTTOM_RELEASE_LEFT_POS;
	sorterManipulators[0].bottomSorter.releaseRightPos = SORTER_SERVO_BOTTOM_RELEASE_RIGHT_POS;
	sorterManipulators[0].latch.id = SORTER_SERVO_LATCH_ID;
	sorterManipulators[0].latch.openedAngle = SORTER_SERVO_LATCH_OPENED_POS;
	sorterManipulators[0].latch.closedAngle = SORTER_SERVO_LATCH_CLOSED_POS;
	sorterManipulators[0].tasksSequencePtr = &sorterTaskTerminator;
	return;
}

static ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (setServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}

static ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (getServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}
//--------------------------------------------- servoChecker functions ---------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker)
{
	if (servoChecker->statusFlag == SERVO_CHECKER_ACTIVE_MODE)
	{	
		// buffer for current angle and torgue
		float angle;

		// Read current angle of the servo
		if (!getServoAngleWithRetries(servoChecker->servoId, &angle))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED;
			return;
		}
		
		// Check if servo reached target position
		if (fabs(servoChecker->targetPos - angle) < SERVO_CHECKER_ANGLE_EPS)
		{
			servoChecker->statusFlag = SERVO_CHECKER_SUCCESFUL_CONFIRMATION;
			return;
		}
		else if (checkTimeout(servoChecker->startTimeMillis, SERVO_CHECKER_TIMEOUT_MS))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_WRONG_POSITION;
			return;
		}
	}
	return;
}

void resetChecker(Servo_Checker_Typedef* servoChecker)
{
	servoChecker->startTimeMillis = 0x00;
	servoChecker->servoId = 0x00;
	servoChecker->targetPos = 0.0;
	servoChecker->statusFlag = SERVO_CHECKER_WAITING_MODE;
	return;
}
//--------------------------------------------- Tasks executor's functions -------------------------------------//
void execManipTasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{	
	// Check if task executor is active
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		Checker_Status_Typedef checkerStatusFlag = servoCheckerCubeManip[number].statusFlag;
		switch (checkerStatusFlag)
		{
			case SERVO_CHECKER_WAITING_MODE:
				// First command
				execManipSubtasks(number, manipulator);
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_ACTIVE_MODE:
				// Wait for servo checker
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_SUCCESFUL_CONFIRMATION:
				// Check if all subtasks were executed
				manipulator->tasksSequencePtr++;
				if(manipulator->tasksSequencePtr[0] == SUBTASK_TERMINATOR)
				{
					manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_SUCCESFUL_EXECUTION;
					// Reset servo checker
					resetChecker(&servoCheckerCubeManip[number]);
				}
				else
				{
					// Next subtask
					execManipSubtasks(number, manipulator);
				}
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoCheckerCubeManip[number]);
				break;
			case SERVO_CHECKER_ERROR_WRONG_POSITION:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_WRONG_POSITION;
				// Reset servo checker
				resetChecker(&servoCheckerCubeManip[number]);
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoCheckerCubeManip[number]);
				break;
		}
	}
	return;
}
void execManipSubtasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	// Extract current subtask
	Manipulator_Subtasks_Typedef subtask = *(manipulator->tasksSequencePtr);
	
	// Buffers for servo id, target position and current position
	uint16_t servoTargetPos;
	float servoCurrentPos;
	uint8_t servoId;
	
	// Unload values that corresponds to partivular subtask
	switch (subtask)
	{
		case SUBTASK_OPEN_MANIPULATOR:
			servoId  = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.openedAngle;
			break;
		case SUBTASK_CLOSE_MANIPULATOR:
			servoId  = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.closedAngle;
			break;
		case SUBTASK_LOWER_MANIPULATOR:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.botPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.topPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR_INTERM:
			servoId  = manipulator->slider.id;
			servoTargetPos = manipulator->slider.intermPos;
			break;
		case SUBTASK_OPEN_DOOR:
			servoId  = manipulator->door.id;
			servoTargetPos = manipulator->door.openedAngle;
			break;
		case SUBTASK_CLOSE_DOOR:
			servoId  = manipulator->door.id;
			servoTargetPos = manipulator->door.closedAngle;
			break;
		case SUBTASK_PUSH_MAGIC_CUBE:
			servoId  = manipulator->magicCube.id;
			servoTargetPos = manipulator->magicCube.finalPosition;
			break;
		case SUBTASK_RETURN_MAGIC_CUBE:
			servoId  = manipulator->magicCube.id;
			servoTargetPos = manipulator->magicCube.initialPosition;
			break;
		case SUBTASK_TERMINATOR:
			manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED;
			return;
	}
	// Read current angle of the servo
	if (!getServoAngleWithRetries(servoId, &servoCurrentPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoCheckerCubeManip[number]);
		return;
	}
	// Send set angle command to servo
	if (!setServoAngleWithRetries(servoId, servoTargetPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoCheckerCubeManip[number]);
		return;
	}
	// Reset servo checker
	resetChecker(&servoCheckerCubeManip[number]);
	// Load angle, id, current time into servoChecker and turn it on
	servoCheckerCubeManip[number].servoId = servoId;
	servoCheckerCubeManip[number].targetPos = servoTargetPos;
	servoCheckerCubeManip[number].previousPos = servoCurrentPos;
	servoCheckerCubeManip[number].statusFlag = SERVO_CHECKER_ACTIVE_MODE;
	servoCheckerCubeManip[number].startTimeMillis = getLocalTime();
	// Extend timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}

// FOR SORTER
void execSorterTasks(uint8_t numberOfsorter, Sorter_Manipulator_Typedef* sorter)
{
	// Check if task executor is active
	if (sorter->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		Checker_Status_Typedef checkerStatusFlag = servoCheckerSorterManip[numberOfsorter].statusFlag;
		switch (checkerStatusFlag)
		{
			case SERVO_CHECKER_WAITING_MODE:
				// First command
				execSorterSubtasks(numberOfsorter, sorter);
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_ACTIVE_MODE:
				// Wait for servo checker
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_SUCCESFUL_CONFIRMATION:
				// Check if all subtasks were executed
				sorter->tasksSequencePtr++;
				if(sorter->tasksSequencePtr[0] == SUBTASK_SORTER_TERMINATOR)
				{
					sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_SUCCESFUL_EXECUTION;
					// Reset servo checker
					resetChecker(&servoCheckerSorterManip[numberOfsorter]);
				}
				else
				{
					// Next subtask
					execSorterSubtasks(numberOfsorter, sorter);
				}
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED:
				sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoCheckerSorterManip[numberOfsorter]);
				break;
			case SERVO_CHECKER_ERROR_WRONG_POSITION:
				sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_WRONG_POSITION;
				// Reset servo checker
				resetChecker(&servoCheckerSorterManip[numberOfsorter]);
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED:
				sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoCheckerSorterManip[numberOfsorter]);
				break;
		}
	}
	return;
}
void execSorterSubtasks(uint8_t numberOfSorter, Sorter_Manipulator_Typedef* sorter)
{
	// Extract current subtask
	Sorter_Subtasks_Typedef subtask = *(sorter->tasksSequencePtr);
	
	// Buffers for servo id, target position and current position
	uint16_t servoTargetPos;
	float servoCurrentPos;
	uint8_t servoId;
	
	// Unload values that corresponds to partivular subtask
	switch (subtask)
	{
		case SUBTASK_TOP_SORTER_SORT_BAD_BALL:
			servoId  = sorter->topSorter.id;
			servoTargetPos = sorter->topSorter.rightPos;
			break;
		case SUBTASK_TOP_SORTER_SORT_GOOD_BALL:
			servoId  = sorter->topSorter.id;
			servoTargetPos = sorter->topSorter.leftPos;
			break;
		case SUBTASK_TOP_SORTER_GO_TO_INTERM_POS:
			servoId  = sorter->topSorter.id;
			servoTargetPos = sorter->topSorter.intermPos;
			break;
		case SUBTASK_BOTTOM_SORTER_SORT_TO_RIGHT:
			servoId  = sorter->bottomSorter.id;
			servoTargetPos = sorter->bottomSorter.rightPos;
			break;
		case SUBTASK_BOTTOM_SORTER_SORT_TO_LEFT:
			servoId  = sorter->bottomSorter.id;
			servoTargetPos = sorter->bottomSorter.leftPos;
			break;
		case SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS:
			servoId  = sorter->bottomSorter.id;
			servoTargetPos = sorter->bottomSorter.intermPos;
			break;
		case SUBTASK_BOTTOM_SORTER_RELEASE_LEFT:
			servoId  = sorter->bottomSorter.id;
			servoTargetPos = sorter->bottomSorter.releaseLeftPos;
			break;
		case SUBTASK_BOTTOM_SORTER_RELEASE_RIGHT:
			servoId  = sorter->bottomSorter.id;
			servoTargetPos = sorter->bottomSorter.releaseRightPos;
			break;
		case SUBTASK_OPEN_LATCH:
			servoId  = sorter->latch.id;
			servoTargetPos = sorter->latch.openedAngle;
			break;
		case SUBTASK_CLOSE_LATCH:
			servoId  = sorter->latch.id;
			servoTargetPos = sorter->latch.closedAngle;
			break;
		case SUBTASK_SORTER_TERMINATOR:
			sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED;
			return;
	}
	// Read current angle of the servo
	if (!getServoAngleWithRetries(servoId, &servoCurrentPos))
	{
		sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoCheckerSorterManip[numberOfSorter]);
		return;
	}
	// Send set angle command to servo
	if (!setServoAngleWithRetries(servoId, servoTargetPos))
	{
		sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoCheckerSorterManip[numberOfSorter]);
		return;
	}
	// Reset servo checker
	resetChecker(&servoCheckerSorterManip[numberOfSorter]);
	// Load angle, id, current time into servoChecker and turn it on
	servoCheckerSorterManip[numberOfSorter].servoId = servoId;
	servoCheckerSorterManip[numberOfSorter].targetPos = servoTargetPos;
	servoCheckerSorterManip[numberOfSorter].previousPos = servoCurrentPos;
	servoCheckerSorterManip[numberOfSorter].statusFlag = SERVO_CHECKER_ACTIVE_MODE;
	servoCheckerSorterManip[numberOfSorter].startTimeMillis = getLocalTime();
	// Extend timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
//--------------------------------------------- High-level command assignment ----------------------------------//
void setManipHighLevelCommand(Manipulator_Command_Typedef command, uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	if (number > NUMBER_OF_MANIPULATORS)
	{
		// Wrong manipulator's id
		return;
	}
	//Check if manipulator is still working
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		return;
	}
	// Set Command
	switch(command)
	{
		case TAKE_CUBE_COMMAND:
			manipulator->tasksSequencePtr = takeCubeTaskSeq;
			break;
		case TAKE_LAST_CUBE_COMMAND:
			manipulator->tasksSequencePtr = takeLastCubeTaskSeq;
			break;
		case UNLOAD_TOWER_COMMAND:
			manipulator->tasksSequencePtr = unloadTowerTaskSeq;
			break;
		case LIFT_TO_INTERMEDIATE_POS_COMMAND:
			manipulator->tasksSequencePtr = liftToIntermPosTaskSeq;
			break;
		case OPEN_DOOR_COMMAND:
			manipulator->tasksSequencePtr = openDoorTaskSeq;
			break;
		case CLOSE_DOOR_COMMAND:
			manipulator->tasksSequencePtr = closeDoorTaskSeq;
			break;
		case RELEASE_MAGIC_CUBE_COMMAND:
			manipulator->tasksSequencePtr = releaseMagicCubeTaskSeq;
			break;
	}
	// Set flag for task sequence execution
	manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ACTIVE_MODE;
	// Turn timer on
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}

void setSorterHighLevelCommand(Sorter_Command_Typedef command, uint8_t numberOfSorter, Sorter_Manipulator_Typedef* sorter)
{
if (numberOfSorter > NUMBER_OF_MANIPULATORS)
	{
		// Wrong manipulator's id
		return;
	}
	//Check if manipulator is still working
	if (sorter->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		return;
	}
	// Set Command
	switch(command)
	{
		case TOP_SORT_BAD_BALL:
			sorter->tasksSequencePtr = topSorterSortBadBallTaskSeq;
			break;
		case TOP_SORT_GOOD_BALL:
			sorter->tasksSequencePtr = topSorterSortGoodBallTaskSeq;
			break;
		case TOP_SORT_GO_TO_INTERM:
			sorter->tasksSequencePtr = topSorterGoToIntermTaskSeq;
			break;
		case BOTTOM_SORT_BALL_TO_RIGHT:
			sorter->tasksSequencePtr = bottomSorterSortToRightTaskSeq;
			break;
		case BOTTOM_SORT_BALL_TO_LEFT:
			sorter->tasksSequencePtr = bottomSorterSortToLeftTaskSeq;
			break;
		case BOTTOM_SORT_GO_TO_INTERM:
			sorter->tasksSequencePtr = bottomSorterGoToIntermTaskSeq;
			break;
		case BOTTOM_SORT_RELEASE_BALL_TO_LEFT:
			sorter->tasksSequencePtr = bottomSorterReleaseToLeftTaskSeq;
			break;
		case BOTTOM_SORT_RELEASE_BALL_TO_RIGHT:
			sorter->tasksSequencePtr = bottomSorterReleaseToRightTaskSeq;
			break;
		case OPEN_LATCH:
			sorter->tasksSequencePtr = openLatchTaskSeq;
			break;
		case CLOSE_LATCH:
			sorter->tasksSequencePtr = closeLatchTaskSeq;
			break;
	}
	// Set flag for task sequence execution
	sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ACTIVE_MODE;
	// Turn timer on
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
