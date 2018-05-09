#include "Manipulators.h"
#include "Interrupts.h"

// For ball sorters
Sorter_Manipulator_Typedef sorterManipulators[NUMBER_OF_SORTERS];
Servo_Checker_Typedef servoCheckerSorterManip[NUMBER_OF_SORTERS];

// FOR SORTERS MANIPULATORS
Sorter_Subtasks_Typedef      bottomSorterSortToRightTaskSeq[] = {SUBTASK_BOTTOM_SORTER_SORT_TO_RIGHT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterSortToLeftTaskSeq[] = {SUBTASK_BOTTOM_SORTER_SORT_TO_LEFT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterGoToIntermTaskSeq[] = {SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterReleaseToLeftTaskSeq[] = {SUBTASK_BOTTOM_SORTER_RELEASE_LEFT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      bottomSorterReleaseToRightTaskSeq[] = {SUBTASK_BOTTOM_SORTER_RELEASE_RIGHT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      openLatchTaskSeq[] = {SUBTASK_OPEN_LATCH, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      closeLatchTaskSeq[] = {SUBTASK_CLOSE_LATCH, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      openHeapGripperTaskSeq[] = {SUBTASK_CLOSE_LATCH, SUBTASK_OPEN_HEAP_GRIPPER_LEFT, SUBTASK_OPEN_HEAP_GRIPPER_RIGHT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      fixHeapGripperTaskSeq[] = {SUBTASK_FIX_HEAP_GRIPPER_LEFT, SUBTASK_FIX_HEAP_GRIPPER_RIGHT,
                                                       SUBTASK_MOVE_LATCH_HEAP_GRIPPER, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      closeHeapGripperTaskSeq[] = {SUBTASK_CLOSE_HEAP_GRIPPER_RIGHT, SUBTASK_CLOSE_HEAP_GRIPPER_LEFT, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      closeAllFunnnyActionsTaskSeq[] = {SUBTASK_CLOSE_LATCH, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      initManipulatorTaskSeq[] = {SUBTASK_CLOSE_LATCH, SUBTASK_CLOSE_HEAP_GRIPPER_RIGHT, SUBTASK_CLOSE_HEAP_GRIPPER_LEFT,
                                                        SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS, SUBTASK_SORTER_TERMINATOR};
Sorter_Subtasks_Typedef      sorterTaskTerminator = SUBTASK_SORTER_TERMINATOR;

// Timer interrupt handler for servoChecker
void TIM8_UP_TIM13_IRQHandler(void)
{

	if (SERVO_CHECKER_TIM_MODULE->SR & TIM_SR_UIF)
	{	
		timClearStatusRegisterFlag(SERVO_CHECKER_TIM_MODULE, TIM_SR_UIF);
		// Check manipulators
		execSorterTasks(0x00, &sorterManipulators[0]);
		checkPosServo(&servoCheckerSorterManip[0]);
	}
	return;
}

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void)
{
	sorterManipulators[0].bottomSorter.id = SORTER_SERVO_BOTTOM_ID;
	sorterManipulators[0].bottomSorter.rightPos = SORTER_SERVO_BOTTOM_RIGHT_POS;
	sorterManipulators[0].bottomSorter.leftPos = SORTER_SERVO_BOTTOM_LEFT_POS;
	sorterManipulators[0].bottomSorter.intermPos = SORTER_SERVO_BOTTOM_INTERM_POS;
	sorterManipulators[0].bottomSorter.releaseLeftPos = SORTER_SERVO_BOTTOM_RELEASE_LEFT_POS;
	sorterManipulators[0].bottomSorter.releaseRightPos = SORTER_SERVO_BOTTOM_RELEASE_RIGHT_POS;
	sorterManipulators[0].latch.id = SORTER_SERVO_LATCH_ID;
	sorterManipulators[0].latch.openedAngle = SORTER_SERVO_LATCH_OPENED_POS;
	sorterManipulators[0].latch.closedAngle = SORTER_SERVO_LATCH_CLOSED_POS;
	sorterManipulators[0].latch.slightlyOpenedAngle = SORTER_SERVO_LATCH_HEAP_GRIPPER_POS;
	sorterManipulators[0].tasksSequencePtr = &sorterTaskTerminator;
	sorterManipulators[0].heapGripperRight.id = SORTER_SERVO_HEAP_GRIPPER_RIGHT_ID;
	sorterManipulators[0].heapGripperRight.openedAngle = SORTER_SERVO_HEAP_GRIPPER_RIGHT_OPENED_POS;
	sorterManipulators[0].heapGripperRight.closedAngle = SORTER_SERVO_HEAP_GRIPPER_RIGHT_CLOSED_POS;
	sorterManipulators[0].heapGripperRight.slightlyOpenedAngle = SORTER_SERVO_HEAP_GRIPPER_RIGHT_FIXED_POS;
	sorterManipulators[0].heapGripperLeft.id = SORTER_SERVO_HEAP_GRIPPER_LEFT_ID;
	sorterManipulators[0].heapGripperLeft.openedAngle = SORTER_SERVO_HEAP_GRIPPER_LEFT_OPENED_POS;
	sorterManipulators[0].heapGripperLeft.closedAngle = SORTER_SERVO_HEAP_GRIPPER_LEFT_CLOSED_POS;
	sorterManipulators[0].heapGripperLeft.slightlyOpenedAngle = SORTER_SERVO_HEAP_GRIPPER_LEFT_FIXED_POS;
	
	// Turn bottom sorter into start position
	setSorterHighLevelCommand(INIT_MANIPULATOR, 0x00, &sorterManipulators[0]);
	
	uint32_t startTime = getLocalTime();
	
	while(!checkTimeout(startTime, MANIPULATOR_INIT_TIMEOUT_TENTH_OF_MS))
	{
		// Check status
		if ((sorterManipulators[0].subtasksExecutorStatusFlag != TASKS_EXECUTOR_SUCCESFUL_EXECUTION)
			&& (sorterManipulators[0].subtasksExecutorStatusFlag != TASKS_EXECUTOR_ACTIVE_MODE))
		{
			// Show error
			showError();
			return;
		}
	}
	// No error
	showNoError();
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
		else if (checkTimeout(servoChecker->startTime, SERVO_CHECKER_TIMEOUT_TENTH_OF_MS))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_WRONG_POSITION;
			return;
		}
	}
	return;
}

void resetChecker(Servo_Checker_Typedef* servoChecker)
{
	servoChecker->startTime = 0x00;
	servoChecker->servoId = 0x00;
	servoChecker->targetPos = 0.0f;
	servoChecker->statusFlag = SERVO_CHECKER_WAITING_MODE;
	return;
}
//--------------------------------------------- Tasks executor's functions -------------------------------------//
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
		case SUBTASK_MOVE_LATCH_HEAP_GRIPPER:
			servoId  = sorter->latch.id;
			servoTargetPos = sorter->latch.slightlyOpenedAngle;
			break;
		case SUBTASK_OPEN_HEAP_GRIPPER_RIGHT:
			servoId  = sorter->heapGripperRight.id;
			servoTargetPos = sorter->heapGripperRight.openedAngle;
			break;
		case SUBTASK_FIX_HEAP_GRIPPER_RIGHT:
			servoId  = sorter->heapGripperRight.id;
			servoTargetPos = sorter->heapGripperRight.slightlyOpenedAngle;
			break;
		case SUBTASK_CLOSE_HEAP_GRIPPER_RIGHT:
			servoId  = sorter->heapGripperRight.id;
			servoTargetPos = sorter->heapGripperRight.closedAngle;
			break;
		case SUBTASK_OPEN_HEAP_GRIPPER_LEFT:
			servoId  = sorter->heapGripperLeft.id;
			servoTargetPos = sorter->heapGripperLeft.openedAngle;
			break;
		case SUBTASK_FIX_HEAP_GRIPPER_LEFT:
			servoId  = sorter->heapGripperLeft.id;
			servoTargetPos = sorter->heapGripperLeft.slightlyOpenedAngle;
			break;
		case SUBTASK_CLOSE_HEAP_GRIPPER_LEFT:
			servoId  = sorter->heapGripperLeft.id;
			servoTargetPos = sorter->heapGripperLeft.closedAngle;
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
	servoCheckerSorterManip[numberOfSorter].startTime = getLocalTime();
	// Extend timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
//--------------------------------------------- High-level command assignment ----------------------------------//
void setSorterHighLevelCommand(Sorter_Command_Typedef command, uint8_t numberOfSorter, Sorter_Manipulator_Typedef* sorter)
{
if (numberOfSorter > NUMBER_OF_SORTERS)
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
		case CLOSE_ALL_FUNNY_ACTIONS:
			sorter->tasksSequencePtr = closeAllFunnnyActionsTaskSeq;
			break;
		case CLOSE_HEAP_GRIPPER:
			sorter->tasksSequencePtr = closeHeapGripperTaskSeq;
			break;
		case OPEN_HEAP_GRIPPER:
			sorter->tasksSequencePtr = openHeapGripperTaskSeq;
			break;
		case FIX_HEAP_GRIPPER:
			sorter->tasksSequencePtr = fixHeapGripperTaskSeq;
			break;
		case INIT_MANIPULATOR:
			sorter->tasksSequencePtr = initManipulatorTaskSeq;
			break;
	}
	// Set flag for task sequence execution
	sorter->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ACTIVE_MODE;
	// Turn timer on
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
