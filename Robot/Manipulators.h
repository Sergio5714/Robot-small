#ifndef MANIPULATORS
#define MANIPULATORS
#include "Robot.h"
#include "Dynamixel_control.h"
#include "stm32f4xx.h"

#define NUMBER_OF_SORTERS                    0x01
#define MANIPULATOR_INIT_TIMEOUT_TENTH_OF_MS 0xEA60  // equals 6000  ms

//--------------------------------------------- Typedefs and enums for tasks executor ---------------------------//

#define TASKS_EXECUTOR_MAX_SEND_RETRIES      0x0A

// Tasks executor's status
typedef enum
{
	TASKS_EXECUTOR_SUCCESFUL_EXECUTION,
	TASKS_EXECUTOR_ACTIVE_MODE,
	TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED,
	TASKS_EXECUTOR_ERROR_WRONG_POSITION,
	TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED,
	TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED,
} Tasks_Executor_Status;

// Enumeration for possible subtasks(sorter)
typedef enum
{
	// For ball's sorter
	SUBTASK_BOTTOM_SORTER_SORT_TO_RIGHT,
	SUBTASK_BOTTOM_SORTER_SORT_TO_LEFT,
	SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS,
	SUBTASK_BOTTOM_SORTER_RELEASE_LEFT,
	SUBTASK_BOTTOM_SORTER_RELEASE_RIGHT,
	SUBTASK_OPEN_LATCH,
	SUBTASK_CLOSE_LATCH,
	SUBTASK_MOVE_LATCH_HEAP_GRIPPER,
	SUBTASK_OPEN_HEAP_GRIPPER_RIGHT,
	SUBTASK_FIX_HEAP_GRIPPER_RIGHT,
	SUBTASK_CLOSE_HEAP_GRIPPER_RIGHT,
	SUBTASK_OPEN_HEAP_GRIPPER_LEFT,
	SUBTASK_FIX_HEAP_GRIPPER_LEFT,
	SUBTASK_CLOSE_HEAP_GRIPPER_LEFT,
	SUBTASK_SORTER_TERMINATOR
} Sorter_Subtasks_Typedef;

//--------------------------------------------- Typedefs and enums for servoChecker timer ----------------------//

#define SERVO_CHECKER_ANGLE_EPS              1.0f
#define SERVO_CHECKER_MAX_READ_REQUESTS      0x0A
// Maximum timeout in ms
#define SERVO_CHECKER_TIMEOUT_TENTH_OF_MS    0x4650  // 1800 ms

// Checker status
typedef enum
{
	SERVO_CHECKER_WAITING_MODE,
	SERVO_CHECKER_ACTIVE_MODE,
	SERVO_CHECKER_SUCCESFUL_CONFIRMATION,
	SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED,
	SERVO_CHECKER_ERROR_WRONG_POSITION,
	SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED,
} Checker_Status_Typedef;

// Checker structure
typedef struct
{
	uint8_t                 servoId;
	uint32_t                startTime;
	float                   targetPos;
	float                   previousPos;
	Checker_Status_Typedef  statusFlag;
} Servo_Checker_Typedef;

//--------------------------------------------- Typedefs for sorter control -------------------------------------//

// Struct for door's parameters
typedef struct
{
	uint8_t id;
	uint16_t openedAngle;
	uint16_t slightlyOpenedAngle;
	uint16_t closedAngle;
} Door_Typedef;

// Struct for top or bottom sorter's parameters
typedef struct
{
	uint8_t id;
	uint16_t leftPos;
	uint16_t rightPos;
	uint16_t intermPos;
	uint16_t releaseLeftPos;
	uint16_t releaseRightPos;
} Sorter_Typedef;

// Struct for cube ball sorter's parameters
typedef struct
{

	Sorter_Typedef                 bottomSorter;
	Door_Typedef                   latch;
	Door_Typedef                   heapGripperRight;
	Door_Typedef                   heapGripperLeft;
	Sorter_Subtasks_Typedef*       tasksSequencePtr;
	Tasks_Executor_Status          subtasksExecutorStatusFlag;
} Sorter_Manipulator_Typedef;

// Sorter's commands
typedef enum
{
	BOTTOM_SORT_BALL_TO_RIGHT,
	BOTTOM_SORT_BALL_TO_LEFT,
	BOTTOM_SORT_GO_TO_INTERM,
	BOTTOM_SORT_RELEASE_BALL_TO_RIGHT,
	BOTTOM_SORT_RELEASE_BALL_TO_LEFT,
	OPEN_LATCH,
	CLOSE_LATCH,
	CLOSE_ALL_FUNNY_ACTIONS,
	CLOSE_HEAP_GRIPPER,
	OPEN_HEAP_GRIPPER,
	FIX_HEAP_GRIPPER,
	INIT_MANIPULATOR,
} Sorter_Command_Typedef;
//--------------------------------------------- Macros for servos ------------------------------------------------//

// FOR SORTER
#define SORTER_SERVO_TOP_ID                         0x01  // 
#define SORTER_SERVO_BOTTOM_ID                      0x03  // 
#define SORTER_SERVO_LATCH_ID                       0x04  // 
#define SORTER_SERVO_HEAP_GRIPPER_RIGHT_ID          0x05  // 
#define SORTER_SERVO_HEAP_GRIPPER_LEFT_ID           0x06  // 

// Boundary angles
#define SORTER_SERVO_BOTTOM_LEFT_POS                0x12C // 300°
#define SORTER_SERVO_BOTTOM_RIGHT_POS               0xBE  // 190°
#define SORTER_SERVO_BOTTOM_INTERM_POS              0xFF  // 255°
#define SORTER_SERVO_BOTTOM_RELEASE_RIGHT_POS       0x21
// 33°
#define SORTER_SERVO_BOTTOM_RELEASE_LEFT_POS        0x78  // 120°
#define SORTER_SERVO_LATCH_OPENED_POS               0x45  // 69°
#define SORTER_SERVO_LATCH_CLOSED_POS               0x8C  // 140°
#define SORTER_SERVO_LATCH_HEAP_GRIPPER_POS         0x0E  // 15°
#define SORTER_SERVO_HEAP_GRIPPER_RIGHT_OPENED_POS  0x82  // 130°
#define SORTER_SERVO_HEAP_GRIPPER_RIGHT_FIXED_POS   0x9B  // 155° 
#define SORTER_SERVO_HEAP_GRIPPER_RIGHT_CLOSED_POS  0xF5  // 245°
#define SORTER_SERVO_HEAP_GRIPPER_LEFT_OPENED_POS   0xAA  // 170°
#define SORTER_SERVO_HEAP_GRIPPER_LEFT_FIXED_POS    0x91  // 145°
#define SORTER_SERVO_HEAP_GRIPPER_LEFT_CLOSED_POS   0x46  // 70°

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void);
static ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle);
static ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle);

//--------------------------------------------- ServoChecker functions -----------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker);
void resetChecker(Servo_Checker_Typedef* servoChecker);

//--------------------------------------------- Tasks executor's functions -------------------------------------//
// For sorter
void execSorterTasks(uint8_t numberOfsorter, Sorter_Manipulator_Typedef* sorter);
void execSorterSubtasks(uint8_t numberOfsorter, Sorter_Manipulator_Typedef* sorter);

//--------------------------------------------- High-level command assignment ----------------------------------//
void setSorterHighLevelCommand(Sorter_Command_Typedef command, uint8_t numberOfSorter, Sorter_Manipulator_Typedef* sorters);
#endif
