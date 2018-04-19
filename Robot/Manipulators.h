#ifndef MANIPULATORS
#define MANIPULATORS
#include "Robot.h"
#include "Dynamixel_control.h"
#include "stm32f4xx.h"

#define NUMBER_OF_MANIPULATORS               0x01
#define NUMBER_OF_SORTERS                    0x01
#define MANIPULATOR_INIT_TIMEOUT_TENTH_OF_MS 0x4E20  // equals 2000  ms

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

// Enumeration for possible subtasks (manipulator)
typedef enum
{
	// For cube manipulator
	SUBTASK_OPEN_MANIPULATOR,
	SUBTASK_CLOSE_MANIPULATOR,
	SUBTASK_LOWER_MANIPULATOR,
	SUBTASK_LIFT_MANIPULATOR,
	SUBTASK_LIFT_MANIPULATOR_INTERM,
	SUBTASK_OPEN_DOOR,
	SUBTASK_CLOSE_DOOR,
	SUBTASK_PUSH_MAGIC_CUBE,
	SUBTASK_RETURN_MAGIC_CUBE,
	SUBTASK_TERMINATOR
} Manipulator_Subtasks_Typedef;

// Enumeration for possible subtasks(sorter)
typedef enum
{
	// For ball's sorter
	SUBTASK_TOP_SORTER_SORT_BAD_BALL,
	SUBTASK_TOP_SORTER_SORT_GOOD_BALL,
	SUBTASK_TOP_SORTER_GO_TO_INTERM_POS,
	SUBTASK_TOP_SORTER_GO_TO_INTERM_GOOD_POS,
	SUBTASK_TOP_SORTER_GO_TO_INTERM_BAD_POS,
	SUBTASK_BOTTOM_SORTER_SORT_TO_RIGHT,
	SUBTASK_BOTTOM_SORTER_SORT_TO_LEFT,
	SUBTASK_BOTTOM_SORTER_GO_TO_INTERM_POS,
	SUBTASK_BOTTOM_SORTER_RELEASE_LEFT,
	SUBTASK_BOTTOM_SORTER_RELEASE_RIGHT,
	SUBTASK_OPEN_LATCH,
	SUBTASK_CLOSE_LATCH,
	SUBTASK_CLOSE_BUTTON_FUNNY_ACTION,
	SUBTASK_OPEN_BUTTON_FUNNY_ACTION,
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
	uint32_t                startTimeMillis;
	float                   targetPos;
	float                   previousPos;
	Checker_Status_Typedef  statusFlag;
} Servo_Checker_Typedef;

//--------------------------------------------- Typedefs for manipulator control --------------------------------//

// Struct for gripper's parameters
typedef struct
{
	uint8_t id;
	uint16_t openedAngle;
	uint16_t closedAngle;
} Gripper_Typedef;

// Struct for magic cube's parameters
typedef struct
{
	uint8_t id;
	uint16_t initialPosition;
	uint16_t finalPosition;
} Magic_Cube_Typedef;

// Struct for sliders's parameters
typedef struct
{
	uint8_t id;
	uint16_t topPos;
	uint16_t botPos;
	uint16_t intermPos;
} Slider_Typedef;

// Struct for door's parameters
typedef struct
{
	uint8_t id;
	uint16_t openedAngle;
	uint16_t closedAngle;
} Door_Typedef;

// Struct for cube manipulators's parameters
typedef struct
{
	Gripper_Typedef               gripper;
	Slider_Typedef                slider;
	Door_Typedef                  door;
	Magic_Cube_Typedef            magicCube;
	Manipulator_Subtasks_Typedef* tasksSequencePtr;
	Tasks_Executor_Status         subtasksExecutorStatusFlag;
	
} Cube_Manipulator_Typedef;

// Manipulator's commands
typedef enum
{
	TAKE_CUBE_COMMAND,
	TAKE_LAST_CUBE_COMMAND,
	UNLOAD_TOWER_COMMAND,
	LIFT_TO_INTERMEDIATE_POS_COMMAND,
	OPEN_DOOR_COMMAND,
	CLOSE_DOOR_COMMAND,
	RELEASE_MAGIC_CUBE_COMMAND,
} Manipulator_Command_Typedef;
//--------------------------------------------- Typedefs for sorter control -------------------------------------//
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
	Sorter_Typedef                 topSorter;
	Sorter_Typedef                 bottomSorter;
	Door_Typedef                   latch;
	Door_Typedef                   buttonServo;
	Sorter_Subtasks_Typedef*       tasksSequencePtr;
	Tasks_Executor_Status          subtasksExecutorStatusFlag;
} Sorter_Manipulator_Typedef;

// Sorter's commands
typedef enum
{
	TOP_SORT_BAD_BALL,
	TOP_SORT_GOOD_BALL,
	TOP_SORT_GO_TO_INTERM,
	TOP_SORT_GO_TO_INTERM_BAD,
	TOP_SORT_GO_TO_INTERM_GOOD,
	BOTTOM_SORT_BALL_TO_RIGHT,
	BOTTOM_SORT_BALL_TO_LEFT,
	BOTTOM_SORT_GO_TO_INTERM,
	BOTTOM_SORT_RELEASE_BALL_TO_RIGHT,
	BOTTOM_SORT_RELEASE_BALL_TO_LEFT,
	OPEN_LATCH,
	CLOSE_LATCH,
	OPEN_BUTTON_FUNNY_ACTION,
	CLOSE_ALL_FUNNY_ACTIONS,
} Sorter_Command_Typedef;
//--------------------------------------------- Macros for servos ------------------------------------------------//
// FOR CUBE MANIPULATOR
// ID of servo motors

#define MANIP_LEFT_SERVO_SLIDER_ID                0x00  // id = 2
#define MANIP_LEFT_SERVO_GRIPPER_ID               0x11  // id = 17
#define MANIP_LEFT_SERVO_MAGIC_CUBE_ID            0x15  // id = 21
#define MANIP_LEFT_SERVO_DOOR_ID                  0x0B  // id = 11

// Boundary angles

#define MANIP_LEFT_SERVO_SLIDER_TOP_POS           0x11D // TBD!!!
#define MANIP_LEFT_SERVO_SLIDER_BOT_POS           0x14  // TBD!!!
#define MANIP_LEFT_SERVO_SLIDER_INTERM_POS        0x3C  // TBD!!!
#define MANIP_LEFT_SERVO_GRIPPER_OPENED_POS       0x69  // TBD!!!
#define MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS       0xF5  // TBD!!!
#define MANIP_LEFT_SERVO_MAGIC_CUBE_INITIAL_POS   0x46  // TBD!!!
#define MANIP_LEFT_SERVO_MAGIC_CUBE_FINAL_POS     0x00  // TBD!!!
#define MANIP_LEFT_SERVO_DOOR_OPENED_POS          0x46  // TBD!!!
#define MANIP_LEFT_SERVO_DOOR_CLOSED_POS          0x93  // TBD!!!

// FOR SORTER
#define SORTER_SERVO_TOP_ID                       0x01  // 
#define SORTER_SERVO_BOTTOM_ID                    0x03  // 
#define SORTER_SERVO_LATCH_ID                     0x02  // 
#define SORTER_SERVO_BUTTON_FUNNY_ID              0x0A  //

// Boundary angles
#define SORTER_SERVO_TOP_LEFT_POS                 0x00  // 0°
#define SORTER_SERVO_TOP_RIGHT_POS                0xF5  // 245°
#define SORTER_SERVO_TOP_INTERM_POS               0x96  // 150°
#define SORTER_SERVO_TOP_INTERM_LEFT_POS          0x8C  // 140°
#define SORTER_SERVO_TOP_INTERM_RIGHT_POS         0xA5  // 165°
#define SORTER_SERVO_BOTTOM_LEFT_POS              0xD5  // 213°
#define SORTER_SERVO_BOTTOM_RIGHT_POS             0x53  // 83°
#define SORTER_SERVO_BOTTOM_INTERM_POS            0x96  // 150°
#define SORTER_SERVO_BOTTOM_RELEASE_RIGHT_POS     0x127 // 295°
#define SORTER_SERVO_BOTTOM_RELEASE_LEFT_POS      0x05  // 5 °
#define SORTER_SERVO_LATCH_OPENED_POS             0x50  // 80°
#define SORTER_SERVO_LATCH_CLOSED_POS             0x96  // 150°
#define SORTER_SERVO_BUTTON_FUNNY_CLOSED_POS      0x96  // 150°
#define SORTER_SERVO_BUTTON_FUNNY_OPENED_POS      0xF0  // 240°

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void);
static ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle);
static ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle);

//--------------------------------------------- ServoChecker functions -----------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker);
void resetChecker(Servo_Checker_Typedef* servoChecker);

//--------------------------------------------- Tasks executor's functions -------------------------------------//
// For cube manipulators
void execManipTasks(uint8_t numberOfmanipulator, Cube_Manipulator_Typedef* manipulator);
void execManipSubtasks(uint8_t numberOfManipulator, Cube_Manipulator_Typedef* manipulator);

// For sorter
void execSorterTasks(uint8_t numberOfsorter, Sorter_Manipulator_Typedef* sorter);
void execSorterSubtasks(uint8_t numberOfsorter, Sorter_Manipulator_Typedef* sorter);

//--------------------------------------------- High-level command assignment ----------------------------------//
void setManipHighLevelCommand(Manipulator_Command_Typedef command, uint8_t numberOfManipulator, Cube_Manipulator_Typedef* manipulator);
void setSorterHighLevelCommand(Sorter_Command_Typedef command, uint8_t numberOfSorter, Sorter_Manipulator_Typedef* sorters);
#endif
