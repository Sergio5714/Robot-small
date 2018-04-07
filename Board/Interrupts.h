#ifndef STM32F4_INTERRUPTS
#define STM32F4_INTERRUPTS

#include "stm32f4xx.h"
#include "Board.h"
#include "Robot.h"
#include "Shooter_motors.h"

// Interrupt handler for receiving data from Raspberry Pi (DEBUG_USART_MODULE = USART1)
// This Interrupt is implemented in Communication.c
void USART1_IRQHandler(void);

// Interrupt handler for communication with Dynamixel servo
// This Interrupt is implemented in Dynamixel_control.c
void UART4_IRQHandler (void);

// Interrupt handler for motor control
void TIM6_DAC_IRQHandler(void);

// Interrupt handler for manipulators control
// This interrupt is implemented in Manipulators.c
void TIM8_UP_TIM13_IRQHandler(void);

// Interrupt handler for PWM for shooter motors
void TIM8_BRK_TIM12_IRQHandler(void);

// Interrupt handler for I2C errors
void I2C2_ER_IRQHandler(void);

// Interrupt for Local time timer (that counts in ms)
void TIM7_IRQHandler(void);

// Interrupt handler for external startup interrupt
void EXTI1_IRQHandler(void);

//--------------------------------------------- Some funtions for local time calculations --------------------------------------//
uint32_t getLocalTime(void);
uint8_t checkTimeout(uint32_t startTime, uint32_t timeout);
void delayMs(uint16_t delay);

#endif
