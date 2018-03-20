#include "Collision_avoidance.h"
extern I2C_Module_With_State_Typedef I2CModule;

// debug
extern uint8_t values;
ErrorStatus initAllRangefinders()
{
//	// Setup register
//	if (expanderWriteReg(EXPANDER_CONFIG_REG, 0xA2, EXPANDER_I2C_ADDRESS) != SUCCESS)
//	{
//		return ERROR;
//	}
	// Init direction of A bank (0 - output, 1 - input) A - input
	if (expanderWriteReg(EXPANDER_REG_DIR_A, 0x00, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	
	// Init direction of B bank (0 - output, 1 - input) B - input
	if (expanderWriteReg(EXPANDER_REG_DIR_B, 0x00, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}

	// Output value of B bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_B, values, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of A bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_A, 0x01, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	
//	// Init Interrupt (A0, A1)
//	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_A, 0x03, EXPANDER_I2C_ADDRESS) != SUCCESS)
//	{
//		return ERROR;
//	}
//	// Value to compare for A bank
//	if (expanderWriteReg(EXPANDER_REG_COMPARE_VALUE_FOR_INTER_A, 0x00, EXPANDER_I2C_ADDRESS) != SUCCESS)
//	{
//		return ERROR;
//	}
//	
//	// Interrupt control for A bank
//	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_A, 0xff, EXPANDER_I2C_ADDRESS) != SUCCESS)
//	{
//		return ERROR;
//	}
	return SUCCESS;
}

ErrorStatus expanderReadInterrupt(void)
{
	uint8_t buf;
//	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_A, &buf, EXPANDER_I2C_ADDRESS) != SUCCESS)
//	{
//		return ERROR;
//	}
	if (expanderReadReg(EXPANDER_REG_VALUE_B, &buf, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	if (expanderReadReg(EXPANDER_REG_DIR_B, &buf, EXPANDER_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}

	return SUCCESS;
}

//--------------------------------------------- Low level functions to access registers ------------------------//

// Write an 8-bit value to expander's register
ErrorStatus expanderWriteReg(uint8_t reg, uint8_t value, uint8_t addr)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	
	if (I2CSendBytes(&I2CModule, buf, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read a 8-bit value from  register
ErrorStatus expanderReadReg(uint8_t reg, uint8_t* value, uint8_t addr)
{
	
	if (I2CSendBytes(&I2CModule, &reg, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CReadBytes(&I2CModule, value, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
