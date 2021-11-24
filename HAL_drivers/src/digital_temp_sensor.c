/*
 * digital_temp_sensor.c
 *
 *  Created on: Feb 5, 2021
 *      Author: khlod
 */
#include "digital_temp_sensor.h"


//GPIO PINs
GPIO_Handle_t I2CPins;

//I2C init
I2C_Handle_t LM75_I2C;

/*
 * PB6-> SCL
 * PB9-> SDA
 */
void LM75_Init() {

	//GPIO PINs
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

	//I2C init
	LM75_I2C.pI2Cx = I2C1;
	LM75_I2C.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	LM75_I2C.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	LM75_I2C.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	LM75_I2C.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;


	I2C_Init(&LM75_I2C);


}

// Read 16-bit LM75 register
uint16_t LM75_ReadReg(uint8_t reg) {
	uint16_t value;

	I2C_ManageAcking(LM75_I2C.pI2Cx,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	I2C_ExecuteAddressPhaseWrite(LM75_I2C.pI2Cx, LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(LM75_I2C.pI2Cx,reg); // Send register address
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);; // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	I2C_ExecuteAddressPhaseRead(LM75_I2C.pI2Cx, LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = (I2C_ReceiveData(LM75_I2C.pI2Cx) << 8); // Receive high byte
	I2C_ManageAcking(LM75_I2C.pI2Cx,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateStopCondition(LM75_I2C.pI2Cx); // Send STOP condition
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	value |= I2C_ReceiveData(LM75_I2C.pI2Cx); // Receive low byte

	return value;
}

// Write 16-bit LM75 register
void LM75_WriteReg(uint8_t reg, uint16_t value) {

	I2C_ManageAcking(LM75_I2C.pI2Cx,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	I2C_ExecuteAddressPhaseWrite(LM75_I2C.pI2Cx, LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(LM75_I2C.pI2Cx,reg); // Send register address
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(LM75_I2C.pI2Cx,(uint8_t)(value >> 8)); // Send high byte
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(LM75_I2C.pI2Cx,(uint8_t)value); // Send low byte
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateStopCondition(LM75_I2C.pI2Cx);
}

// Read value from LM75 configuration register (8 bit)
uint8_t LM75_ReadConf(void) {
	uint8_t value;

	I2C_ManageAcking(LM75_I2C.pI2Cx,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);;
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	I2C_ExecuteAddressPhaseWrite(LM75_I2C.pI2Cx,LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(LM75_I2C.pI2Cx,LM75_REG_CONF); // Send register address
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);; // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	I2C_ExecuteAddressPhaseRead(LM75_I2C.pI2Cx,LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	I2C_ManageAcking(LM75_I2C.pI2Cx,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateStopCondition(LM75_I2C.pI2Cx); // Send STOP condition
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(LM75_I2C.pI2Cx);

	return value;
}

// Write value to LM75 configuration register  (8 bit)
void LM75_WriteConf(uint8_t value) {
	I2C_ManageAcking(LM75_I2C.pI2Cx,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateStartCondition(LM75_I2C.pI2Cx);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	//I2C_Send7bitAddress(LM75_I2C.pI2Cx,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	I2C_ExecuteAddressPhaseWrite(LM75_I2C.pI2Cx,LM75_ADDR);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(LM75_I2C.pI2Cx,LM75_REG_CONF); // Send register address
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(LM75_I2C.pI2Cx,value);
	while (!I2C_CheckEvent(LM75_I2C.pI2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateStopCondition(LM75_I2C.pI2Cx);
}

// Set LM75 shutdown mode
// newstate:
//    ENABLE = put LM75 into powerdown mode
//    DISABLE = wake up LM75
void LM75_Shutdown(FunctionalState newstate) {
	uint8_t value;

	value = LM75_ReadConf();
	LM75_WriteConf(newstate == ENABLE ? value | 0x01 : value & 0xFE);
}

// Read temperature readings from LM75 in decimal format
// IIIF where:
//   III - integer part
//   F   - fractional part
// e.g. 355 means 35.5C
int16_t LM75_Temperature(void) {
	uint16_t raw;
	int16_t temp;

	raw = LM75_ReadReg(LM75_REG_TEMP) >> 7;
	if (raw & 0x0100) {
		// Negative temperature
		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
	} else {
		// Positive temperature
		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
	}

	return temp;
}


