/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
 */

/* This file implements the I2C based driver for an image sensor that uses I2C
 for control in the FX3 HD 720p camera kit.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "sensor.h"
#include "uvc.h"

/* This function inserts a delay between successful I2C transfers to prevent
 false errors due to the slave being busy.
 */
static void SensorI2CAccessDelay(CyU3PReturnStatus_t status) {
	/* Add a 10us delay if the I2C operation that preceded this call was successful. */
	if (status == CY_U3P_SUCCESS)
	{
		CyU3PBusyWait(2000); //change into 2ms org is 100us (100).
	}else //if I2C operation is not success reconfig the I2C
	{
		//CyFxUVCApplnI2CInit ();
		//CyU3PBusyWait(1000);
	}
}

/* Write to an I2C slave with two bytes of data. */
CyU3PReturnStatus_t SensorWrite2B(
	uint8_t slaveAddr,
	uint8_t boradAddr,
	uint8_t highAddr,
	uint8_t lowAddr, 
	uint8_t numData,
	uint8_t *buf) {
	
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;
	uint8_t inbuf[2];

	/* Validate the I2C slave address. */
	if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}
	preamble.buffer[0] = slaveAddr;						/************** command block ***************************************/
	preamble.buffer[1] = boradAddr;
	preamble.buffer[2] = highAddr;
	preamble.ctrlMask = 0x0000;
	preamble.length = 3; /*  Three byte preamble. */
	inbuf[0] = lowAddr;

	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, inbuf, 1, 0);
#ifdef DbgInfo
	CyU3PDebugPrint(4, "sensor write2B(0) %d %d %d\r\n", lowAddr, buf[0], lowData); //additional debug
#endif
	SensorI2CAccessDelay(apiRetStatus);

	//buf[0] = lowData;								/****************** data block *****************************************/
	preamble.ctrlMask = 0x0000;
	preamble.length = 1;
	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, numData, 0);
#ifdef DbgInfo
	CyU3PDebugPrint(4, "sensor write2B(1) %d %d %d\r\n", lowAddr, buf[0], buf[1]); //additional debug
#endif
	/* Set the parameters for the I2C API access and then call the write API. */
	SensorI2CAccessDelay(apiRetStatus);
	return apiRetStatus;
}

CyU3PReturnStatus_t SensorWrite(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t count, uint8_t *buf) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	/* Validate the I2C slave address. */
	if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}

	if (count > 64) {
		CyU3PDebugPrint(4, "ERROR: SensorWrite count > 64\n");
		return 1;
	}

	/* Set up the I2C control parameters and invoke the write API. */
	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = 0xab;//highAddr;
	preamble.buffer[2] = 0xcd;//lowAddr;
	preamble.length = 3;
	preamble.ctrlMask = 0x0000;

	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, count, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

CyU3PReturnStatus_t SensorRead2B(
		uint8_t slaveAddr, 
		uint8_t highAddr,
		uint8_t lowAddr, 
		uint8_t RegAdd,
		uint8_t numData,
		uint8_t *buf) {
	
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}
	preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; /*  Mask out the transfer type bit. */
	preamble.buffer[1] = highAddr; //highAddr;
	preamble.buffer[2] = lowAddr; //lowAddr;
	preamble.length = 3;
	preamble.ctrlMask = 0x0000; /*  Send start bit after third byte of preamble. */
	buf[0] = RegAdd;
#ifdef DbgInfo
	CyU3PDebugPrint(4, "sensor read2B(0) %d %d %d %d %d\r\n", lowAddr, RegAdd, numData, buf[0], buf[1]); //additional debug
#endif
	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, 1, 0); //send command block to prepare for reading
	/*** test I2C bus ready ****/
	if(apiRetStatus != CY_U3P_SUCCESS){
		CyU3PDebugPrint(4, "sensor read2B(T) %d %d %d\r\n", apiRetStatus, buf[0], buf[1]);
	}

#ifdef DbgInfo
	CyU3PDebugPrint(4, "sensor read2B(1) %d %d %d\r\n", lowAddr, buf[0], buf[1]); //additional debug
#endif
	SensorI2CAccessDelay(apiRetStatus);
	preamble.buffer[0] = slaveAddr;
	preamble.length = 1;
	preamble.ctrlMask = 0x0000;

	apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, numData, 0);//send data block read one byte
	/*** test I2C bus ready ****/
	if(apiRetStatus != CY_U3P_SUCCESS){
		CyU3PDebugPrint(4, "sensor read2B(R) %d %d %d\r\n", apiRetStatus, buf[0], buf[1]);
	}
	SensorI2CAccessDelay(apiRetStatus);
#ifdef DbgInfo
	CyU3PDebugPrint(4, "sensor read2B(2) %d %d %d\r\n", apiRetStatus, buf[0], buf[1]); //additional debug
#endif
	return apiRetStatus;
}

CyU3PReturnStatus_t SensorRead(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t count, uint8_t *buf) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	/* Validate the parameters. */
	if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}
	if (count > 64) {
		CyU3PDebugPrint(4, "ERROR: SensorWrite count > 64\n");
		return 1;
	}
	preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; /*  Mask out the transfer type bit. */
	preamble.buffer[1] = 0x55;//highAddr;
	preamble.buffer[2] = 0xaa;//lowAddr;
	preamble.buffer[3] = slaveAddr;
	preamble.length = 4;
	preamble.ctrlMask = 0x0004; /*  Send start bit after third byte of preamble. */

	apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, count, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

/*
 * Reset the image sensor using GPIO.
 */
void SensorReset(void) {
	CyU3PReturnStatus_t apiRetStatus;
	uint16_t preTick, posTick;
	/* Drive the GPIO low to reset the sensor. */
	//apiRetStatus = CyU3PGpioSetValue(SENSOR_POWER_GPIO, CyFalse);
	apiRetStatus = CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyFalse);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n",
				apiRetStatus);
		return;
	}
	CyU3PDebugPrint(4, "GPIO Set Value\r\n");
	/* Wait for some time to allow proper reset. */
	uint8_t i = 0;
	while (i++ < 2){
		preTick = CyU3PGetTime();
		CyU3PThreadSleep(500);  // change the value into 100 from 10.
		posTick = CyU3PGetTime();
		CyU3PDebugPrint(4, "The ticks %d %d \r\n", preTick, posTick); //additional debug
		//;//CyU3PDebugPrint(4, "cpu pause \r\n");
	}

	/* Drive the GPIO high to bring the sensor out of reset. */
	//apiRetStatus = CyU3PGpioSetValue(SENSOR_POWER_GPIO, CyTrue);
	apiRetStatus = CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n",
				apiRetStatus);
		return;
	}
/* pause the cpu */
	while (i++ < 4){
		CyU3PThreadSleep(600);  // change the value into 100 from 10.
		//;//CyU3PDebugPrint(4, "cpu pause \r\n");
	}

	return;
}

/* Image sensor initialization sequence. */
void SensorInit(void) {
	if (SensorI2cBusTest() != CY_U3P_SUCCESS) /* Verify that the sensor is connected. */
	{
		CyU3PDebugPrint(4, "Error: Reading Sensor ID failed!\r\n");
		return;
	}

	/* Generic settings (which are common for all resolutions) for bringing up the image sensor to stream
	 video data should be populated here.
	 */

	/* Update sensor configuration based on desired video stream parameters. Using 720p 30fps as default setting.*/
	//SensorScaling_HD720p_30fps();
}

/*
   Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t SensorI2cBusTest(void) {
	/* The sensor ID register can be read here to verify sensor connectivity. */
	uint8_t buf[2];

	/* Reading sensor ID */
	if (SensorRead2B(SENSOR_ADDR_RD, I2C_DSPBOARD_ADDR_WR, I2C_EAGLESDP_ADDR, 0xf2, 1, buf) == CY_U3P_SUCCESS) {
		if ((buf[0] == 0x56) /*&& (buf[1] == 0x02)*/) {
			return CY_U3P_SUCCESS;
		}
	}
#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "The Sensor test 0x%x 0x%x\r\n", buf[0], buf[1]); // additional debug
#endif
	return 1;
}

/*************************************************************
 *  the modularized control get routine. IDext is the control ID.
 *
 * ********************************************************* */

uint8_t SensorGetControl(uint8_t IDext, uint8_t devAdd)  //for register w/r, the IDext is Reg. addrss.
{
	uint8_t buf[2];
	SensorRead2B(SENSOR_ADDR_RD, I2C_DSPBOARD_ADDR_RD, devAdd, IDext, 1, buf);
//#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "The Get control ID 0x%x %d\r\n", IDext, buf[0]); // additional debug
//#endif
	return buf[0];
};

/* *********************************************************
 * the modularized control Set routine. IDuvc: the control ID;
 * value: set value, range check.
 *
 ************************************************************ */

uint8_t SensorSetControl(uint8_t IDext, uint8_t devAdd, uint8_t value) //for register w/r, the IDext is Reg. addrss.
{
	uint8_t buf[2];
	buf[0] = value;
	SensorWrite2B(SENSOR_ADDR_WR, I2C_DSPBOARD_ADDR_WR, devAdd, IDext, 1, buf);
//#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "The Set control regAdd 0x%x 0x%x\r\n", IDext, value); // additional debug
//#endif
	return 0;
};
/*************************************************************
 *  the Iris control get routine. IDext is the control ID, boardID: the Iris control board address.
 *
 * ********************************************************* */

uint8_t SensorGetIrisControl(uint8_t IDext, uint8_t devAdd, uint8_t boardID)  //for register w/r, the IDext is Reg. addrss.
{
	uint8_t buf[2];
	SensorRead2B(SENSOR_ADDR_RD, boardID, devAdd, IDext, 1, buf);
#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "The Get control ID 0x%x 0x%x %d\r\n", boardID, IDext, buf[0]); // additional debug
#endif
	return buf[0];
};

/* *********************************************************
 * the Iris control Set routine. IDuvc: the control ID, boardID: the Iris control board address;
 * value: set value, range check.
 *
 ************************************************************ */

uint8_t SensorSetIrisControl(uint8_t IDext, uint8_t devAdd, uint8_t value, uint8_t boardID) //for register w/r, the IDext is Reg. addrss.
{
	uint8_t buf[2];
	buf[0] = value;
	SensorWrite2B(SENSOR_ADDR_WR, boardID, devAdd, IDext, 1, buf);
#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "The Set control ID 0x%x 0x%x 0x%x\r\n", boardID, IDext, value); // additional debug
#endif
	return 0;
};

