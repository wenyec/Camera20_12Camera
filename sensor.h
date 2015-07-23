/*
 ## Cypress FX3 Camera Kit header file (sensor.h)
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

/* This file defines the parameters and the interface for the image sensor driver. */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>

/* I2C Slave address for the image sensor. */
#define SENSOR_ADDR_WR 0x70             /* Slave address used to write sensor registers. Default set to EEPROM. 0xA0 */
#define SENSOR_ADDR_RD 0x71             /* Slave address used to read from sensor registers. Default set to EEPROM  0xA1 */

#define I2C_SLAVEADDR_MASK 0xFE         /* Mask to get actual I2C slave address value without direction bit. */

#define I2C_MEMORY_ADDR_WR 0xA0         /* I2C slave address used to write to an EEPROM. */
#define I2C_MEMORY_ADDR_RD 0xA1         /* I2C slave address used to read from an EEPROM. */

#define I2C_DSPBOARD_ADDR_WR 0x52		/* I2C address from DSP Eagle Board WRITE */
#define I2C_DSPBOARD_ADDR_RD 0x53		/* I2C address from DSP Eagle Board READ  */

#define I2C_AFBOARD_ADDR_WR 0x82		/* I2C address from AF Board WRITE */
#define I2C_AFBOARD_ADDR_RD 0x83		/* I2C address from AF Board READ  */

#define I2C_EAGLESDP_ADDR 0x30			/* I2C address from Eagle DSP */

#define I2C_DevAdd_C6			0xc6	/* I2C address for device 0xc6 bright, contrast, hue, etc*/
#define I2C_DevAdd_F2           0xf2    /* I2C address for device 0xf2 saturation */

#define I2C_RD_MASK             0x1     /* I2C bus address direction bit for read */
#define I2C_WR_MASK             0xFE    /* I2C bus address direction bit for write */

/******	 Definition of feature registers for Eagle DSP board	**************************************************************/

#define BLCModeReg				0x10    // off:0 blc:1 hblc:2 wdr:3
#define BrightnessReg0   		0x00	// brightness RegAAdd0 [7:0]. devadd 0xc6
#define BrightnessReg1   		0x01	// brightness RegAAdd1 [1:0] bit1 is sign bit
#define ContrastReg				0x02	// devadd 0xc6
#define MainsFreqReg 			0x07    // PAL:0 NTSC:1
#define HuectrlRegMg            0xdc    // hue regadd for Mg     devadd 0xc6
#define HuectrlRegRed           0xdd    // hue regadd for red    devadd 0xc6
#define HuectrlRegYel           0xde    // hue regadd for yellow devadd 0xc6
#define HuectrlRegGr            0xdf    // hue regadd for green  devadd 0xc6
#define HuectrlRegCy            0xe0    // hue regadd for Cy     devadd 0xc6
#define HuectrlRegBlu           0xe1    // hue regadd for blue   devadd 0xc6
#define SaturationRegR 		    0x85	// for red devadd 0xf2
#define SaturationRegB 		    0x86	// for blue devadd 0xf2
#define SharpnessReg 			0x06    //
#define GainModeReg 			0x00	// tbd (?)
#define WBModeReg 				0x08    // AWD mode. ATW:0 AWC_SET:1 INDOOR:2 OUTDOOR:3 MANUAL:4 PUSH_TO_WHITE:5 (for AWC_SET usage)(?)

#define ManuBWBReg	    		0x09	// Blue when wb mode set to manual(4).
#define ManuRWBReg     	    	0x0a	// Red when wb mode set to manual(4).

#define AExReferleveReg		    0x04	// reference level Temp
#define AExModeReg	        	0x02	// AEx mode. 0:auto 1:manual.
#define AExAGCReg	        	0x03    // AGC level. 0 ~ 0xff. read only when AEx mode is auto, otherwise, write only.
#define DigZoomReg              0x2a    // digital zoom 1 ~ 25 (no autofocus system support)
#define ShutterReg  			0x00    // shutter control. 0x00 ~ 0x12
#define SenseUpReg  			0x01    // sense up mode. 0:off 1:X2 2:X4 3:X6 4:X8 5:X10 6:X15 7:X20 8:X25 9:X30
#define MirrModeReg 			0x05    // 0:off 1:mirror 2:V_flip 3:rotate (180 degree)
#define SensorModeReg           0x10    // 0:1080p and BLC OFF; 1:1080p and BLC ON; 2:1080p and HBLCD; 3:WRD 1080p and no BLC; 4:720p and BLC OFF; 5:720p and BLC ON; 6:720p and HBLC ON
#define NoiRedu3DModReg 		0x18    // 3D noise reduce Mode control. 0:off 1:on
#define NoiRedu3DLevReg  		0x19    // 3D noise reduce level. 0 ~ 0x64
#define DayNightModReg 			0x20    // day and night selection. 0:auto 1:day mode 2:night mode
#define DayNightDlyReg 			0x22    // day and night switch delay. 0sec ~ 0x3f sec
#define DayNightLevReg 			0x23    // day to night start level. 0 ~ 0x64
#define NightDayLevReg 			0x24    // night to day start level. 0 ~ 0x64

#define IrisAFReg 			    0x23    // Iris range. 0 ~ 0x30. fist byte represents MSB, sencond byte represents LSB
#define OpZoomReg 			    0x10    // Optical zoom register

/******** OSD Menu Control ************************************************/
#define OSDMenuReg 	     		0x40    // write only. stop(no change):0 left:1 right:2 up:3 down:4 set(OSD_pup_up/return/enter):5
#define OSDLogoReg 	     		0x66    // vidology logo. off:0 on:1
#define BLCGainReg				0x11	// low:0 medium:1 high:2
#define BLCEgeSetReg    		0x13	// [7:4]:Top/bottom [3:0]:left/right
#define BLCCenterReg			0x14    // [7:4]:height [3:0]:width
#define BLCGridReg  			0x17	// BLC grid 1:on 0:off (area view)
#define DayNightModeReg			0x20    // auto:0 day:1 night:2

/******* save settings control *********************************************/
#define SeveParsReg  			0x50
							// the register used for save current (data:0) or default (dat:1) DSP settings.
							// Note: the operation needs several hundreds milliseconds. don't turn off the power during the operation.

/****************************************************************************************************************************/

/*****  Hue value base ******/
#define RED_BASE               (0x80-0x11)
#define MAGENTA_BASE           (0x80-0x0a)
#define YELLOW_BASE            (0x80-0xfe)
#define GREEN_BASE             (0x80-0x00)
#define CYAN_BASE              (0x80-0xf2)
#define BLUE_BASE              (0x80-0xff)

//#define DbgInfo //for UART out

/* GPIO 22 on FX3 is used to reset the Image sensor. */
#define SENSOR_RESET_GPIO 22
/* GPIO 20 on FX3 is used to turn on/off the power of the Image sensor. */
#define SENSOR_POWER_GPIO 20
/* GPIO 24 on FX3 is used to detect snap shot button press/release. */
#define SENSOR_SNAPSHOT_GPIO 24


/* Function    : SensorWrite2B
   Description : Write two bytes of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 highData  - High byte of data to be written.
                 lowData   - Low byte of data to be written.
 */
extern CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData);

/* Function    : SensorWrite
   Description : Write arbitrary amount of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     - Size of write data in bytes. Limited to a maximum of 64 bytes.
                 buf       - Pointer to buffer containing data.
 */
extern CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorRead2B
   Description : Read 2 bytes of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 buf       - Buffer to be filled with data. MSB goes in byte 0.
 */
extern CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t RegAdd,
        uint8_t *buf);

/* Function    : SensorRead
   Description : Read arbitrary amount of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     = Size of data to be read in bytes. Limited to a max of 64.
                 buf       - Buffer to be filled with data.
 */
extern CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorInit
   Description : Initialize the image sensor.
   Parameters  : None
 */
extern void
SensorInit (
        void);

/* Function    : SensorReset
   Description : Reset the image sensor using FX3 GPIO.
   Parameters  : None
 */
extern void
SensorReset (
        void);

/* Function     : SensorScaling_HD720p_30fps
   Description  : Configure the image sensor for 720p 30 fps video stream.
   Parameters   : None
 */
extern void
SensorScaling_HD720p_30fps (
        void);

/* Function     : SensorScaling_VGA
   Description  : Configure the image sensor for VGA video stream.
   Parameters   : None
 */
extern void
SensorScaling_VGA (
        void);

/* Function    : SensorI2cBusTest
   Description : Test whether the image sensor is connected on the I2C bus.
   Parameters  : None
 */
extern uint8_t SensorI2cBusTest (void);

/**************** the Sensor controls ********************/
extern uint8_t SensorGetControl(uint8_t IDext, uint8_t devAdd);
extern uint8_t SensorSetControl(uint8_t IDext, uint8_t devAdd, uint8_t value);

extern uint8_t SensorGetIrisControl(uint8_t IDext, uint8_t devAdd, uint8_t boardID);
extern uint8_t SensorSetIrisControl(uint8_t IDext, uint8_t devAdd, uint8_t value, uint8_t boardID);
#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/
