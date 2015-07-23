/********************************************************************
 * The cmdqu.h for the ring buffer that can be used any case that needs a buffer.
 * Copyright Videology Imaging Solution Inc. 2013
 * All Rights Reserved
 *
 *  Created on: Oct 31, 2013
 *      Author: wcheng
 *
 *  the structure and functions used in cmdqu.c are declared here.
 *******************************************************************/

#ifndef CMDQU_H_
#define CMDQU_H_

#include <cyu3types.h>
#include <cyu3usbconst.h>
#include <cyu3externcstart.h>

#define  MAXCMD      64
#define  MAXSTA      32
#define  MAXPAR      8
#define  CMDQU0      0
/* for optical zoom operation */
#define  TELEDATA    0x04
#define  WIDEDATA    0x08
#define  STOP        0x00
/* end of the optical zoom define */
#define  EXUAOFFSET  0x20

/************* control handler **************/
typedef enum UVCCtrlID{
	BLCCtlID0  = 0x0,
	BrgtCtlID1,
	ConsCtlID2,
	UVCCtlID3,
	MFreqCtlID4,
	HueCtlID5,
	SaturCtlID6,
	ShapCtlID7,
	GamCtlID8,
	WBTMdCtlID9,
	UVCCtlID10,
	WBTLevCtlID11,
	UVCCtlID12,
	UVCCtlID13,
	DigZmCtlID14,
	UVCCtlID15
}UVCCtrlID_t;

typedef enum ExtCtrlID{
	ExtShutCtlID0 = 0x10,   // shutter
	ExtSenCtlID1,           // sense up
	ExtMirrCtlID2,          // mirror mode
	Ext3DNReduMCtlID3,      // 3D noise reduce mode
	Ext3DNReduLvCtlID4,     // // 3D noise reduce level
	ExtDNModCtlID5,         // Day Night mode
	ExtDNDelytlID6,         // Day Night delay
	ExtDNlevCtlID7,         // Day Night level
	ExtNDlevCtlID8,         // Night Day level
	ExtAexModCtlID9,        // Aex mode/agc
	ExtExRefCtlID10,         // Ex reference level
	ExtCtlID11,
	ExtCamMCtlID12,
	ExtshotCtlID13,			//snap shot
	ExtSensorParCtlID14,    //sensor parameters operation: recovery default settings; customer settings; store current settings
	ExtI2CCtlID15			//I2C command
}ExtCtrlID_t;

typedef enum ExtCtrlID1{
	Ext1BLCRangeCtlID0 = 0x20,      // back light compensation range
	Ext1BLCWeightCtlID1,           // back light compensation wieght factor
	Ext1BLCGridCtlID2,
	Ext1CtlID3,
	Ext1CtlID4,
	Ext1CtlID5,
	Ext1CtlID6,
	Ext1CtlID7,
	Ext1CtlID8,
	Ext1CtlID9,
	Ext1CtlID10,
	Ext1CtlID11,
	Ext1CtlID12,
	Ext1CtlID13,
	Ext1CtlID14,
	Ext1CtlID15
}ExtCtrlID1_t;


typedef enum CTCtrlID{
	ScanMCtlID0  = 0x0,
	AutoExMCtlID1,
	AutoExPCtlID2,
	ExTmACtlID3,
	ExTmRCtlID4,
	FocACtlID5,
	FocRCtlID6,
	IriACtlID7,
	IriRCtlID8,
	ZmOpACtlID9,
	ZmOpRCtlID10,
	CTCtlID11,
	CTCtlID12,
	CTCtlID13,
	CTCtlID14
}CTCtrlID_t;

typedef enum dataIdx{
	First  = 0x0,
	Second,
	Third,
	Fourth,
	Fifth,
	Sixth,
	Seventh,
	Eighth
}DIdx;

typedef enum desFlag{
	deswait = 0x00,
	desusing = 0x0F,
	desReady = 0xFF
}desflag;

typedef struct camPara_t{          //camera parameters structure
	uint8_t     RegAdd;            //it's the register's address in camera unit.
	uint8_t     DevAdd;            //it's the address of the device in camera unit.
	uint8_t     Data;              //the array used to store the data being set to the camera.
	uint8_t     ParaRes;           //a reserver byte
	uint16_t    DelayT;            //command performance delay
}camPara;

typedef struct cmdDescriptor_t{
	uint8_t     CmdID;             //it's the same as the CtrlID.
	uint8_t     curNum;            //current number of the parameter. it is 0 if cmdFlag == 0x00 or 0xFF.
	uint8_t     NumPara;           //a number of the parameters in a command
	camPara     CmdPar[MAXPAR];    //command parameters
	struct   cmdDescriptor_t
		*cmdDesNext,
		*cmdDesPrevious;
	CyBool_t    cmdFlag;           //the command is available.
}VdcmdDes;

typedef struct stateDescriptor_t{
	uint8_t     StatID;            //it's the same as the CtrlID.
	uint8_t     curNum;            //current number of the parameter. it is 0 if cmdFlag == 0x00 or 0xFF.
	uint8_t     NumData;           //a number of a state data.
	camPara     staPar[MAXPAR];    //state data
	struct   stateDescriptor_t
		*staDesNext,
		*staDesPrevious;
	CyBool_t    statFlag;           //the state is available.
}VdstateDes;

typedef struct RingBuffer_t{
	uint8_t     ringbufID;
	char        *bufferName;
	VdcmdDes        *startAdd;
	VdcmdDes        *endAdd;
	VdcmdDes        *readPtr;
	VdcmdDes        *writePtr;
	uint16_t    numUnit;
	uint8_t     bugFlag;
	CyU3PMutex  *ringMux;

}VdRingBuf;

static uint8_t glEp0Buffer[32];

/******** routines for the ring buffer operation **********/
void  cmdquTest(VdRingBuf *cmdbuf, uint8_t state);  //it's used test the cmdqu data structure.
VdRingBuf  cmdbufCreate(uint16_t size, CyU3PMutex *muxPtr);                  //create a command buffer.
void  cmdquInit(VdRingBuf *cmdqu);                    //initialize the command queue.
CyBool_t  cmdbufDestroy(VdRingBuf *cmdqu);

CyBool_t  cmdSet(VdRingBuf *cmdqu, uint8_t cmdID, uint8_t RegAdd, uint8_t DevAdd, uint8_t Data, uint8_t dataIdx);
void  statGet(VdRingBuf *statqu, uint8_t statId);                     //it might be unused, if state request performs immediately.

VdcmdDes *cmdFind(VdRingBuf *cmdqu, uint8_t cmdID);

#endif /* CMDQU_H_ */
