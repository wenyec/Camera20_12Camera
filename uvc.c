/*
 ## Cypress FX3 Camera Kit Source file (uvc.c)
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

/* This project implements a USB Video Class device that streams uncompressed video
   data from an image sensor to a USB host PC.

   Please refer to the Cypress Application Note: "AN75779: Interfacing an Image
   Sensor to EZ-USB FX3 in a USB video class (UVC) Framework" (http://www.cypress.com/?rID=62824)
   for a detailed design description of this application.

   As the UVC class driver on Windows hosts does not support burst enabled Isochronous
   endpoints on USB 3.0, this implementation makes use of Bulk endpoints for the video
   streaming.
 */
/*****************************************
 *
 * The code is modified at 1/2014
 * 1. add a thread for I2C commands handle
 * 2. add UVC Camera Terminal Requests handle
 * 3. add UVC Extension Unit Requests handle
 * 4. more UVC Processing Unit Requests added
 * 5. Support USB3.0 1080p 25/30fps and USB2.0 960x540p 25/30 fps
 *
 ****************************************/

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>

#include "uvc.h"
#include "sensor.h"
#include "camera_ptzcontrol.h"
#include "cyfxgpif2config.h"

#ifndef CAM720
#ifdef GPIFIIM
#include "cyfxgpif2config_usb2.h"//
#else
#include "cyfxgpif2config_usb2_720.h"//
#endif
#else
#include "cyfxgpif2config_usb2_720.h"
#endif

#include "cmdqu.h"
/*
 ##Videology Imaging Solution Inc. USB UVC Stack

 ## source file : CX3RDKOV5640.c
 ## ===========================
 ##
 ##  Copyright E-Con Systems, 2013-2014,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  PROPERTY OF ECON SYSTEMS

 ## www.e-consystems.com
 ##
 ##
 ## ===========================
*/

#include "cyu3socket.h"
#include "sock_regs.h"
//#include "cyu3mipicsi.h"

//#include "CX3OV5640Lib.h"
//#include "CX3RDKOV5640.h"

/* Event generated on Timer overflow*/
#define ES_TIMER_RESET_EVENT		(1<<4)

/* Event generated on a USB Suspend Request*/
#define ES_USB_SUSP_EVENT_FLAG		(1<<5)

/* Firmware version*/
#define MajorVersion 				1
#define MinorVersion 				3
#define SubVersion					133
#define SubVersion1					309
//#define RESET_TIMER_ENABLE 1

/*************************************************************************************************
                                         Global Variables
 *************************************************************************************************/
static CyU3PThread   uvcAppThread;                      /* UVC video streaming thread. */
static CyU3PThread   uvcAppEP0Thread;                   /* UVC control request handling thread. */
static CyU3PEvent    glFxUVCEvent;                      /* Event group used to signal threads. */
CyU3PDmaMultiChannel glChHandleUVCStream;               /* DMA multi-channel handle. */
CyU3PDmaMultiChannel glChHandleStillStream;             /* DMA multi-channel handle for still image. */
CyU3PDmaChannel glChHandleInterStat;                    /* DMA channel handle for interrupt status. */

uint8_t     *glInterStaBuffer;                          /* Buffer used to send interrrupt status. */
uint8_t     snapButFlag = 1;							/* snap shot button flag: 0 = masked; 1 = unmasked;*/
uint8_t     testSnap = 0;				                /* used for debugging */
/**************** variables relative to the command queue operation ****************/
static CyU3PThread   i2cAppThread;      //i2c control command handling thread
VdRingBuf        cmdQu;                 //the command queue
VdRingBuf        statQu;                //the state queue
CyU3PMutex       cmdQuMux;
CyU3PMutex       staQuMux;
CyU3PMutex       timMux;
CyU3PMutex       imgHdMux;

uint8_t  bRequest, bType,bRType, bTarget;
uint16_t wValue, wIndex, wLength;


#ifdef RESET_TIMER_ENABLE
#define TIMER_PERIOD	(500)

static CyU3PTimer        UvcTimer;

static void UvcAppProgressTimer (uint32_t arg)
{
    /* This frame has taken too long to complete.
     * Abort it, so that the next frame can be started. */
    CyU3PEventSet(&glTimerEvent, ES_TIMER_RESET_EVENT,CYU3P_EVENT_OR);
}
#endif

volatile int32_t glDMATxCount = 0;          /* Counter used to count the Dma Transfers */
volatile int32_t glDmaDone = 0;
volatile uint8_t glActiveSocket = 0;
volatile CyBool_t doLpmDisable = CyTrue;	/* Flag used to Enable/Disable low USB 3.0 LPM */
CyBool_t glHitFV = CyFalse;             	/* Flag used for state of FV signal. */
CyBool_t glMipiActive = CyFalse;        	/* Flag set to true when Mipi interface is active. Used for Suspend/Resume. */
CyBool_t glIsClearFeature = CyFalse;    	/* Flag to signal when AppStop is called from the ClearFeature request. Need to Reset Toggle*/
CyBool_t glPreviewStarted = CyFalse;		/* Flag to support Mac os */

CyBool_t        isUsbConnected = CyFalse;               /* Whether USB connection is active. */
CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED;        /* Current USB connection speed. */

/* UVC Header */
uint8_t glUVCHeader[CY_FX_UVC_MAX_HEADER] =
{
    0x0C,                           /* Header Length */
    0x8C,                           /* Bit field header field */
    0x00,0x00,0x00,0x00,            /* Presentation time stamp field */
    0x00,0x00,0x00,0x00,0x00,0x00   /* Source clock reference field */
};

/* Video Probe Commit Control */
uint8_t glCommitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];
uint8_t glCurrentFrameIndex = 1;
uint8_t glStillCommitCtrl[CY_FX_MAX_STILL_PROBE_SETTING_ALIGNED];
uint8_t glCurrentStillFrameIndex = 1;
uint8_t glStillTriggerCtrl = 0;
uint8_t glFrameIndexToSet = 0;
CyBool_t glStillCaptureStart = CyFalse;
CyBool_t glStillCaptured = CyFalse;
uint8_t glStillSkip = 0;

CyBool_t glIsApplnActive = CyFalse;             /* Whether the Mipi->USB application is active or not. */
CyBool_t glIsConfigured = CyFalse;              /* Whether Application is in configured state or not */
CyBool_t glIsStreamingStarted = CyFalse;        /* Whether streaming has started - Used for MAC OS support*/

/* DMA Channel */
CyU3PDmaMultiChannel glChHandleUVCStream;       /* DMA Channel Handle for UVC Stream  */
uint16_t ES_UVC_STREAM_BUF_SIZE=0;
uint16_t ES_UVC_DATA_BUF_SIZE=0;
uint8_t ES_UVC_STREAM_BUF_COUNT=0;

uint8_t g_IsAutoFocus=1;						/* Check the AutoFocus is Enabled or not*/

/* USB control request processing variables*/
#if 1

uint8_t glGet_Info = 0;
int16_t gl8GetControl = 0;
int16_t gl8SetControl = 0;
int16_t gl16GetControl = 0;

int32_t gl32GetControl = 0;

#endif

#ifndef CAM720
volatile static CyBool_t WDRflag = CyTrue;              /* the flag for WDR mode. It's initialized based on the CtrlParArry[0][13]: 3=set; others=clear. */
#else
volatile static CyBool_t WDRflag = CyFalse;
#endif
volatile static CyBool_t stiflag = CyFalse;             /* Whether the image is still image */

uint8_t glcommitcount=0,glcheckframe=1;
/************ control parameters array ***********
 *  the first D is the index of functionality, the second D is the index of parameters.
 *    e.g.
 *     1th D: backlight compensation, brightness, contrast, hue, saturation, sharpness, gamma, WBT, ~, BLC, main freq, ...
 *     2nd D: RegAdd1, RegAdd2, length, Min1, Min2, Max1, Max2, Res1, Res2, InfoReq1, InfoReq2, DefReq1, DefReq2,
 *            curVal1, curVal2 (index:14th), device address, checked flag, command available flag
 **************************************************/
#define BLCIndex  0 // the back light compensation index
#define CamModeIndex 28 // the index of camera mode
static uint8_t CtrlParArry[32][24]={
#ifndef CAM720
		{/*0*/BLCModeReg          , BLCModeReg           , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
#else
		{/*0*/BLCModeReg          , BLCModeReg           , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
#endif
		{/*1*/BrightnessReg1      , BrightnessReg0       , 2,    0,    0,  63,     0, 1, 0, 3, 0, 31, 0,   31, 199, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},
		{/*2*/0x7/*ContrastReg*/  , 0x7/*ContrastReg*/   , 2,    16,   0,  64,     0, 1, 0, 3, 0, 40, 0,   40,   0, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},
		{/*3*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*4*/MainsFreqReg        , MainsFreqReg         , 2,    0,    0,    1,    0, 1, 0, 3, 0,   1, 0,   1,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*5*/HuectrlRegGr        , HuectrlRegBlu        , 2,    0,    0,  255,    0, 1, 0, 3, 0, 128, 0,   0,   0, I2C_DevAdd_C6,      CyTrue,  CyTrue, 0},  //Hue control
		{/*6*/SaturationRegR      , SaturationRegB       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  50, 0,  50,   0, I2C_DevAdd_F2,      CyTrue,  CyTrue, 0},  //Saturation control
		{/*7*/SharpnessReg        , SharpnessReg         , 2,    0,    0,   14,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},
		{/*8*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*9*/0x8/*WBModeReg*/    , 0x8/*WBModeReg*/     , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //white balance control
		{/*A*/0                   , 0                    , 2,    0,    0,   64,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*B*/ManuBWBReg          , ManuRWBReg           , 4,    0,    0,   64,    0, 1, 0, 3, 0,  32,56,  32,  56, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //white balance component: Red, Blue. Only manual mode
		{/*C*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*D*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*E*/DigZoomReg          , DigZoomReg           , 2,    0,    0,   27,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*F*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // end of the UVC PU
		{/*10*/ShutterReg          , ShutterReg           , 2,    0,    0,   18,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // start of the extension unit (0x10)/ shutter control 0 ~ 0x12
		{/*11*/SenseUpReg          , SenseUpReg           , 2,    0,    0,    9,    0, 1, 0, 3, 0,   0, 0,   1,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // sense up control 0 ~ 0x09
#ifndef CAM720
		{/*12*/MirrModeReg         , MirrModeReg          , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // mirror mode control 0 ~ 0x03
#else
		{/*12*/MirrModeReg         , MirrModeReg          , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // mirror mode control 0 ~ 0x03
#endif
		{/*13*/NoiRedu3DModReg     , NoiRedu3DModReg      , 2,    0,    0,    1,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // 3D noise reduce mode(data0)/level(data1). 0:off 1:on. 0 ~ 0x64
		{/*14*/NoiRedu3DLevReg     , NoiRedu3DLevReg      , 1,    0,    0,   64,    0, 1, 0, 3, 0,  32, 0,  32,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*15*/DayNightModReg      , DayNightModReg       , 2,    0,    0,    2,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // Day night mode. 0:auto 1:day mode 2:night mode
		{/*16*/DayNightDlyReg      , DayNightDlyReg       , 2,    0,    0,   63,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // day night switch delay control. 0 ~ 0x3f second
		{/*17*/DayNightLevReg      , DayNightLevReg       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  16, 0,  16,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // day to night start level. 0 ~ 0x64
		{/*18*/NightDayLevReg      , NightDayLevReg       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  16, 0,  16,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // night to day start level. 0 ~ 0x64
		{/*19*/AExModeReg          , AExAGCReg            , 4,    0,    0,  255,    0, 1, 0, 3, 0,   0,32,   0,  32, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE mode setting & AGC level: 0:auto 1~18:manual; 0 ~ 0xff:level. read(auto), write(menu).
		{/*1A*/AExReferleveReg0    , AExReferleveReg0     , 2,    0,    0,   63,    0, 1, 0, 3, 0,  32, 0,  32,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE reference level 0 ~ 0x40
		{/*1B*/0                   , 0                    , 2,    0,    0,   25,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*1C*/SensorModeReg       , SensorModeReg        , 2,    0,    0,    6,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*1D*/0/*StillImg*/       , 0                    , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*1E*/SeveParsReg         , SeveParsReg          , 1,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0}, //
		/**********************************
		 * the I2C commands control for generic I2C module control.
		 * the data format: wLength 10 bytes, the first part is address. the significant length is presented by first byte.
		 * 					Maximum is 6. so the total length in this part is 7 bytes.
		 * 					The second part is data. the significant length is presented by 8th byte.
		 * 					Maximum is 2. so the total length of this part is 3.
		 * 					Total length of the request is 10 bytes.
		 *
		 *********************************/
		{/*1F*/0/*I2CCtrl*/        , 0                    ,11,    0,    0,  0xff, 0xff, 1, 0, 3, 0,   0, 0,   0,   0,                0,  CyTrue, CyFalse, 0}  // index is 0x1f
};

#ifndef CAM720
	static uint8_t CamMode = 0; //0:1080p
#else
	static uint8_t CamMode = 1; //1:720p
#endif
	static uint8_t setRes = 0;  // 1:1280x960; 2:1280x720; 0:n/a
	static uint8_t setstilRes = 0;  // 1:1280x960; 2:1280x720; 0:n/a

static uint8_t ExUCtrlParArry[16][24]={
		{/*20 set Iris auto (AF Lens)*/0,               0   , 4,    0x1,    0, 0x38, 0x01, 1, 0, 3, 0,0x4e, 0,0x4e,   0, I2C_EAGLESDP_ADDR,   CyTrue, CyFalse, 0},   //
		{/*21 set Iris auto (non AF Lens)*/0,           0   , 1,    0,    0,    0,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,     CyTrue, CyFalse, 0},
		{/*22 set Iris value (DC manual)*/0,            0   , 2,    0,    0,  255,    0, 1, 0, 3, 0,   1, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{/*23 opt zoom*/0,                              0   , 2,    0,    0,    0,    0, 0, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{/*24*/0x13/*Ext1BLCRangeCtlID4 position*/ , 0x14/*size*/ , 2,    1,    0,    3,    0, 1, 0, 3, 0, 0x23, 0x37, 0x23, 0x37, I2C_EAGLESDP_ADDR,     CyTrue, CyFalse, 0},
		{/*25*/0x11/*Ext1BLCWeightCtlID5*/         , 0   , 2,    1,    0,    3,    0, 1, 0, 3, 0,   1, 0,   1,   0, I2C_EAGLESDP_ADDR,     CyTrue,  CyTrue, 0},
		{/*26*/0x17/*Ext1BLCGridCtlID6*/           , 0   , 1,    1,    0,    2,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,     CyTrue,  CyTrue, 0},
		{/*27*/0,                                     0   , 4,    0x1,    0, 0x38, 0x01, 1, 0, 3, 0,0x4e, 0,0x4e,   0, I2C_EAGLESDP_ADDR,   CyTrue, CyFalse, 0},   //ExTmACtlID3
		{/*28*/0,                                     0   , 1,    0,    0,    0,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,     CyTrue, CyFalse, 0},
		{/*29*/0,                                     0   , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{/*2A*/0,                                     0   , 3,    0,    0,   10,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*2B*/0                   , 0                    , 2,    0,    0,   64,    0, 1, 0, 3, 0,  15, 17,  0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{/*2C*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*2D*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*2E*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*2F*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0}  // end of the UVC CT
};

/*      RegAdd1,             RegAdd2,              length, Min1,  Min2, Max1, Max2, Res1, Res2, InfoReq1, InfoReq2, DefReq1, DefReq2,
 *            curVal1, curVal2 (index:14th), device address, checked flag, command available flag*/
static uint8_t CTCtrlParArry[16][24]={
		{ScanMCtlID0            , 0                    , 1,    0,    0,    3,    0, 1, 0, 3, 0,   3, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{ShutterReg             , ShutterReg           , 1,    1,    0,   15,    0,15, 0, 3, 0,   2, 0,   2,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},
		{AutoExPCtlID2          , 0                    , 1,    0,    0,    1,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},
		{ShutterReg             , ShutterReg           , 4,    0x1,    0, 0x38, 0x01, 1, 0, 3, 0,0x4e, 0,0x4e,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},   //ExTmACtlID3
		{ExTmRCtlID4            , 0                    , 1,    0,    0,    0,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{FocACtlID5             , 0                    , 2,    0,    0,  255,    0, 1, 0, 3, 0,   1, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{FocRCtlID6             , 0                    , 2,    0,    0,    0,    0, 0, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{IrisAFReg              , 0                    , 2,    0,    0,   48,    0, 1, 0, 3, 0x0a,0, 0, 0xa,   0, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},  //IriACtlID7
		{IriRCtlID8             , 0                    , 1,    0,    0,  127,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{ZmOpACtlID9            , 0                    , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{OpZoomReg              , 0                    , 3,    0,    0,    0,    0, 0, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,   64,    0, 1, 0, 3, 0,  15, 17,  0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0}  // end of the UVC CT
};
static uint16_t ShutValueArry[8]={200, 100, 39, 20, 10, 5, 2, 1};
static uint8_t ExTime[8][2]={{0x9c, 0x00}, {0x4e, 0x00}, {0x27, 0x00}, {0x14, 0x00}, {0x0a, 0x00}, {0x05, 0x00}, {0x02, 0x00}, {0x01, 0x00}};

static uint8_t curFlag[64]={0}; //the curFlag for each controls current records available. 0: unable. the data should be read from sensor and put into the records. 1: available. the data is read from records.

/*
 * WBMenuCmpArry is set for white storing balance component requests values.
 * first two bytes represent blue and last two are for red. The defaults are set to 0.
 */
static uint8_t WBMenuCmpArry[4]={
		0xa0, 0x0f, 0xf, 0xf0
};
static uint8_t I2CCMDArry[12]={//the index 12 points to data available; 0: no used; 0xf: unavailable; 0xff: available.
		0
};

void I2CCmdHandler(){
	uint8_t buf[2];
	uint8_t CmdType, CmdRegLen, CmdDataLen;
	CmdType = I2CCMDArry[0];
	CmdRegLen = I2CCMDArry[1];
	CmdDataLen = I2CCMDArry[8];
	VdRingBuf *cmdQuptr = &cmdQu;

	CyU3PDebugPrint (4, "The I2C command is 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
			I2CCMDArry[0], I2CCMDArry[1], I2CCMDArry[2], I2CCMDArry[3], I2CCMDArry[4], I2CCMDArry[5],
			I2CCMDArry[6], I2CCMDArry[7], I2CCMDArry[8], I2CCMDArry[9], I2CCMDArry[10]);

	if(CmdType == 0)//I2C read
	{
		I2CCMDArry[11] = 0xf; //setting I2C data is not available.
#if 0 //for debugging
		/* test still image operation */
		if(I2CCMDArry[2] == 0xff){
			snapButFlag = 0; //press
			//CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR); //set sending interrupt status event for snap button press
		}else if(I2CCMDArry[2] == 0x0){
			snapButFlag = 0xf; //release
			//CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR); //set sending interrupt status event for snap button release
		}

		/* end of the test */
#endif
		if(1||(CmdRegLen == 4)){
			SensorRead2B(I2CCMDArry[2]|I2C_RD_MASK, I2CCMDArry[3]|I2C_RD_MASK, I2CCMDArry[4], I2CCMDArry[5], I2CCMDArry[8], buf);
			I2CCMDArry[9] = buf[0];
			if(CmdDataLen == 2){
				I2CCMDArry[10] = buf[1];
			}
			I2CCMDArry[11] = 0xff; //setting I2C data is available.
		}else{//not support currently
			CyU3PDebugPrint (4, "The I2C command length is not supported. value %d\r\n", CmdRegLen);
		}
	}else if(CmdType == 1){
		if(1||(CmdRegLen == 4)){//TODO cmdque mutual
			buf[0] = I2CCMDArry[9];
			buf[1] = I2CCMDArry[10];
			if(0 && (I2CCMDArry[3]&I2C_WR_MASK)==0x82 && (I2CCMDArry[4]==0x30) && (I2CCMDArry[5]==0x10)){
				CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);
				cmdSet(cmdQuptr, 0x23, 0x10, 0x30, STOP, 0);
				CyU3PMutexPut(cmdQuptr->ringMux);
			}
			else SensorWrite2B(I2CCMDArry[2]&I2C_WR_MASK, I2CCMDArry[3]&I2C_WR_MASK, I2CCMDArry[4], I2CCMDArry[5], I2CCMDArry[8], buf);
		}else{//not support currently
			CyU3PDebugPrint (4, "The I2C command length is not supported. value %d\r\n", CmdRegLen);
		}

	}
}

inline void ControlHandle(uint8_t CtrlID){
    CyU3PReturnStatus_t apiRetStatus = !CY_U3P_SUCCESS;
    VdRingBuf *cmdQuptr = &cmdQu;
    uint16_t readCount;
    uint8_t RegAdd0, RegAdd1, Data0, Data1, Len, idx, locCtrlID;
    uint8_t devAdd;
    locCtrlID = CtrlID-EXUAOFFSET+4;
    if(CtrlID >= EXUAOFFSET){//the extension command over 32.
    	RegAdd0 = ExUCtrlParArry[locCtrlID][0];
        RegAdd1 = ExUCtrlParArry[locCtrlID][1];
        devAdd = ExUCtrlParArry[locCtrlID][15];
        Len = ExUCtrlParArry[locCtrlID][2];
    }else{
		RegAdd0 = CtrlParArry[CtrlID][0];
		RegAdd1 = CtrlParArry[CtrlID][1];
		devAdd = CtrlParArry[CtrlID][15];
		Len = CtrlParArry[CtrlID][2];
    }
    uint8_t dataIdx, getData=0xFF, getData1=0xff, sendData=0xff, sendData1=0xFF, reqData;
#ifdef USB_DEBUG_PRINT
    CyU3PDebugPrint (4, "The cur sensor value %d 0x%x 0x%x\r\n", CtrlID, CtrlParArry[CtrlID][13], CtrlParArry[CtrlID][14]); // additional debug
#endif
    reqData = bRequest;
    /*
     * Ext manual mode is not supported in 1080p camera
     */
    if (0 && (CtrlID == ExtAexModCtlID9)){
    	//CyU3PDebugPrint (4, "The Aex manual mode and AGC level are not support with 1080p camera.\r\n");
    	goto EndofSet;
    }
    switch (bRequest)
		 {

		 case CY_FX_USB_UVC_GET_LEN_REQ: /* the length of get length request always setting to 2 */
			  glEp0Buffer[0] = Len;
			  glEp0Buffer[1] = 0;
			  CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_CUR_REQ: /* Current value. */

			 switch(CtrlID)
			 {
			 	 if(CtrlID >= EXUAOFFSET){
			 	 	 case Ext1BLCRangeCtlID4:
			 	 	 case Ext1BLCWeightCtlID5:
			 	 	 case Ext1BLCGridCtlID6:
						 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][13];//ext_control array;
						 glEp0Buffer[1] = ExUCtrlParArry[locCtrlID][14];
						 sendData = glEp0Buffer[0];
						 sendData1 = glEp0Buffer[1];
			 	 		 break;
			 	 }
			 	 case ExtCamMCtlID12:
					 sendData = CtrlParArry[CtrlID][13];
					 if(CamMode == 1){//720p or invendo
						if(sendData >= 3){
							CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
							sendData = 0; //set back to default
							CtrlParArry[CtrlID][13] = 0;
						}
						sendData += 4;
					 }
					//CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
					 glEp0Buffer[0] = sendData;
					 glEp0Buffer[1] = 0;
					 break;
			 	 case ExtI2CCtlID15:
			 		 for(idx=0; idx<Len; idx++){
			 			glEp0Buffer[idx] = I2CCMDArry[idx];
			 		 }
			 		 sendData = glEp0Buffer[9];
			 		 sendData1 = glEp0Buffer[10];
#ifdef USB_DEBUG_PRINT
			 		CyU3PDebugPrint (4, "The I2C command is 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
			 				I2CCMDArry[0], I2CCMDArry[1], I2CCMDArry[2], I2CCMDArry[3], I2CCMDArry[4], I2CCMDArry[5],
			 				I2CCMDArry[6], I2CCMDArry[7], I2CCMDArry[8], I2CCMDArry[9], I2CCMDArry[10]);
#endif
			 		 if(I2CCMDArry[11] != 0xff)//the data availabel.
			 		 {
			 			CyU3PDebugPrint (4, "The I2C current data is not available. try again. %d %d\r\n", I2CCMDArry[9], I2CCMDArry[10]);
			 		 }
			 		 break;
				 case ExtAexModCtlID9:

		 	 		 if(curFlag[CtrlID]){
						 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//exposure mode
						 glEp0Buffer[2] = CtrlParArry[CtrlID][14];//AGC
		 	 		 }else{
		 	 			//remove for invendo
		 	 			//glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
		 	 			//glEp0Buffer[0] = glEp0Buffer[0]&0x3; // get least two bits for Aex Mode
		 	 			//CtrlParArry[CtrlID][13] = glEp0Buffer[0];

		 	 			glEp0Buffer[2] = SensorGetControl(RegAdd1, devAdd);
		 	 			CtrlParArry[CtrlID][14] = glEp0Buffer[2];
		 	 			curFlag[CtrlID] = CyTrue;
		 	 		 }

					 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//exposure mode
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = CtrlParArry[CtrlID][14];//AGC
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 sendData1 = glEp0Buffer[2];
					 CyU3PDebugPrint (4, "ExpM&AGC sent to host. %d %d; %d %d\r\n", glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3]);
					 break;
#if 0	//the brightness is placed by Axreference for invendo camera
			 	 case BrgtCtlID1:

		 	 		 if(curFlag[CtrlID]){
						 Data0 = CtrlParArry[CtrlID][13];  //SensorGetControl(RegAdd0, devAdd); //SensorGetBLCMode();
						 Data1 = CtrlParArry[CtrlID][14];  //SensorGetControl(RegAdd1, devAdd);
		 	 		 }else{
		 	 			glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
		 	 			glEp0Buffer[0] = glEp0Buffer[0]&0x3; // get least two bits for Aex Mode
		 	 			CtrlParArry[CtrlID][13] = glEp0Buffer[0];

		 	 			glEp0Buffer[2] = SensorGetControl(RegAdd1, devAdd);
		 	 			CtrlParArry[CtrlID][14] = glEp0Buffer[2];
		 	 			curFlag[CtrlID] = CyTrue;
		 	 		 }

					 if (Data1&0x2){ //check the sign bit (bit1)
						 Data1 = ((Data1<<6)&0x40)| (Data0 >> 2);//clear MSB
					 }else{
						 Data1 = ((Data1<<6)|0x80)| (Data0 >> 2);//set MSB
					 }

					 glEp0Buffer[0] = Data1;
					 glEp0Buffer[1] = 0;
					 sendData = glEp0Buffer[0];
					 break;
#endif
				 case HueCtlID5://TODO check sensor register
					 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//SensorGetControl(HuectrlRegRed, devAdd);
					 glEp0Buffer[0] = glEp0Buffer[0] + GREEN_BASE;
					 glEp0Buffer[1] = 0;
					 sendData = glEp0Buffer[0];
					 break;
				 case WBTLevCtlID11:

		 	 		 if(curFlag[CtrlID]){
						 glEp0Buffer[0] = WBMenuCmpArry[0];//using for blue part
						 glEp0Buffer[2] = WBMenuCmpArry[2];//using for red part
		 	 		 }else{
		 	 			Data0 = SensorGetControl(RegAdd1, devAdd);
		 	 			Data1 = SensorGetControl(RegAdd0, devAdd);
						glEp0Buffer[0] = Data0;
						WBMenuCmpArry[0] = glEp0Buffer[0];//using for blue part
						glEp0Buffer[2] = Data1;
						WBMenuCmpArry[2]= glEp0Buffer[2];//using for red part
		 	 			curFlag[CtrlID] = CyTrue;
		 	 		 }

					 glEp0Buffer[1] = 0;
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 sendData1 = glEp0Buffer[2];
					 break;
				 case SaturCtlID6://TODO check sensor register
				 default:

		 	 		 if(curFlag[CtrlID]){
						 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//ext_control array;
						 glEp0Buffer[1] = CtrlParArry[CtrlID][14];
		 	 		 }else{
		 	 			glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
		 	 			CtrlParArry[CtrlID][13] = glEp0Buffer[0];
		 	 			glEp0Buffer[1] = CtrlParArry[CtrlID][14];
		 	 			curFlag[CtrlID] = CyTrue;
		 	 		 }

					 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//SensorGetControl(RegAdd0, devAdd);
					 glEp0Buffer[1] = 0;
					 sendData = glEp0Buffer[0];
					 break;
			 }

			 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);

#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The get sensor value %d 0x%x 0x%x, %d\r\n", CtrlID, CtrlParArry[CtrlID][13], CtrlParArry[CtrlID][14], glEp0Buffer[0]); // additional debug
#endif
			  break;
		 case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum BLC = 0. */
		 	 if(CtrlID >= EXUAOFFSET){
				 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][3];//ext_control array;
				 glEp0Buffer[1] = ExUCtrlParArry[locCtrlID][4];
		 	 }

		 	 else if(CtrlID == WBTLevCtlID11){
				 glEp0Buffer[0] = 1;//WBMenuCmpArry[0];//using for blue part
				 glEp0Buffer[1] = 0;
				 glEp0Buffer[2] = 1;//WBMenuCmpArry[2];//using for red part
				 glEp0Buffer[3] = 0;
			 }else
			 {
			  glEp0Buffer[0] = CtrlParArry[CtrlID][3];
			  glEp0Buffer[1] = CtrlParArry[CtrlID][4];
			 }
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_MAX_REQ:
		 	 if(CtrlID >= EXUAOFFSET){
				 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][5];//ext_control array;
				 glEp0Buffer[1] = ExUCtrlParArry[locCtrlID][6];
		 	 }
		 	 else if(CtrlID == WBTLevCtlID11){
				 glEp0Buffer[0] = 0xff;//WBMenuCmpArry[0];//using for blue part
				 glEp0Buffer[1] = 0;
				 glEp0Buffer[2] = 0xff;//WBMenuCmpArry[2];//using for red part
				 glEp0Buffer[3] = 0;
			 }else
			 {
				  glEp0Buffer[0] = CtrlParArry[CtrlID][5];
				  glEp0Buffer[1] = CtrlParArry[CtrlID][6];
			 }
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_RES_REQ:
		 	 if(CtrlID >= EXUAOFFSET){
				 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][7];//ext_control array;
				 glEp0Buffer[1] = ExUCtrlParArry[locCtrlID][8];
				 glEp0Buffer[2] = 0;
				 glEp0Buffer[3] = 0;
		 	 }
		 	 else{
			  glEp0Buffer[0] = CtrlParArry[CtrlID][7];
			  glEp0Buffer[1] = CtrlParArry[CtrlID][8];
			  glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
		 	 }
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_INFO_REQ:
		 	 if(CtrlID >= EXUAOFFSET){
				 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][9];//ext_control array;
		 	 }
		 	 else{
			  glEp0Buffer[0] = CtrlParArry[CtrlID][9];
		 	 }
			  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  Len = 1;
			  break;
		 case CY_FX_USB_UVC_GET_DEF_REQ:
		 	 if(CtrlID >= EXUAOFFSET){
				 glEp0Buffer[0] = ExUCtrlParArry[locCtrlID][11];//ext_control array;
				 glEp0Buffer[1] = ExUCtrlParArry[locCtrlID][12];
		 	 }

		 	 else if(CtrlID == WBTLevCtlID11){
				  glEp0Buffer[0] = CtrlParArry[CtrlID][11];
				  glEp0Buffer[1] = 0;
				  glEp0Buffer[2] = CtrlParArry[CtrlID][12];
				  glEp0Buffer[3] = 0;
			 }else{
			  glEp0Buffer[0] = CtrlParArry[CtrlID][11];
			  glEp0Buffer[1] = CtrlParArry[CtrlID][12];
			 }
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_SET_CUR_REQ:
			  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				  glEp0Buffer, &readCount);
			  if (apiRetStatus == CY_U3P_SUCCESS )
			   {
				  Data0 = glEp0Buffer[0];
				  Data1 = glEp0Buffer[1];
				  getData = glEp0Buffer[0];
				  getData1 = glEp0Buffer[2];
#ifdef USB_DEBUG_PRINT
				  CyU3PDebugPrint (4, "The setup sensor value (0) %d 0x%x 0x%x 0x%x\r\n", CtrlID, readCount, glEp0Buffer[0], glEp0Buffer[1]); // additional debug
#endif
				  switch(CtrlID)
					 {
						 case ExtShutCtlID0:
							 CtrlParArry[CtrlID][13] = Data0;
							 if(Data0 == 0){//set exposure mode auto
								 if((CTCtrlParArry[AutoExMCtlID1][13] != 8) && (CTCtrlParArry[AutoExMCtlID1][13] != 2)){
									 if(CTCtrlParArry[AutoExMCtlID1][13] == 1) {
										 CTCtrlParArry[AutoExMCtlID1][13] = 8; //aperture priority
									 }else{
										 CTCtrlParArry[AutoExMCtlID1][13] = 2; //auto mode
									 }
								 }
							 }else{
								 Data1 = Data0 - 1;
								 if((CTCtrlParArry[AutoExMCtlID1][13] != 1) && (CTCtrlParArry[AutoExMCtlID1][13] != 4)){
									 if(CTCtrlParArry[AutoExMCtlID1][13] == 8) {
										 CTCtrlParArry[AutoExMCtlID1][13] = 1; //manual mode
									 }else{
										 CTCtrlParArry[AutoExMCtlID1][13] = 4; //shutter priority
									 }
								 }
								 if(Data1 < 8){
									 CTCtrlParArry[ExTmACtlID3][13] = ExTime[Data1][0];
									 CTCtrlParArry[ExTmACtlID3][14] = ExTime[Data1][1];
								 }else{
									 CTCtrlParArry[ExTmACtlID3][13] = ExTime[7][0];
									 CTCtrlParArry[ExTmACtlID3][14] = ExTime[7][1];
								 }
							 }
							 CtrlParArry[CtrlID][16] = CyTrue;
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 //CyU3PDebugPrint (4, "The shutter&exposure 0x%x 0x%x 0x%x ox%x\r\n", Data1, Data0, CTCtrlParArry[ExTmACtlID3][13], CtrlParArry[CtrlID][13]);
							 break;
						 case ExtAexModCtlID9://exposure&AGC
							 CtrlParArry[CtrlID][13] = getData;//exposure mode
							 CtrlParArry[CtrlID][14] = getData1;//AGC
							 CtrlParArry[CtrlID][16] = CyTrue;
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 //cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData, dataIdx);  //Exposure
							 if(1 || (getData != 0)){
								 //dataIdx++;
								 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, getData1, dataIdx);  //AGC
							 }
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CyU3PDebugPrint (4, "ExpM&AGC gotten from host. %d %d; %d %d\r\n", glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3]);
							 break;
						 case ExtExRefCtlID10:
							 dataIdx = 0;
							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 if(WDRflag)
								 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data0, dataIdx);  //First
							 else
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CyU3PDebugPrint (4, "Exe level. %d %d; %d %d\r\n", Data0, WDRflag, glEp0Buffer[2], glEp0Buffer[3]);
						 case ExtCamMCtlID12:
							 /*
							 dataIdx = 0;
							 if(Data0 <= 3){
								 CamMode = 0; //set 1080p flag
								 Data1 = Data0;
							 }else{
								 CamMode = 1; //set 720p flag
								 Data1 = Data0-4;
							 }
							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[BLCIndex][13] = Data1;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 //CyU3PDebugPrint (4, "The CamMode value %d %d %d %d\r\n", Data1, Data0, CamMode, CtrlParArry[CtrlID][13]);
							 */
							 break;
						 case ExtSensorParCtlID14://TODO
							 dataIdx = 0;
							 if(Data0 == 0){ //set default sensor parameters.
								 Data0 = 1;
							 }else{ //save current sensor parameters.
								 Data0 = 0;
							 }
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;
						 case ExtI2CCtlID15:
					 		 for(idx=0; idx<Len; idx++){
					 			I2CCMDArry[idx] = glEp0Buffer[idx];
					 		 }
					 		I2CCmdHandler();
							 break;
						 case Ext1BLCRangeCtlID4: //registers value BLD window enable (0x17); position (0x13); size (0x14).
							 dataIdx = 0;
#if 0 //seperate version
							 getData = Data0&0xF; //get LSB H-Pos.
							 getData1 = Data0>>4; //get MSB V-Pos.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 if(getData1&0x8){//enable BLD window
								 cmdSet(cmdQuptr, CtrlID, 0x17, devAdd, 1, dataIdx); //show BLC window
							 }else{ //disable BLD window
								 cmdSet(cmdQuptr, CtrlID, 0x17, devAdd, 0, dataIdx); //close BLC window
							 }
							 getData1 = getData1&0x7; //mask bit7 ~ bit3/
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData, dataIdx);  //set H-Pos
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData1, dataIdx);  //set V-Pos
							 dataIdx++;
							 getData = Data1&0xf; //get LSB H-size.
							 getData1 = Data1>>4; //get MSB V-size.
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, getData, dataIdx);  //set H-size
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, getData1, dataIdx);  //set V-size
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
#else //combination version
							 Data0 = Data0&0x7F; //mask window show flag bit.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
						     /* end test */
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set H/V-Pos
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //set H/V-size
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 getData1 = Data1;
#endif
							 ExUCtrlParArry[locCtrlID][13] = Data0;//ext_control array;
							 ExUCtrlParArry[locCtrlID][14] = Data1;
							 ExUCtrlParArry[locCtrlID][16] = CyTrue;
							 break;
						 case Ext1BLCWeightCtlID5: //register value 0x11 (need check).
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set weight factor
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 ExUCtrlParArry[locCtrlID][13] = Data0;
							 ExUCtrlParArry[locCtrlID][16] = CyTrue;
							 break;
						 case Ext1BLCGridCtlID6:
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set grid status
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 ExUCtrlParArry[locCtrlID][13] = Data0;
							 ExUCtrlParArry[locCtrlID][16] = CyTrue;
							 break;
#if 0	//the brightness is placed by Axreference for invendo camera
				  	  	 case BrgtCtlID1:
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							  /****** double check the register0 Data1 ******/
							  if(Data0&0x80){
								  Data1 = ((Data0 >> 6)&0x01)|(CtrlParArry[CtrlID][14]&0xfc);
							  }else{
								  Data1 = ((Data0 >> 6)|0x02)|(CtrlParArry[CtrlID][14]&0xfc);
							  }
							 Data1 |= ~0x03;
							 Data1 &= 0xC7;
						  	 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //First
						  	 dataIdx++;

							 Data0 = (Data0 << 2);
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);   //Second
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[CtrlID][14] = Data1;
							 CtrlParArry[CtrlID][16] = CyTrue;

							 break;
#endif
						 case HueCtlID5:  //mapping to hue control registers
							 dataIdx = 0;

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, (Data0-GREEN_BASE), dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegMg, devAdd, (Data0-MAGENTA_BASE), dataIdx);  //Second
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegYel, devAdd, (Data0-YELLOW_BASE), dataIdx);  //Third
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegCy, devAdd, (Data0-CYAN_BASE), dataIdx);  //Fourth
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegRed, devAdd, (Data0-RED_BASE), dataIdx);  //Fifth
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, (glEp0Buffer[0]-BLUE_BASE), dataIdx);   //Sixth
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 CtrlParArry[CtrlID][13] = glEp0Buffer[0] - GREEN_BASE;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;
						 case SaturCtlID6:
							 dataIdx = 0;
							 Data1 = Data0 = glEp0Buffer[0]; //red and blue set the same value.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data0, dataIdx);  //Second
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;

						 case WBTLevCtlID11:
							 Data0 = glEp0Buffer[0]; //blue to 0x9 or low to 0xa
							 Data1 = glEp0Buffer[2]; //red to 0xa or high to 0x9
							 dataIdx = 0;

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data0, dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data1, dataIdx);  //Second
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 WBMenuCmpArry[0] = Data0;//using for blue part
							 WBMenuCmpArry[2] = Data1;//using for red part
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;
						 case MFreqCtlID4:
							 dataIdx = 0;
							 Data0 = Data0 - 1;
							 if(Data0 < 0)  //for specific check. if it's minor value, set to 0.
								 Data0 = 0;
							 else if(Data0 >2)
								 Data0 = 1;

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 //remove for Invendo
							 //cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;
					 	 case BLCCtlID0:
							 CtrlParArry[CtrlID][13] = Data0;
							 if(Data0 == 3)
								 WDRflag = CyTrue; //WDR mode
							 else
								 WDRflag = CyFalse;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 if(CamMode == 1) //mode 720p
							 {
								 if(Data0 < 2){
					 				 ;//Data0 += 4;
					 			 }else{
									CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, getData);
									Data0 = 0; //set to default.
					 			 }
					 		 }
							 //CtrlParArry[CamModeIndex][13] = Data0+4;
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CyU3PDebugPrint (4, "BLC set. %d %d; %d %d\r\n", Data0, WDRflag, glEp0Buffer[2], glEp0Buffer[3]);

					 		 break;
						 default:
							 dataIdx = 0;

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 CtrlParArry[CtrlID][13] = Data0;
							 CtrlParArry[CtrlID][16] = CyTrue;
							 break;
					 }
			   }else{
				   CyU3PDebugPrint (4, "The get data from host fail error code %d.\r\n", apiRetStatus);
			   }
#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The setup sensor value %d, 0x%x 0x%x 0x%x 0x%x %d\r\n", CtrlID, readCount, Data0, Data1, CtrlParArry[CtrlID][14], 0xff); // additional debug
#endif

			  break;
		  default:
			  CyU3PUsbStall (0, CyTrue, CyFalse);
			  break;
		 }
EndofSet:    CyU3PDebugPrint (4, "The Request 0x%x parameter get from host 0x%x 0x%x / send to host 0x%x 0x%x\r\n", reqData, getData, getData1, sendData, sendData1);
}

/* Application critical error handler */
    void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t status        /* API return status */
        )
{
    /* Application failed with the error code status */

    /* Add custom debug or recovery actions here */

    /* Loop indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}


/* UVC header addition function */
    static void
esUVCUvcAddHeader (
        uint8_t *buffer_p,      /* Buffer pointer */
        uint8_t frameInd        /* EOF or normal frame indication */
        )
{
    /* Copy header to buffer */
    CyU3PMemCopy (buffer_p, (uint8_t *)glUVCHeader, CY_FX_UVC_MAX_HEADER);

    /* Check if last packet of the frame. */
    if (frameInd == CY_FX_UVC_HEADER_EOF)
    {
        /* Modify UVC header to toggle Frame ID */
        glUVCHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;

        /* Indicate End of Frame in the buffer */
        buffer_p[1] |=  CY_FX_UVC_HEADER_EOF;
    }
}


/* This function starts the video streaming application. It is called
 * when there is a SET_INTERFACE event for alternate interface 1
 * (in case of UVC over Isochronous Endpoint usage) or when a
 * COMMIT_CONTROL(SET_CUR) request is received (when using BULK only UVC).
 */
    CyU3PReturnStatus_t
esUVCUvcApplnStart (void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    glIsApplnActive = CyTrue;
    glDmaDone = 0;
    glDMATxCount = 0;
    glHitFV = CyFalse;
    doLpmDisable = CyTrue;

#ifdef RESET_TIMER_ENABLE
    CyU3PTimerStop (&UvcTimer);
#endif


    /* Place the EP in NAK mode before cleaning up the pipe. */
    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
    CyU3PBusyWait (100);

    /* Reset USB EP and DMA */
    CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
    status = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4,"\n\rAplnStrt:ChannelReset Err = 0x%x", status);
        return status;
    }

    status = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAplnStrt:SetXfer Err = 0x%x", status);
        return status;
    }

    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
    CyU3PBusyWait (200);
//
//    /* Place the EP in NAK mode before cleaning up the pipe. */
//    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
//    CyU3PBusyWait (100);
//
//    /* Reset USB EP and DMA */
//    CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
//    status = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
//    if (status != CY_U3P_SUCCESS)
//    {
//        CyU3PDebugPrint (4,"\n\rAplnStrt:ChannelReset Err = 0x%x", status);
//        return status;
//    }
//    status = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
//    if (status != CY_U3P_SUCCESS)
//    {
//        CyU3PDebugPrint (4, "\n\rAplnStrt:SetXfer Err = 0x%x", status);
//        return status;
//    }
//    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
//    CyU3PBusyWait (200);

	 /* Night Mode function
	 *  --------------------
	 *  esOV5640_Nightmode API is used to enable the Nightmode
	 *  of OV5640 sensor.
	 *  Set Enable -- Cytrue to enable Nightmode
	 * 				  CyFalse to Disable Nightmode
	 *
	 *  Set NightMode_option -- 1 to 6 to set different night modes
	 *
	 * To test different night modes, uncomment the below statement and build the firmware
	 */
    //TODO Change this Function to "Sensor Specific" Nightmode Function to enable the nightmode(If supported by the sensor)
	/*esOV5640_Nightmode(CyTrue,3);*/

    //TODO Change this Function with "Sensor Specific" AutoFocus Function to Set the AutoFocus of the sensor

    /* Resume the Fixed Function GPIF State machine */
    CyU3PGpifSMControl(CyFalse);

    glActiveSocket = 0;
    CyU3PGpifSMSwitch(257, 0, 257, 0, 2/*ES_UVC_INVALID_GPIF_STATE, CX3_START_SCK0,
    		ES_UVC_INVALID_GPIF_STATE, ALPHA_CX3_START_SCK0, ES_UVC_GPIF_SWITCH_TIMEOUT*/);

    CyU3PThreadSleep(10);

    /* Wake Mipi interface and Image Sensor */
    //CyU3PMipicsiWakeup();

    //TODO Change this function with "Sensor Specific" PowerUp function to PowerUp the sensor
    //esCamera_Power_Up();

    glMipiActive = CyTrue; //(???)

	if(glStillCaptureStart!= CyTrue)
	{
		if(g_IsAutoFocus)
			;//esOV5640_SetAutofocus(g_IsAutoFocus);
	}
    return CY_U3P_SUCCESS;
}

/* This function stops the video streaming. It is called from the USB event
 * handler, when there is a reset / disconnect or SET_INTERFACE for alternate
 * interface 0 in case of ischronous implementation or when a Clear Feature (Halt)
 * request is received (in case of bulk only implementation).
 */
    void
esUVCUvcApplnStop(void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Update the flag so that the application thread is notified of this. */
    glIsApplnActive = CyFalse;

    /* Stop the image sensor and CX3 mipi interface */
    //status = CyU3PMipicsiSleep();

    //TODO Change this function with "Sensor Specific" PowerDown function to PowerDown the sensor
    //esCamera_Power_Down();

    //glMipiActive = CyFalse;

#ifdef RESET_TIMER_ENABLE
    CyU3PTimerStop (&UvcTimer);
#endif

    /* Pause the GPIF interface*/
    //CyU3PGpifSMControl(CyTrue);

    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
    CyU3PBusyWait (100);

    /* Abort and destroy the video streaming channel */
    /* Reset the channel: Set to DSCR chain starting point in PORD/CONS SCKT; set DSCR_SIZE field in DSCR memory*/
    status = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4,"\n\rAplnStop:ChannelReset Err = 0x%x",status);
    }
    CyU3PThreadSleep(25);

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
    /* Clear the stall condition and sequence numbers if ClearFeature. */
    if (glIsClearFeature)
    {
        CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
        glIsClearFeature = CyFalse;
    }
    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
	CyU3PBusyWait (200);

    glDMATxCount = 0;
    glDmaDone = 0;

    /* Enable USB 3.0 LPM */
    CyU3PUsbLPMEnable ();
}

/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
    void
esUVCGpifCB (
        CyU3PGpifEventType event,
        uint8_t currentState
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    /* Handle interrupt from the State Machine */
    if (event == CYU3P_GPIF_EVT_SM_INTERRUPT)
    {
        /* Wrapup Socket 0*/
        if(currentState == PARTIAL_BUF_IN_SCK0)
        {
            status = CyU3PDmaMultiChannelSetWrapUp(&glChHandleUVCStream,0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (4, "\n\rGpifCB:WrapUp SCK0 Err = 0x%x",status);
            }
        }
        /* Wrapup Socket 1 */
        else if(currentState == PARTIAL_BUF_IN_SCK1)
        {
            status = CyU3PDmaMultiChannelSetWrapUp(&glChHandleUVCStream,1);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (4, "\n\rGpifCB:WrapUp SCK1 Err = 0x%x",status);
            }
        }
    }
}


/* DMA callback function to handle the produce and consume events. */
    void
esUVCUvcAppDmaCallback (
        CyU3PDmaMultiChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PDmaBuffer_t DmaBuffer;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is
         * received upon reception of every buffer. The buffer will not be sent
         * out unless it is explicitly committed. The call shall fail if there
         * is a bus reset / usb disconnect or if there is any application error. */

        /* Disable USB 3.0 LPM while Buffer is being transmitted out*/
        if ((CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED) && (doLpmDisable))
        {
            CyU3PUsbLPMDisable ();
            CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);
            CyU3PBusyWait (200);

            doLpmDisable = CyFalse;
        }
#ifdef RESET_TIMER_ENABLE
            CyU3PTimerStart (&UvcTimer);
#endif

        status = CyU3PDmaMultiChannelGetBuffer(chHandle, &DmaBuffer, CYU3P_NO_WAIT);
        while (status == CY_U3P_SUCCESS)
        {
            /* Add Headers*/
            if(DmaBuffer.count < ES_UVC_DATA_BUF_SIZE)
            {
                esUVCUvcAddHeader ((DmaBuffer.buffer - CY_FX_UVC_MAX_HEADER), CY_FX_UVC_HEADER_EOF);
                glHitFV = CyTrue;
            }
            else
            {
                esUVCUvcAddHeader ((DmaBuffer.buffer - CY_FX_UVC_MAX_HEADER), CY_FX_UVC_HEADER_FRAME);
            }

            /* Commit Buffer to USB*/
            status = CyU3PDmaMultiChannelCommitBuffer (chHandle, (DmaBuffer.count + 12), 0);
            if (status != CY_U3P_SUCCESS)
            {
                   CyU3PEventSet(&glFxUVCEvent, ES_TIMER_RESET_EVENT,CYU3P_EVENT_OR);
                   break;
            }
            else
            {
                glDMATxCount++;
                glDmaDone++;
            }

            glActiveSocket ^= 1; /* Toggle the Active Socket */
            status = CyU3PDmaMultiChannelGetBuffer(chHandle, &DmaBuffer, CYU3P_NO_WAIT);
        }
    }
    else if(type == CY_U3P_DMA_CB_CONS_EVENT)
    {
        glDmaDone--;

        /* Check if Frame is completely transferred */
        glIsStreamingStarted = CyTrue;

        if((glHitFV == CyTrue) && (glDmaDone == 0))
        {
            glHitFV = CyFalse;
            glDMATxCount=0;
#ifdef RESET_TIMER_ENABLE
            CyU3PTimerStop (&UvcTimer);
#endif
            CyU3PGpifSMSwitch (257, 0, 257, 0, 2);
            /*
            if (glActiveSocket)
                CyU3PGpifSMSwitch(ES_UVC_INVALID_GPIF_STATE, CX3_START_SCK1,
                		ES_UVC_INVALID_GPIF_STATE, ALPHA_CX3_START_SCK1, ES_UVC_GPIF_SWITCH_TIMEOUT);
            else
                CyU3PGpifSMSwitch(ES_UVC_INVALID_GPIF_STATE, CX3_START_SCK0,
                		ES_UVC_INVALID_GPIF_STATE, ALPHA_CX3_START_SCK0, ES_UVC_GPIF_SWITCH_TIMEOUT);
             */
            CyU3PUsbLPMEnable ();
            doLpmDisable = CyTrue;
#ifdef RESET_TIMER_ENABLE
            CyU3PTimerModify (&UvcTimer, TIMER_PERIOD, 0);
#endif

            if(glStillCaptured == CyTrue)
            {
            	glStillCaptured = CyFalse;
            	glUVCHeader[1]^=CY_FX_UVC_HEADER_STILL_IMAGE;
            	glFrameIndexToSet = glCurrentFrameIndex;
            	CyU3PEventSet(&glFxUVCEvent, ES_TIMER_RESET_EVENT,CYU3P_EVENT_OR);
            }
            if(glStillCaptureStart == CyTrue)
            {
            	if(glStillSkip == 3)
				{
            		glStillSkip--;
            		glFrameIndexToSet = 4;
					CyU3PEventSet(&glFxUVCEvent, ES_TIMER_RESET_EVENT,CYU3P_EVENT_OR);
				}
            	else if(glStillSkip == 0)
            	{
            		glStillCaptureStart = CyFalse;
					glStillCaptured = CyTrue;
					glUVCHeader[1]^=CY_FX_UVC_HEADER_STILL_IMAGE;
            	}
            	else
            		glStillSkip--;
            }
        }
    }
}

/* This is the Callback function to handle the USB Events */
    static void
esUVCUvcApplnUSBEventCB (
        CyU3PUsbEventType_t evtype,     /* Event type */
        uint16_t            evdata      /* Event data */
        )
{
    uint8_t interface = 0, altSetting = 0;

    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SUSPEND:
            /* Suspend the device with Wake On Bus Activity set */
            glIsStreamingStarted = CyFalse;
            CyU3PEventSet (&glFxUVCEvent, ES_USB_SUSP_EVENT_FLAG, CYU3P_EVENT_OR);
            break;
        case CY_U3P_USB_EVENT_SETINTF:
            /* Start the video streamer application if the
             * interface requested was 1. If not, stop the
             * streamer. */
            interface = CY_U3P_GET_MSB(evdata);
            altSetting = CY_U3P_GET_LSB(evdata);

            glIsStreamingStarted = CyFalse;

            if ((altSetting == CY_FX_UVC_STREAM_INTERFACE) && (interface == 1))
            {
                /* Stop the application before re-starting. */
                if (glIsApplnActive)
                {
                	glIsClearFeature = CyTrue;
                    esUVCUvcApplnStop ();
                }
                esUVCUvcApplnStart ();

            }
            else if ((altSetting == 0x00) && (interface == 1))
            {
            	glPreviewStarted = CyFalse;
            	/* Stop the application before re-starting. */
            	glIsClearFeature = CyTrue;
				esUVCUvcApplnStop ();
				glcommitcount = 0;
            }
            break;

            /* Fall-through. */

        case CY_U3P_USB_EVENT_SETCONF:
        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
        case CY_U3P_USB_EVENT_CONNECT:
            glIsStreamingStarted = CyFalse;
            if (evtype == CY_U3P_USB_EVENT_SETCONF)
                glIsConfigured = CyTrue;
            else
                glIsConfigured = CyFalse;

            /* Stop the video streamer application and enable LPM. */
            CyU3PUsbLPMEnable ();
            if (glIsApplnActive)
            {
            	glIsClearFeature = CyTrue;
                esUVCUvcApplnStop ();
            }
            break;
        default:
            break;
    }
}

/* Callback for LPM requests. Always return true to allow host to transition device
 * into required LPM state U1/U2/U3. When data transmission is active LPM management
 * is explicitly disabled to prevent data transmission errors.
 */
static CyBool_t esUVCApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode         /*USB 3.0 linkmode requested by Host */
        )
{
    return CyTrue;
}

//TODO Change this function with "Sensor Specific" function to write the sensor settings & configure the CX3 for supported resolutions
void esSetCameraResolution(uint8_t FrameIndex)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyU3PDebugPrint (4, "\n\resSetCameraResolution");
	/* Super Speed USB Streams*/
	if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
	{
		if(FrameIndex == 0x01)
		{
			/* Write 1080pSettings */
			//status = CyU3PMipicsiSetIntfParams (&cfgUvc1080p30NoMclk, CyFalse);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams SS1 Err = 0x%x", status);
			}
			//esOV5640_1080P_config();
		}
		else if(FrameIndex == 0x02)
		{
			/* Write VGA Settings */
			//status = CyU3PMipicsiSetIntfParams (&cfgUvcVga30NoMclk, CyFalse);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams FS Err = 0x%x", status);
			}
			//esOV5640_VGA_config();
		}
		else if(FrameIndex == 0x03)
		{
			/* Write 720pSettings */
			//status = CyU3PMipicsiSetIntfParams (&cfgUvc720p60NoMclk, CyFalse);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams SS2 Err = 0x%x", status);
			}
			//esOV5640_720P_config();
		}
		else if(FrameIndex == 0x04)
		{
			//status = CyU3PMipicsiSetIntfParams (&cfgUvc5Mp15NoMclk, CyFalse);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams SS2 Err = 0x%x", status);
			}
			//esOV5640_5MP_config();
		}
	}
	/* High Speed USB Streams*/
	else if (CyU3PUsbGetSpeed () == CY_U3P_HIGH_SPEED)
	{
		/* Write VGA Settings */
		//status = CyU3PMipicsiSetIntfParams (&cfgUvcVga30NoMclk, CyFalse);
		if (status != CY_U3P_SUCCESS)
		{
			CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams HS Err = 0x%x", status);
		}
		//esOV5640_VGA_config();
		//esOV5640_VGA_HS_config();
	}
	/* Full Speed USB Streams*/
	else
	{
		/* Write VGA Settings */
		//esOV5640_VGA_config();
		//status = CyU3PMipicsiSetIntfParams (&cfgUvcVga30NoMclk, CyFalse);
		if (status != CY_U3P_SUCCESS)
		{
			CyU3PDebugPrint (4, "\n\rUSBStpCB:SetIntfParams FS Err = 0x%x", status);
		}
	}
}

/* Callback to handle the USB Setup Requests and UVC Class events */
    static CyBool_t
esUVCUvcApplnUSBSetupCB (
        uint32_t setupdat0,     /* SETUP Data 0 */
        uint32_t setupdat1      /* SETUP Data 1 */
        )
{
    uint8_t  bRequest, bType,bRType, bTarget;
    uint16_t wValue, wIndex, wLength;
    uint16_t readCount = 0;
    uint8_t  ep0Buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyBool_t uvcHandleReq = CyFalse;
    uint8_t temp = 0;
    CyBool_t isHandled = CyFalse;
    uint8_t RequestOption = 0;

    /* Decode the fields from the setup request. */
    bRType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bRType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bRType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

#if 1
   	CyU3PDebugPrint(4, "\n\rbRType = 0x%x, bRequest = 0x%x, wValue = 0x%x, wIndex = 0x%x, wLength= 0x%x",bRType, bRequest, wValue, wIndex, wLength);
#endif

    /* ClearFeature(Endpoint_Halt) received on the Streaming Endpoint. Stop Streaming */
    if((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
            && (wIndex == CY_FX_EP_BULK_VIDEO) && (wValue == CY_U3P_USBX_FS_EP_HALT))
    {
        if ((glIsApplnActive) && (glIsStreamingStarted))
        {
        	glPreviewStarted = CyFalse;
            glIsClearFeature = CyTrue;
            esUVCUvcApplnStop();
            glcommitcount = 0;
        }
        return CyFalse;
    }

    if( bRType == CY_U3P_USB_GS_DEVICE)
    {
        /* Make sure that we bring the link back to U0, so that the ERDY can be sent. */
        if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
            CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);
    }

    if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
    {
        if (glIsConfigured)
        {
            CyU3PUsbAckSetup ();
        }
        else
        {
            CyU3PUsbStall (0, CyTrue, CyFalse);
        }
        return CyTrue;
    }

    if ((bRequest == CY_U3P_USB_SC_GET_STATUS) &&
            (bTarget == CY_U3P_USB_TARGET_INTF))
    {
        /* We support only interface 0. */
        if (wIndex == 0)
        {
            ep0Buf[0] = 0;
            ep0Buf[1] = 0;
            CyU3PUsbSendEP0Data (0x02, ep0Buf);
        }
        else
            CyU3PUsbStall (0, CyTrue, CyFalse);
        return CyTrue;
    }

    /* Check for UVC Class Requests */
    if (bType == CY_U3P_USB_CLASS_RQT)
    {

        /* UVC Class Requests */
        /* Requests to the Video Streaming Interface (IF 1) */
		if ((wIndex & 0x00FF) == CY_FX_UVC_STREAM_INTERFACE)
		{
#if 0
			{//if VS, set VS event flag
				uvcHandleReq = CyTrue;
				status = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
					CYU3P_EVENT_OR);
				if (status != CY_U3P_SUCCESS)
				{
					CyU3PDebugPrint(4, "Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed %x\n", status);
					CyU3PUsbStall(0, CyTrue, CyFalse);
				}
			}
#else
	        {
	            /* GET_CUR Request Handling Probe/Commit Controls*/
	            if ((bRequest == CY_FX_USB_UVC_GET_CUR_REQ)||(bRequest == CY_FX_USB_UVC_GET_MIN_REQ) || (bRequest == CY_FX_USB_UVC_GET_MAX_REQ)||(bRequest == CY_FX_USB_UVC_GET_DEF_REQ))
	            {
	                isHandled = CyTrue;
	                if((wValue == CY_FX_UVC_PROBE_CTRL) || (wValue == CY_FX_UVC_COMMIT_CTRL))
	                {
	                	//TODO Modify this "glProbeCtrl" according to the Supported Preview Resolutions that are supported by the sensor

						/* Host requests for probe data of 34 bytes (UVC 1.1) or 26 Bytes (UVC1.0). Send it over EP0. */
						if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
						{
							if(glCurrentFrameIndex == 4)
							{
								CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl5MpProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
							}
							/* Probe Control for 1280x720 stream*/
							else if(glCurrentFrameIndex == 3)
							{
								CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl720pProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
							}
							/* Probe Control for 640x480 stream*/
							else  if(glCurrentFrameIndex == 2)
							{
								CyU3PMemCopy(glProbeCtrl, (uint8_t *)glVga60ProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
							}
							/* Probe Control for 1920x1080 stream*/
							else  if(glCurrentFrameIndex == 1)
							{
								CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl1080pProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
							}

						}
						else if (CyU3PUsbGetSpeed () == CY_U3P_HIGH_SPEED)
						{
							/* Probe Control for 640x480 stream*/
							CyU3PMemCopy(glProbeCtrl, (uint8_t *)glVga30ProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
						}
						else /* FULL-Speed*/
						{
							CyU3PDebugPrint (4, "\n\rFull Speed Not Supported!");
						}

						status = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, glProbeCtrl);
						if (status != CY_U3P_SUCCESS)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:GET_CUR:SendEP0Data Err = 0x%x", status);
						}
	                }
	                else if((wValue == VD_FX_UVC_STILL_PROB_CTRL) || (wValue == VD_FX_UVC_STILL_COMIT_CTRL))
	                {
	                	if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
	                	{
							status = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING, glStillProbeCtrl);
							if (status != CY_U3P_SUCCESS)
							{
								CyU3PDebugPrint (4, "\n\rUSBStpCB:GET_CUR:SendEP0Data Err = 0x%x", status);
							}
	                	}
	                }
	            }
	            /* SET_CUR request handling Probe/Commit controls */
	            else if (bRequest == CY_FX_USB_UVC_SET_CUR_REQ)
	            {
	                isHandled = CyTrue;
	                if((wValue == CY_FX_UVC_PROBE_CTRL) || (wValue == CY_FX_UVC_COMMIT_CTRL))
	                {
						/* Get the UVC probe/commit control data from EP0 */
						status = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
								glCommitCtrl, &readCount);
						if (status != CY_U3P_SUCCESS)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
						}
						/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
						if (readCount > (uint16_t)CY_FX_UVC_MAX_PROBE_SETTING)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
						}
						else
						{
							/* Set Probe Control */
							if(wValue == CY_FX_UVC_PROBE_CTRL)
							{
								glCurrentFrameIndex = glCommitCtrl[3];
							}
							/* Set Commit Control and Start Streaming*/
							else if(wValue == CY_FX_UVC_COMMIT_CTRL)
							{

								if((glcommitcount==0)||(glcheckframe!=glCommitCtrl[3]))
								{
									glcommitcount++;
									glcheckframe=glCommitCtrl[3];
								glCurrentFrameIndex = glCommitCtrl[3];
								glFrameIndexToSet = glCurrentFrameIndex;
								glPreviewStarted = CyTrue;

								//TODO Change this function with "Sensor Specific" function to write the sensor settings & configure the CX3 for supported resolutions
							//	esSetCameraResolution(glCurrentFrameIndex);
								//esSetCameraResolution(glCommitCtrl[3]);//TODO the camera resolution calling

								if (glIsApplnActive)
								{
									if(glcommitcount)
										glIsClearFeature = CyFalse;
									else
										glIsClearFeature = CyTrue;

									esUVCUvcApplnStop();
								}
								esUVCUvcApplnStart();
								}
							}
						}
	                }
	                else if((wValue == VD_FX_UVC_STILL_PROB_CTRL) || (wValue == VD_FX_UVC_STILL_COMIT_CTRL))
	                {
	                	/* Get the UVC STILL probe/commit control data from EP0 */
						status = CyU3PUsbGetEP0Data(CY_FX_STILL_TRIGGER_ALIGNED,glStillCommitCtrl, &readCount);
						if (status != CY_U3P_SUCCESS)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
						}
						/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
						if (readCount > (uint16_t)CY_FX_UVC_MAX_STILL_PROBE_SETTING)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
						}
						else
						{
							/* Set Probe Control */
							if(wValue == VD_FX_UVC_STILL_PROB_CTRL)
							{
								glCurrentStillFrameIndex = glStillCommitCtrl[1];
							}
							/* Set Commit Control and Start Streaming*/
							else if(wValue == VD_FX_UVC_STILL_COMIT_CTRL)
							{
								glCurrentStillFrameIndex = glStillCommitCtrl[1];
							}
						}

	                }
	                else if(wValue == VD_FX_UVC_STILL_TRIG_CTRL)
	                {
						status = CyU3PUsbGetEP0Data(CY_FX_STILL_TRIGGER_ALIGNED,&glStillTriggerCtrl, &readCount);
						if (status != CY_U3P_SUCCESS)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
						}
						/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
						if (readCount > (uint16_t)CY_FX_STILL_TRIGGER_COUNT)
						{
							CyU3PDebugPrint (4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
						}
						else
						{
							if(glStillTriggerCtrl == 0x01)
							{
								glStillSkip = 3;
								glStillCaptureStart = CyTrue;
							}
						}
	                }
	            }
	            else
	            {
	                /* Mark with error. */
	                status = CY_U3P_ERROR_FAILURE;
	            }
	        }
#endif
		}
		else if ((wIndex & 0x00FF) == CY_FX_UVC_CONTROL_INTERFACE) /* Video Control Interface */
		{
#if 0
			{
				uvcHandleReq = CyTrue;
				status = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
					CYU3P_EVENT_OR);
				if (status != CY_U3P_SUCCESS)
				{
					/* Error handling */
					CyU3PDebugPrint(4, "Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n", status);
					CyU3PUsbStall(0, CyTrue, CyFalse);
				}
			}
#endif
		}
    }
    return isHandled;
}


/* This function initializes the USB Module, creates event group,
   sets the enumeration descriptors, configures the Endpoints and
   configures the DMA module for the UVC Application */
    void
esUVCUvcApplnInit (void)
{
    CyU3PEpConfig_t endPointConfig;
    CyU3PDmaMultiChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	//TODP I2C & GPIO & PIB for FX3
    /* Initialize the I2C interface for Mipi Block Usage and Camera. */
    //status = CyU3PMipicsiInitializeI2c (CY_U3P_MIPICSI_I2C_400KHZ);
    if( status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:I2CInit Err = 0x%x.",status);
        CyFxAppErrorHandler(status);
    }

    /* Initialize GPIO module. */
    //status = CyU3PMipicsiInitializeGPIO ();
    if( status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:GPIOInit Err = 0x%x",status);
        CyFxAppErrorHandler(status);
    }

    /* Initialize the PIB block */
    //status = CyU3PMipicsiInitializePIB ();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:PIBInit Err = 0x%x",status);
        CyFxAppErrorHandler(status);
    }

    /* Start the USB functionality */
    status = CyU3PUsbStart();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:UsbStart Err = 0x%x",status);
        CyFxAppErrorHandler(status);
    }
    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(esUVCUvcApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events */
    CyU3PUsbRegisterEventCallback(esUVCUvcApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback (esUVCApplnLPMRqtCB);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscrSS);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_SS_Device_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* High speed device descriptor. */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_HS_Device_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* BOS descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_BOS_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* Device qualifier descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_DEVQUAL_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* Super speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_SS_CFG_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* High speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_HS_CFG_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* Full speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_FS_CFG_Dscr Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* String descriptor 0 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr0 Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* String descriptor 1 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr1 Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* String descriptor 2 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr2 Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }
    /* String descriptor 3 */
    /*
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)esUVCUSBConfigSSDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr3 Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }
	*/
    /* String descriptor 4 */
    /*
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 4, (uint8_t *)esUVCUSBConfigHSDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr4 Err = 0x%x", status);
        CyFxAppErrorHandlerr(status);
    }
    */
    /* String descriptor 2 */
    /*
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 5, (uint8_t *)esUVCUSBConfigFSDscr);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:Set_STRNG_Dscr5 Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }
	*/
    //TODO Change this function with the "Sensor specific" function to Write the Base I2C settings into the sensor
    /* Setup Image Sensor */
	//esOV5640_Base_Config();
	 //TODO Change this function with the "Sensor specific" function to Write the Base I2C settings for autofocus into the sensor
	//esOV5640_Auto_Focus_Config();
	//TODO Change this function with "Sensor Specific" PowerDown function to PowerDown the sensor
	//esCamera_Power_Down();

    /* Connect the USB pins and enable super speed operation */
    status = CyU3PConnectState(CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:ConnectState Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    /* Since the status interrupt endpoint is not used in this application,
     * just enable the EP in the beginning. */
    /* Control status interrupt endpoint configuration */
    endPointConfig.enable = 1;
    endPointConfig.epType = CY_U3P_USB_EP_INTR;
    endPointConfig.pcktSize = 64;
    endPointConfig.isoPkts  = 1;
    endPointConfig.burstLen = 1;

    status = CyU3PSetEpConfig(CY_FX_EP_CONTROL_STATUS, &endPointConfig);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:CyU3PSetEpConfig CtrlEp Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    CyU3PUsbFlushEp(CY_FX_EP_CONTROL_STATUS);

    /* Setup the Bulk endpoint used for Video Streaming */
    endPointConfig.enable = CyTrue;
    endPointConfig.epType = CY_U3P_USB_EP_BULK;

    endPointConfig.isoPkts  = 0;
    endPointConfig.streams = 0;

    CyU3PThreadSleep(1000);

    switch(CyU3PUsbGetSpeed())
    {
        case CY_U3P_HIGH_SPEED:
            endPointConfig.pcktSize = 0x200;
            endPointConfig.burstLen = 1;
            ES_UVC_STREAM_BUF_SIZE 	= CY_FX_UVC_STREAM_BUF_SIZE;
            ES_UVC_DATA_BUF_SIZE 	= CY_FX_UVC_BUF_FULL_SIZE;
            ES_UVC_STREAM_BUF_COUNT	= CY_FX_UVC_STREAM_BUF_COUNT;
            break;

        case CY_U3P_FULL_SPEED:
            endPointConfig.pcktSize = 0x40;
            endPointConfig.burstLen = 1;
            break;

        case CY_U3P_SUPER_SPEED:
        default:
            endPointConfig.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
            endPointConfig.burstLen = 16;
            ES_UVC_STREAM_BUF_SIZE 	= CY_FX_UVC_STREAM_BUF_SIZE;
            ES_UVC_DATA_BUF_SIZE 	= CY_FX_UVC_BUF_FULL_SIZE;
            ES_UVC_STREAM_BUF_COUNT	= CY_FX_UVC_STREAM_BUF_COUNT;
            break;
    }

    status = CyU3PSetEpConfig(CY_FX_EP_BULK_VIDEO, &endPointConfig);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:CyU3PSetEpConfig BulkEp Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    CyU3PUsbEPSetBurstMode (CY_FX_EP_BULK_VIDEO, CyTrue);

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);

    /* Create a DMA Manual OUT channel for streaming data */
    /* Video streaming Channel is not active till a stream request is received */
    dmaCfg.size                 = ES_UVC_STREAM_BUF_SIZE;
    dmaCfg.count                = ES_UVC_STREAM_BUF_COUNT;
    dmaCfg.validSckCount        = 2;

    dmaCfg.prodSckId[0]         = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_0;//ES_UVC_PRODUCER_PPORT_SOCKET_0;
    dmaCfg.prodSckId[1]         = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_1;//ES_UVC_PRODUCER_PPORT_SOCKET_1;

    dmaCfg.consSckId[0]         = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET);//ES_UVC_EP_VIDEO_CONS_SOCKET;
    dmaCfg.dmaMode              = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification         = CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
    dmaCfg.cb                   = esUVCUvcAppDmaCallback;
    dmaCfg.prodHeader           = CY_FX_UVC_PROD_HEADER;
    dmaCfg.prodFooter           = CY_FX_UVC_PROD_FOOTER;
    dmaCfg.consHeader           = 0;
    dmaCfg.prodAvailCount       = 0;

    status = CyU3PDmaMultiChannelCreate (&glChHandleUVCStream, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE , &dmaCfg);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:DmaMultiChannelCreate Err = 0x%x", status);
    }
    CyU3PThreadSleep(100);

    /* Reset the channel: Set to DSCR chain starting point in PORD/CONS SCKT; set
       DSCR_SIZE field in DSCR memory */
    status = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4,"\n\rAppInit:MultiChannelReset Err = 0x%x", status);
    }

    /* TODO same configuration for GPIF and DMA for FX3 */
	/* Configure the Fixed Function GPIF on the CX3 to use a 16 bit bus, and
     * a DMA Buffer of size CX3_UVC_DATA_BUF_SIZE
     */
    //status = CyU3PMipicsiGpifLoad(CY_U3P_MIPICSI_BUS_16, ES_UVC_DATA_BUF_SIZE);
    CyU3PGpifLoad ((CyU3PGpifConfig_t *) &CyFxGpifConfig);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:MipicsiGpifLoad Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }
    CyU3PThreadSleep(50);

    CyU3PGpifRegisterCallback(esUVCGpifCB); //need to check
    CyU3PThreadSleep(50);

    /* Start the state machine. */
    status = CyU3PGpifSMStart (START, ALPHA_START);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:GpifSMStart Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }
    CyU3PThreadSleep(50);

    /* Pause the GPIF*/
    CyU3PGpifSMControl(CyTrue);
#if 0
    /* Initialize the MIPI block */
    status =  CyU3PMipicsiInit();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:MipicsiInit Err = 0x%x", status);
        CyFxAppErrorHandler(status);
    }

    status = CyU3PMipicsiSetIntfParams(&cfgUvcVgaNoMclk, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\rAppInit:MipicsiSetIntfParams Err = 0x%x",status);
        CyFxAppErrorHandler(status);
    }
#endif
#ifdef RESET_TIMER_ENABLE
    CyU3PTimerCreate (&UvcTimer, UvcAppProgressTimer, 0x00, TIMER_PERIOD, 0, CYU3P_NO_ACTIVATE);
#endif

    CyU3PDebugPrint (4, "\n\rFirmware Version: %d.%d.%d.%d",MajorVersion,MinorVersion,SubVersion,SubVersion1);
}

/* This function initializes the debug module for the UVC application */
    void
esUVCUvcApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    status = CyU3PUartInit();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\resUVCUvcApplnDebugInit:CyU3PUartInit failed Error = 0x%x",status);
    }

    /* Set UART Configuration */
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    /* Set the UART configuration */
    status = CyU3PUartSetConfig (&uartConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\resUVCUvcApplnDebugInit:CyU3PUartSetConfig failed Error = 0x%x",status);
    }

    /* Set the UART transfer */
    status = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\resUVCUvcApplnDebugInit:CyU3PUartTxSetBlockXfer failed Error = 0x%x",status);
    }

    /* Initialize the debug application */
    status = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 4/*8*/);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\n\resUVCUvcApplnDebugInit:CyU3PDebugInit failed Error = 0x%x",status);
    }
    CyU3PDebugPreamble (CyFalse);

}

	/* I2C initialization. */
	//static void
	void
		CyFxUVCApplnI2CInit(void)
	{
			CyU3PI2cConfig_t i2cConfig;;
			CyU3PReturnStatus_t status;

			status = CyU3PI2cInit();
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint(4, "I2C initialization failed!\n");
				CyFxAppErrorHandler(status);
			}

			/*  Set I2C Configuration */
			i2cConfig.bitRate = 100000;      /*  100 KHz */
			i2cConfig.isDma = CyFalse;
			i2cConfig.busTimeout = 0xffffffffU;
			i2cConfig.dmaTimeout = 0xffff;

			status = CyU3PI2cSetConfig(&i2cConfig, 0);
			if (CY_U3P_SUCCESS != status)
			{
				CyU3PDebugPrint(4, "I2C configuration failed!\n");
				CyFxAppErrorHandler(status);
			}
		}


/* Entry function for the UVC application thread. */
    void
UVCAppThread_Entry(      //esUVCUvcAppThread_Entry
        uint32_t input)
{
    uint16_t wakeReason;
    uint32_t eventFlag;
    CyU3PReturnStatus_t status;
    uint8_t i;
    /* Initialize the Debug Module */
    esUVCUvcApplnDebugInit();
    //CyU3PDebugPrint(4," the UART init \r\n");

	//while (i++ < 6){// is it too long???
		//CyU3PThreadSleep(500);
	//}

	/* Initialize the I2C interface */
	CyFxUVCApplnI2CInit();

    /* Initialize the UVC Application */
    esUVCUvcApplnInit();

    //for(;;)
	{
		CyU3PDebugPrint(4,"test loop.\r\n");
	}
    for (;;)
    {
        CyU3PEventGet (&glFxUVCEvent,ES_USB_SUSP_EVENT_FLAG|ES_TIMER_RESET_EVENT, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);

        /* Handle TimerReset Event*/
        if( eventFlag & ES_TIMER_RESET_EVENT)
        {
            if (glIsApplnActive)
            {
            	glIsClearFeature = CyFalse;
                esUVCUvcApplnStop();
            }
            if(glPreviewStarted == CyTrue)
            {
            	//TODO Change this function with "Sensor Specific" function to write the sensor settings & configure the CX3 for supported resolutions
            	//esSetCameraResolution(glFrameIndexToSet);//TODO resoltion set
            	esUVCUvcApplnStart();
            }
#ifdef RESET_TIMER_ENABLE
            CyU3PTimerModify (&UvcTimer, TIMER_PERIOD, 0);
#endif
        }
        /* Handle Suspend Event*/
        if(eventFlag & ES_USB_SUSP_EVENT_FLAG)
        {
            /* Place CX3 in Low Power Suspend mode, with USB bus activity as the wakeup source. */
            //CyU3PMipicsiSleep();
            //TODO Change this function with "Sensor Specific" PowerDown function to PowerDown the sensor
            //esCamera_Power_Down();

            status = CyU3PSysEnterSuspendMode (CY_U3P_SYS_USB_BUS_ACTVTY_WAKEUP_SRC, 0, &wakeReason);
#if 0
            if(0/*glMipiActive*/)//need to check
            {
                CyU3PMipicsiWakeup();
                //TODO Change this function with "Sensor Specific" PowerUp function to PowerUp the sensor
                esCamera_Power_Up();
            }
#endif
            continue;
        }
    } /* End of for(;;) */
}

	/*
	* Handler for control requests addressed to the Processing Unit.
	*/

static void
	UVCHandleProcessingUnitRqts(
	void)
{
		uint8_t CtrlAdd;
#ifdef DbgInfo
		CyU3PDebugPrint(4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
#endif
		switch (wValue)
		{
		case CY_FX_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL:
			CtrlAdd = CtrlParArry[BLCCtlID0][0];
			ControlHandle(BLCCtlID0);
			break;
		case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:
			CtrlAdd = CtrlParArry[ExtExRefCtlID10/*BrgtCtlID1*/][0]; //Exreference places brightness.
			ControlHandle(ExtExRefCtlID10/*BrgtCtlID1*/);
			break;
		case CY_FX_UVC_PU_CONTRAST_CONTROL:
			CtrlAdd = CtrlParArry[ConsCtlID2][0];
			ControlHandle(ConsCtlID2);
			break;

		case CY_FX_UVC_PU_GAIN_CONTROL: break;

		case CY_FX_UVC_PU_POWER_LINE_FREQUENCY_CONTROL:
			CtrlAdd = CtrlParArry[MFreqCtlID4][0];
			ControlHandle(MFreqCtlID4);
			break;
		case CY_FX_UVC_PU_HUE_CONTROL:
			CtrlAdd = CtrlParArry[HueCtlID5][0];
			ControlHandle(HueCtlID5);
			break;
		case CY_FX_UVC_PU_SATURATION_CONTROL:
			CtrlAdd = CtrlParArry[SaturCtlID6][0];
			ControlHandle(SaturCtlID6);
			break;
		case CY_FX_UVC_PU_SHARPNESS_CONTROL:
			CtrlAdd = CtrlParArry[ShapCtlID7][0];
			ControlHandle(ShapCtlID7);
			break;
		case CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL://
			//case CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL:
		case CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
			CtrlAdd = CtrlParArry[WBTMdCtlID9][0];
			ControlHandle(WBTMdCtlID9);
			break;
		case CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL:
			CtrlAdd = CtrlParArry[WBTLevCtlID11][0];
			ControlHandle(WBTLevCtlID11);
			break;
		case CY_FX_UVC_PU_DIGITAL_MULTIPLIER_CONTROL:
			CtrlAdd = CtrlParArry[DigZmCtlID14][0];
			ControlHandle(DigZmCtlID14);
			break;

		default:
			/*
			* Only the  control is supported as of now. Add additional code here to support
			* other controls.
			*/
			CyU3PDebugPrint(4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
}

	/*
	* Handler for control requests addressed to the UVC Camera Terminal unit.
	*/
#if 0
static void
	UVCHandleCameraTerminalRqts(
	void)
{
		uint8_t CtrlAdd;

		switch (wValue)
		{
		case CY_FX_UVC_CT_SCANNING_MODE_CONTROL:
			CtrlAdd = CTCtrlParArry[ScanMCtlID0][0];
			CTControlHandle(ScanMCtlID0);
			break;
		case CY_FX_UVC_CT_AE_MODE_CONTROL:
			CtrlAdd = CTCtrlParArry[AutoExMCtlID1][0];
			CTControlHandle(AutoExMCtlID1);
			break;
		case CY_FX_UVC_CT_AE_PRIORITY_CONTROL:
			CtrlAdd = CTCtrlParArry[AutoExPCtlID2][0];
			CTControlHandle(AutoExPCtlID2);
			break;

		case CY_FX_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
			CtrlAdd = CTCtrlParArry[ExTmACtlID3][0];
			CTControlHandle(ExTmACtlID3);
			break;

		case CY_FX_UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL:
			CtrlAdd = CTCtrlParArry[ExTmRCtlID4][0];
			CTControlHandle(ExTmRCtlID4);
			break;
		case CY_FX_UVC_CT_FOCUS_ABSOLUTE_CONTROL:
			CtrlAdd = CTCtrlParArry[FocACtlID5][0];
			CTControlHandle(FocACtlID5);
			break;
		case CY_FX_UVC_CT_FOCUS_RELATIVE_CONTROL:
			CtrlAdd = CTCtrlParArry[FocRCtlID6][0];
			CTControlHandle(FocRCtlID6);
			break;
		case CY_FX_UVC_CT_FOCUS_AUTO_CONTROL:
			break;
		case CY_FX_UVC_CT_IRIS_ABSOLUTE_CONTROL://
			CtrlAdd = CTCtrlParArry[IriACtlID7][0];
			CTControlHandle(IriACtlID7);
			break;

		case CY_FX_UVC_CT_IRIS_RELATIVE_CONTROL:
			CtrlAdd = CTCtrlParArry[IriRCtlID8][0];
			CTControlHandle(IriRCtlID8);
			break;
		case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
			CtrlAdd = CTCtrlParArry[ZmOpACtlID9][0];
			CTControlHandle(ZmOpACtlID9);
			break;
		case CY_FX_UVC_CT_ZOOM_RELATIVE_CONTROL:
			CtrlAdd = CTCtrlParArry[ZmOpRCtlID10][0];
			CTControlHandle(ZmOpRCtlID10);
			break;

		default:
			/*
			* Only the  control is supported as of now. Add additional code here to support
			* other controls.
			*/
			CyU3PDebugPrint(4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}

}
#endif
	/*
	* Handler for UVC Interface control requests.
	*/
static void
	UVCHandleInterfaceCtrlRqts(
	void)
{

		switch (wValue)
		{
		case CY_FX_UVC_POWER_MODE_CTRL: // shutter CONTROL1
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		case CY_FX_UVC_ERROR_CODE_CTRL: // sense up mode CONTROL2
			CyU3PUsbStall(0, CyTrue, CyFalse);
			//ControlHandle(0xff);//for control interface error code control.
			break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		/* No requests supported as of now. Just stall EP0 to fail the request. */

	}

	/*
	* Handler for control requests addressed to the Extension Unit.
	*/
static void
	UVCHandleExtensionUnitRqts(
	void)
{
		uint8_t CtrlAdd;  //set control ID -add

#ifdef DbgInfo
		CyU3PDebugPrint(4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
#endif
		switch (wValue)
		{
		case CY_FX_EXT_CONTROL_1SHUTTER: // shutter CONTROL1
			CtrlAdd = CtrlParArry[ExtShutCtlID0][0];
			ControlHandle(ExtShutCtlID0);
			break;
		case CY_FX_EXT_CONTROL_2SENUPMODE: // sense up mode CONTROL2
			CtrlAdd = CtrlParArry[ExtSenCtlID1][0];
			ControlHandle(ExtSenCtlID1);
			break;
		case CY_FX_EXT_CONTROL_3MIRROR: // mirror mode CONTROL3
			CtrlAdd = CtrlParArry[ExtMirrCtlID2][0];
			ControlHandle(ExtMirrCtlID2);
			break;
		case CY_FX_EXT_CONTROL_43DNOISEREDUC_MODE: //3D noise reduce control CONTROL4
			CtrlAdd = CtrlParArry[Ext3DNReduMCtlID3][0];
			ControlHandle(Ext3DNReduMCtlID3);
			break;
		case CY_FX_EXT_CONTROL_53DNOISEREDUC_CTRL: //3D noise reduce level CONTROL5
			CtrlAdd = CtrlParArry[Ext3DNReduLvCtlID4][0];
			ControlHandle(Ext3DNReduLvCtlID4);
			break;
		case CY_FX_EXT_CONTROL_6DAYNIGHT_MODE: // day night mode CONTROL6
			CtrlAdd = CtrlParArry[ExtDNModCtlID5][0];
			ControlHandle(ExtDNModCtlID5);
			break;
		case CY_FX_EXT_CONTROL_7DAYNIGHT_DELAY: //day night switch delay CONTROL7
			CtrlAdd = CtrlParArry[ExtDNDelytlID6][0];
			ControlHandle(ExtDNDelytlID6);
			break;
		case CY_FX_EXT_CONTROL_8DAYNIGHT_LEVEL: //day to night level CONTROL8
			CtrlAdd = CtrlParArry[ExtDNlevCtlID7][0];
			ControlHandle(ExtDNlevCtlID7);
			break;
		case CY_FX_EXT_CONTROL_9NIGHTDAY_LEVEL: //night to day level CONTROL9
			CtrlAdd = CtrlParArry[ExtNDlevCtlID8][0];
			ControlHandle(ExtNDlevCtlID8);
			break;
		case CY_FX_EXT_CONTROL_10EXPOSURE_MODE: //AEx mode CONTROL10
			if (CamMode == 1){//only 720p support
				CtrlAdd = CtrlParArry[ExtAexModCtlID9][0];
				ControlHandle(ExtAexModCtlID9);
			}
			else/* no support for 1080p camera */
				CyU3PDebugPrint(4, "The host command is not correct for 1080p camera 0x%x 0x%x %d\r\n", wValue, bRequest, CamMode);
			break;
		case CY_FX_EXT_CONTROL_11AEREFERENCE_LEVEL: //AEx reference level CONTROL11
			CtrlAdd = CtrlParArry[ExtExRefCtlID10][0];
			ControlHandle(ExtExRefCtlID10);
			break;
		case CY_FX_EXT_CONTROL_13CAMERA_MODE: //Camera Mode CONTROL13
			CtrlAdd = CtrlParArry[ExtCamMCtlID12][0];
			ControlHandle(ExtCamMCtlID12);
			break;
			//case CY_FX_EXT_CONTROL_14SNAP_SHOT: //Still image set CONTROL14
			//CtrlAdd = CtrlParArry[ExtshotCtlID13][0];
			//ControlHandle(ExtshotCtlID13);
			//break;
		case CY_FX_EXT_CONTROL_15SENSOR_PARS: //Sensor Parameters set CONTROL15
			CtrlAdd = CtrlParArry[ExtSensorParCtlID14][0];
			ControlHandle(ExtSensorParCtlID14);
			break;
		case CY_FX_EXT_CONTROL_16I2C_COMMAND: //I2C commands operation CONTROL16
			CtrlAdd = CtrlParArry[ExtI2CCtlID15][0];
			ControlHandle(ExtI2CCtlID15);
			break;
		case CY_FX_EXT_CONTROL_17BLC_RANGE:   //BLD range CONTROL17
			CtrlAdd = ExUCtrlParArry[Ext1BLCRangeCtlID4 - EXUAOFFSET + 4][0];
			ControlHandle(Ext1BLCRangeCtlID4);
			break;
		case CY_FX_EXT_CONTROL_18BLC_POSITION:   //BLD gain CONTROL18
			CtrlAdd = ExUCtrlParArry[Ext1BLCWeightCtlID5 - EXUAOFFSET + 4][0];
			ControlHandle(Ext1BLCWeightCtlID5);
			break;
		case CY_FX_EXT_CONTROL_18BLC_GRID:   //BLD gain CONTROL19
			CtrlAdd = ExUCtrlParArry[Ext1BLCGridCtlID6 - EXUAOFFSET + 4][0];
			ControlHandle(Ext1BLCGridCtlID6);
			break;
		default:
			/* No requests supported as of now. Just stall EP0 to fail the request. */
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}

	}

	/*
	* Handler for the video streaming control requests.
	*/
static void
UVCHandleVideoStreamingRqts(
void)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyBool_t isHandled = CyFalse;
	uint16_t readCount;
#if 0
	switch (wValue)
	{
	case CY_FX_UVC_PROBE_CTRL:
		switch (bRequest)
		{
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			glEp0Buffer[1] = 0;
			CyU3PUsbSendEP0Data(2, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
		case CY_FX_USB_UVC_GET_MIN_REQ:
		case CY_FX_USB_UVC_GET_MAX_REQ:
		case CY_FX_USB_UVC_GET_DEF_REQ: 	/* There is only one setting per USB speed. */
			if (1 || usbSpeed == CY_U3P_SUPER_SPEED)//supports both SS and HS
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
			}
			else
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
				//if (usbSpeed == CY_U3P_SUPER_SPEED)//for both SS and HS
				{
					/* Copy the relevant settings from the host provided data into the
					active data structure. */
					glProbeCtrl[2] = glCommitCtrl[2];
					glProbeCtrl[3] = glCommitCtrl[3];
					glProbeCtrl[4] = glCommitCtrl[4];
					glProbeCtrl[5] = glCommitCtrl[5];
					glProbeCtrl[6] = glCommitCtrl[6];
					glProbeCtrl[7] = glCommitCtrl[7];
					CyU3PDebugPrint(4, "Get UVC Prob(set) control %d %d %d %d %d %d %d\r\n", readCount,
						glCommitCtrl[0], glCommitCtrl[3], glCommitCtrl[4], glCommitCtrl[5], glCommitCtrl[6], glCommitCtrl[7]);
				}
			}
			break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	case CY_FX_UVC_COMMIT_CTRL:
		switch (bRequest)
		{
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			glEp0Buffer[1] = 0;
			CyU3PUsbSendEP0Data(2, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
			if (1 || usbSpeed == CY_U3P_SUPER_SPEED) //support both SS and HS
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
			}
			else
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			/* The host has selected the parameters for the video stream. Check the desired
			resolution settings, configure the sensor and start the video stream.
			*/
			apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS)//supports both SS and HS
			{
				if (setRes != glCommitCtrl[3])
				{
					switch (glCommitCtrl[3])
					{
					case 1: //720 or 360
						SensorSetIrisControl(0x0b, 0x30, 0x1, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
						CyU3PThreadSleep(500);
						CyU3PDebugPrint(4, "Set the video mode format %x %d\n", 0x1, 0x0b);
						break;
					case 2: //960 or 480
						SensorSetIrisControl(0x0b, 0x30, 0x0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
						CyU3PThreadSleep(500);
						CyU3PDebugPrint(4, "Set the video mode format %x %d\n", 0x0, 0x0b);
						break;
					default:
						break;
					}
					setRes = glCommitCtrl[3];
				}
				CyU3PDebugPrint(4, "Set the video mode format setRes %d\n", setRes);

	#if 0
				if (usbSpeed == CY_U3P_SUPER_SPEED)
				{
					SensorScaling_HD720p_30fps();
				}
				else
				{
					SensorScaling_VGA();
				}
	#endif
				/* We can start streaming video now. */
				apiRetStatus = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					CyU3PDebugPrint(4, "Set CY_FX_UVC_STREAM_EVENT failed %x\n", apiRetStatus);
				}
			}
			break;

		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

		/* still image streaming handler */
	case VD_FX_UVC_STILL_PROB_CTRL:
		switch (bRequest)
		{
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			glEp0Buffer[1] = 0;
			CyU3PUsbSendEP0Data(2, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
		case CY_FX_USB_UVC_GET_MIN_REQ:
		case CY_FX_USB_UVC_GET_MAX_REQ:
		case CY_FX_USB_UVC_GET_DEF_REQ: 	/* There is only one setting per USB speed. */
			if (usbSpeed == CY_U3P_SUPER_SPEED)
			{
				CyU3PUsbSendEP0Data(VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl);
			}
			else
			{
				CyU3PUsbSendEP0Data(VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
				//if (usbSpeed == CY_U3P_SUPER_SPEED)//for both SS and HS
				{
					/* Copy the relevant settings from the host provided data into the
					active data structure. */
					glProbeStilCtrl[1] = glCommitCtrl[1];
					glProbeStilCtrl[2] = glCommitCtrl[2];
					glProbeStilCtrl[3] = glCommitCtrl[3];
					glProbeStilCtrl[4] = glCommitCtrl[4];
					glProbeStilCtrl[5] = glCommitCtrl[5];
					glProbeStilCtrl[6] = glCommitCtrl[6];
				}
				CyU3PDebugPrint(4, "Get UVC still Prob(set) control %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);
			}
			break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	case VD_FX_UVC_STILL_COMIT_CTRL:
		switch (bRequest)
		{
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			glEp0Buffer[1] = 0;
			CyU3PUsbSendEP0Data(2, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
			if (usbSpeed == CY_U3P_SUPER_SPEED)
			{
				CyU3PUsbSendEP0Data(VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl);
			}
			else
			{
				CyU3PUsbSendEP0Data(VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			/* The host has selected the parameters for the video stream. Check the desired
			resolution settings, configure the sensor and start the video stream.
			*/
			apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
	#if 0
				if (usbSpeed == CY_U3P_SUPER_SPEED)
				{
					SensorScaling_HD720p_30fps();
				}
				else
				{
					SensorScaling_VGA();
				}
				/* We can start streaming video now. */
				apiRetStatus = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);

				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					CyU3PDebugPrint(4, "Set CY_FX_UVC_STREAM_EVENT failed %x\n", apiRetStatus);
				}
	#endif
	#if 0 //remove the still resolution set for invendo because the still res. is always the same as the video res.
				switch (glCommitCtrl[1])
				{
				case 1: //720
					SensorSetIrisControl(0x0b, 0x30, 0x1, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
					//CyU3PThreadSleep(500);
					CyU3PDebugPrint(4, "Set the still mode format %x %d\n", 0x0b, 0x1);
					break;
				case 2: //960
					SensorSetIrisControl(0x0b, 0x30, 0x0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
					//CyU3PThreadSleep(500);
					CyU3PDebugPrint(4, "Set the still mode format %x %d\n", 0x0b, 0x0);
					break;
				default:
					break;
				}
				setstilRes = glCommitCtrl[1];

				CyU3PDebugPrint(4, "UVC still commit control set %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);
	#endif
			}
			break;

		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	case VD_FX_UVC_STILL_TRIG_CTRL:
		switch (bRequest)
		{
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = 1;//CY_FX_UVC_MAX_PROBE_SETTING;
			glEp0Buffer[1] = 0;
			CyU3PUsbSendEP0Data(2, (uint8_t *)glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ://TODO for still trigger control
			if (1 || usbSpeed == CY_U3P_SUPER_SPEED)// support both SS and HS
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
			}
			else
			{
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			/* The host has selected the parameters for the video stream. Check the desired
			resolution settings, configure the sensor and start the video stream.
			*/
			apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
	#if 1
				/* We can start still streaming video now. */
				apiRetStatus = CyU3PEventSet(&glFxUVCEvent, VD_FX_UVC_STIL_EVENT, CYU3P_EVENT_OR);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					CyU3PDebugPrint(4, "Set CY_FX_UVC_STIL_EVENT failed %x\n", apiRetStatus);
				}
	#endif
				else{
					stiflag = 0xF0;//set still trigger flag
					//stillcont = 0;
				}
				CyU3PDebugPrint(4, "Get UVC still trigger control %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);
			}
			else{
				CyU3PDebugPrint(4, "UVC still trigger control fail %d %d\r\n", readCount, glCommitCtrl[0]);
				CyU3PUsbStall(0, CyTrue, CyFalse);
			}
			break;

		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	default:
		CyU3PUsbStall(0, CyTrue, CyFalse);
		break;
	}
#endif

	/* the new merge*/
		{
			/* GET_CUR Request Handling Probe/Commit Controls*/
			if ((bRequest == CY_FX_USB_UVC_GET_CUR_REQ) || (bRequest == CY_FX_USB_UVC_GET_MIN_REQ) || (bRequest == CY_FX_USB_UVC_GET_MAX_REQ) || (bRequest == CY_FX_USB_UVC_GET_DEF_REQ))
			{
				isHandled = CyTrue;
				if ((wValue == CY_FX_UVC_PROBE_CTRL) || (wValue == CY_FX_UVC_COMMIT_CTRL))
				{
					//TODO Modify this "glProbeCtrl" according to the Supported Preview Resolutions that are supported by the sensor

					/* Host requests for probe data of 34 bytes (UVC 1.1) or 26 Bytes (UVC1.0). Send it over EP0. */
					if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
					{
						if (glCurrentFrameIndex == 4)
						{
							CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl5MpProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
						}
						/* Probe Control for 1280x720 stream*/
						else if (glCurrentFrameIndex == 3)
						{
							CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl720pProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
						}
						/* Probe Control for 640x480 stream*/
						else  if (glCurrentFrameIndex == 2)
						{
							CyU3PMemCopy(glProbeCtrl, (uint8_t *)glVga60ProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
						}
						/* Probe Control for 1920x1080 stream*/
						else  if (glCurrentFrameIndex == 1)
						{
							CyU3PMemCopy(glProbeCtrl, (uint8_t *)gl1080pProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
						}

					}
					else if (CyU3PUsbGetSpeed() == CY_U3P_HIGH_SPEED)
					{
						/* Probe Control for 640x480 stream*/
						CyU3PMemCopy(glProbeCtrl, (uint8_t *)glVga30ProbeCtrl, CY_FX_UVC_MAX_PROBE_SETTING);
					}
					else /* FULL-Speed*/
					{
						CyU3PDebugPrint(4, "\n\rFull Speed Not Supported!");
					}

					status = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, glProbeCtrl);
					if (status != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:GET_CUR:SendEP0Data Err = 0x%x", status);
					}
				}
				else if ((wValue == VD_FX_UVC_STILL_PROB_CTRL) || (wValue == VD_FX_UVC_STILL_COMIT_CTRL))
				{
					if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
					{
						status = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING, glStillProbeCtrl);
						if (status != CY_U3P_SUCCESS)
						{
							CyU3PDebugPrint(4, "\n\rUSBStpCB:GET_CUR:SendEP0Data Err = 0x%x", status);
						}
					}
				}
			}
			/* SET_CUR request handling Probe/Commit controls */
			else if (bRequest == CY_FX_USB_UVC_SET_CUR_REQ)
			{
				isHandled = CyTrue;
				if ((wValue == CY_FX_UVC_PROBE_CTRL) || (wValue == CY_FX_UVC_COMMIT_CTRL))
				{
					/* Get the UVC probe/commit control data from EP0 */
					status = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
						glCommitCtrl, &readCount);
					if (status != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
					}
					/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
					if (readCount > (uint16_t)CY_FX_UVC_MAX_PROBE_SETTING)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
					}
					else
					{
						/* Set Probe Control */
						if (wValue == CY_FX_UVC_PROBE_CTRL)
						{
							glCurrentFrameIndex = glCommitCtrl[3];
						}
						/* Set Commit Control and Start Streaming*/
						else if (wValue == CY_FX_UVC_COMMIT_CTRL)
						{

							if ((glcommitcount == 0) || (glcheckframe != glCommitCtrl[3]))
							{
								glcommitcount++;
								glcheckframe = glCommitCtrl[3];
								glCurrentFrameIndex = glCommitCtrl[3];
								glFrameIndexToSet = glCurrentFrameIndex;
								glPreviewStarted = CyTrue;

								//TODO Change this function with "Sensor Specific" function to write the sensor settings & configure the CX3 for supported resolutions
								//	esSetCameraResolution(glCurrentFrameIndex);
								//esSetCameraResolution(glCommitCtrl[3]);//TODO the camera resolution calling

								if (glIsApplnActive)
								{
									if (glcommitcount)
										glIsClearFeature = CyFalse;
									else
										glIsClearFeature = CyTrue;

									esUVCUvcApplnStop();
								}
								esUVCUvcApplnStart();
							}
						}
					}
				}
				else if ((wValue == VD_FX_UVC_STILL_PROB_CTRL) || (wValue == VD_FX_UVC_STILL_COMIT_CTRL))
				{
					/* Get the UVC STILL probe/commit control data from EP0 */
					status = CyU3PUsbGetEP0Data(CY_FX_MAX_STILL_PROBE_SETTING_ALIGNED, glStillCommitCtrl, &readCount);
					if (status != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
					}
					/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
					if (readCount > (uint16_t)CY_FX_UVC_MAX_STILL_PROBE_SETTING)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
					}
					else
					{
						/* Set Probe Control */
						if (wValue == VD_FX_UVC_STILL_PROB_CTRL)
						{
							glCurrentStillFrameIndex = glStillCommitCtrl[1];
						}
						/* Set Commit Control and Start Streaming*/
						else if (wValue == VD_FX_UVC_STILL_COMIT_CTRL)
						{
							glCurrentStillFrameIndex = glStillCommitCtrl[1];
						}
					}

				}
				else if (wValue == VD_FX_UVC_STILL_TRIG_CTRL)
				{
					status = CyU3PUsbGetEP0Data(CY_FX_STILL_TRIGGER_ALIGNED, &glStillTriggerCtrl, &readCount);
					if (status != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:SET_CUR:GetEP0Data Err = 0x%x.", status);
					}
					/* Check the read count. Expecting a count of CX3_UVC_MAX_PROBE_SETTING bytes. */
					if (readCount > (uint16_t)CY_FX_STILL_TRIGGER_COUNT)
					{
						CyU3PDebugPrint(4, "\n\rUSBStpCB:Invalid SET_CUR Rqt Len.");
					}
					else
					{
						if (glStillTriggerCtrl == 0x01)
						{
							glStillSkip = 3;
							glStillCaptureStart = CyTrue;
						}
					}
				}
			}
			else
			{
				/* Mark with error. */
				status = CY_U3P_ERROR_FAILURE;
			}
		}
}


	/*
 * Entry function for the UVC control request processing thread.
 */
void
UVCAppEP0Thread_Entry (
        uint32_t input)
{
    uint32_t eventMask = (CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT | CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT);
    uint32_t eventFlag;
	CyBool_t value;
	CyBool_t *valueptr = &value;


#ifdef USB_DEBUG_INTERFACE
    CyU3PReturnStatus_t apiRetStatus;
    CyU3PDmaBuffer_t    dmaInfo;

    eventMask |= CY_FX_USB_DEBUG_CMD_EVENT;
#endif

    /* for interrupt status test */
    CyU3PReturnStatus_t apiRetStatus;
    eventMask |= VD_FX_INT_STA_EVENT;
    CyU3PDmaBuffer_t    interStabuf;

    for (;;)
    {
        /* Wait for a Video control or streaming related request on the control endpoint. */
        if (CyU3PEventGet (&glFxUVCEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag,
                    CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS)
        {
            /* If this is the first request received during this connection, query the connection speed. */
            if (!isUsbConnected)
            {
                usbSpeed = CyU3PUsbGetSpeed ();
                if (usbSpeed != CY_U3P_NOT_CONNECTED)
                {
                    isUsbConnected = CyTrue;
                }
            }
//#ifdef DbgInfo
#if 0
            if((eventFlag & eventMask) & ~VD_FX_INT_STA_EVENT)
            CyU3PDebugPrint (4, "USB speed = %d evenflag = 0x%x bmReqType = 0x%x\r\n"
            		"bRequest = 0x%x wValue = 0x%x wIndex = 0x%x wLength = 0x%x isflag 0x%x\r\n",
            		usbSpeed, eventFlag, bmReqType, bRequest, wValue, wIndex, wLength, 0/*isFlag*/); /* additional debug message */
            //CyU3PDebugPrint (4, "fb = %d pb = %d pbc = %d pbcp = %d\r\n", fbbak, pbbak, pbcbak, pbcpbak);
            //fbbak=0;pbbak=0;pbcbak=0;pbcpbak=0;
#endif
//#endif
            if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)
            {
            	switch ((wIndex >> 8))
                {

                    case CY_FX_UVC_PROCESSING_UNIT_ID:
                        UVCHandleProcessingUnitRqts ();
                        break;

                    case CY_FX_UVC_CAMERA_TERMINAL_ID:
                        ;//UVCHandleCameraTerminalRqts ();
                        break;

                    case CY_FX_UVC_INTERFACE_CTRL:
                        UVCHandleInterfaceCtrlRqts ();
                        break;

                    case CY_FX_UVC_EXTENSION_UNIT_ID:
                        UVCHandleExtensionUnitRqts ();
                        break;

                    default:
                        /* Unsupported request. Fail by stalling the control endpoint. */
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
            }

            if (eventFlag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT)
            {
                //CyU3PDebugPrint (4, "start a stream req. ctrl. wIndex 0x%x\r\n", wIndex);

                if (wIndex != CY_FX_UVC_STREAM_INTERFACE)
                {
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                }
                else
                {
                    UVCHandleVideoStreamingRqts ();
                }
            }

            /* handle interrupt status event */
            if (eventFlag & VD_FX_INT_STA_EVENT)
            {

            	//CyU3PDebugPrint (4, "start a interrupt req. ctrl. snap flag 0x%x\r\n", snapButFlag);
            	/** preparing interrupt status data **/
            	CyU3PGpioSimpleGetValue (SENSOR_SNAPSHOT_GPIO, valueptr);// get button value 1:release 0:press

				//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);

#if 1 //for real button
				if(value&&(!snapButFlag)){
					//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);
					glInterStaBuffer[0] = 0x02;  //VS interface
					glInterStaBuffer[1] = 0x01;  //number of VS interface
					glInterStaBuffer[2] = 0x00;
					glInterStaBuffer[3] = 0x00; //button release

					interStabuf.buffer = glInterStaBuffer;
					interStabuf.size   = 1024;
					interStabuf.status = 0;

					interStabuf.count = 4;

					/** wait unitll the responses has gone out **/
					CyU3PDmaChannelWaitForCompletion(&glChHandleInterStat, CYU3P_WAIT_FOREVER);

					/** send a interrupt status data **/
					apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleInterStat, &interStabuf);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint (4, "Failed to send interrupt status, Error code = %d\r\n", apiRetStatus);
						CyFxAppErrorHandler (apiRetStatus);
					}
					snapButFlag = 1;//snap button is masked.
				}else if(snapButFlag&&(!value)){
					//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);
					glInterStaBuffer[0] = 0x02;  //VS interface
					glInterStaBuffer[1] = 0x01;  //number of VS interface
					glInterStaBuffer[2] = 0x00;
					glInterStaBuffer[3] = 0x01; //button release

					interStabuf.buffer = glInterStaBuffer;
					interStabuf.size   = 1024;
					interStabuf.status = 0;

					interStabuf.count = 4;

					/** wait unitll the responses has gone out **/
					CyU3PDmaChannelWaitForCompletion(&glChHandleInterStat, CYU3P_WAIT_FOREVER);

					/** send a interrupt status data **/
					apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleInterStat, &interStabuf);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint (4, "Failed to send interrupt status, Error code = %d\r\n", apiRetStatus);
						CyFxAppErrorHandler (apiRetStatus);
					}

					snapButFlag = 0; //snap button is not masked.
					stiflag = 0xFF;
				}
#else			//for botton simulation
				if(snapButFlag == 0x0f){
					//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);
					glInterStaBuffer[0] = 0x02;  //VS interface
					glInterStaBuffer[1] = 0x01;  //number of VS interface
					glInterStaBuffer[2] = 0x00;
					glInterStaBuffer[3] = 0x00; //button release

					interStabuf.buffer = glInterStaBuffer;
					interStabuf.size   = 1024;
					interStabuf.status = 0;

					interStabuf.count = 4;

					/** wait unitll the responses has gone out **/
					CyU3PDmaChannelWaitForCompletion(&glChHandleInterStat, CYU3P_WAIT_FOREVER);

					/** send a interrupt status data **/
					apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleInterStat, &interStabuf);
					//CyU3PDebugPrint (4, "send interrupt status\r\n");
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint (4, "Failed to send interrupt status, Error code = %d\r\n", apiRetStatus);
						CyFxAppErrorHandler (apiRetStatus);
					}
						SensorSetControl(0x5, 0x30, 0); //mirror set to 0

						snapButFlag = 1;//snap button is masked.
				}else if(!snapButFlag){
					//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);
					glInterStaBuffer[0] = 0x02;  //VS interface
					glInterStaBuffer[1] = 0x01;  //number of VS interface
					glInterStaBuffer[2] = 0x00;
					glInterStaBuffer[3] = 0x01; //button release

					interStabuf.buffer = glInterStaBuffer;
					interStabuf.size   = 1024;
					interStabuf.status = 0;

					interStabuf.count = 4;

					/** wait unitll the responses has gone out **/
					CyU3PDmaChannelWaitForCompletion(&glChHandleInterStat, CYU3P_WAIT_FOREVER);

					/** send a interrupt status data **/
					apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleInterStat, &interStabuf);
					//CyU3PDebugPrint (4, "send interrupt status\r\n");
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						CyU3PDebugPrint (4, "Failed to send interrupt status, Error code = %d\r\n", apiRetStatus);
						CyFxAppErrorHandler (apiRetStatus);
					}

					SensorSetControl(0x5, 0x30, 1); //mirror set to 1
					snapButFlag = 1; //snap button is not masked.
				}
#endif

            }


#ifdef USB_DEBUG_INTERFACE
            if (eventFlag & CY_FX_USB_DEBUG_CMD_EVENT)
            {
                /* Get the command buffer */
                apiRetStatus = CyU3PDmaChannelGetBuffer (&glDebugCmdChannel, &dmaInfo, CYU3P_WAIT_FOREVER);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to receive debug command, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Decode the command from the command buffer, error checking is not implemented,
                 * so the command is expected to be correctly sent from the host application. First byte indicates
                 * read (0x00) or write (0x01) command. Second and third bytes are register address high byte and
                 * register address low byte. For read commands the fourth byte (optional) can be N>0, to read N
                 * registers in sequence. Response first byte is status (0=Pass, !0=Fail) followed by N pairs of
                 * register value high byte and register value low byte.
                 */
                CyU3PDebugPrint (4, "Debug interface conut %d data %d %d %d\r\n", dmaInfo.count, dmaInfo.buffer[0], dmaInfo.buffer[1], dmaInfo.buffer[2]); //additional debug
                if (dmaInfo.buffer[0] == 0)
                {
                    if (dmaInfo.count == 3)
                    {
                        /*glDebugRspBuffer[0] = SensorRead2B (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(glDebugRspBuffer+1));*/
                        dmaInfo.count = 3;
                    }
                    else if (dmaInfo.count == 4)
                    {
                        if (dmaInfo.buffer[3] > 0)
                        {
                                glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                                		(dmaInfo.buffer[3]*2), (glDebugRspBuffer+1));
                        }
                        dmaInfo.count = dmaInfo.buffer[3]*2+1;
                    }
                    CyU3PDebugPrint (4, "Debug responsR conut %d data %d %d %d\r\n", dmaInfo.count, glDebugRspBuffer[0], glDebugRspBuffer[1], glDebugRspBuffer[2]); //additional debug
                }
                /*  For write commands, the register address is followed by N pairs (N>0) of register value high byte
                 *  and register value low byte to write in sequence. Response first byte is status (0=Pass, !0=Fail)
                 *  followed by N pairs of register value high byte and register value low byte after modification.
                 */
                else if (dmaInfo.buffer[0] == 1)
                {
                        /*glDebugRspBuffer[0] = SensorWrite (SENSOR_ADDR_WR, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (dmaInfo.buffer+3));  original one*/
                        glDebugRspBuffer[0] = SensorWrite2B (SENSOR_ADDR_WR, dmaInfo.buffer[1], dmaInfo.buffer[2],
                                                		0x00, dmaInfo.buffer[3]); //additional debug
                        CyU3PDebugPrint (4, "Debug write %d data %d %d %d\r\n", dmaInfo.count, dmaInfo.buffer[2], dmaInfo.buffer[3], (dmaInfo.buffer+3));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;
                        /*glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (glDebugRspBuffer+1));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;*/
                    dmaInfo.count -= 2;
                }
                /* Default case, prepare buffer for loop back command in response */
                else
                {
                   /* For now, we just copy the command into the response buffer; and send it back to the
                      USB host. This can be expanded to include I2C transfers. */
                    CyU3PMemCopy (glDebugRspBuffer, dmaInfo.buffer, dmaInfo.count);
                    CyU3PDebugPrint (4, "Debug respons conut %d data %d %d %d\r\n", dmaInfo.count, glDebugRspBuffer[0], glDebugRspBuffer[1], glDebugRspBuffer[2]); //additional debug
                }

                dmaInfo.buffer = glDebugRspBuffer;
                dmaInfo.size   = 1024;
                dmaInfo.status = 0;

                /* Free the command buffer to receive the next command. */
                apiRetStatus = CyU3PDmaChannelDiscardBuffer (&glDebugCmdChannel);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to free up command OUT EP buffer, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Wait until the response has gone out. */
                CyU3PDmaChannelWaitForCompletion (&glDebugRspChannel, CYU3P_WAIT_FOREVER);

                apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glDebugRspChannel, &dmaInfo);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to send debug response, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }
            }
#endif
        }
        /* Allow other ready threads to run. */
        CyU3PThreadRelinquish ();
    }
}

/*
* Entry function for the internal I2C control handler thread.
* added 10/2013
*/
/*
static uint8_t timeDelay[64] = {

};
*/

/*
 * Entry function for the UVC Application Thread
 */

uint32_t posTick;
CyU3PTimer I2CCmdTimer;

void  I2CCmdCb(uint32_t input){
	CyU3PDebugPrint (4, "I2C pos-timer %d %d\r\n", posTick, input);
	CyU3PEventSet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);
}

static uint8_t timercount = 0;
void I2cAppThread_Entry(uint32_t input){

	//uint16_t count = 0, cmdCopyIdx = 0; //count1 = 0, cmdQuIdx = 0,
	VdRingBuf *cmdQuptr = &cmdQu;
	VdRingBuf *statQuptr = &statQu;
	VdcmdDes  *lcCmdDes;
	VdstateDes *lcStaDes;
	uint32_t flag = 0;
	uint8_t  cmdFlag = 0;
	uint8_t regAdd, /*regAdd1,*/ devAdd, data;// data1;
	uint8_t i, curFlagIdx;
	uint16_t delaytime;
	//CyBool_t trigger = CyFalse;

#if 0 //for test the command queue
	lcCmdDes = cmdQuptr->startAdd;
	for (cmdQuIdx = 0; cmdQuIdx < MAXCMD; cmdQuIdx++){
		CyU3PDebugPrint(4, "Command Queue check cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d\r\n",
			lcCmdDes->CmdID, lcCmdDes, lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, cmdQuIdx);
		lcCmdDes += 1;
	}
#endif
	/*** create a timer for I2C commands delay option ***/
	CyU3PTimerCreate(&I2CCmdTimer, I2CCmdCb, 11, 1000, 0, CYU3P_NO_ACTIVATE);
	CyU3PDebugPrint(4, "I2C per-timer %d\r\n", CyU3PGetTime());
	CyU3PThreadSleep(50);
	CyU3PTimerStart(&I2CCmdTimer);

	while (cmdQuptr->bugFlag == (uint8_t)CyFalse){ //waiting for first command
		/* Allow other ready threads to run. */

		CyU3PThreadRelinquish();
	}
	CyU3PDebugPrint(4, "The command queue is ready %d %d\r\n", cmdQuptr->bugFlag, cmdQuptr->readPtr->cmdFlag);
	//CamDefSet(); //set default parameters to camera
	/***** add recovery of the current camera settings ****/
	//CyU3PThreadSleep(100);
	//SetCurCmd();
	/*********** the loop of the thread ***********/
	for (;;){

		CyU3PEventGet(&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_AND_CLEAR, &flag, CYU3P_WAIT_FOREVER);//wait command event
		//CyU3PDebugPrint (4, "In I2C loop timercounter %d cmdFlag 0x%x\r\n", timercount, cmdFlag);
		/*  // for test GPIO output
		if(trigger)
		{
		CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyFalse);
		{
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n", CyFalse);
		}

		}else{
		CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyTrue);
		{
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n", CyTrue);
		}

		}
		*/
		CyU3PMutexGet(statQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
		//CyU3PDebugPrint (4, "get I2C events (0) flag 0x%x cmdflag 0x%x\r\n", flag, cmdFlag);
		lcStaDes = (VdstateDes*)statQuptr->readPtr;
		if (0 && (lcStaDes->statFlag == CyTrue)){ /* for state queue it's not used right now. */
			for (i = 0; i < lcStaDes->NumData; i++){
				regAdd = ((lcStaDes->staPar) + i)->RegAdd;
				devAdd = ((lcStaDes->staPar) + i)->DevAdd;
				((lcStaDes->staPar) + i)->Data = SensorGetControl(regAdd, devAdd); //get state value from I2C bus
#ifdef USB_DEBUG_PRINT
				CyU3PDebugPrint(4, "send I2C state stateID %d cmdCopyIdx %d regAdd 0x%x devAdd 0x%x data 0x%x\r\n",
					lcStaDes->StatID, regAdd, devAdd, data);
#endif
			}
			lcStaDes->statFlag = CyFalse;
			statQuptr->readPtr = (VdcmdDes*)lcStaDes->staDesNext; //update command queue read pointer
			cmdFlag = 0xFF; //I2C command done
		}
		CyU3PMutexPut(statQuptr->ringMux);  //release the command queue mutex
		if (cmdFlag != 0xFF){ //for during handle command
			CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
			lcCmdDes = cmdQuptr->readPtr;

			/*
			CyU3PDebugPrint (4, "get I2C events (1) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
			flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
			*/

			/* find a available command */
			i = 0;
			while ((lcCmdDes->cmdFlag == deswait) && (i < MAXCMD)){
				i++;
				lcCmdDes = lcCmdDes->cmdDesNext;
				cmdQuptr->readPtr = lcCmdDes;
			}
			//CyU3PDebugPrint (4, "i %d Cmf_Flag %d\r\n", i, lcCmdDes->cmdFlag);
			if (lcCmdDes->cmdFlag != deswait){
				i = lcCmdDes->curNum;
				regAdd = ((lcCmdDes->CmdPar) + i)->RegAdd;
				devAdd = ((lcCmdDes->CmdPar) + i)->DevAdd;
				data = ((lcCmdDes->CmdPar) + i)->Data;
				delaytime = ((lcCmdDes->CmdPar) + i)->DelayT;
				switch (lcCmdDes->CmdID){
				case 0x20:
					SensorSetIrisControl(regAdd, devAdd, data, I2C_DSPBOARD_ADDR_WR/*boardID*/);//set Iris auto (AF Lens)
					delaytime = 500;
					break;
				case 0x21:
					SensorSetIrisControl(regAdd, devAdd, data, I2C_DSPBOARD_ADDR_WR/*boardID*/);//set Iris auto (non AF Lens)
					delaytime = 500;
					break;
				case 0x22:
					SensorSetIrisControl(regAdd, devAdd, data, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris value (DC manual)
					delaytime = 300;
					break;
				case 0x23:
					SensorSetIrisControl(regAdd, devAdd, data, I2C_AFBOARD_ADDR_WR/*boardID*/);//opt Zoom
					delaytime = 300;
					break;
				default:
					SensorSetControl(regAdd, devAdd, data);    //send I2C command
					break;
				}
				//SensorSetControl(regAdd, devAdd, data);    //send I2C command
				/** timer's ticket modify **/
				CyU3PTimerModify(&I2CCmdTimer, delaytime, 0);
				CyU3PTimerStart(&I2CCmdTimer);  //start delay timer
				//CyU3PDebugPrint (4, "set timer restart(1) %d 0x%x 0x%x %d %d %d %d\r\n", CyU3PGetTime(), regAdd, devAdd, data, delaytime, lcCmdDes->CmdID, i);
				cmdFlag = 0xFF; //I2C command done
#ifdef USB_DEBUG_PRINT
				CyU3PDebugPrint(4, "send I2C command cmdID %d regAdd 0x%x devAdd 0x%x data 0x%x cmdflag 0x%x\r\n",
					lcCmdDes->CmdID, regAdd, devAdd, data, lcCmdDes->cmdFlag);
#endif
				if (lcCmdDes->NumPara == lcCmdDes->curNum){
					lcCmdDes->cmdFlag = deswait;
					if (lcCmdDes->CmdID >= EXUAOFFSET){
						ExUCtrlParArry[(lcCmdDes->CmdID - EXUAOFFSET + 4)][16] = CyFalse;
					}
					else{
						CtrlParArry[lcCmdDes->CmdID][16] = CyFalse; //set flag to false. wait for check.
					}
					cmdQuptr->readPtr = lcCmdDes->cmdDesNext; //update command queue read pointer for next handled command
				}
				else{
					lcCmdDes->curNum++;
					lcCmdDes->cmdFlag = desusing;
				}
			}
			else{
				CyU3PTimerModify(&I2CCmdTimer, 1000, 0); //the free I2C commands timer pace (no setting command).
				CyU3PTimerStart(&I2CCmdTimer);
				//CyU3PDebugPrint (4, "I2Ctimer counter %d", timercount);
				if (timercount >= 3){
					for (curFlagIdx = 0; curFlagIdx<64; curFlagIdx++){
						curFlag[curFlagIdx] = 0;
					}
					timercount = 0;

				}
				else{
					timercount++;
				}
			}
		}
		CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
		/*
		CyU3PDebugPrint (4, "get I2C events (2) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
		flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
		*/
#ifdef USB_DEBUG_PRINT
		CyU3PDebugPrint(4, "I2C thread checking camera parameters count %d data0 %d data1 %d cmdflag 0x%x.\r\n",
			0/*count*/, CtrlParArry[count][13], CtrlParArry[count][14], cmdFlag);
#endif

		/**** checking the camera registers if it is the same what the current copy is. ****/
		/** this code might be used when a timer is used to schedule the I2C command sent out **/
#if 0
		if ((CtrlParArry[cmdCopyIdx][16] != CyTrue) && (cmdFlag != 0xFF)/*&&(CtrlParArry[cmdCopyIdx][17] != CyFalse)*/){ //checking register value

			regAdd = CtrlParArry[cmdCopyIdx][0];
			regAdd1 = CtrlParArry[cmdCopyIdx][1];
			devAdd = CtrlParArry[cmdCopyIdx][15];
			data = SensorGetControl(regAdd, devAdd); //SensorGetBLCMode();
			i = 0;
			switch (cmdCopyIdx)
			{
			case BrgtCtlID1:
				if (CtrlParArry[cmdCopyIdx][14] != data){
					CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][14], i);
					CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
					i++;
				}
				else{
					;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
				}

				CyU3PBusyWait(500);
				data = SensorGetControl(regAdd1, devAdd);
				if (CtrlParArry[cmdCopyIdx][13] != data){
					CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					cmdSet(cmdQuptr, cmdCopyIdx, regAdd1, devAdd, CtrlParArry[cmdCopyIdx][13], i);
					CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
				}
				else{
					;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
				}
				break;
			case HueCtlID5:
				if (CtrlParArry[cmdCopyIdx][13] != data){
					CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][13], i);
					CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
				}
				else{
					;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
				}
				break;
			case SaturCtlID6:
			case WBTLevCtlID10:
			default:
				if (CtrlParArry[cmdCopyIdx][13] == data){
					CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][13], i);
					CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
				}
				else{
					;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
				}
				break;
			}
			//cmdFlag = 0xFF; //one I2C command one available event.
			CtrlParArry[cmdCopyIdx][16] = CyTrue; //set flag to true. let it sent to camera.
		}
		cmdCopyIdx = (cmdCopyIdx + 1) & 0x1F;    //update checking index.
#endif
		cmdFlag = 0x00; //clear flag
		/* Allow other ready threads to run. */
		//CyU3PDebugPrint (4, "out of the i2cthread flag 0x%x cmdflag 0x%x\r\n", flag, cmdFlag);
		CyU3PThreadRelinquish();
	}
}

/* Application define function which creates the threads. */
    void
CyFxApplicationDefine (
        void)
{
	void *ptr1, *ptr2, *ptr3;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;
	VdRingBuf *cmdQuptr = &cmdQu;
	VdRingBuf *statQuptr = &statQu;

	/* Allocate the memory for the thread stacks. */
	ptr1 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
	ptr2 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
	ptr3 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);

	if ((ptr1 == 0) || (ptr2 == 0) || (ptr3 == 0))
		goto fatalErrorHandler;

	/****** create a ring buffer for command queue *******/
	cmdQu = cmdbufCreate(MAXCMD, &cmdQuMux);
	statQu = cmdbufCreate(MAXSTA, &staQuMux);

	/****** initialize command descriptor ***********/
	cmdquInit(cmdQuptr);
	cmdquInit(statQuptr);

	/* Create the UVC application thread. */
	retThrdCreate = CyU3PThreadCreate(&uvcAppThread,   /* UVC Thread structure */
		"30:UVC App Thread",                        /* Thread Id and name */
		UVCAppThread_Entry,                         /* UVC Application Thread Entry function */
		0,                                          /* No input parameter to thread */
		ptr1,                                       /* Pointer to the allocated thread stack */
		UVC_APP_THREAD_STACK,                       /* UVC Application Thread stack size */
		UVC_APP_THREAD_PRIORITY,                    /* UVC Application Thread priority */
		UVC_APP_THREAD_PRIORITY,                    /* Threshold value for thread pre-emption. */
		CYU3P_NO_TIME_SLICE,                        /* No time slice for the application thread */
		CYU3P_AUTO_START                            /* Start the Thread immediately */
		);
    /* Check the return code */
	if (retThrdCreate != 0)
	{
		goto fatalErrorHandler;
	}
#if 0  //cancel the ep0 thread
	/* Create the control request handling thread. */
	retThrdCreate = CyU3PThreadCreate(&uvcAppEP0Thread,        /* UVC Thread structure */
		"31:UVC App EP0 Thread",                            /* Thread Id and name */
		UVCAppEP0Thread_Entry,                              /* UVC Application EP0 Thread Entry function */
		0,                                                  /* No input parameter to thread */
		ptr2,                                               /* Pointer to the allocated thread stack */
		UVC_APP_EP0_THREAD_STACK,                           /* UVC Application Thread stack size */
		UVC_APP_EP0_THREAD_PRIORITY,                        /* UVC Application Thread priority */
		UVC_APP_EP0_THREAD_PRIORITY,                        /* Threshold value for thread pre-emption. */
		CYU3P_NO_TIME_SLICE,                                /* No time slice for the application thread */
		CYU3P_AUTO_START                                    /* Start the Thread immediately */
		);
	if (retThrdCreate != 0)
	{
		goto fatalErrorHandler;
	}
#endif
	/* Create the I2C control command handling thread. */
	retThrdCreate = CyU3PThreadCreate(&i2cAppThread,   /* UVC Thread structure */
		"32:I2C App CTRL Thread",                        /* Thread Id and name */
		I2cAppThread_Entry,                         /* UVC Application Thread Entry function */
		0,                                          /* No input parameter to thread */
		ptr3,                                       /* Pointer to the allocated thread stack */
		UVC_APP_I2C_THREAD_STACK,                       /* UVC Application Thread stack size */
		UVC_APP_I2C_THREAD_PRIORITY,                    /* UVC Application Thread priority */
		UVC_APP_I2C_THREAD_PRIORITY,                    /* Threshold value for thread pre-emption. */
		CYU3P_NO_TIME_SLICE,                        /* No time slice for the application thread */
		CYU3P_AUTO_START                            /* Start the Thread immediately */
		);
	if (retThrdCreate != 0)
	{
		goto fatalErrorHandler;
	}

    /* Create GPIO application event group */
    retThrdCreate = CyU3PEventCreate(&glFxUVCEvent); //it places the evnet group glFxUVCEvent
    if (retThrdCreate != 0)
    {
        /* Event group creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
		goto fatalErrorHandler;
	}

    return;

    fatalErrorHandler:
        /* Add custom recovery or debug actions here */
        /* Loop indefinitely */
        while (1);
}

/*
 * Main function
 */

int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyU3PSysClockConfig_t clockConfig;
    clockConfig.setSysClk400  = CyTrue;
    clockConfig.cpuClkDiv     = 2;
    clockConfig.dmaClkDiv     = 2;
    clockConfig.mmioClkDiv    = 2;
    clockConfig.useStandbyClk = CyFalse;
    clockConfig.clkSrc         = CY_U3P_SYS_CLK;

    /* Initialize the device */
     status = CyU3PDeviceInit (&clockConfig);

     if (status != CY_U3P_SUCCESS)
     {
         goto handle_fatal_error;
     }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device.*/
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:
    /* Cannot recover from this error. */
    while (1);
}
/* [ ] */
