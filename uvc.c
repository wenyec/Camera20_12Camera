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

#define  NOQU    //for queue debug
#define  SPEED   1// for optical zoom speed setting: 1 no support

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

/* Current UVC control request fields. See USB specification for definition. */
uint8_t  bmReqType, bRequest;                           /* bmReqType and bRequest fields. */
uint16_t wValue, wIndex, wLength;                       /* wValue, wIndex and wLength fields. */

uint16_t fb=0,pb=0,pbc=0;
uint16_t fbbak=0, pbbak=0, pbcbak=0, pbcpbak=0;

CyBool_t        isUsbConnected = CyFalse;               /* Whether USB connection is active. */
CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED;        /* Current USB connection speed. */
CyBool_t        clearFeatureRqtReceived = CyFalse;      /* Whether a CLEAR_FEATURE (stop streaming) request has been
                                                           received. */
CyBool_t        streamingStarted = CyFalse;             /* Whether USB host has started streaming data */
#ifdef BACKFLOW_DETECT
uint8_t back_flow_detected = 0;                         /* Whether buffer overflow error is detected. */
#endif

#ifdef USB_DEBUG_INTERFACE
CyU3PDmaChannel  glDebugCmdChannel;                     /* Channel to receive debug commands on. */
CyU3PDmaChannel  glDebugRspChannel;                     /* Channel to send debug responses on. */
uint8_t         *glDebugRspBuffer;                      /* Buffer used to send debug responses. */
#endif

/* UVC Probe Control Settings for a USB 3.0 connection. */
uint8_t glProbeCtrl[CY_FX_UVC_MAX_PROBE_SETTING] = {
    0x00, 0x00,                 /* bmHint : no hit */
    0x01,                       /* Use 1st Video format index */
    0x01,                       /* Use 1st Video frame index */
    0x15, 0x16, 0x05, 0x00,     /* Desired frame interval in the unit of 100ns: 30 fps */
    0x00, 0x00,                 /* Key frame rate in key frame/video frame units: only applicable
                                   to video streaming with adjustable compression parameters */
    0x00, 0x00,                 /* PFrame rate in PFrame / key frame units: only applicable to
                                   video streaming with adjustable compression parameters */
    0x00, 0x00,                 /* Compression quality control: only applicable to video streaming
                                   with adjustable compression parameters */
    0x00, 0x00,                 /* Window size for average bit rate: only applicable to video
                                   streaming with adjustable compression parameters */
    0x00, 0x00,                 /* Internal video streaming i/f latency in ms */
    0x00, 0x48, 0x3F, 0x00,     /* Max video frame size in bytes */
    0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
};

/* UVC still image Protrol Settings for a USB 3.0 connection */
uint8_t glProbeStilCtrl[11/*CY_FX_UVC_MAX_PROBE_SETTING*/] = {
    0x01,                       /* Use 1st Video format index */
    0x01,                       /* Use 1st Video frame index */
    0x00,						/* Compression index from a frame descriptor */
    0x00, 0x48, 0x3F, 0x00,     /* Max video frame size in bytes */
    0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
};


/* UVC Probe Control Setting for a USB 2.0 connection. */
uint8_t glProbeCtrl20[CY_FX_UVC_MAX_PROBE_SETTING] = {
    0x00, 0x00,                 /* bmHint : no hit */
    0x01,                       /* Use 1st Video format index */
    0x01,                       /* Use 1st Video frame index */
    0x80, 0x1a, 0x06, 0x00,     /* Desired frame interval in the unit of 100ns: 15 fps */
    0x00, 0x00,                 /* Key frame rate in key frame/video frame units: only applicable
                                   to video streaming with adjustable compression parameters */
    0x00, 0x00,                 /* PFrame rate in PFrame / key frame units: only applicable to
                                   video streaming with adjustable compression parameters */
    0x00, 0x00,                 /* Compression quality control: only applicable to video streaming
                                   with adjustable compression parameters */
    0x00, 0x00,                 /* Window size for average bit rate: only applicable to video
                                   streaming with adjustable compression parameters */
    0x00, 0x00,                 /* Internal video streaming i/f latency in ms */
    0x00, 0xD2, 0x0F, 0x00,     /* Max video frame size in bytes */
    0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
};

/* UVC still image Protrol Settings for a USB 2.0 connection */
uint8_t glProbeStilCtrl20[11/*CY_FX_UVC_MAX_PROBE_SETTING*/] = {
    0x01,                       /* Use 1st Video format index */
    0x01,                       /* Use 1st Video frame index */
    0x00,						/* Compression index from a frame descriptor */
    0x00, 0xD2, 0x0F, 0x00,     /* Max video frame size in bytes */
    0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
};


/* Video Probe Commit Control. This array is filled out when the host sends down the SET_CUR request. */
static uint8_t glCommitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];

/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
uint8_t volatile glUVCHeader[CY_FX_UVC_MAX_HEADER] =
{
    0x0C,                               /* Header Length */
    0x8C,                               /* Bit field header field */
    0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Source clock reference field */
};

volatile static CyBool_t hitFV = CyFalse;               /* Whether end of frame (FV) signal has been hit. */
volatile static CyBool_t gpif_initialized = CyFalse;    /* Whether the GPIF init function has been called. */
volatile static uint16_t prodCount = 0, consCount = 0;  /* Count of buffers received and committed during
                                                           the current video frame. */
volatile static CyBool_t stiflag = CyFalse;             /* Whether the image is still image */
#ifndef CAM720
volatile static CyBool_t WDRflag = CyTrue;              /* the flag for WDR mode. It's initialized based on the CtrlParArry[0][13]: 3=set; others=clear. */
#else
volatile static CyBool_t WDRflag = CyFalse;
#endif
//#define isWBMamu   0  // Is white balance control manual mode.

/************ control parameters array ***********
 *  the first D is the index of functionality, the second D is the index of parameters.
 *    e.g.
 *     1th D: backlight compensation, brightness, contrast, hue, saturation, sharpness, gamma, WBT, ~, BLC, main freq, ...
 *     2ed D: RegAdd1, RegAdd2, length, Min1, Min2, Max1, Max2, Res1, Res2, InfoReq1, InfoReq2, DefReq1, DefReq2,
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
		{/*1*/BrightnessReg1      , BrightnessReg0       , 2,    0,    0,  255,    0, 1, 0, 3, 0, 118, 0, 118, 199, I2C_DevAdd_C6,      CyTrue,  CyTrue, 0},
		{/*2*/ContrastReg         , ContrastReg          , 2,    0,    0,  255,    0, 1, 0, 3, 0, 112, 0, 112,   0, I2C_DevAdd_C6,      CyTrue,  CyTrue, 0},
		{/*3*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*4*/MainsFreqReg        , MainsFreqReg         , 2,    0,    0,    1,    0, 1, 0, 3, 0,   1, 0,   1,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*5*/HuectrlRegGr        , HuectrlRegBlu        , 2,    0,    0,  255,    0, 1, 0, 3, 0, 128, 0,   0,   0, I2C_DevAdd_C6,      CyTrue,  CyTrue, 0},  //Hue control
		{/*6*/SaturationRegR      , SaturationRegB       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  50, 0,  50,   0, I2C_DevAdd_F2,      CyTrue,  CyTrue, 0},  //Saturation control
		{/*7*/SharpnessReg        , SharpnessReg         , 2,    0,    0,   64,    0, 1, 0, 3, 0,  32, 0,  32,   0, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},
		{/*8*/0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{/*9*/WBModeReg           , WBModeReg            , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //white balance control
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
		{/*19*/AExModeReg          , AExAGCReg            , 4,    0,    0,  127,    0, 1, 0, 3, 0,   0,32,   0,  32, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE mode setting & AGC level: 0:auto 1~18:manual; 0 ~ 0xff:level. read(auto), write(menu).
		{/*1A*/AExReferleveReg0    , AExReferleveReg1     , 2,    0,    0,   64,    0, 1, 0, 3, 0,  32, 0,  32,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE reference level 0 ~ 0x40
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
/*
 * WBMenuCmpArry is set for white storing balance component requests values.
 * first two bytes represent blue and last two are for red. The defaults are set to 0.
 */
static uint8_t WBMenuCmpArry[4]={
		0x20, 0x0f, 0x38, 0xf0
};
static uint8_t I2CCMDArry[12]={//the index 12 points to data available; 0: no used; 0xf: unavailable; 0xff: available.
		0
};

//static uint32_t  isFlag = 0x0; /*set current value flag*/

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

/************************************
 * set Iris mode
 * input isAuto: 0: set manual; 1: set auto
 */
inline void setIrisauto(VdRingBuf *cmdQuptr, uint8_t isAuto){
	uint8_t dataIdx;
	  dataIdx = 0;
	  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
	  cmdSet(cmdQuptr, 0x20/*AFIrisMode*/, 0x27, 0x30, isAuto?0:1, dataIdx);  //set Iris Mode for AF Lens value to 0
	  cmdSet(cmdQuptr, 0x21/*noAFIrisMode*/, 0x25, 0x30, isAuto?1:2, dataIdx);  //set Iris Mode value for no-AF Lens to 0
	  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
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
					 if(CamMode == 1){//720p
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
					 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//exposure mode
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = CtrlParArry[CtrlID][14];//AGC
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 sendData1 = glEp0Buffer[2];
					 CyU3PDebugPrint (4, "ExpM&AGC sent to host. %d %d; %d %d\r\n", glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3]);
					 break;

			 	 case BrgtCtlID1:
					 Data0 = CtrlParArry[CtrlID][13];  //SensorGetControl(RegAdd0, devAdd); //SensorGetBLCMode();
					 Data1 = CtrlParArry[CtrlID][14];  //SensorGetControl(RegAdd1, devAdd);
					 if (Data1&0x2){ //check the sign bit (bit1)
						 Data1 = ((Data1<<6)&0x40)| (Data0 >> 2);//clear MSB
					 }else{
						 Data1 = ((Data1<<6)|0x80)| (Data0 >> 2);//set MSB
					 }
					 glEp0Buffer[0] = Data1;
					 glEp0Buffer[1] = 0;
					 sendData = glEp0Buffer[0];
					 break;
				 case HueCtlID5:
					 glEp0Buffer[0] = CtrlParArry[CtrlID][13];//SensorGetControl(HuectrlRegRed, devAdd);
					 glEp0Buffer[0] = glEp0Buffer[0] + GREEN_BASE;
					 glEp0Buffer[1] = 0;
					 sendData = glEp0Buffer[0];
					 break;
				 case WBTLevCtlID11:
					 glEp0Buffer[0] = WBMenuCmpArry[0];//using for blue part
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = WBMenuCmpArry[2];//using for red part
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 sendData1 = glEp0Buffer[2];
					 break;
				 case SaturCtlID6:
				 default:
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
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData, dataIdx);  //Exposure
							 if(getData != 0){
								 dataIdx++;
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
							 Data0 = glEp0Buffer[0]; //blue
							 Data1 = glEp0Buffer[2]; //red
							 dataIdx = 0;

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //Second
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
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
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
/************** CT control requests handler *************************/
#define EXLIMIT  200  //shutter value limit in 30 fps

inline void CTControlHandle(uint8_t CtrlID){
    CyU3PReturnStatus_t apiRetStatus = !CY_U3P_SUCCESS;
    VdRingBuf *cmdQuptr = &cmdQu;
    uint16_t readCount;
    uint8_t RegAdd0, RegAdd1, Data0, Data1, Len;
    uint16_t diff, value, diffRd;
    uint8_t i, shutter, index;
    diff = 0xffff;
    shutter = 1;
    index = 1;

    uint8_t devAdd = CTCtrlParArry[CtrlID][15];
    RegAdd0 = CTCtrlParArry[CtrlID][0];
    RegAdd1 = CTCtrlParArry[CtrlID][1];
    Len = CTCtrlParArry[CtrlID][2];
    uint8_t dataIdx, getData=0xFF, getData1=0xff, sendData=0xff, sendData1=0xFF, reqData;
#ifdef USB_DEBUG_PRINT
    CyU3PDebugPrint (4, "The cur sensor value(CT) %d 0x%x 0x%x\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14]); // additional debug
#endif
    reqData = bRequest;

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
				 default:
					 glEp0Buffer[0] = CTCtrlParArry[CtrlID][13];
					 glEp0Buffer[1] = CTCtrlParArry[CtrlID][14];
					 glEp0Buffer[2] = 0;
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 break;
			 }

			 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);

#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The get sensor value(CT) %d 0x%x 0x%x, %d %d\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14], glEp0Buffer[0], Len); // additional debug
#endif
			  break;
		 case CY_FX_USB_UVC_GET_MIN_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][3];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][4];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;//1;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_MAX_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][5];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][6];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_RES_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][7];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][8];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_INFO_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][9];
			  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  Len = 1;
			  break;
		 case CY_FX_USB_UVC_GET_DEF_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][11];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][12];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_SET_CUR_REQ:
			  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
			  glEp0Buffer, &readCount);
			  Data0 = glEp0Buffer[0];
			  Data1 = glEp0Buffer[1];
			  value = Data1;

			  switch(CtrlID)
			  {
		  	      case AutoExMCtlID1:
		  		    //CyU3PDebugPrint (4, "The Ex Mode value(CT) %d 0x%x 0x%x 0x%x 0x%x, %d!\r\n", CtrlID, glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3], readCount); // additional debug

				    CTCtrlParArry[CtrlID][13] = Data0;
				    CTCtrlParArry[CtrlID][16] = CyTrue;
				    getData = glEp0Buffer[0];
		  		    //CyU3PDebugPrint (4, "The Ex Mode set value(CT) %d %d!\r\n", CtrlID, CTCtrlParArry[CtrlID][13]); // additional debug
		  		    switch (getData){
						case 1:
							setIrisauto(cmdQuptr, 0); //set Iris being manual.
							break;
						case 2:
			  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
							dataIdx = 0;
							CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
							CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
				  		    setIrisauto(cmdQuptr, 1); //set Iris being auto.

							break;
						case 4:
			  		    	setIrisauto(cmdQuptr, 1); //set Iris being auto.
							break;
						case 8:
			  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
			  		    	dataIdx = 0;
			  		    	CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
			  		    	cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
			  		    	CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
			  		    	setIrisauto(cmdQuptr, 0); //set Iris being manual.
							break;
		  		    }
#if 0
				    if(getData == 2 || getData == 8){//if exposure mode is auto or aperture priority
		  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
						  dataIdx = 0;
						  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
						  cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
						  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
		  		    }
		  		    if(getData == 2 || getData == 4){//if exposure mode is auto or exprosue priority
		  		    	setIrisauto(cmdQuptr, 1); //set Iris being auto.
		  		    }
#endif
				    break;

			  	  case ExTmACtlID3:
			  		//CyU3PDebugPrint (4, "The Ex Time value(CT) %d 0x%x 0x%x 0x%x 0x%x, %d!\r\n", CtrlID, glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3], readCount); // additional debug

					  value = (value << 8)|Data0;
					  if(((CTCtrlParArry[AutoExMCtlID1][13] == 1) || (CTCtrlParArry[AutoExMCtlID1][13] == 4))
							  && (value < (EXLIMIT+50)))//shutter set accepted
					  {
						  for(i = 0; i < 8; i++)//find closest shutter No.
						  {
							if(value > ShutValueArry[i]){
								diffRd = value-ShutValueArry[i];
							}else{
								diffRd = ShutValueArry[i]-value;
							}
							  if(diff > diffRd){
								  diff = diffRd;
								  index = i;
							  }
						  }
						  shutter = shutter+index;

						  dataIdx = 0;
						  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
						  cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, shutter, dataIdx);  //First
						  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						  //CyU3PDebugPrint (4, "The Ex Time shutter value(CT) %d %d %d %d!\r\n", shutter, index, ShutValueArry[index], diff); // additional debug

						  CTCtrlParArry[CtrlID][13] = Data0;
						  CTCtrlParArry[CtrlID][14] = Data1;
						  CTCtrlParArry[CtrlID][16] = CyTrue;
						  CtrlParArry[ExtShutCtlID0][13] = shutter; //set extension shutter current value
					  }else{
						  CyU3PUsbStall (0, CyTrue, CyFalse);
					  }
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];
					  break;
			  	  case IriACtlID7:
					  if((CTCtrlParArry[AutoExMCtlID1][13] == 1) || (CTCtrlParArry[AutoExMCtlID1][13] == 8))//Iris set accepted
					  {
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, 0x22, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 //CyU3PEventSet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);//set event of the command available.

							 CTCtrlParArry[CtrlID][13] = Data0;
							 CTCtrlParArry[CtrlID][14] = Data1;
							 CTCtrlParArry[CtrlID][16] = CyTrue;
					  }else{
						  CyU3PUsbStall (0, CyTrue, CyFalse);
					  }
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];

					  break;
			  	  case ZmOpRCtlID10:
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];
#if 1
					  dataIdx = 0;
					  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					  if(getData == 1)
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, TELEDATA, dataIdx);  //telephoto direction
					  else if(getData == 0xff)
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, WIDEDATA, dataIdx);  //wide-angle direction
					  else
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, STOP, dataIdx);
					  //dataIdx++;
					  //cmdSet(cmdQuptr, 23, RegAdd0, devAdd, STOP, dataIdx); //for temp implementation for stop zoom
					  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
#endif
					  CyU3PDebugPrint (4, "Zoom Op receives (CT) 0x%x 0x%x 0x%x\r\n", getData, getData1, glEp0Buffer[2]);
					  break;

			  	  default:
					 CTCtrlParArry[CtrlID][13] = glEp0Buffer[0];
					 CyU3PDebugPrint (4, "default selector (CT) 0x%x 0x%x\r\n", CtrlID, bRequest); // additional debug
			  		 break;
			  }
			  break;
		  default:
			  CyU3PUsbStall (0, CyTrue, CyFalse);
			  CyU3PDebugPrint (4, "default request (CT) 0x%x 0x%x\r\n", CtrlID, bRequest); // additional debug
			  break;
		 }
	//CyU3PDebugPrint (4, "The get sensor value(CT) %d 0x%x 0x%x, %d %d\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14], glEp0Buffer[0], Len); // additional debug

    CyU3PDebugPrint (4, "The Request 0x%x parameter get from host (CT) 0x%x 0x%x 0x%x / send to host 0x%x 0x%x 0x%x, %d\r\n", reqData, getData, getData1, glEp0Buffer[2], sendData, sendData1, glEp0Buffer[2], Len);
}

/************** send default parameters to camera at the beginning **************/
void CamDefSet(void) //it's not called at anywhere right now
{
    VdRingBuf *cmdQuptr = &cmdQu;
    uint8_t RegAdd, devAdd, Data;
    uint8_t CtrlID, Data0, Data1;

    CtrlID = BrgtCtlID1;
    RegAdd = CtrlParArry[CtrlID][1];
    devAdd = CtrlParArry[CtrlID][15];
    Data0 = CtrlParArry[CtrlID][11];
    Data1 = Data0;

    CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
    if(Data1&0x80){
    	Data1 = (((Data1 >> 6)&0x01)|0xC4);
    }else{
    	Data1 = (((Data1 >> 6)|0x02)|0xC4);
    }
    Data0 = (Data0 << 2);

	cmdSet(cmdQuptr, CtrlID, RegAdd, devAdd, Data1, First); //brightness

	RegAdd = CtrlParArry[CtrlID][0];
	cmdSet(cmdQuptr, CtrlID, RegAdd, devAdd, Data0, Second);
	CtrlParArry[CtrlID][13] = Data0;
	CtrlParArry[CtrlID][14] = Data1;
	CyU3PDebugPrint (4, "The set def data 0x%x, 0x%x.\r\n", Data1, Data0);

    CtrlID = ConsCtlID2;
    RegAdd = CtrlParArry[CtrlID][0];
    devAdd = CtrlParArry[CtrlID][15];
    Data = CtrlParArry[CtrlID][11];
	cmdSet(cmdQuptr, CtrlID, RegAdd, devAdd, Data, First); //contrast
	CtrlParArry[CtrlID][13] = Data0;
	CyU3PDebugPrint (4, "The set def data 0x%x, 0x%x.\r\n", Data, Data0);

    CtrlID = HueCtlID5;
    RegAdd = CtrlParArry[CtrlID][0];
    devAdd = CtrlParArry[CtrlID][15];
    Data = CtrlParArry[CtrlID][11];
    cmdSet(cmdQuptr, CtrlID, HuectrlRegGr, devAdd, (Data-GREEN_BASE), First);
    cmdSet(cmdQuptr, CtrlID, HuectrlRegMg, devAdd, (Data-MAGENTA_BASE), Second);
    cmdSet(cmdQuptr, CtrlID, HuectrlRegYel, devAdd, (Data-YELLOW_BASE), Third);
    cmdSet(cmdQuptr, CtrlID, HuectrlRegCy, devAdd, (Data-CYAN_BASE), Fourth);
    cmdSet(cmdQuptr, CtrlID, HuectrlRegRed, devAdd, (Data-RED_BASE), Fifth);
    cmdSet(cmdQuptr, CtrlID, HuectrlRegBlu, devAdd, (Data-BLUE_BASE), Sixth);
	CtrlParArry[CtrlID][13] = Data-GREEN_BASE;
	CyU3PDebugPrint (4, "The set def data 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.\r\n",
			(Data-GREEN_BASE), (Data-MAGENTA_BASE), (Data-YELLOW_BASE), (Data-CYAN_BASE), (Data-RED_BASE), (Data-BLUE_BASE));

    CtrlID = SaturCtlID6;
    devAdd = CtrlParArry[CtrlID][15];
    Data = CtrlParArry[CtrlID][11];
	cmdSet(cmdQuptr, CtrlID, SaturationRegR, devAdd, Data, First); //saturation
	cmdSet(cmdQuptr, CtrlID, SaturationRegB, devAdd, Data, Second); //saturation
	CtrlParArry[CtrlID][13] = Data;
	CyU3PDebugPrint (4, "The set def data 0x%x, 0x%x.\r\n", Data, Data0);

    CtrlID = ShapCtlID7;
    RegAdd = CtrlParArry[CtrlID][0];
    devAdd = CtrlParArry[CtrlID][15];
    Data = CtrlParArry[CtrlID][11];
	cmdSet(cmdQuptr, CtrlID, RegAdd, devAdd, Data, First); //shapness
	CtrlParArry[CtrlID][13] = Data0;
	CyU3PDebugPrint (4, "The set def data 0x%x, 0x%x.\r\n", Data, Data0);

	CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
	//CyU3PEventSet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);//set event of the command available.
	return;
}

/* Add the UVC packet header to the top of the specified DMA buffer. */
void
CyFxUVCAddHeader (
        uint8_t *buffer_p,              /* Buffer pointer */
        uint8_t frameInd                /* EOF or normal frame indication */
        )
{
    /* Copy header to buffer */
	CyU3PMutexGet(&imgHdMux, CYU3P_WAIT_FOREVER);
    CyU3PMemCopy (buffer_p, (uint8_t *)glUVCHeader, CY_FX_UVC_MAX_HEADER);
	CyU3PMutexPut(&imgHdMux);

    /* The EOF flag needs to be set if this is the last packet for this video frame. */
    if (frameInd & CY_FX_UVC_HEADER_EOF)
    {
        buffer_p[1] |= CY_FX_UVC_HEADER_EOF;
    }
}


/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* This function is hit when we have hit a critical application error. This is not
       expected to happen, and the current implementation of this function does nothing
       except stay in a loop printing error messages through the UART port.

       This function can be modified to take additional error handling actions such
       as cycling the USB connection or performing a warm reset.
     */
    for (;;)
    {
        CyU3PDebugPrint (4, "Error handler...\r\n");
        CyU3PThreadSleep (1000);
    }
}

/* This function performs the operations for a Video Streaming Abort.
   This is called every time there is a USB reset, suspend or disconnect event.
 */
static void
CyFxUVCApplnAbortHandler (
        void)
{
	uint32_t flag;
	if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag,CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
	{
        /* Clear the Video Stream Request Event */
        CyU3PEventSet (&glFxUVCEvent, ~(CY_FX_UVC_STREAM_EVENT), CYU3P_EVENT_AND);

        /* Set Video Stream Abort Event */
        CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
	}
}

/* This is the Callback function to handle the USB Events */
static void
CyFxUVCApplnUSBEventCB (
        CyU3PUsbEventType_t evtype,  /* Event type */
        uint16_t             evdata  /* Event data */
        )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_RESET:
            CyU3PDebugPrint (4, "RESET encountered...0x%x 0x%x\r\n", evtype, evdata);
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

        case CY_U3P_USB_EVENT_SUSPEND:
            CyU3PDebugPrint (4, "SUSPEND encountered...0x%x 0x%x\r\n", evtype, evdata);
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

        case CY_U3P_USB_EVENT_DISCONNECT:
            CyU3PDebugPrint (4, "USB disconnected...0x%x 0x%x\r\n", evtype, evdata);
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            isUsbConnected = CyFalse;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

#ifdef BACKFLOW_DETECT
        case CY_U3P_USB_EVENT_EP_UNDERRUN:
            CyU3PDebugPrint (4, "CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
            break;
#endif

        default:
            break;
    }
}

/* Callback to handle the USB Setup Requests and UVC Class events */
static CyBool_t
CyFxUVCApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
        )
{
    CyBool_t uvcHandleReq = CyFalse;
    uint32_t status;

    /* Obtain Request Type and Request */
    bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
    bRequest  = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
    wValue    = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
    wIndex    = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
    wLength   = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);

    /* Check for UVC Class Requests */
    switch (bmReqType)
    {
        case CY_FX_USB_UVC_GET_REQ_TYPE:
        case CY_FX_USB_UVC_SET_REQ_TYPE:
            /* UVC Specific requests are handled in the EP0 thread. */
            switch (wIndex & 0xFF)
            {
                case CY_FX_UVC_CONTROL_INTERFACE:
                    {
                        uvcHandleReq = CyTrue;
                        status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
                                CYU3P_EVENT_OR);
                        if (status != CY_U3P_SUCCESS)
                        {
                            CyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed %x\n", status);
                            CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                    }
                    break;

                case CY_FX_UVC_STREAM_INTERFACE:
                    {
                        uvcHandleReq = CyTrue;
                        status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
                                CYU3P_EVENT_OR);
                        if (status != CY_U3P_SUCCESS)
                        {
                            /* Error handling */
                            CyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n", status);
                            CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                    }
                    break;

                default:
                    break;
            }
            break;

        case CY_FX_USB_SET_INTF_REQ_TYPE:
            if (bRequest == CY_FX_USB_SET_INTERFACE_REQ)
            {
            	/* MAC OS sends Set Interface Alternate Setting 0 command after
            	 * stopping to stream. This application needs to stop streaming. */
                if ((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0))
                {
                	/* Stop GPIF state machine to stop data transfers through FX3 */
                	CyU3PDebugPrint (4, "Alternate setting 0..\r\n");
                    CyU3PGpifDisable (CyTrue);
                    gpif_initialized = 0;
                    streamingStarted = CyFalse;
                    /* Place the EP in NAK mode before cleaning up the pipe. */
                    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                    CyU3PBusyWait (100);

                    /* Reset and flush the endpoint pipe. */
                    CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                    CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                    CyU3PBusyWait (100);

                    /* Clear the stall condition and sequence numbers. */
                    CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
                    uvcHandleReq = CyTrue;
                    /* Complete Control request handshake */
                    CyU3PUsbAckSetup ();
                    /* Indicate stop streaming to main thread */
                    clearFeatureRqtReceived = CyTrue;
                    CyFxUVCApplnAbortHandler ();

                }
            }
            break;

        case CY_U3P_USB_TARGET_ENDPT:
            if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
            {
                if (wIndex == CY_FX_EP_BULK_VIDEO)
                {
                	/* Windows OS sends Clear Feature Request after it stops streaming,
                	 * however MAC OS sends clear feature request right after it sends a
                	 * Commit -> SET_CUR request. Hence, stop streaming only of streaming
                	 * has started. */
                    if (streamingStarted == CyTrue)
                    {
                        CyU3PDebugPrint (4, "Clear feature request detected..\r\n");

                        /* Disable the GPIF state machine. */
                        CyU3PGpifDisable (CyTrue);
                        gpif_initialized = 0;
                        streamingStarted = CyFalse;

                        /* Place the EP in NAK mode before cleaning up the pipe. */
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                        CyU3PBusyWait (100);

                        /* Reset and flush the endpoint pipe. */
                        CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                        CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                        CyU3PBusyWait (100);

                        /* Clear the stall condition and sequence numbers. */
                        CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

                        uvcHandleReq = CyTrue;
                        /* Complete Control request handshake */
                        CyU3PUsbAckSetup ();
                        /* Indicate stop streaming to main thread */
                        clearFeatureRqtReceived = CyTrue;
                        CyFxUVCApplnAbortHandler ();
                    }
                    else
                    {
                        uvcHandleReq = CyTrue;
                        CyU3PUsbAckSetup ();
                    }
                }
            }
            break;

        default:
            break;
    }

    /* Return status of request handling to the USB driver */
    return uvcHandleReq;
}

#define CyFxUvcAppInterInCallback 0  //no callback function of the interrupt status endpoint

/* DMA callback providing notification when each buffer has been sent out to the USB host.
 * This is used to track whether all of the data has been sent out.
 */
void
CyFxUvcApplnDmaCallback (
        CyU3PDmaMultiChannel *multiChHandle,
        CyU3PDmaCbType_t      type,
        CyU3PDmaCBInput_t    *input
        )
{
   // CyU3PDmaBuffer_t    produced_buffer;
#if 1
    CyU3PReturnStatus_t status;
    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
            if (input->buffer_p.count == CY_FX_UVC_BUF_FULL_SIZE)
            {
                CyFxUVCAddHeader (input->buffer_p.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
                fb++;
            }
            else
            {
                /* If we have a partial buffer, this is guaranteed to be the end of the video frame for uncompressed images. */
                CyFxUVCAddHeader (input->buffer_p.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);
                pb++;
                pbc = input->buffer_p.count;
               // hitFV = CyTrue;
#if 1   //remove the still flag clearing here
                if(stiflag == 0x0F){
                	CyU3PMutexGet(&imgHdMux, CYU3P_WAIT_FOREVER);
                	glUVCHeader[1] &= ~(1<<5);    //clear still image flag
                	CyU3PMutexPut(&imgHdMux);
                	stiflag = 0xAA;
                }
#endif
                hitFV = CyTrue;  //set the hitFV flag to indicate the the partial buffer has been committed.
            }

            /* Commit the updated DMA buffer to the USB endpoint. */
            prodCount++;
            status = CyU3PDmaMultiChannelCommitBuffer (&glChHandleUVCStream,
            		input->buffer_p.count + CY_FX_UVC_MAX_HEADER, 0);
            //CyU3PDebugPrint(1,"\r\n %d",input->buffer_p.count);
            if (status != CY_U3P_SUCCESS)
            {
                prodCount--;
                CyU3PDebugPrint (4, "Error in multichannelcommitbuffer: Code = %d, size = %x, dmaDone %x\r\n",
                        status, input->buffer_p.count, prodCount - consCount);
            }
    }
#endif
    if (type == CY_U3P_DMA_CB_CONS_EVENT)
    {
        consCount++;
        streamingStarted = CyTrue;
    }
}

/*
 * This function is called from the GPIF callback when we have reached the end of a video frame.
 * The DMA buffer containing the last part of the frame may not have been committed, and need to
 * be manually wrapped up. This function uses the current GPIF state ID to identify the socket on
 * which this last buffer is pending, and then uses the CyU3PDmaMultiChannelSetWrapUp function
 * to commit the buffer.
 */
static uint8_t
CyFxUvcAppCommitEOF (
        CyU3PDmaMultiChannel *handle,           /* Handle to DMA channel. */
        uint8_t stateId                         /* Current GPIF state ID. */
        )
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint8_t socket = 0xFF;      /*  Invalid value. */

    /* Verify that the current state is a terminal state for the GPIF state machine. */

    if(usbSpeed == CY_U3P_SUPER_SPEED)
    {
        switch (stateId)
        {
            case FULL_BUF_IN_SCK0:
            case FULL_BUF_IN_SCK1:
                /* Buffer is already full and would have been committed. Do nothing. */
                break;

            case PARTIAL_BUF_IN_SCK0:
                socket = 0;
                break;

            case PARTIAL_BUF_IN_SCK1:
                socket = 1;
                break;

            default:
            	CyU3PDebugPrint(1,"\r\n commiteof state = %d",stateId);
                /* Unexpected current state. Return error. */
                return 1;
        }
    }

    else if(usbSpeed == CY_U3P_HIGH_SPEED)
    {
        switch (stateId)
        {
#ifndef CAM720
#ifdef GPIFIIM
            case 13:
            case 24:
                /* Buffer is already full and would have been committed. Do nothing. */
                break;

            case 8:
                socket = 0;
                break;

            case 20:
                socket = 1;
                break;
#else
            case 11:
            case 18:
                /* Buffer is already full and would have been committed. Do nothing. */
                break;

            case 8:
                socket = 0;
                break;

            case 15:
                socket = 1;
                break;
#endif
#else
            case 11:
            case 18:
                /* Buffer is already full and would have been committed. Do nothing. */
                break;

            case 8:
                socket = 0;
                break;

            case 15:
                socket = 1;
                break;

#endif
             default:
            	CyU3PDebugPrint(1,"\r\n commiteof state = %d",stateId);
                /* Unexpected current state. Return error. */
               return 1;
        }
    }

    if (socket != 0xFF)
    {
        /* We have a partial buffer. Commit the buffer manually. The Wrap Up API, here, helps produce a
           partially filled buffer on the producer side. This action will cause CyU3PDmaMultiChannelGetBuffer API
           in the UVCAppThread_Entry function to succeed one more time with less than full producer buffer count */
        apiRetStatus = CyU3PDmaMultiChannelSetWrapUp (handle, socket);
        if (apiRetStatus != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "Channel Set WrapUp failed, Error code = %d\r\n", apiRetStatus);
            CyFxAppErrorHandler (apiRetStatus);
        }
    }

    return 0;
}

/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
void
CyFxGpifCB (
        CyU3PGpifEventType event,
        uint8_t currentState
        )
{
    if (event == CYU3P_GPIF_EVT_SM_INTERRUPT)
    {
        //hitFV = CyTrue;
        if (CyFxUvcAppCommitEOF (&glChHandleUVCStream, currentState) != CY_U3P_SUCCESS)
            CyU3PDebugPrint (4, "Commit EOF failed!\n");
    }
}

/* This function initializes the Debug Module for the UVC Application */
static void
CyFxUVCApplnDebugInit (
        void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "UART initialization failed!\n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Set UART Configuration */
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity   = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma    = CyTrue;

    /* Set the UART configuration */
    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Set the UART transfer */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Initialize the Debug logger module. */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 4);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Disable log message headers. */
    CyU3PDebugPreamble (CyFalse);
}

/* I2C initialization. */
//static void
void
CyFxUVCApplnI2CInit (void)
{
    CyU3PI2cConfig_t i2cConfig;;
    CyU3PReturnStatus_t status;

    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "I2C initialization failed!\n");
        CyFxAppErrorHandler (status);
    }

    /*  Set I2C Configuration */
    i2cConfig.bitRate    = 100000;      /*  100 KHz */
    i2cConfig.isDma      = CyFalse;
    i2cConfig.busTimeout = 0xffffffffU;
    i2cConfig.dmaTimeout = 0xffff;

    status = CyU3PI2cSetConfig (&i2cConfig, 0);
    if (CY_U3P_SUCCESS != status)
    {
        CyU3PDebugPrint (4, "I2C configuration failed!\n");
        CyFxAppErrorHandler (status);
    }
}

#ifdef BACKFLOW_DETECT
static void CyFxUvcAppPibCallback (
        CyU3PPibIntrType cbType,
        uint16_t cbArg)
{
    if ((cbType == CYU3P_PIB_INTR_ERROR) && ((cbArg == 0x1005) || (cbArg == 0x1006)))
    {
        if (!back_flow_detected)
        {
            CyU3PDebugPrint (4, "Backflow detected...\r\n");
            back_flow_detected = 1;
        }
    }
}
#endif

#ifdef USB_DEBUG_INTERFACE
static void
CyFxUvcAppDebugCallback (
        CyU3PDmaChannel   *handle,
        CyU3PDmaCbType_t   type,
        CyU3PDmaCBInput_t *input)
{
    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* Data has been received. Notify the EP0 thread which handles the debug commands as well. */
        CyU3PEventSet (&glFxUVCEvent, CY_FX_USB_DEBUG_CMD_EVENT, CYU3P_EVENT_OR);
    }
}
#endif

#if 0
static void CyFxAppIntEpCb(
		CyU3PUsbEpEvtType evType,
		CyU3PUSBSpeed_t  usbSpeed,
		uint8_t  ebNum)
		{
			//CyBool_t value;
			if((evType == 1)&&(ebNum == 0x82))
				CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR); //set sending interrupt status event for snap button press

			//CyU3PDebugPrint (4, "Interrpt EP event 0x%x, %d, 0x%x\r\n", evType, usbSpeed, ebNum);
		}
#endif

/* This function initializes the USB Module, creates event group,
   sets the enumeration descriptors, configures the Endpoints and
   configures the DMA module for the UVC Application */
static void
CyFxUVCApplnInit (void)
{
    CyU3PDmaMultiChannelConfig_t dmaMultiConfig;
    CyU3PEpConfig_t              endPointConfig;
    CyU3PReturnStatus_t          apiRetStatus;
    CyU3PGpioClock_t             gpioClock;
    CyU3PGpioSimpleConfig_t      gpioConfig;
    CyU3PPibClock_t              pibclock;

    CyU3PDmaChannelConfig_t dmaInterConfig; 			//for interrupt endpoint
    //CyU3PDmaMultiChannelConfig_t dmaMultiStillConfig;   //for still image channel

#ifdef USB_DEBUG_INTERFACE
    CyU3PDmaChannelConfig_t channelConfig;
#endif

    /* Create UVC event group */
    apiRetStatus = CyU3PEventCreate (&glFxUVCEvent);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "UVC Create Event failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

#ifdef UVC_PTZ_SUPPORT
    CyFxUvcAppPTZInit ();
#endif

    isUsbConnected = CyFalse;
    clearFeatureRqtReceived = CyFalse;

    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 2;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK;
    gpioClock.halfDiv    = 0;

    /* Initialize Gpio interface */
    apiRetStatus = CyU3PGpioInit (&gpioClock, NULL);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO Init failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* CTL pins are restricted and cannot be configured using I/O matrix configuration function,
     * must use GpioOverride to configure it */
    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_RESET_GPIO, CyTrue);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO Override failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_POWER_GPIO, CyTrue);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO(20) Override failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_SNAPSHOT_GPIO, CyTrue);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO(24) Override failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* SENSOR_RESET_GPIO is the Sensor reset pin */
    gpioConfig.outValue    = CyTrue;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus           = CyU3PGpioSetSimpleConfig (SENSOR_RESET_GPIO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "GPIO Set (reset 22) Config Error, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* SENSOR_POWER_GPIO is the Sensor power on/off pin */
//    gpioConfig.outValue    = CyFalse; /* set output value is low and the rest of the setting is the same as what of the reset gpio pin */
    gpioConfig.outValue    = CyTrue;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus           = CyU3PGpioSetSimpleConfig (SENSOR_POWER_GPIO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "GPIO Set (power 20) Config Error, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* SENSOR_SNAPSHOT_GPIO is the Sensor snap shot button detecting pin */
//    gpioConfig.outValue    = CyFalse; /* set output value is low and the rest of the setting is the same as what of the reset gpio pin */
    gpioConfig.outValue    = CyFalse;
    gpioConfig.driveLowEn  = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.inputEn     = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus           = CyU3PGpioSetSimpleConfig (SENSOR_SNAPSHOT_GPIO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "GPIO Set (snap shot 24) Config Error, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Initialize the P-port. */
    pibclock.clkDiv      = 2;
    pibclock.clkSrc      = CY_U3P_SYS_CLK;
    pibclock.isDllEnable = CyFalse;
    pibclock.isHalfDiv   = CyFalse;

    apiRetStatus = CyU3PPibInit (CyTrue, &pibclock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "PIB Function Failed to Start, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Setup the Callback to Handle the GPIF INTR event */
    CyU3PGpifRegisterCallback (CyFxGpifCB);

#ifdef BACKFLOW_DETECT
    back_flow_detected = 0;
    CyU3PPibRegisterCallback (CyFxUvcAppPibCallback, CYU3P_PIB_INTR_ERROR);
#endif

    /* Image sensor initialization. Reset and then initialize with appropriate configuration. */
    SensorReset ();
    SensorInit ();

    /* USB initialization. */
    apiRetStatus = CyU3PUsbStart ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Function Failed to Start, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
    /* Setup the Callback to Handle the USB Setup Requests */
    CyU3PUsbRegisterSetupCallback (CyFxUVCApplnUSBSetupCB, CyFalse);

    /* Setup the Callback to Handle the USB Events */
    CyU3PUsbRegisterEventCallback (CyFxUVCApplnUSBEventCB);

    /* setup the callback to handle the interrupt endpoint events */
    //CyU3PUsbRegisterEpEvtCallback(CyFxAppIntEpCb, 0x000000FF, 0, 0xFF); //0x04 for interrupt endpoint (SuperSpeed)

    /* Register the USB device descriptors with the driver. */
    CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSBDeviceDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSBDeviceDscrSS);

    /* BOS and Device qualifier descriptors. */
    CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)CyFxUSBDeviceQualDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)CyFxUSBBOSDscr);

    /* Configuration descriptors. */
    CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBHSConfigDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBFSConfigDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBSSConfigDscr);

    /* String Descriptors */
    CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);

    /* Configure the status interrupt endpoint.
       Note: This endpoint is not being used by the application as of now. This can be used in case
       UVC device needs to notify the host about any error conditions. A MANUAL_OUT DMA channel
       can be associated with this endpoint and used to send these data packets.
     */
    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_INTR;
    endPointConfig.pcktSize = 64;
    endPointConfig.isoPkts  = 0;
    endPointConfig.streams  = 0;
    endPointConfig.burstLen = 1;
    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_CONTROL_STATUS, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* create a DMA for interrupt endpoint */
    dmaInterConfig.size           = 1024;
    dmaInterConfig.count          = 1;
    dmaInterConfig.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
    dmaInterConfig.consSckId      = CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_CONTROL_STATUS_SOCKET;
    dmaInterConfig.prodAvailCount = 0;
    dmaInterConfig.prodHeader     = 0;
    dmaInterConfig.prodFooter     = 0;
    dmaInterConfig.consHeader     = 0;
    dmaInterConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaInterConfig.notification   = CY_U3P_DMA_CB_CONS_EVENT;
    dmaInterConfig.cb             = CyFxUvcAppInterInCallback;
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleInterStat, CY_U3P_DMA_TYPE_MANUAL_OUT,
            &dmaInterConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "DMA Interrupt Status Channel Creation Failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    glInterStaBuffer = (uint8_t *)CyU3PDmaBufferAlloc (1024);
    if (glInterStaBuffer == 0)
    {
        CyU3PDebugPrint (4, "Failed to allocate memory for interrupt status buffer\r\n");
        CyFxAppErrorHandler (CY_U3P_ERROR_MEMORY_ERROR);
    }

    /* Create a DMA Manual channel for sending the video data to the USB host. */
    CyU3PMutexCreate(&imgHdMux, CYU3P_NO_INHERIT);// create a mutex for the image header operation.
    dmaMultiConfig.size           = CY_FX_UVC_STREAM_BUF_SIZE;
    dmaMultiConfig.count          = CY_FX_UVC_STREAM_BUF_COUNT;
    dmaMultiConfig.validSckCount  = 2;
    dmaMultiConfig.prodSckId [0]  = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_0;
    dmaMultiConfig.prodSckId [1]  = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_1;
    dmaMultiConfig.consSckId [0]  = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET);
    dmaMultiConfig.prodAvailCount = 0;
    dmaMultiConfig.prodHeader     = 12;                 /* 12 byte UVC header to be added. */
    dmaMultiConfig.prodFooter     = 4;                  /* 4 byte footer to compensate for the 12 byte header. */
    dmaMultiConfig.consHeader     = 0;
    dmaMultiConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaMultiConfig.notification   = CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
    dmaMultiConfig.cb             = CyFxUvcApplnDmaCallback;
    apiRetStatus = CyU3PDmaMultiChannelCreate (&glChHandleUVCStream, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE,
            &dmaMultiConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "DMA Channel Creation Failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

#ifdef USB_DEBUG_INTERFACE
    /* Configure the endpoints and create DMA channels used by the USB debug interface.
       The command (OUT) endpoint is configured in packet mode and enabled to receive data.
       Once the CY_U3P_DMA_CB_PROD_EVENT callback is received, the received data packet is
       processed and the data is returned through the CyU3PDmaChannelSetupSendBuffer API call.
     */

    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_BULK;
    endPointConfig.pcktSize = 1024;                     /* Use SuperSpeed settings here. */
    endPointConfig.isoPkts  = 0;
    endPointConfig.streams  = 0;
    endPointConfig.burstLen = 1;

    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_DEBUG_CMD, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Debug Command endpoint config failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    CyU3PUsbSetEpPktMode (CY_FX_EP_DEBUG_CMD, CyTrue);

    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_DEBUG_RSP, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Debug Response endpoint config failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    channelConfig.size           = 1024;
    channelConfig.count          = 1;
    channelConfig.prodSckId      = CY_U3P_UIB_SOCKET_PROD_0 | CY_FX_EP_DEBUG_CMD_SOCKET;
    channelConfig.consSckId      = CY_U3P_CPU_SOCKET_CONS;
    channelConfig.prodAvailCount = 0;
    channelConfig.prodHeader     = 0;
    channelConfig.prodFooter     = 0;
    channelConfig.consHeader     = 0;
    channelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    channelConfig.notification   = CY_U3P_DMA_CB_PROD_EVENT;
    channelConfig.cb             = CyFxUvcAppDebugCallback;

    apiRetStatus = CyU3PDmaChannelCreate (&glDebugCmdChannel, CY_U3P_DMA_TYPE_MANUAL_IN, &channelConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Debug Command channel create failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer (&glDebugCmdChannel, 0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Debug channel SetXfer failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    channelConfig.size           = 1024;
    channelConfig.count          = 0;           /* No buffers allocated. We will only use the SetupSend API. */
    channelConfig.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
    channelConfig.consSckId      = CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_DEBUG_RSP_SOCKET;
    channelConfig.prodAvailCount = 0;
    channelConfig.prodHeader     = 0;
    channelConfig.prodFooter     = 0;
    channelConfig.consHeader     = 0;
    channelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    channelConfig.notification   = 0;
    channelConfig.cb             = 0;

    apiRetStatus = CyU3PDmaChannelCreate (&glDebugRspChannel, CY_U3P_DMA_TYPE_MANUAL_OUT, &channelConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Debug Response channel create failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    glDebugRspBuffer = (uint8_t *)CyU3PDmaBufferAlloc (1024);
    if (glDebugRspBuffer == 0)
    {
        CyU3PDebugPrint (4, "Failed to allocate memory for debug buffer\r\n");
        CyFxAppErrorHandler (CY_U3P_ERROR_MEMORY_ERROR);
    }
#endif

    /* Enable USB connection from the FX3 device, preferably at USB 3.0 speed. */
    apiRetStatus = CyU3PConnectState (CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Connect failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    CyU3PBusyWait(100);

    usbSpeed = CyU3PUsbGetSpeed();

    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_BULK;
    if(usbSpeed == CY_U3P_SUPER_SPEED)  /*for usb3.0 super-speed mode */
    {
    	endPointConfig.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
    	endPointConfig.burstLen = 16;
    }
    else		/* for usb2.0 high-speed mode */
    {
    	endPointConfig.pcktSize = 0x200;
    	endPointConfig.burstLen = 1;
    }
    endPointConfig.streams  = 0;
    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_BULK_VIDEO, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
#if 0    //for still image method 3 using
    apiRetStatus = CyU3PSetEpConfig (0x87, &endPointConfig); //configure still image endpoint
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "USB Set Endpoint config failed(0x87), Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
#endif

}

/*
 * Load the GPIF configuration on the GPIF-II engine. This operation is performed whenever a new video
 * streaming session is started.
 */
static void
CyFxUvcAppGpifInit (
        void)
{
    CyU3PReturnStatus_t apiRetStatus=0;

    if(usbSpeed == CY_U3P_SUPER_SPEED)
    {
    	CyU3PDebugPrint(1,"\r\n super gpif");
    	apiRetStatus =  CyU3PGpifLoad ((CyU3PGpifConfig_t *) &CyFxGpifConfig);
    }
    else if(usbSpeed == CY_U3P_HIGH_SPEED)
    {
    	CyU3PDebugPrint(1,"\r\n high gpif");
    	apiRetStatus =  CyU3PGpifLoad ((CyU3PGpifConfig_t *) &CyFxGpifConfig_usb2);
    }
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "Loading GPIF Configuration failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Start the state machine from the designated start state. */
    if(usbSpeed == CY_U3P_SUPER_SPEED)
    {
    	apiRetStatus = CyU3PGpifSMStart (START, ALPHA_START);
    }
    else if (usbSpeed == CY_U3P_HIGH_SPEED)
    {
    	apiRetStatus = CyU3PGpifSMStart (START_USB2, ALPHA_START_USB2);
    }
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "Starting GPIF state machine failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
}

/*
 * Entry function for the UVC Application Thread
 */

uint32_t posTick;
CyU3PTimer I2CCmdTimer;

void  I2CCmdCb(uint32_t input){
	//CyU3PDebugPrint (4, "I2C pos-timer %d %d\r\n", posTick, input);
	CyU3PEventSet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);
}


void
UVCAppThread_Entry (
        uint32_t input)
{
    //CyU3PDmaBuffer_t    produced_buffer; //for streaming header handle here
    //CyU3PDmaBuffer_t    interStabuf; for sencond interrupt status schem
	CyU3PReturnStatus_t apiRetStatus;
    uint8_t i = 0;
    uint32_t flag;
    uint32_t prinflag = 0;
static uint8_t IMcount = 0;
#ifdef DEBUG_PRINT_FRAME_COUNT
    uint32_t frameCnt = 0;
#endif
    /* Initialize the Uart Debug Module */
    CyFxUVCApplnDebugInit ();

    /* Initialize the I2C interface */
	while (i++ < 6){
		CyU3PThreadSleep(500);
	}

    CyFxUVCApplnI2CInit ();

    /* Initialize the UVC Application */
    CyFxUVCApplnInit ();

    /*
       This thread continually checks whether video streaming is enabled, and commits video data if so.

       The CY_FX_UVC_STREAM_EVENT and CY_FX_UVC_STREAM_ABORT_EVENT event flags are monitored by this
       thread. The CY_FX_UVC_STREAM_EVENT event flag is enabled when the USB host sends a COMMIT control
       request to the video streaming interface, and stays ON as long as video streaming is enabled.

       The CY_FX_UVC_STREAM_ABORT_EVENT event indicates that we need to abort the video streaming. This
       only happens when we receive a CLEAR_FEATURE request indicating that streaming is to be stopped,
       or when we have a critical error in the data path. In both of these cases, the CY_FX_UVC_STREAM_EVENT
       event flag will be cleared before the CY_FX_UVC_STREAM_ABORT_EVENT event flag is enabled.

       This sequence ensures that we do not get stuck in a loop where we are trying to send data instead
       of handling the abort request.
     */

    for (;;)
    {
        /* Waiting for the Video Stream Event */
        if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag,
                    CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
        {
#if 0 //test for new firmware no video bring up
        	//CyU3PDebugPrint(4,"\r\n gpif switch(2) 0x%x %d\r\n", apiRetStatus, curstate);// track the low res
        	/* Check if we have a buffer ready to go. */
            apiRetStatus = CyU3PDmaMultiChannelGetBuffer (&glChHandleUVCStream, &produced_buffer, CYU3P_NO_WAIT);
            if (apiRetStatus == CY_U3P_SUCCESS)
            {
            	//CyU3PDebugPrint(4," gpif switch(3)\r\n");// track the low res
            	if (produced_buffer.count == CY_FX_UVC_BUF_FULL_SIZE)
                {
                    CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
#ifdef  USB_LOWRES_IMG
#ifdef  USB_DEBUG_PRINT
                    CyU3PDebugPrint(4," gpif switch(full) 0x%x\r\n", produced_buffer.count);// track the low res
#endif
#endif
                    }
                else
                {
                    /* If we have a partial buffer, this is guaranteed to be the end of the video frame for uncompressed images. */
                    CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);
#ifdef USB_LOWRES_IMG
                    gotPartial = CyFalse; /* (low res)Flag is reset to indicate that the partial buffer was committed to USB */
#ifdef USB_DEBUG_PRINT
//                    CyU3PDebugPrint(4," gpif switch(partial) 0x%x %d %d\r\n", produced_buffer.count, prodCount, consCount);// track the low res
#endif
#endif
                }

                /* Commit the updated DMA buffer to the USB endpoint. */
                prodCount++;
                //CyU3PDebugPrint(1,"buffer count %d\r\n", produced_buffer.count);
                apiRetStatus = CyU3PDmaMultiChannelCommitBuffer (&glChHandleUVCStream,
                        produced_buffer.count + CY_FX_UVC_MAX_HEADER, 0);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    prodCount--;
                    CyU3PDebugPrint (4, "Error in multichannelcommitbuffer: Code = %d, size = %x, dmaDone %x\r\n",
                            apiRetStatus, produced_buffer.count, prodCount - consCount);
                }
            }
#endif
            /* If we have the end of frame signal and all of the committed data has been read by the USB host;
               we can reset the DMA channel and prepare for the next video frame. */
            if ((hitFV) && (prodCount == consCount))
            {
            	if(0&&(prinflag == 0)){
            		CyU3PDebugPrint (4, "(1) fb %d pb % pbc %\n", fb, pb, pbc);
            		prinflag = 1;
            	}
            	//fbbak=fb; pbbak=pb; pbcbak=pbc;
            	fb=0;
            	pb=0;
            	pbc=0;
                prodCount = 0;
                consCount = 0;
                hitFV     = CyFalse;

#ifdef BACKFLOW_DETECT
                back_flow_detected = 0;
#endif
#ifdef DEBUG_PRINT_FRAME_COUNT //it should be enabled as the frameCnt is used as timer in this version.
                frameCnt++;
#endif
                /* set image ready flag isFlag and set default camera parameters */
                //if(frameCnt%5 == 0){//2 frame interval
                	//;//CyU3PEventSet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR); //each frame trigger I2C thread sending command.
                //}
                /* Toggle UVC header FRAME ID bit */
            	CyU3PMutexGet(&imgHdMux, CYU3P_WAIT_FOREVER);
                glUVCHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;
            	//CyU3PMutexPut(&imgHdMux);
                 	if ((stiflag == 0xF0) && CyU3PEventGet (&glFxUVCEvent, VD_FX_UVC_STIL_EVENT, CYU3P_EVENT_AND_CLEAR, &flag,
                	                    CYU3P_NO_WAIT) == CY_U3P_SUCCESS){ //start full res.
                		//glUVCHeader[1] |= (1<<5);    //set still image flag
                       	//SensorSetIrisControl(0x1, 0x30, is60Hz? 0x64:0xE4, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                     	//CyU3PThreadSleep(100);
                		stiflag = 0xFF;
                		IMcount = 0;
                	}
                 	else if(stiflag==0xFF){//setting still marker in the stream head after one frame late

                 		if(IMcount++ >= 0x3){
                 		glUVCHeader[1] |= (1<<5);    //set still image flag
                		stiflag = 0x0F;
                		IMcount = 0;
                		}
                 		/*if(IMcount > 0x4){
                			stiflag = 0x0F;
                			IMcount = 0;
                		}*/

                }else if(stiflag==0xAA){//recovery video stream res. after one still frame set.
                    //CyU3PThreadSleep(400);
                	//CyU3PMutexGet(&imgHdMux, CYU3P_WAIT_FOREVER);
                   	//glUVCHeader[1] &= ~(1<<5);    //clear still image flag
                	//CyU3PMutexPut(&imgHdMux);

                	if(IMcount++ >= 0x3)
                	{
                    switch (setRes)
                     {
                 	case 1: //960
                 		SensorSetIrisControl(0x0b, 0x30, 0x0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                 		//CyU3PThreadSleep(100);
                        CyU3PDebugPrint (4, "Set the video mode format1 %x %d\n", 0xb, 0x0);
                 		break;
                 	case 2: //720
                 		SensorSetIrisControl(0x0b, 0x30, 0x1, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                 		//CyU3PThreadSleep(100);
                        CyU3PDebugPrint (4, "Set the video mode format1 %x %d\n", 0x0b, 0x1);
                 		break;
                 	default:
                 		break;
                     }
                    IMcount = 0;
                	//glUVCHeader[1] &= ~(1<<5);    //clear still image flag
                	stiflag = 0x0;
                	}
                }
                CyU3PMutexPut(&imgHdMux);
                /* Reset the DMA channel. */
                apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "DMA Channel Reset Failed, Error Code = %d\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Start Channel Immediately */
                apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "DMA Channel Set Transfer Failed, Error Code = %d\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Jump to the start state of the GPIF state machine. 257 is used as an
                   arbitrary invalid state (> 255) number. */
                CyU3PGpifSMSwitch (257, 0, 257, 0, 2);
                }
        }
        else
        {
            /* If we have a stream abort request pending. */
            if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_AND_CLEAR,
                        &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
            {
                hitFV     = CyFalse;
                prodCount = 0;
                consCount = 0;
                if(0&&(prinflag == 0)){
                	CyU3PDebugPrint (4, "(0) fb %d pb % pbc %\n", fb, pb, pbc);
                	prinflag = 1;
                }
                //fbbak=fb; pbbak=pb; pbcbak=pbc;
                fb=0;
                pb=0;
                pbc=0;

                if (!clearFeatureRqtReceived)
                {
                    apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                    if (apiRetStatus != CY_U3P_SUCCESS)
                    {
                        CyFxAppErrorHandler (apiRetStatus);
                    }

                    /* Flush the Endpoint memory */
                    CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                }

                clearFeatureRqtReceived = CyFalse;
            }
            else
            {
                /* We are essentially idle at this point. Wait for the reception of a start streaming request. */
                CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_WAIT_FOREVER);
                //CyU3PTimerStart(&I2CCmdTimer); //start timer again.
                //CyU3PDebugPrint (4, "start time tick  = %d\r\n", CyU3PGetTime());
                /* Set DMA Channel transfer size, first producer socket */
                apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    /* Error handling */
                    CyU3PDebugPrint (4, "DMA Channel Set Transfer Failed, Error Code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Initialize gpif configuration and waveform descriptors */
                if (gpif_initialized == CyFalse)
                {
#if 0
                	//for start up of the AF Lens
                   	SensorSetIrisControl(0x21, 0x30, 1, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris manual (AF Lens)
                    CyU3PThreadSleep(500);
                   	SensorSetIrisControl(0x25, 0x30, 2, I2C_DSPBOARD_ADDR_WR/*boardID*/);//set Iris manual (non AF Lens)
                    CyU3PThreadSleep(500);
                   	SensorSetIrisControl(0x23, 0x30, 0xa, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris value (DC manual)
                   	CyU3PThreadSleep(300);
                   	SensorSetIrisControl(0x21, 0x30, 0, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris auto (AF Lens)
                    CyU3PThreadSleep(500);
                   	SensorSetIrisControl(0x25, 0x30, 0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//set Iris auto (non AF Lens)
                    CyU3PThreadSleep(500);
#endif
                    CyFxUvcAppGpifInit ();
                    gpif_initialized = CyTrue;
                    CyU3PThreadSleep(200);
                    
                }
                else
                {
                    /* Jump to the start state of the GPIF state machine. 257 is used as an
                       arbitrary invalid state (> 255) number. */
                    CyU3PGpifSMSwitch (257, 0, 257, 0, 2);
                }
            }
        }
        CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR);//check snap shot button

        /* Allow other ready threads to run before proceeding. */
        CyU3PThreadRelinquish ();
    }
}

/*
 * Handler for control requests addressed to the Processing Unit.
 */

static void
UVCHandleProcessingUnitRqts (
        void)
{
    uint8_t CtrlAdd;
#ifdef DbgInfo
    CyU3PDebugPrint (4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
#endif
    switch (wValue)
    {
    	case CY_FX_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL:
    		CtrlAdd = CtrlParArry[BLCCtlID0][0];
    		ControlHandle(BLCCtlID0);
    		break;
        case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:
        	CtrlAdd = CtrlParArry[BrgtCtlID1][0];
   			ControlHandle(BrgtCtlID1);
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
        	CyU3PDebugPrint (4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
}

/*
 * Handler for control requests addressed to the UVC Camera Terminal unit.
 */
static void
UVCHandleCameraTerminalRqts (
        void)
{
#ifdef UVC_PTZ_SUPPORT
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;
    uint16_t zoomVal;
    int32_t  panVal, tiltVal;
    CyBool_t sendData = CyFalse;
#endif
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
        	CyU3PDebugPrint (4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }

#ifdef UVC_PTZ_SUPPORT
    switch (wValue)
    {
        case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* Support GET/SET queries. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ: /* Current zoom control value. */
                    zoomVal  = CyFxUvcAppGetCurrentZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMinimumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMaximumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution is one unit. */
                    zoomVal  = CyFxUvcAppGetZoomResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ: /* Default zoom setting. */
                    zoomVal  = CyFxUvcAppGetDefaultZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        zoomVal = (glEp0Buffer[0]) | (glEp0Buffer[1] << 8);
                        CyFxUvcAppModifyZoom (zoomVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 2-byte data in zoomVal back to the USB host. */
                glEp0Buffer[0] = CY_U3P_GET_LSB (zoomVal);
                glEp0Buffer[1] = CY_U3P_GET_MSB (zoomVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;

        case CY_FX_UVC_CT_PANTILT_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* GET/SET requests supported for this control */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                    panVal   = CyFxUvcAppGetCurrentPan ();
                    tiltVal  = CyFxUvcAppGetCurrentTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ:
                    panVal   = CyFxUvcAppGetMinimumPan ();
                    tiltVal  = CyFxUvcAppGetMinimumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ:
                    panVal   = CyFxUvcAppGetMaximumPan ();
                    tiltVal  = CyFxUvcAppGetMaximumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ:
                    panVal   = CyFxUvcAppGetPanResolution ();
                    tiltVal  = CyFxUvcAppGetTiltResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ:
                    panVal   = CyFxUvcAppGetDefaultPan ();
                    tiltVal  = CyFxUvcAppGetDefaultTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        panVal = (glEp0Buffer[0]) | (glEp0Buffer[1]<<8) |
                            (glEp0Buffer[2]<<16) | (glEp0Buffer[2]<<24);
                        tiltVal = (glEp0Buffer[4]) | (glEp0Buffer[5]<<8) |
                            (glEp0Buffer[6]<<16) | (glEp0Buffer[7]<<24);

                        CyFxUvcAppModifyPan (panVal);
                        CyFxUvcAppModifyTilt (tiltVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 8-byte PAN and TILT values back to the USB host. */
                glEp0Buffer[0] = CY_U3P_DWORD_GET_BYTE0 (panVal);
                glEp0Buffer[1] = CY_U3P_DWORD_GET_BYTE1 (panVal);
                glEp0Buffer[2] = CY_U3P_DWORD_GET_BYTE2 (panVal);
                glEp0Buffer[3] = CY_U3P_DWORD_GET_BYTE3 (panVal);
                glEp0Buffer[4] = CY_U3P_DWORD_GET_BYTE0 (tiltVal);
                glEp0Buffer[5] = CY_U3P_DWORD_GET_BYTE1 (tiltVal);
                glEp0Buffer[6] = CY_U3P_DWORD_GET_BYTE2 (tiltVal);
                glEp0Buffer[7] = CY_U3P_DWORD_GET_BYTE3 (tiltVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;
            //CyU3PDebugPrint (4, "The camera request received 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
        default:
            //CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
#endif
}

/*
 * Handler for UVC Interface control requests.
 */
static void
UVCHandleInterfaceCtrlRqts (
        void)
{

    switch (wValue)
    {
    	case CY_FX_UVC_POWER_MODE_CTRL: // shutter CONTROL1
    		CyU3PUsbStall (0, CyTrue, CyFalse);
    		break;
		case CY_FX_UVC_ERROR_CODE_CTRL: // sense up mode CONTROL2
      		CyU3PUsbStall (0, CyTrue, CyFalse);
      		//ControlHandle(0xff);//for control interface error code control.
    		break;
    	default:
     		CyU3PUsbStall (0, CyTrue, CyFalse);
     		break;
    }
/* No requests supported as of now. Just stall EP0 to fail the request. */

}

/*
 * Handler for control requests addressed to the Extension Unit.
 */
static void
UVCHandleExtensionUnitRqts (
        void)
{
    uint8_t CtrlAdd;  //set control ID -add

#ifdef DbgInfo
    CyU3PDebugPrint (4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
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
    		if(CamMode == 1){//only 720p support
				CtrlAdd = CtrlParArry[ExtAexModCtlID9][0];
				ControlHandle(ExtAexModCtlID9);
    		}else/* no support for 1080p camera */
    			CyU3PDebugPrint (4, "The host command is not correct for 1080p camera 0x%x 0x%x %d\r\n", wValue, bRequest, CamMode);
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
    		CtrlAdd = ExUCtrlParArry[Ext1BLCRangeCtlID4-EXUAOFFSET+4][0];
      		ControlHandle(Ext1BLCRangeCtlID4);
    		break;
		case CY_FX_EXT_CONTROL_18BLC_POSITION:   //BLD gain CONTROL18
    		CtrlAdd = ExUCtrlParArry[Ext1BLCWeightCtlID5-EXUAOFFSET+4][0];
      		ControlHandle(Ext1BLCWeightCtlID5);
    		break;
		case CY_FX_EXT_CONTROL_18BLC_GRID:   //BLD gain CONTROL19
    		CtrlAdd = ExUCtrlParArry[Ext1BLCGridCtlID6-EXUAOFFSET+4][0];
      		ControlHandle(Ext1BLCGridCtlID6);
    		break;
   	default:
    		/* No requests supported as of now. Just stall EP0 to fail the request. */
    		CyU3PUsbStall (0, CyTrue, CyFalse);
    		break;
    }

}

/*
 * Handler for the video streaming control requests.
 */
static void
UVCHandleVideoStreamingRqts (
        void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;

    switch (wValue)
    {
        case CY_FX_UVC_PROBE_CTRL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_LEN_REQ:
                    glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                    glEp0Buffer[1] = 0;
                    CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                case CY_FX_USB_UVC_GET_MIN_REQ:
                case CY_FX_USB_UVC_GET_MAX_REQ:
                case CY_FX_USB_UVC_GET_DEF_REQ: 	/* There is only one setting per USB speed. */
                    if (1 || usbSpeed == CY_U3P_SUPER_SPEED)//supports both SS and HS
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
                    }
                    else
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
                    }
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
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
                            CyU3PDebugPrint (4, "Get UVC Prob(set) control %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[3]);
                        }
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }
            break;

        case CY_FX_UVC_COMMIT_CTRL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_LEN_REQ:
                    glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                    glEp0Buffer[1] = 0;
                    CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                    if (1 || usbSpeed == CY_U3P_SUPER_SPEED) //support both SS and HS
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
                    }
                    else
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
                    }
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    /* The host has selected the parameters for the video stream. Check the desired
                       resolution settings, configure the sensor and start the video stream.
                       */
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glCommitCtrl, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)//supports both SS and HS
                    {
                        switch (glCommitCtrl[3])
                         {
                         	case 1: //960 or 480
                         		SensorSetIrisControl(0x0b, 0x30, 0x0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                         		CyU3PThreadSleep(500);
                                CyU3PDebugPrint (4, "Set the video mode format %x %d\n", 0x0, 0x0b);
                         		break;
                         	case 2: //720 or 360
                         		SensorSetIrisControl(0x0b, 0x30, 0x1, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                         		CyU3PThreadSleep(500);
                                CyU3PDebugPrint (4, "Set the video mode format %x %d\n", 0x1, 0x0b);
                         		break;
                         	default:
                         		break;
                         }
                        setRes = glCommitCtrl[3];
                        CyU3PDebugPrint (4, "Set the video mode format setRes %d\n", setRes);

#if 0
                    	if (usbSpeed == CY_U3P_SUPER_SPEED)
                        {
                            SensorScaling_HD720p_30fps ();
                        }
                        else
                        {
                            SensorScaling_VGA ();
                        }
#endif
                        /* We can start streaming video now. */
                        apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
                        if (apiRetStatus != CY_U3P_SUCCESS)
                        {
                            CyU3PDebugPrint (4, "Set CY_FX_UVC_STREAM_EVENT failed %x\n", apiRetStatus);
                        }
                    }
                    break;

                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }
            break;

/* still image streaming handler */
            case VD_FX_UVC_STILL_PROB_CTRL:
                switch (bRequest)
                {
                    case CY_FX_USB_UVC_GET_INFO_REQ:
                        glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
                        CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_LEN_REQ:
                        glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                        glEp0Buffer[1] = 0;
                        CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_CUR_REQ:
                    case CY_FX_USB_UVC_GET_MIN_REQ:
                    case CY_FX_USB_UVC_GET_MAX_REQ:
                    case CY_FX_USB_UVC_GET_DEF_REQ: 	/* There is only one setting per USB speed. */
                        if (usbSpeed == CY_U3P_SUPER_SPEED)
                        {
                            CyU3PUsbSendEP0Data (VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl);
                        }
                        else
                        {
                            CyU3PUsbSendEP0Data (VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl20);
                        }
                        break;
                    case CY_FX_USB_UVC_SET_CUR_REQ:
                        apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
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
                            CyU3PDebugPrint (4, "Get UVC still Prob(set) control %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);
                        }
                        break;
                    default:
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
                break;

            case VD_FX_UVC_STILL_COMIT_CTRL:
                switch (bRequest)
                {
                    case CY_FX_USB_UVC_GET_INFO_REQ:
                        glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
                        CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_LEN_REQ:
                        glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                        glEp0Buffer[1] = 0;
                        CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_CUR_REQ:
                        if (usbSpeed == CY_U3P_SUPER_SPEED)
                        {
                            CyU3PUsbSendEP0Data (VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl);
                        }
                        else
                        {
                            CyU3PUsbSendEP0Data (VD_FX_UVC_MAX_STLPROBE_SETTING, (uint8_t *)glProbeStilCtrl20);
                        }
                        break;
                    case CY_FX_USB_UVC_SET_CUR_REQ:
                        /* The host has selected the parameters for the video stream. Check the desired
                           resolution settings, configure the sensor and start the video stream.
                           */
                        apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                glCommitCtrl, &readCount);
                        if (apiRetStatus == CY_U3P_SUCCESS)
                        {
    #if 0
                        	if (usbSpeed == CY_U3P_SUPER_SPEED)
                            {
                                SensorScaling_HD720p_30fps ();
                            }
                            else
                            {
                                SensorScaling_VGA ();
                            }
                            /* We can start streaming video now. */
                            apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);

                            if (apiRetStatus != CY_U3P_SUCCESS)
                            {
                                CyU3PDebugPrint (4, "Set CY_FX_UVC_STREAM_EVENT failed %x\n", apiRetStatus);
                            }
	#endif
                           switch (glCommitCtrl[1])
                             {
                             	case 1: //720
                             		SensorSetIrisControl(0x0b, 0x30, 0x1, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                             		//CyU3PThreadSleep(500);
                                    CyU3PDebugPrint (4, "Set the still mode format %x %d\n", 0x0b, 0x1);
                             		break;
                            	case 2: //960
                             		SensorSetIrisControl(0x0b, 0x30, 0x0, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
                             		//CyU3PThreadSleep(500);
                                    CyU3PDebugPrint (4, "Set the still mode format %x %d\n", 0x0b, 0x0);
                             		break;
                              	default:
                             		break;
                             }
                            setstilRes = glCommitCtrl[1];

                        	CyU3PDebugPrint (4, "UVC still commit control set %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);

                        }
                        break;

                    default:
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
                break;

            case VD_FX_UVC_STILL_TRIG_CTRL:
                switch (bRequest)
                {
                    case CY_FX_USB_UVC_GET_INFO_REQ:
                        glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
                        CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_LEN_REQ:
                        glEp0Buffer[0] = 1;//CY_FX_UVC_MAX_PROBE_SETTING;
                        glEp0Buffer[1] = 0;
                        CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
                        break;
                    case CY_FX_USB_UVC_GET_CUR_REQ://TODO for still trigger control
                        if (1 || usbSpeed == CY_U3P_SUPER_SPEED)// support both SS and HS
                        {
                            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
                        }
                        else
                        {
                            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
                        }
                        break;
                    case CY_FX_USB_UVC_SET_CUR_REQ:
                        /* The host has selected the parameters for the video stream. Check the desired
                           resolution settings, configure the sensor and start the video stream.
                           */
                        apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                glCommitCtrl, &readCount);
                        if (apiRetStatus == CY_U3P_SUCCESS)
                        {
    #if 1
                            /* We can start still streaming video now. */
                            apiRetStatus = CyU3PEventSet (&glFxUVCEvent, VD_FX_UVC_STIL_EVENT, CYU3P_EVENT_OR);
                            if (apiRetStatus != CY_U3P_SUCCESS)
                            {
                                CyU3PDebugPrint (4, "Set CY_FX_UVC_STIL_EVENT failed %x\n", apiRetStatus);
                            }
    #endif
                            else{
                            stiflag = 0xF0;//set still trigger flag
                            //stillcont = 0;
                            }
                            CyU3PDebugPrint (4, "Get UVC still trigger control %d %d %d\r\n", readCount, glCommitCtrl[0], glCommitCtrl[1]);
                        }else{
                        	CyU3PDebugPrint (4, "UVC still trigger control fail %d %d\r\n", readCount, glCommitCtrl[0]);
                        	CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                        break;

                    default:
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
                break;

        default:
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
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
            if((eventFlag & eventMask) & ~VD_FX_INT_STA_EVENT)
            CyU3PDebugPrint (4, "USB speed = %d evenflag = 0x%x bmReqType = 0x%x\r\n"
            		"bRequest = 0x%x wValue = 0x%x wIndex = 0x%x wLength = 0x%x isflag 0x%x\r\n",
            		usbSpeed, eventFlag, bmReqType, bRequest, wValue, wIndex, wLength, 0/*isFlag*/); /* additional debug message */
            //CyU3PDebugPrint (4, "fb = %d pb = %d pbc = %d pbcp = %d\r\n", fbbak, pbbak, pbcbak, pbcpbak);
            //fbbak=0;pbbak=0;pbcbak=0;pbcpbak=0;
//#endif
            if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)
            {
            	switch ((wIndex >> 8))
                {

                    case CY_FX_UVC_PROCESSING_UNIT_ID:
                        UVCHandleProcessingUnitRqts ();
                        break;

                    case CY_FX_UVC_CAMERA_TERMINAL_ID:
                        UVCHandleCameraTerminalRqts ();
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
void I2cAppThread_Entry(uint32_t input){

	//uint16_t count = 0, cmdCopyIdx = 0; //count1 = 0, cmdQuIdx = 0,
    VdRingBuf *cmdQuptr = &cmdQu;
    VdRingBuf *statQuptr = &statQu;
	VdcmdDes  *lcCmdDes;
	VdstateDes *lcStaDes;
	uint32_t flag = 0;
	uint8_t  cmdFlag = 0;
	uint8_t regAdd, /*regAdd1,*/ devAdd, data;// data1;
	uint8_t i;
	uint16_t delaytime;
	//CyBool_t trigger = CyFalse;

#if 0 //for test the command queue
	lcCmdDes = cmdQuptr->startAdd;
	for(cmdQuIdx = 0; cmdQuIdx < MAXCMD; cmdQuIdx++){
		CyU3PDebugPrint (4, "Command Queue check cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d\r\n",
				lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, cmdQuIdx);
		lcCmdDes += 1;
	}
#endif
/*** create a timer for I2C commands delay option ***/
	CyU3PTimerCreate(&I2CCmdTimer, I2CCmdCb, 11, 1000, 0, CYU3P_NO_ACTIVATE);
	CyU3PDebugPrint (4, "I2C per-timer %d\r\n", CyU3PGetTime());
	CyU3PThreadSleep(50);
	CyU3PTimerStart(&I2CCmdTimer);

	while(cmdQuptr->bugFlag == (uint8_t)CyFalse){ //waiting for first command
        /* Allow other ready threads to run. */

        CyU3PThreadRelinquish ();
	}
	CyU3PDebugPrint (4, "The command queue is ready %d %d\r\n", cmdQuptr->bugFlag, cmdQuptr->readPtr->cmdFlag);
	//CamDefSet(); //set default parameters to camera
	/***** add recovery of the current camera settings ****/
	//CyU3PThreadSleep(100);
	//SetCurCmd();
	/*********** the loop of the thread ***********/
	for(;;){

		CyU3PEventGet (&glFxUVCEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_AND_CLEAR, &flag, CYU3P_WAIT_FOREVER);//wait command event
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
			if(0 && (lcStaDes->statFlag == CyTrue)){ /* for state queue it's not used right now. */
				for(i = 0; i < lcStaDes->NumData; i++){
					regAdd = ((lcStaDes->staPar)+i)->RegAdd;
					devAdd = ((lcStaDes->staPar)+i)->DevAdd;
					((lcStaDes->staPar)+i)->Data = SensorGetControl(regAdd, devAdd); //get state value from I2C bus
#ifdef USB_DEBUG_PRINT
					CyU3PDebugPrint (4, "send I2C state stateID %d cmdCopyIdx %d regAdd 0x%x devAdd 0x%x data 0x%x\r\n",
								lcStaDes->StatID, regAdd, devAdd, data);
#endif
				}
				lcStaDes->statFlag = CyFalse;
				statQuptr->readPtr = (VdcmdDes*)lcStaDes->staDesNext; //update command queue read pointer
				cmdFlag = 0xFF; //I2C command done
			}
			CyU3PMutexPut(statQuptr->ringMux);  //release the command queue mutex
			if(cmdFlag != 0xFF){ //for during handle command
				CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
				lcCmdDes = cmdQuptr->readPtr;

				/*
				CyU3PDebugPrint (4, "get I2C events (1) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
						flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
				*/

				/* find a available command */
				i = 0;
				while((lcCmdDes->cmdFlag == deswait) && (i < MAXCMD)){
					i++;
					lcCmdDes = lcCmdDes->cmdDesNext;
					cmdQuptr->readPtr = lcCmdDes;
				}
				//CyU3PDebugPrint (4, "i %d Cmf_Flag %d\r\n", i, lcCmdDes->cmdFlag);
				if(lcCmdDes->cmdFlag != deswait){
					i = lcCmdDes->curNum;
					regAdd = ((lcCmdDes->CmdPar)+i)->RegAdd;
					devAdd = ((lcCmdDes->CmdPar)+i)->DevAdd;
					data = ((lcCmdDes->CmdPar)+i)->Data;
					delaytime = ((lcCmdDes->CmdPar)+i)->DelayT;
					switch(lcCmdDes->CmdID){
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
					CyU3PDebugPrint (4, "send I2C command cmdID %d regAdd 0x%x devAdd 0x%x data 0x%x cmdflag 0x%x\r\n",
							lcCmdDes->CmdID, regAdd, devAdd, data, lcCmdDes->cmdFlag);
#endif
					if(lcCmdDes->NumPara == lcCmdDes->curNum){
						lcCmdDes->cmdFlag = deswait;
						if(lcCmdDes->CmdID >= EXUAOFFSET){
							ExUCtrlParArry[(lcCmdDes->CmdID-EXUAOFFSET+4)][16] = CyFalse;
						}else{
							CtrlParArry[lcCmdDes->CmdID][16] = CyFalse; //set flag to false. wait for check.
						}
						cmdQuptr->readPtr = lcCmdDes->cmdDesNext; //update command queue read pointer for next handled command
					}else{
						lcCmdDes->curNum ++;
						lcCmdDes->cmdFlag = desusing;
					}
				}else{
					CyU3PTimerModify(&I2CCmdTimer, 1000, 0);
					CyU3PTimerStart(&I2CCmdTimer);
				}
			}
			CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
/*
			CyU3PDebugPrint (4, "get I2C events (2) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
					flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
*/
#ifdef USB_DEBUG_PRINT
			CyU3PDebugPrint (4, "I2C thread checking camera parameters count %d data0 %d data1 %d cmdflag 0x%x.\r\n",
						0/*count*/, CtrlParArry[count][13], CtrlParArry[count][14], cmdFlag);
#endif

			/**** checking the camera registers if it is the same what the current copy is. ****/
			/** this code might be used when a timer is used to schedule the I2C command sent out **/
#if 0
				if((CtrlParArry[cmdCopyIdx][16] != CyTrue)&&(cmdFlag != 0xFF)/*&&(CtrlParArry[cmdCopyIdx][17] != CyFalse)*/){ //checking register value

				regAdd = CtrlParArry[cmdCopyIdx][0];
			    regAdd1 = CtrlParArry[cmdCopyIdx][1];
			    devAdd = CtrlParArry[cmdCopyIdx][15];
			    data = SensorGetControl(regAdd, devAdd); //SensorGetBLCMode();
			    i = 0;
				 switch(cmdCopyIdx)
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
						 data =SensorGetControl(regAdd1, devAdd);
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
			cmdCopyIdx = (cmdCopyIdx + 1 )& 0x1F;    //update checking index.
#endif
			cmdFlag = 0x00; //clear flag
		/* Allow other ready threads to run. */
			//CyU3PDebugPrint (4, "out of the i2cthread flag 0x%x cmdflag 0x%x\r\n", flag, cmdFlag);
			CyU3PThreadRelinquish ();
		}
}


/*
 * This function is called by the FX3 framework once the ThreadX RTOS has started up.
 * The application specific threads and other OS resources are created and initialized here.
 */
void
CyFxApplicationDefine (
        void)
{
    void *ptr1, *ptr2, *ptr3;
    uint32_t retThrdCreate;
    VdRingBuf *cmdQuptr = &cmdQu;
    VdRingBuf *statQuptr = &statQu;

    /* Allocate the memory for the thread stacks. */
    ptr1 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
    ptr2 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
    ptr3 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);

    if ((ptr1 == 0) || (ptr2 == 0) || (ptr3 == 0))
        goto fatalErrorHandler;

	/****** create a ring buffer for command queue *******/
	cmdQu = cmdbufCreate(MAXCMD, &cmdQuMux);
	statQu = cmdbufCreate(MAXSTA, &staQuMux);

	/****** initialize command descriptor ***********/
	cmdquInit(cmdQuptr);
	cmdquInit(statQuptr);

    /* Create the UVC application thread. */
    retThrdCreate = CyU3PThreadCreate (&uvcAppThread,   /* UVC Thread structure */
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
    if (retThrdCreate != 0)
    {
        goto fatalErrorHandler;
    }

    /* Create the control request handling thread. */
    retThrdCreate = CyU3PThreadCreate (&uvcAppEP0Thread,        /* UVC Thread structure */
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
#if 1
    /* Create the I2C control command handling thread. */
    retThrdCreate = CyU3PThreadCreate (&i2cAppThread,   /* UVC Thread structure */
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
#endif

    return;

fatalErrorHandler:
    /* Add custom recovery or debug actions here */
    /* Loop indefinitely */
    while (1);
}

/* Main entry point for the C code. We perform device initialization and start
 * the ThreadX RTOS here.
 */
int
main (
        void)
{
    CyU3PReturnStatus_t apiRetStatus;
    CyU3PIoMatrixConfig_t io_cfg;

       CyU3PSysClockConfig_t clockConfig;
       clockConfig.setSysClk400  = CyTrue;
       clockConfig.cpuClkDiv     = 2;
       clockConfig.dmaClkDiv     = 2;
       clockConfig.mmioClkDiv    = 2;
       clockConfig.useStandbyClk = CyFalse;
       clockConfig.clkSrc         = CY_U3P_SYS_CLK;

    /* Initialize the device */
    apiRetStatus = CyU3PDeviceInit (&clockConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Turn on instruction cache to improve firmware performance. Use Release build to improve it further */
    apiRetStatus = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

    /* Configure the IO matrix for the device. */
    io_cfg.isDQ32Bit        = CyTrue;
    io_cfg.lppMode          = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    io_cfg.useUart          = CyTrue;   /* Uart is enabled for logging. */
    io_cfg.useI2C           = CyTrue;   /* I2C is used for the sensor interface. */
    io_cfg.useI2S           = CyFalse;
    io_cfg.useSpi           = CyFalse;

    apiRetStatus = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
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

