/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2016, All rights reserved.               */
/*                                                                            */
/*    Module:     iq_apitypes.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    20 March 2019                                               */
/*                                                                            */
/*    Revisions:  V0.1                                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef IQ_APITYPES_H_
#define IQ_APITYPES_H_

#include <stdint.h>
#include <stdbool.h>

/*-----------------------------------------------------------------------------*/
/** @file    iq_apitypes.h
  * @brief   Header for IQ API - type definitions
*//*---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------*/
/** @brief      IQ Device type definitions                                    */
/*----------------------------------------------------------------------------*/
// hard coded for SDK as we don't want to include VEX_Api.h
// same names as V5
//
#ifndef IQ_API_H_
typedef enum {
  kDeviceTypeNoSensor        = 0,
  kDeviceTypeMotorSensor     = 2,
  kDeviceTypeLedSensor       = 3,
  kDeviceTypeRgbSensor       = 4,
  kDeviceTypeBumperSensor    = 5,
  kDeviceTypeGyroSensor      = 6,
  kDeviceTypeSonarSensor     = 7,
  kDeviceTypeRadioSensor     = 8,
  kDeviceTypeTetherSensor    = 9,
  kDeviceTypeVisionSensor    = 11,

  kDeviceTypeGenericSensor   = 0x80,
  kDeviceTypeUserSensor      = 0xFF
} IQ_DeviceType;
#endif

// Some ports will be virtual, some physical, same number as V5 even though we have less
#define IQ_MAX_DEVICE_PORTS   32
typedef IQ_DeviceType IQ_DeviceTypeBuffer[IQ_MAX_DEVICE_PORTS];

/*----------------------------------------------------------------------------*/
/** @brief      IQ Vision sensor definitions                                  */
/*----------------------------------------------------------------------------*/

// subject to change
typedef enum {
  kVisionModeNormal      = 0,
  kVisionModeMixed       = 1,
  kVisionModeLineDetect  = 2,
  kVisionTypeTest        = 3
} IQVisionMode;

typedef enum {
  kVisionTypeNormal      = 0,
  kVisionTypeColorCode   = 1,
  kVisionTypeLineDetect  = 2
} IQVisionBlockType;

// White balance
typedef enum {
  kVisionWBNormal       = 0,
  kVisionWBStart        = 1,
  kVisionWBManual       = 2
} IQVisionWBMode;

// LED control, from V5 or by the vision sensor
typedef enum {
  kVisionLedModeAuto    = 0,
  kVisionLedModeManual  = 1
} IQVisionLedMode;

// Wifi mode
typedef enum {
  kVisionWifiModeOff    = 0,
  kVisionWifiModeOn     = 1
} IQVisionWifiMode;

// signature should be 40 bytes and is the same data
// as sent to the vision sensor

// if bit0 is now set you know signature is read back
#define VISION_SIG_FLAG_STATUS    (1<<0) 


typedef struct __attribute(( packed )) _IQ_DeviceVisionObject {
    uint16_t            signature;        /// block signature
    IQVisionBlockType   type;             /// block type
    uint16_t            xoffset;          /// left side of block
    uint16_t            yoffset;          /// top of block
    uint16_t            width;            /// width of block
    uint16_t            height;           /// height of block
    uint16_t            angle;            /// angle of CC block in 0.1 deg units
} IQ_DeviceVisionObject;

// Color structure, used for LED and white balance
typedef struct __attribute(( packed )) _IQ_DeviceVisionRgb {
    uint8_t             red;
    uint8_t             green;
    uint8_t             blue;
    uint8_t             brightness;    /// not used yet
} IQ_DeviceVisionRgb;

//
// local copies of the external vision structures on IQ
//
typedef struct  __attribute__ ((__packed__))
{
	uint16_t 	ID;
	uint8_t 	XPos;
	uint8_t 	YPos;
	uint8_t		Width;
	uint8_t		Height;
	uint16_t	Angle;
} IQ_VisionDetectionObj;

typedef struct __attribute__ ((__packed__)) 
{
	uint8_t		id;
	float		  range;
	int32_t		uMin;
	int32_t		uMax;
	int32_t		uMean;
	int32_t		vMin;
	int32_t		vMax;
	int32_t		vMean;
	uint32_t	mRgb;
	uint32_t	mType;
} IQ_VisionDetectionSettings;


#ifdef __cplusplus
}
#endif
#endif /* IQ_APITYPES_H_*/