/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     iq_cpp.h                                                    */
/*    Author:     James Pearman                                               */
/*    Created:    5 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef IQ_CPP_H_
#define IQ_CPP_H_

#include "iq_apitypes.h"

#include "vex_units.h"
#include "vex_event.h"
#include "vex_brain.h"
#include "vex_controller.h"
#include "vex_task.h"
#include "vex_thread.h"
#include "vex_timer.h"

#include "vex_device.h"
#include "vex_motor.h"
#include "vex_vision.h"
#include "vex_touchled.h"
#include "vex_bumper.h"
#include "vex_colorsensor.h"
#include "vex_sonar.h"
#include "vex_gyro.h"
#include "vex_distance.h"
#include "vex_optical.h"
#include "vex_console.h"
#include "vex_generic.h"
#include "vex_pneumatic.h"
#include "vex_motorgroup.h"
#include "vex_drivetrain.h"
#include "vex_smartdrive.h"

#include "vex_global.h"

/*----------------------------------------------------------------------------*/
//    Do not remove, this sets the IQ program name                            */
#ifdef IQNAME
#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)
extern "C" __attribute__((weak))  const char userProgramName[] = {EXPAND_AND_QUOTE(IQNAME)};
#endif
/*----------------------------------------------------------------------------*/

#endif /* IQ_CPP_H_*/