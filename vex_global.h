/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_global.h                                                */
/*    Author:     James Pearman                                               */
/*    Created:    5 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_GLOBAL_H
#define   VEX_GLOBAL_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_global.h
  * @brief   Globals for C++ API
*//*---------------------------------------------------------------------------*/

// more natural language based globals
namespace vex {
    extern const rotationUnits         degrees;
    extern const rotationUnits         turns;
    extern const percentUnits          percent;
    extern const timeUnits             seconds;
    extern const distanceUnits         inches;
    extern const distanceUnits         mm;
    extern const directionType         forward;
    extern const directionType         reverse;
    extern const turnType              left;
    extern const turnType              right;

    extern const colorType             black;
    extern const colorType             white;
    extern const colorType             red;
    extern const colorType             green;
    extern const colorType             blue;
    extern const colorType             yellow;
    extern const colorType             orange;
    extern const colorType             purple;
    extern const colorType             cyan;
    extern const colorType             transparent;

    extern const colorType             red_violet;
    extern const colorType             violet;
    extern const colorType             blue_violet;
    extern const colorType             blue_green;
    extern const colorType             yellow_green;
    extern const colorType             yellow_orange;
    extern const colorType             red_orange;

    extern const fadeType              slow;
    extern const fadeType              fast;
    extern const fadeType              off;

    extern const gyroCalibrationType   calNormal;
    extern const gyroCalibrationType   calSlow;
    extern const gyroCalibrationType   calExtended;

    extern const soundType             siren;
    extern const soundType             wrongWay;
    extern const soundType             wrongWaySlow;
    extern const soundType             fillup;
    extern const soundType             headlightsOn;
    extern const soundType             headlightsOff;
    extern const soundType             tollBooth;
    extern const soundType             alarm;
    extern const soundType             tada;
    extern const soundType             doorClose;
    extern const soundType             ratchet;
    extern const soundType             wrench;
    extern const soundType             siren2;
    extern const soundType             ratchet2;
    extern const soundType             alarm2;
    extern const soundType             powerDown;
     
    extern const cylinderType          cylinder1;
    extern const cylinderType          cylinder2;
    extern const cylinderType          cylinderAll;

    extern void wait( double time, timeUnits units );
    
    extern void  vexSystemExitRequest(void);
};

namespace vex {
    extern const int32_t               PORT1;
    extern const int32_t               PORT2;
    extern const int32_t               PORT3;
    extern const int32_t               PORT4;
    extern const int32_t               PORT5;
    extern const int32_t               PORT6;
    extern const int32_t               PORT7;
    extern const int32_t               PORT8;
    extern const int32_t               PORT9;
    extern const int32_t               PORT10;
    extern const int32_t               PORT11;
    extern const int32_t               PORT12;

    extern const percentUnits          pct;
    extern const timeUnits             sec;
    extern const timeUnits             msec;
    extern const voltageUnits          volt;
    extern const currentUnits          amp;
    extern const powerUnits            watt;
    extern const torqueUnits           Nm;
    extern const torqueUnits           InLb;
    extern const rotationUnits         deg;
    extern const rotationUnits         rev;
    extern const velocityUnits         rpm;
    extern const velocityUnits         dps;
    extern const temperatureUnits      celsius;
    extern const temperatureUnits      fahrenheit;

    extern const directionType         fwd;
    extern const brakeType             coast;
    extern const brakeType             brake;
    extern const brakeType             hold;
};

#endif // VEX_GLOBAL_H