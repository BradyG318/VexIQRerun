/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_units.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    5 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_UNITS_H
#define   VEX_UNITS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_units.h
  * @brief   Header for global units
*//*---------------------------------------------------------------------------*/

namespace vex {
    enum class percentUnits {
      pct = 0
    };
    
    enum class timeUnits {
      sec,
      msec
    };
    
    enum class currentUnits {
      amp
    };

    enum class voltageUnits {
      volt,
      mV
    };
    
    enum class powerUnits {
      watt
    };

    enum class torqueUnits {
      Nm,
      InLb
    };

    enum class rotationUnits {
      deg,
      rev,
      raw = 99
    };

    enum class velocityUnits {
      pct = (int)percentUnits::pct,
      rpm,
      dps
    };

    enum class distanceUnits {
      mm,
      in,
      cm,
      counts
    };

    enum class temperatureUnits {
      celsius,
      fahrenheit
    };

    enum class directionType {
      fwd = 0,
      rev,
      undefined
    };

    enum class turnType {
      left,
      right
      };
    
    enum class brakeType {
      coast = 0,
      brake = 1,
      hold  = 2,
      undefined
    };

    enum class sizeType {
      none,
      small,
      medium,
      large
    };
    
    enum class colorType {
      none,
      red,
      green,
      blue,
      white,
      yellow,
      orange,
      purple, 
      cyan,
      
      red_violet,
      violet,
      blue_violet,
      blue_green,
      yellow_green,
      yellow_orange,
      red_orange,
      
      black,
      transparent
    };

    enum class fadeType {
      off,
      slow,
      fast
    };

    enum class gyroCalibrationType {
      calNormal,
      calSlow,
      calExtended
    };
    
    enum class rateUnits {
      dps,
      rps
    };

    enum class ledState {
      off,
      on,
      blink
    };

    enum class soundType {
      siren            =  0,
      wrongWay         =  1,
      wrongWaySlow     =  2,
      fillup           =  3,
      headlightsOn     =  4,
      headlightsOff    =  5,
      tollBooth        =  6,
      alarm            =  7,
      tada             =  8,
      doorClose        =  9,
      ratchet          = 10,
      wrench           = 11,
      siren2           = 12,
      ratchet2         = 13,
      alarm2           = 14,
      powerDown        = 15
      };    

    /**
     @brief The defined units for optical sensor gesture types.
    */
    enum class gestureType {
      none,
      up,
      down,
      left,
      right
    };
    
    /**
     @brief The defined types for pneumatic cylinders.
    */
    enum class cylinderType {
      cylinder1 = 0,
      cylinder2 = 1,
      cylinderAll = 0xff
    };
};

#endif // VEX_UNITS_H