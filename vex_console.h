/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_console.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    1 Nov 2017                                                  */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_CONSOLE_CLASS_H
#define   VEX_CONSOLE_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_console.h
  * @brief   IQ console class header
*//*---------------------------------------------------------------------------*/

//
// allows printing on the second serial port
// subclass of brain
//
namespace vex {
  class console : public brain {
      public:
        console();
        ~console();
      
        /** 
         * @brief print formatted output on the serial console
         * @param format This is a reference to a char format that prints the value of variables.
         * @param ... A variable list of parameters to insert into format string.
        */          
        static int32_t write( const char *fmt, ... ); 
  };
}

#endif // VEX_CONSOLE_CLASS_H