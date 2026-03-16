/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_event.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    7 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_EVENT_CLASS_H
#define   VEX_EVENT_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_event.h
  * @brief   event handler class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  class event {
    private:
      static  int16_t _usereventid;
      void         (* _callback)(void);  
      int           _userid;
      
    public:
      event();
      event( int16_t index, uint32_t mask, void (* callback)(void) );
      event( void (* callback)(void) );
      event( event v, void (* callback)(void) );
      event( void (* callback)(void *), void *arg );
      event( event v, void (* callback)(void *), void *arg );
      ~event();
      
      static void     init( int16_t index, uint32_t mask, void (* callback)(void) );
      // Do not use for now - here for testing
      static void     init( int16_t index, uint32_t mask, void (* callback)(int) );
      static void     init( int16_t index, uint32_t mask, void (* callback)(void *), void *arg );
      static int32_t  userindex(void);

      void            set( void (* callback)(void) );
      void            operator()( void (* callback)(void) );

      void            broadcast();


      void            broadcastAndWait( int32_t timeout = 60000 );
      static void     broadcast( int16_t index );
      static void     broadcastAndWait( int16_t index, int32_t timeout = 60000 );
  };
};

#endif // VEX_EVENT_CLASS_H