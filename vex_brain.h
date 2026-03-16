/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2017, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_brain.h                                                 */
/*    Author:     James Pearman                                               */
/*    Created:    8 July 2017                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_BRAIN_CLASS_H
#define   VEX_BRAIN_CLASS_H

#include "vex_timer.h"

/*-----------------------------------------------------------------------------*/
/** @file    vex_brain.h
  * @brief   IQ brain class header
*//*---------------------------------------------------------------------------*/

namespace vex {
  class brain {
    private:
      static  int32_t _index;
      
    public:
      /** 
        * @brief Creates a new brain object.
      */
      brain();
      ~brain();

    private:
      enum class tEventType {
        EVENT_UP_PRESSED     = 1,
        EVENT_UP_RELEASED    = 2,
        EVENT_DOWN_PRESSED   = 4,
        EVENT_DOWN_RELEASED  = 8,
        EVENT_CHECK_PRESSED  = 16,
        EVENT_CHECK_RELEASED = 32
      };

    public:
      /** 
       * @brief Use the button class to get values from the brain's buttons.
      */
      class button {
        private:
          uint8_t  _id;
          
        public:
          button() : _id( -1 ) {};
          button( const int id ) : _id( id ) {};
          ~button() {};
          
          /**
           * @brief Sets the function to be called when the button is pressed.
           * @param callback A reference to a function.
          */
          void    pressed( void (* callback)(void) );
          /**
           * @brief Sets the function to be called when the button is released.
           * @param callback A reference to a function.
          */
          void    released( void (* callback)(void) );
          /**
           * @brief Sets the function to be called when the button is released.
           * @param callback A reference to a function.
          */
         bool     pressing( void );        
      };
      
      /**
       * @brief A button that represents the Up/Left button on the brain
       */
      button       buttonUp    = button(0);
      button      &buttonLeft  = buttonUp;
      /**
       * @brief A button that represents the Down/Right button on the brain
       */
      button       buttonDown  = button(1);
      button      &buttonRight = buttonDown;
      /**
       * @brief A button that represents the Check button on the brain
       */
      button       buttonCheck = button(2);
      
      /** 
        * @brief Use this class to write or draw to the brain's LCD screen.
      */      
      class lcd {
        private:
          int16_t   _row;
          int16_t   _maxrows;
          int16_t   _rowheight;
          
          int16_t   _col;
          int16_t   _maxcols;

          int32_t   _penWidth;

          char      _textStr[32];
          
          bool      _fg_color;
          bool      _bg_color;
          bool      _transparent;
          bool      _aspect_compensation;
          
          int       _origin_x;
          int       _origin_y;

          int32_t   rowToPixel( int32_t row );
          int32_t   colToPixel( int32_t col );
          int32_t   scaley( int32_t y );
          
        public:
          lcd();
          ~lcd() {};
                  
          /** 
           * @brief Sets the cursor to the row and column number set in the parameters.
           * @param row Sets the row number for where the cursor is placed.
           * @param col Sets the column number for where the cursor is placed. 
          */
          void     setCursor( int32_t row, int32_t col ); 

          /** 
           * @brief Sets the pen's width.
           * @param width This represents the width of the pen when drawing. A larger width equals a wider pen stroke.
          */
          void     setPenWidth( uint32_t width ); 

          /** 
           * @brief Sets the origin of the screen to the parameters defined in the function.
           * @param x The x location of the origin.
           * @param y The y location of the origin.
          */
          void     setOrigin( int32_t x, int32_t y );

          /**
           * @brief compensate for non square pixels when drawing
           * @param value true or false
          */
          void     setAspectCompensation( bool value );

          /** 
           * @brief An integer that tracks the current cursor position's column, starting at 1.
           * @return Returns an integer that tracks the current cursor position's column, starting at 1.
          */
          int32_t  column();

          /** 
           * @brief An integer that tracks the current cursor position's row, starting at 1.
           * @return Returns an integer that tracks the current cursor position's row, starting at 1.
          */
          int32_t  row();

          /** 
           * @cpp_sig{setPenColor(vex::color::red);}
           * @brief Sets the color of the pen to a specified color.
           * @param color A color unit where colors can be defined as names.
          */          
          void     setPenColor( colorType color );

          /** 
           * @cpp_sig{setFillColor(vex::color::red);}
           * @brief Sets the default fill color for any subsequent draw command.
           * @param color In a color unit, colors can be defined as names.
          */          
          void     setFillColor( colorType color );

          /** 
           * @brief Prints a number, string, or Boolean.
           * @param format This is a reference to a char format that prints the value of variables.
           * @param ... A variable list of parameters to insert into format string.
          */          
          void     print( const char *format, ... );
          void     print( char *format, ... );

          /** 
           * @brief Prints a number, string, or Boolean at an x, y cursor location.
           * @param x The x-coordinate at which to print a message on the screen.
           * @param y The y-coordinate at which to print a message on the screen.
           * @param format A reference to a char format to print the value of variables.
           * @param ... A variable list of parameters to insert into format string.
           * @notes this uses cursor location rather than pixel location as on the V5
           * it's shouthand for using setCursor followed by print.
          */
          void     printAt( int32_t x, int32_t y, const char *format, ... );

          /** 
           * @brief Clears the whole Screen to white.
          */          
          void     clearScreen( void );

          /** 
           * @brief Clears the specified line.
           * @param number The integer sets the line that is to be cleared.
          */
          void     clearLine( int number );

          /** 
           * @brief Clears the rest of the line from where the cursor is located. 
          */
          void     clearLine( void );

          /** 
           * @brief Clears the rest of the line from where the cursor is located and then moves the cursor to the beginning of the next line.
           * (actually not, just moves cursor at the moment as that was what the V5 does, we will address later)
          */
          void     newLine( void );

          /** 
           * @brief Draws a single pixel to the screen in the specified x and y location.
           * @param x The x location of where to draw the pixel.
           * @param y The y location of where to draw the pixel.
          */
          void     drawPixel( int x, int y );

          /** 
           * @brief Draws a line connecting the two specified points in the parameters.
           * @param x1 The x location of the first point.
           * @param y1 The y location of the first point.
           * @param x2 The x location of the second point.
           * @param y2 The y location of the second point.
          */
          void     drawLine( int x1, int y1, int x2, int y2 );

          /** 
           * @brief Draws a rectangle using the specified points and attributes set in the parameters. Fills the rectangle with the current fill color.
           * @param x The x-coordinate at which the left edge of the rectangle will be drawn on the screen.
           * @param y The y-coordinate at which the top edge of the rectangle will be drawn on the screen.
           * @param width The width of the rectangle.
           * @param height The height of the rectangle.
          */
          void     drawRectangle( int x, int y, int width, int height );
          
          /** 
           * @brief Draws a rectangle using the specified points and attributes set in the parameters. Fills the rectangle with the color specified.
           * @param x The x-coordinate at which the left edge of the rectangle will be drawn on the screen.
           * @param y The y-coordinate at which the top edge of the rectangle will be drawn on the screen.
           * @param width The width of the rectangle.
           * @param height The height of the rectangle.
           * @param color A color unit where colors can be defined as names.
          */
          void     drawRectangle( int x, int y, int width, int height, colorType color );


          /** 
           * @brief Inverts a rectangular area of the screen using the specified points and attributes set in the parameters.
           * @param x The x-coordinate at which the left edge of the rectangle will be drawn on the screen.
           * @param y The y-coordinate at which the top edge of the rectangle will be drawn on the screen.
           * @param width The width of the rectangle.
           * @param height The height of the rectangle.
          */
          void     invertRectangle( int x, int y, int width, int height );

          /** 
           * @brief Draws a circle using the specified points and attributes set in the parameters. Fills the circle with the current pen and fill color.
           * @param x The central x location of the circle.
           * @param y The central y location of the circle.
           * @param radius Sets the radius of the circle to be drawn on the Screen.
          */
          void     drawCircle( int x, int y, int radius );

          /** 
           * @brief Draws a circle using the specified points and attributes set in the parameters. Fills the circle with the color specified.
           * @param x The central x location of the circle.
           * @param y The central y location of the circle.
           * @param radius Sets the radius of the circle to be drawn on the Screen.
           * @param color A color unit where colors can be defined as names.
          */
          void     drawCircle( int x, int y, int radius, colorType color );

          /** 
           * @brief Inverts a circular area of the screen using the specified points and attributes set in the parameters.
           * @param x The central x location of the circle.
           * @param y The central y location of the circle.
           * @param radius Sets the radius of the circle to be drawn on the Screen.
          */
          void     invertCircle( int x, int y, int radius );
      };
      
      /** 
       * @brief Use the Screen class to write or draw to the screen.
       */
      lcd         Screen;

      /** 
       * @brief Gets the value of the timer in the units specified.
       * @param units The unit of time that will be returned.
       * @return Returns a double that represents the value of the timer.
      */
      double      timer( timeUnits units );

      /** 
       * @brief Resets the timer to zero.
      */
      void        resetTimer();

      /** 
       * @brief Sets the timer to a value and time unit.
       * @param value The value that the timer is set to.
       * @param units The unit of time that will be returned.
      */
      void        setTimer( double value, timeUnits units );

      /** 
       * @brief Use the Timer class to obtain the system time.
       */
      vex::timer  Timer;

      /** 
       * @brief Use the battery class to see information about the battery.
      */      
      class battery {
        public:
          battery() {};
          ~battery() {};

          /** 
           * @brief Gets the battery capacity.
           * @return Returns an integer that represents the batteries capacity in the units specified by the parameter.
           * @param units (Optional) The type of unit that will be returned. By default, this parameter is a percentage. 
          */
          uint16_t  capacity( percentUnits units = percentUnits::pct );

          /** 
           * @brief Gets the battery voltage.
           * @return Returns a double that represents the batteries voltage in the units specified by the parameter.
           * @param units The unit of voltage to be returned.
          */
          double    voltage( voltageUnits units = voltageUnits::volt );
      };
      
      battery     Battery;
      
      // we need to avoid using printf or std::cout
      // as it uses too much memory
      /** 
       * @brief Use the terminal class to send text to the brain USB port
      */      
      class terminal {
        public:
          terminal() {};
          ~terminal() {};
          
          /** 
           * @brief print on the vexcode terminal
           */
          void print( const char *fmt, ... );
      };
      
      terminal    Terminal;
                
      /** 
       * @brief play one of the predefined sounds on the brain
       */
      void        playSound( soundType sound );
      /** 
       * @brief play a note in the given octave on the brain
       */
      void        playNote( int32_t octave, int32_t note );
      /** 
       * @brief play a note in the given octave on the brain and wait for the given time before returning
       */
      void        playNote( int32_t octave, int32_t note, int32_t ms );
      /** 
       * @brief turn sound off on the brain
       */
      void        soundOff( void );
      /** 
       * @brief request that vexos stop the running program.  The program will stop after approximately 500mS.
       */
      void        programStop();
  };
};

#endif // VEX_BRAIN_CLASS_H
