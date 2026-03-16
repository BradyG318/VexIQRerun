/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2019, All rights reserved.               */
/*                                                                            */
/*    Module:     vex_generic.h                                               */
/*    Author:     James Pearman                                               */
/*    Created:    6 June 2019                                                 */
/*                                                                            */
/*    Revisions:                                                              */
/*                V1.00     TBD - Initial release                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef   VEX_GENERIC_CLASS_H
#define   VEX_GENERIC_CLASS_H

/*-----------------------------------------------------------------------------*/
/** @file    vex_generic.h
  * @brief   IQ generic sensor device class header
*//*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief generic sensor device class                                         */
/*-----------------------------------------------------------------------------*/
namespace vex {
  class generic : public device {      
      public:
        /**
         * @brief Creates a new generic sensor object on the port specified in the parameter.
         * @param port index to the brain port.
       */
        generic( int32_t index );  
        ~generic();
    
        bool  installed();
        
        /*--------------------------------------------------------------------*/
        //
        // The following methods are for communication with generic devices
        // that conform to the IQ sensor reference spec and have been enumerated
        // by vexos as either generic or user sensors.
        //
        /*--------------------------------------------------------------------*/
        
        /**
         * @brief Gets the firmware version of the generic sensor.
         * @return Returns an integer with version or -1 on error
         */
        int32_t version();
        /**
         * @brief Gets the vendor id of the generic sensor.
         * @return Returns an integer with vendor id or -1 on error
         */
        int32_t vendorId();
        /**
         * @brief Gets the product id of the generic sensor.
         * @return Returns an integer with product id or -1 on error
         */
        int32_t productId();
        
        /**
         * @brief Read 1 byte from the generic sensor.
         * @param  reg the register to read from 
         * @return register value on success, -1 if error
         */
        int32_t readByte( uint8_t reg );
        /**
         * @brief Write 1 byte to the generic sensor.
         * @param  reg the register to write to 
         * @param  value the data to write
         * @return register value on success, -1 if error
         */
        int32_t writeByte( uint8_t reg, uint8_t value );
        
        /**
         * @brief Read 1 word (2 bytes) from the generic sensor.
         * @param  reg the register to read from 
         * @param  bLittleEndian the endianess of the word in the sensor
         * @return register value on success, -1 if error
         */
        int32_t readWord( uint8_t reg, bool bLittleEndian = false );
        /**
         * @brief  Write 1 word (2 bytes) to the generic sensor.
         * @param  reg the register to write to 
         * @param  value the data to write
         * @param  bLittleEndian the endianess of the word in the sensor
         * @return register value on success, -1 if error
         */
        int32_t writeWord( uint8_t reg, uint16_t value, bool bLittleEndian = false );

        /**
         * @brief  Read bytes from the generic sensor.
         * @param  reg the starting register to read from 
         * @param  pBytes pointer to buffer to read into 
         * @param  nLength number of bytes to read
         * @return true on success, false if error
         */
        bool    readReg( uint8_t reg, uint8_t *pBytes, uint8_t nLength );
        /**
         * @brief  Write bytes to the generic sensor.
         * @param  reg the starting register to write to 
         * @param  pBytes pointer to buffer with data
         * @param  nLength number of bytes to write
         * @return true on success, false if error
         */
        bool    writeReg( uint8_t reg, uint8_t *pBytes, uint8_t nLength );

        typedef enum _i2cRegType {
          // All sensors have these registers
          kRegAsciiVersion    = 0x00, // 8 bytes
          kRegAsciiVendor     = 0x08, // 8 bytes
          kRegAsciiDeviceId   = 0x10, // 8 bytes

          kRegUserVendorId    = 0x18, // user sensor specific
          kRegUserProductId   = 0x19, // user sensor specific
          
          kRegFirmwareVersion = 0x20,
          kRegDeviceType      = 0x21,
          kRegDeviceId        = 0x22,
          kRegDeviceStatus    = 0x23
        } i2cRegType;

        /*--------------------------------------------------------------------*/
        //
        // The following methods are for communication with generic devices
        // that do not conform to the IQ sensor reference spec and have an I2C
        // address that does not conflict with other sensors on the same bus.
        // If the addressed device is not present, or gives an error during
        // use, the IQ may abort with an "I2C error" message in the same way
        // as when an IQ sensor is disconnected.
        // raw sensors should not use I2C addresses that may be assigned to IQ
        // devices if sharing the bus with them.
        // (8 bit addresses)
        // 0x60
        // 0x20, 0x22, 0x24
        /*--------------------------------------------------------------------*/
        
        /**
         * @brief  Read bytes from the device.
         * @param  addr the 8 bit i2C address of the device
         * @param  pBytes pointer to buffer to read into 
         * @param  nLength number of bytes to read
         * @return true on success, false if error
         */
        bool    readRaw( uint8_t addr, uint8_t *pBytes, uint8_t nLength );
        /**
         * @brief  Write bytes to the device.
         * @param  addr the 8 bit i2C address of the device
         * @param  pBytes pointer to buffer with data
         * @param  nLength number of bytes to write
         * @return true on success, false if error
         */
        bool    writeRaw( uint8_t addr, uint8_t *pBytes, uint8_t nLength );
        
        /**
         * @brief  Wait for raw write to complete
         * usually this is not needed, writeRaw will block when called waiting
         * for previous transactions to complete.
         * but during testing, if errors are occurring, then this method
         * will clears errors and suppress the "I2C error" message
         */
        void    writeRawWait();
      private:
  };
};

#endif // VEX_GENERIC_CLASS_H