/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522DriverI2C.h"

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522DriverI2C
/////////////////////////////////////////////////////////////////////////////////////

bool MFRC522DriverI2C::init() {
  // TODO avoid double init.
  _wire.begin(); // Returns type void, no check possible.
  return true;
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4. 
 */
void MFRC522DriverI2C::PCD_WriteRegister(const PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte value           ///< The value to write.
                                        ) {
  _wire.beginTransmission(_slaveAdr);
  _wire.write(reg);
  _wire.write(value);
  _wire.endTransmission();
} // End PCD_WriteRegister().

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
void MFRC522DriverI2C::PCD_WriteRegister(const MFRC522Constants::PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte count,        ///< The number of bytes to write to the register.
                                         byte *const values        ///< The values to write. Byte array.
                                        ) {
  _wire.beginTransmission(_slaveAdr);
  _wire.write(reg);
  _wire.write(values, count);
  _wire.endTransmission();
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
byte MFRC522DriverI2C::PCD_ReadRegister(const PCD_Register reg    ///< The register to read from. One of the PCD_Register enums.
                                       ) {
  byte value;
  
  _wire.beginTransmission(_slaveAdr);
  _wire.write(reg);
  _wire.endTransmission();
  
  _wire.requestFrom(_slaveAdr, (uint8_t)1);
  //while(!_wire.available()); // Dangerous! Might block! Wait for byte to be available. TODO timeout detection.
  value = (uint8_t)_wire.read();
  _wire.endTransmission();
  
  return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverI2C::PCD_ReadRegister(const PCD_Register reg,    ///< The register to read from. One of the PCD_Register enums.
                                        const byte count,          ///< The number of bytes to read.
                                        byte *const values,        ///< Byte array to store the values in.
                                        const byte rxAlign         ///< Only bit positions rxAlign..7 in values[0] are updated.
                                       ) {
  // Sanity check.
  if(count == 0 || values == nullptr) {
    return;
  }
  
  byte index = 0;
  
  _wire.beginTransmission(_slaveAdr);
  _wire.write(reg);  // Tell MFRC522 which register address we want to read.
  _wire.endTransmission();
  
  _wire.requestFrom(_slaveAdr, count);
  
  // Todo: is waiting for byte to be available required?
  // With thanks to arozcan (https://github.com/arozcan/MFRC522-I2C-Library), but slightly modified:
  while(_wire.available() && index < count) {
    if(index == 0 && rxAlign) {    // Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      
      for(byte i     = rxAlign; i <= 7; i++) {
        mask |= (1 << i);
      }
      // Read value and tell that we want to read the same address again.
      byte     value = (byte)_wire.read(); // returns int but only with uint8 content
      
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    } else { // Normal case
      values[index] = (byte)_wire.read(); // returns int but only with uint8 content
    }
    index++;
  }
  
  // Fixme: flush wire if still data available?
} // End PCD_ReadRegister()
