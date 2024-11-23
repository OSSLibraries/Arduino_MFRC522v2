/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522Debug.h"
#include "MFRC522DriverUART.h"

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522DriverUART
/////////////////////////////////////////////////////////////////////////////////////

bool MFRC522DriverUART::init() {
  // Surrounding code should initialize the serial port.
  _serial.setTimeout(10);
  return true;
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4. 
 */
void MFRC522DriverUART::PCD_WriteRegister(const PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte value           ///< The value to write.
                                        ) {
  // PCD_ReadRegister(reg);
  Serial.print(MFRC522Debug::toString(reg));
  Serial.printf(" < 0x%02x \n", value);
  _serial.write(reg);
  // read back the echoes address:
  byte readBack = 0xff;
  _serial.readBytes(&readBack, 1);
  if (readBack != reg)
    Serial.printf("Different address was read back 0x%02x != 0x%02x \n", reg, readBack);
  _serial.write(value);
} // End PCD_WriteRegister().

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
void MFRC522DriverUART::PCD_WriteRegister(const MFRC522Constants::PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte count,        ///< The number of bytes to write to the register.
                                         byte *const values        ///< The values to write. Byte array.
                                        ) {
  for (byte i = 0; i < count; i++)
  {
    PCD_WriteRegister(reg, values[i]);
  }
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
byte MFRC522DriverUART::PCD_ReadRegister(const PCD_Register reg    ///< The register to read from. One of the PCD_Register enums.
                                       ) {
  byte value = 0;
  byte regRead = reg | 0xC0;
  _serial.write(regRead);
  _serial.readBytes(&value, 1);
  Serial.print(MFRC522Debug::toString(reg));
  Serial.printf(" : 0x%02x \n", value);
  return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverUART::PCD_ReadRegister(const PCD_Register reg,    ///< The register to read from. One of the PCD_Register enums.
                                        const byte count,          ///< The number of bytes to read.
                                        byte *const values,        ///< Byte array to store the values in.
                                        const byte rxAlign         ///< Only bit positions rxAlign..7 in values[0] are updated.
                                       ) {
  // Sanity check.
  if(count == 0 || values == nullptr) {
    return;
  }
  
  byte index = 0;
  
  while(index < count) {
    if(index == 0 && rxAlign) {    // Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      
      for(byte i     = rxAlign; i <= 7; i++) {
        mask |= (1 << i);
      }
      byte value = PCD_ReadRegister(reg);
      
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    } else { // Normal case
      values[index] = PCD_ReadRegister(reg);
    }
    index++;
  }
} // End PCD_ReadRegister()
