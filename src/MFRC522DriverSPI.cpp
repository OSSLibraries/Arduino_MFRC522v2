/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522DriverSPI.h"

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522DriverSPI
/////////////////////////////////////////////////////////////////////////////////////

bool MFRC522DriverSPI::init() {
  // Initialise SPI, Arduino implementation is protected against double initialisation.
  _spiClass.begin(); // Returns type void, no check possible.
  
  // Set the chipSelectPin as digital output, do not select the slave yet.
  if(_chipSelectPin.init() == false) {
    return false;
  }
  
  _chipSelectPin.high();
  return true;
}

/**
 * Writes a byte to the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_WriteRegister(const PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte value            ///< The value to write.
                                        ) {
  _spiClass.beginTransaction(_spiSettings);  // Set the settings to work with SPI bus
  _chipSelectPin.low();    // Select slave
  // When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
  _spiClass.transfer(reg << 1);            // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  _spiClass.transfer(value);
  _chipSelectPin.high();    // Release slave again
  _spiClass.endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_WriteRegister(const PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte count,            ///< The number of bytes to write to the register.
                                         byte *const values        ///< The values to write. Byte array.
                                        ) {
  _spiClass.beginTransaction(_spiSettings);  // Set the settings to work with SPI bus
  _chipSelectPin.low();    // Select slave
  _spiClass.transfer(reg << 1);            // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  for(byte index = 0; index < count; index++) {
    _spiClass.transfer(values[index]);
  }
  _chipSelectPin.high();    // Release slave again
  _spiClass.endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte MFRC522DriverSPI::PCD_ReadRegister(const PCD_Register reg    ///< The register to read from. One of the PCD_Register enums.
                                       ) {
  byte value;
  _spiClass.beginTransaction(_spiSettings);  // Set the settings to work with SPI bus
  _chipSelectPin.low();      // Select slave
  _spiClass.transfer((byte)0x80 | (reg << 1));          // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  value = _spiClass.transfer(0);          // Read the value back. Send 0 to stop reading.
  _chipSelectPin.high();      // Release slave again
  _spiClass.endTransaction(); // Stop using the SPI bus
  return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_ReadRegister(const PCD_Register reg,    ///< The register to read from. One of the PCD_Register enums.
                                        const byte count,            ///< The number of bytes to read.
                                        byte *const values,        ///< Byte array to store the values in.
                                        const byte rxAlign        ///< Only bit positions rxAlign..7 in values[0] are updated.
                                       ) {
  if(count == 0) {
    return;
  }
  //Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
  byte address  = (byte)0x80 | (reg << 1);        // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index    = 0;              // Index in values array.
  _spiClass.beginTransaction(_spiSettings);  // Set the settings to work with SPI bus
  _chipSelectPin.low();    // Select slave
  //count--;								// One read is performed outside of the loop // TODO is this correct?
  _spiClass.transfer(address);          // Tell MFRC522 which address we want to read
  if(rxAlign) {    // Only update bit positions rxAlign..7 in values[0]
    // Create bit mask for bit positions rxAlign..7
    byte mask  = (byte)(0xFF << rxAlign) & 0xFF;
    // Read value and tell that we want to read the same address again.
    byte value = _spiClass.transfer(address);
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask);
    index++;
  }
  //while (index < count) { // changed because count changed to const
  while(index < count-1) {
    values[index] = _spiClass.transfer(address);  // Read value and tell that we want to read the same address again.
    index++;
  }
  values[index] = _spiClass.transfer(0);      // Read the final byte. Send 0 to stop reading.
  _chipSelectPin.high();      // Release slave again
  _spiClass.endTransaction(); // Stop using the SPI bus
} // End PCD_ReadRegister()

