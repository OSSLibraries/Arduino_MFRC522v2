/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <SPI.h>
#include <MFRC522Driver.h>
#include <MFRC522DriverPin.h>

class MFRC522DriverSPI : public MFRC522Driver {
public:
  //using PCD_Register = MFRC522Constants::PCD_Register;
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for setting up the Arduino.
  /////////////////////////////////////////////////////////////////////////////////////
  
  bool init() override;
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Basic interface functions for communicating with the MFRC522.
  /////////////////////////////////////////////////////////////////////////////////////
  void PCD_WriteRegister(const PCD_Register reg, const byte value) override;
  void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) override;
  byte PCD_ReadRegister(const PCD_Register reg) override;
  void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) override;
  
  
  // Advanced init with custom chip select method, e.g. with i2c port expander or multiplexer.
  MFRC522DriverSPI(MFRC522DriverPin &chipSelectPin,
                   SPIClass &spiClass = SPI,
                   const SPISettings spiSettings = SPISettings(4000000u /* 4MHz */, MSBFIRST, SPI_MODE0)
                  ) : MFRC522Driver(),
                      _chipSelectPin(chipSelectPin),
                      _spiClass(spiClass),
                      _spiSettings(spiSettings) {};

protected:
  // Pins.
  //byte _chipSelectPin;        // Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
  MFRC522DriverPin &_chipSelectPin; // Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
  
  // SPI communication.
  SPIClass &_spiClass;        // SPI class which abstracts hardware.
  const SPISettings _spiSettings;    // SPI settings.
};
