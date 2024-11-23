/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <Stream.h>
#include <MFRC522Driver.h>

class MFRC522DriverUART : public MFRC522Driver {
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
  
  
  MFRC522DriverUART(Stream& serial) : 
    MFRC522Driver(),
    _serial(serial)
  {}

protected:
  Stream& _serial;
};
