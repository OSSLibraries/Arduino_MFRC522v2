/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <Wire.h>
#include <MFRC522Driver.h>

class MFRC522DriverI2C : public MFRC522Driver {
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
  
  
  MFRC522DriverI2C(
  const byte slaveAdr = 0x28,
  TwoWire &wire = Wire
                  ) : MFRC522Driver(),
                      _slaveAdr(slaveAdr),
                      _wire(wire) {};

protected:
  // Address of mfrc522.
  const byte _slaveAdr;
  
  // Wire instance.
  TwoWire &_wire;
};
