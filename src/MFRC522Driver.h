/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <Arduino.h>
#include <MFRC522Constants.h>

#define __STDC_LIMIT_MACROS

#include <stdint.h>

class MFRC522Driver {
public:
  //typedef MFRC522Constants::PCD_Register PCD_Register;
  using PCD_Register = MFRC522Constants::PCD_Register;
  
  // Default value for unused pin
  static constexpr uint8_t UNUSED_PIN = UINT8_MAX;
  
  virtual bool init() = 0;
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Basic interface functions for communicating with the MFRC522
  /////////////////////////////////////////////////////////////////////////////////////
  virtual void PCD_WriteRegister(const PCD_Register reg, const byte value) = 0;
  virtual void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) = 0;
  virtual byte PCD_ReadRegister(const PCD_Register reg) = 0;
  virtual void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) = 0;
  
  /////////////////////////////////////////////////////////////////////////////////////
  MFRC522Driver() = default;
};
