/* SPDX-License-Identifier: LGPL-2.1 */
/**
 * Library to use Arduino MFRC522 module.
 * 
 * @authors Dr.Leong, Miguel Balboa, Søren Thing Andersen, Tom Clement, many more! See GitLog.
 * 
 * Please read this file for an overview and then MFRC522v2.cpp for comments on the specific functions.
 */
#ifndef MFRC522_h
#define MFRC522_h

#include <require_cpp11.h>
// Enable integer limits
#define __STDC_LIMIT_MACROS

#include <stdint.h>
#include <Arduino.h>
#include <MFRC522Constants.h>
#include <MFRC522Driver.h>


// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const byte MFRC522_firmware_referenceV0_0[]
           PROGMEM = {
           0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
           0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
           0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
           0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
           0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
           0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
           0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
           0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
           };
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const byte MFRC522_firmware_referenceV1_0[]
           PROGMEM = {
           0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
           0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
           0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
           0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
           0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
           0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
           0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
           0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
           };
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const byte MFRC522_firmware_referenceV2_0[]
           PROGMEM = {
           0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
           0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
           0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
           0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
           0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
           0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
           0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
           0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
           };
// Clone
// Fudan Semiconductor FM17522 (0x88)
const byte FM17522_firmware_reference88[]
           PROGMEM = {
           0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
           0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
           0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
           0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
           0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
           0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
           0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
           0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
           };
// Another "FM17522" ic form Aliexpress
const byte FM17522_firmware_referenceB2[]
           PROGMEM = {
           0x00, 0xeb, 0x44, 0x85, 0xfa, 0x9a, 0x78, 0x01,
           0x74, 0xe5, 0x1c, 0x7a, 0x0a, 0xa0, 0x71, 0xe1,
           0xf3, 0xfa, 0x96, 0x6d, 0x28, 0xa1, 0x34, 0x46,
           0x3a, 0x1c, 0x32, 0x96, 0xb9, 0xe6, 0x44, 0x87,
           0x0a, 0x45, 0x98, 0xa9, 0x36, 0x60, 0x89, 0x0f,
           0x06, 0x9b, 0x7b, 0x17, 0xb3, 0x0c, 0x1a, 0x6c,
           0x1a, 0xae, 0x2c, 0xac, 0x0e, 0x6f, 0x2e, 0x02,
           0x2b, 0xcb, 0x8a, 0xb2, 0x45, 0xdd, 0x7e, 0x3c
           };
// Fudan Semiconductor FM17522E (0x89)
const byte FM17522E_firmware_reference[]
           PROGMEM = {
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x80, 0x00, 0x04, 0xcc, 0xc8, 0x00,
           0x10, 0x04, 0x00, 0xc0, 0x00, 0x90, 0x00, 0x20,
           0x00, 0x00, 0x23, 0x00, 0x38, 0x06, 0x01, 0x33,
           0x98, 0xf3, 0x80, 0x06, 0xc0, 0xf9, 0x80, 0x08,
           0x27, 0x04, 0x23, 0x82, 0x21, 0x12, 0xf9, 0xc7
           };

class MFRC522 {
public:
  //using PCD_Register = MFRC522Constants::PCD_Register;
  using PCD_Register = MFRC522Constants::PCD_Register;
  using PCD_RxGain = MFRC522Constants::PCD_RxGain;
  using PICC_Command = MFRC522Constants::PICC_Command;
  using MIFARE_Misc = MFRC522Constants::MIFARE_Misc;
  using PICC_Type = MFRC522Constants::PICC_Type;
  using StatusCode = MFRC522Constants::StatusCode;
  using PCD_Command = MFRC522Constants::PCD_Command;
  using PCD_Version = MFRC522Constants::PCD_Version;
  using Uid = MFRC522Constants::Uid;
  using MIFARE_Key = MFRC522Constants::MIFARE_Key;
  
  // Size of the MFRC522 FIFO
  static constexpr byte FIFO_SIZE = 64;    // The FIFO is 64 bytes.
  
  // Member variables
  Uid uid;                // Used by PICC_ReadCardSerial().
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for setting up the Arduino
  /////////////////////////////////////////////////////////////////////////////////////
  MFRC522(MFRC522Driver &driver)
  : _driver(driver) {};
  /////////////////////////////////////////////////////////////////////////////////////
  // Basic interface functions for communicating with the MFRC522
  /////////////////////////////////////////////////////////////////////////////////////
  void PCD_SetRegisterBitMask(PCD_Register reg, byte mask);
  void PCD_ClearRegisterBitMask(PCD_Register reg, byte mask);
  StatusCode PCD_CalculateCRC(byte *data, byte length, byte *result);
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for manipulating the MFRC522
  /////////////////////////////////////////////////////////////////////////////////////
  bool PCD_Init();
  void PCD_Reset();
  void PCD_AntennaOn();
  void PCD_AntennaOff();
  byte PCD_GetAntennaGain();
  void PCD_SetAntennaGain(byte mask);
  
  PCD_Version PCD_GetVersion();
  bool PCD_PerformSelfTest();
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Power control functions
  /////////////////////////////////////////////////////////////////////////////////////
  void PCD_SoftPowerDown();
  void PCD_SoftPowerUp();
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for communicating with PICCs
  /////////////////////////////////////////////////////////////////////////////////////
  StatusCode PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits = nullptr, byte rxAlign = 0, bool checkCRC = false);
  StatusCode PCD_CommunicateWithPICC(byte command, byte waitIRq, byte *sendData, byte sendLen, byte *backData = nullptr, byte *backLen = nullptr, byte *validBits = nullptr, byte rxAlign = 0, bool checkCRC = false);
  StatusCode PICC_RequestA(byte *bufferATQA, byte *bufferSize);
  StatusCode PICC_WakeupA(byte *bufferATQA, byte *bufferSize);
  StatusCode PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize);
  virtual StatusCode PICC_Select(Uid *uid, byte validBits = 0);
  StatusCode PICC_HaltA();
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for communicating with MIFARE PICCs
  /////////////////////////////////////////////////////////////////////////////////////
  StatusCode PCD_Authenticate(byte command, byte blockAddr, MIFARE_Key *key, Uid *uid);
  void PCD_StopCrypto1();
  StatusCode MIFARE_Read(byte blockAddr, byte *buffer, byte *bufferSize);
  StatusCode MIFARE_Write(byte blockAddr, byte *buffer, byte bufferSize);
  StatusCode MIFARE_Ultralight_Write(byte page, byte *buffer, byte bufferSize);
  StatusCode MIFARE_Decrement(byte blockAddr, int32_t delta);
  StatusCode MIFARE_Increment(byte blockAddr, int32_t delta);
  StatusCode MIFARE_Restore(byte blockAddr);
  StatusCode MIFARE_Transfer(byte blockAddr);
  StatusCode MIFARE_GetValue(byte blockAddr, int32_t *value);
  StatusCode MIFARE_SetValue(byte blockAddr, int32_t value);
  StatusCode PCD_NTAG216_AUTH(const byte password[4], byte pACK[]);
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Support functions
  /////////////////////////////////////////////////////////////////////////////////////
  StatusCode PCD_MIFARE_Transceive(byte *sendData, byte sendLen, bool acceptTimeout = false);
  static PICC_Type PICC_GetType(byte sak);
  
  // Advanced functions for MIFARE
  void MIFARE_CalculateAccessBits(byte accessBitBuffer[3], const byte g0, const byte g1, const byte g2, const byte g3) const;
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Convenience functions - does not add extra functionality
  /////////////////////////////////////////////////////////////////////////////////////
  virtual bool PICC_IsNewCardPresent();
  virtual bool PICC_ReadCardSerial();

protected:
  MFRC522Driver &_driver;
  
  // Functions for communicating with MIFARE PICCs
  StatusCode MIFARE_TwoStepHelper(byte command, byte blockAddr, int32_t data);
};

#endif
