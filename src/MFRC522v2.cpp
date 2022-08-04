/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522v2.h"

/////////////////////////////////////////////////////////////////////////////////////
// Functions for setting up the Arduino
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522::PCD_SetRegisterBitMask(PCD_Register reg,  ///< The register to update. One of the PCD_Register::Enums.
                                     byte mask      ///< The bits to set.
                                    ) {
  byte tmp;
  tmp = _driver.PCD_ReadRegister(reg);
  _driver.PCD_WriteRegister(reg, tmp | mask);      // set bit mask
} // End PCD_SetRegisterBitMask(PCD_Register )

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522::PCD_ClearRegisterBitMask(PCD_Register reg,  ///< The register to update. One of the PCD_Register::Enums.
                                       byte mask      ///< The bits to clear.
                                      ) {
  byte tmp;
  tmp = _driver.PCD_ReadRegister(reg);
  _driver.PCD_WriteRegister(reg, tmp & (~mask));    // clear bit mask
} // End PCD_ClearRegisterBitMask(PCD_Register )


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_CalculateCRC(byte *data,    ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                              byte length,  ///< In: The number of bytes to transfer.
                                              byte *result  ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
                                             ) {
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Idle);    // Stop any active command.
  _driver.PCD_WriteRegister(PCD_Register::DivIrqReg, 0x04);        // Clear the CRCIRq interrupt request bit
  _driver.PCD_WriteRegister(PCD_Register::FIFOLevelReg, 0x80);      // FlushBuffer = 1, FIFO initialization
  _driver.PCD_WriteRegister(PCD_Register::FIFODataReg, length, data);  // Write data to the FIFO
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_CalcCRC);    // Start the calculation
  
  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
  // TODO check/modify for other architectures than Arduino Uno 16bit
  for(uint16_t i = 5000; i > 0; i--) {
    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    byte n = _driver.PCD_ReadRegister(PCD_Register::DivIrqReg);
    if(n & 0x04) {                  // CRCIRq bit set - calculation done
      _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Idle);  // Stop calculating CRC for new content in the FIFO.
      // Transfer the result from the registers to the result buffer
      result[0] = _driver.PCD_ReadRegister(PCD_Register::CRCResultRegL);
      result[1] = _driver.PCD_ReadRegister(PCD_Register::CRCResultRegH);
      return StatusCode::STATUS_OK;
    }
  }
  // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
  return StatusCode::STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
bool MFRC522::PCD_Init() {
  // Init connection to PCD.
  if(_driver.init() == false) {
    return false;
  }

//  So far just using software based reset had no disadvantage so skip any reset pin related code.
//  bool hardReset = false;
//  
//  // If a valid pin number has been set, pull device out of power down / reset state.
//  if(_driver._resetPowerDownPin != MFRC522Driver::UNUSED_PIN) {
//    // First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
//    pinMode(_driver._resetPowerDownPin, INPUT);
//    
//    if(digitalRead(_driver._resetPowerDownPin) == LOW) {  // The MFRC522 chip is in power down mode.
//      pinMode(_driver._resetPowerDownPin, OUTPUT);    // Now set the resetPowerDownPin as digital output.
//      digitalWrite(_driver._resetPowerDownPin, HIGH);    // Exit power down mode. This triggers a hard reset.
//      // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
//      delay(50);
//      hardReset = true;
//    }
//  }
//  if(!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
//    PCD_Reset();
//  }
  
  PCD_Reset();
  
  // Reset baud rates
  _driver.PCD_WriteRegister(PCD_Register::TxModeReg, 0x00);
  _driver.PCD_WriteRegister(PCD_Register::RxModeReg, 0x00);
  // Reset ModWidthReg
  _driver.PCD_WriteRegister(PCD_Register::ModWidthReg, 0x26);
  
  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  _driver.PCD_WriteRegister(PCD_Register::TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  _driver.PCD_WriteRegister(PCD_Register::TPrescalerReg, 0xA9);    // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
  _driver.PCD_WriteRegister(PCD_Register::TReloadRegH, 0x03);    // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  _driver.PCD_WriteRegister(PCD_Register::TReloadRegL, 0xE8);
  
  _driver.PCD_WriteRegister(PCD_Register::TxASKReg, 0x40);    // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  _driver.PCD_WriteRegister(PCD_Register::ModeReg, 0x3D);    // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();            // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
  
  delay(4); // Optional delay of 4ms. Some board do need more time after init to be ready, see Readme.
  
  // If we get a valid version from board, the init was successful.
  return PCD_GetVersion() != PCD_Version::Version_Unknown;
} // End PCD_Init()


/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 * TODO return result
 */
void MFRC522::PCD_Reset() {
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_SoftReset);  // Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) .
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
  uint8_t countTries = 0;
  do {
    // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms).
    // Todo: check what lower delay is effective. 50ms block is very long.
    delay(50);
  } while((_driver.PCD_ReadRegister(PCD_Register::CommandReg) & (1 << 4)) && ((++countTries) < 3 /* Timeout after 3 tries. */));
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522::PCD_AntennaOn() {
  byte value = _driver.PCD_ReadRegister(PCD_Register::TxControlReg);
  if((value & 0x03) != 0x03) {
    _driver.PCD_WriteRegister(PCD_Register::TxControlReg, value | 0x03);
  }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522::PCD_AntennaOff() {
  PCD_ClearRegisterBitMask(PCD_Register::TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
byte MFRC522::PCD_GetAntennaGain() {
  return _driver.PCD_ReadRegister(PCD_Register::RFCfgReg) & (0x07 << 4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC522::PCD_SetAntennaGain(byte mask) {
  if(PCD_GetAntennaGain() != mask) {            // Only bother if there is a change.
    PCD_ClearRegisterBitMask(PCD_Register::RFCfgReg, (0x07 << 4));    // Clear needed to allow 000 pattern.
    PCD_SetRegisterBitMask(PCD_Register::RFCfgReg, mask & (0x07 << 4));  // Only set RxGain[2:0] bits
  }
} // End PCD_SetAntennaGain()

/**
 * Determine firmware version. Firmware is equal to chip version.
 * @return (Known) firmware version of MFRC522.
 */
MFRC522::PCD_Version MFRC522::PCD_GetVersion() {
  // Determine firmware version (see section 9.3.4.8 in spec).
  byte version = _driver.PCD_ReadRegister(PCD_Register::VersionReg);
  
  switch(version) {
    case 0x12:
      return PCD_Version::Version_Counterfeit;
    case 0xb2:
      return PCD_Version::Version_FM17522_1;
    case 0x88:
      return PCD_Version::Version_FM17522;
    case 0x89:
      return PCD_Version::Version_FM17522E;
    case 0x90:
      return PCD_Version::Version_0_0;
    case 0x91:
      return PCD_Version::Version_1_0;
    case 0x92:
      return PCD_Version::Version_2_0;
    default:
      return PCD_Version::Version_Unknown;
  }
} // End PCD_GetVersion()

/**
 * Performs a self-test of the MFRC522.
 * See 16.1.1 in https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
 * Warning: Re-inits the PCD.
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
bool MFRC522::PCD_PerformSelfTest() {
  // This follows directly the steps outlined in 16.1.1
  // 1. Perform a soft reset.
  PCD_Reset();
  
  // 2. Clear the internal buffer by writing 25 bytes of 00h.
  byte ZEROES[25] = {0x00};
  _driver.PCD_WriteRegister(PCD_Register::FIFOLevelReg, 0x80);    // flush the FIFO buffer
  _driver.PCD_WriteRegister(PCD_Register::FIFODataReg, 25, ZEROES);  // write 25 bytes of 00h to FIFO
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Mem);    // transfer to internal buffer
  
  // 3. Enable self-test.
  _driver.PCD_WriteRegister(PCD_Register::AutoTestReg, 0x09);
  
  // 4. Write 00h to FIFO buffer.
  _driver.PCD_WriteRegister(PCD_Register::FIFODataReg, 0x00);
  
  // 5. Start self-test by issuing the CalcCRC command.
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_CalcCRC);
  
  // 6. Wait for self-test to complete.
  byte        n;
  for(uint8_t i   = 0; i < 0xFF; i++) {
    // The datasheet does not specify exact completion condition except
    // that FIFO buffer should contain 64 bytes.
    // While selftest is initiated by CalcCRC command
    // it behaves differently from normal CRC computation,
    // so one can't reliably use DivIrqReg to check for completion.
    // It is reported that some devices does not trigger CRCIRq flag
    // during selftest.
    n = _driver.PCD_ReadRegister(PCD_Register::FIFOLevelReg);
    if(n >= 64) {
      break;
    }
  }
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Idle);    // Stop calculating CRC for new content in the FIFO.
  
  // 7. Read out resulting 64 bytes from the FIFO buffer.
  byte result[64];
  _driver.PCD_ReadRegister(PCD_Register::FIFODataReg, 64, result, 0);
  
  // Auto self-test done
  // Reset AutoTestReg register to be 0 again. Required for normal operation.
  _driver.PCD_WriteRegister(PCD_Register::AutoTestReg, 0x40 & 0x00);
  
  // Determine firmware version (see section 9.3.4.8 in spec).
  byte version = _driver.PCD_ReadRegister(PCD_Register::VersionReg);
  
  // Pick the appropriate reference values
  const byte *reference;
  switch(version) {
  // Fudan Semiconductor clone:
    case 0xb2:  // FM17522
      reference = FM17522_firmware_referenceB2;
      break;
    case 0x88:  // FM17522
      reference = FM17522_firmware_reference88;
      break;
    case 0x89:  // FM17522E
      reference = FM17522E_firmware_reference;
      break;
    case 0x90:  // Version 0.0
      reference = MFRC522_firmware_referenceV0_0;
      break;
    case 0x91:  // Version 1.0
      reference = MFRC522_firmware_referenceV1_0;
      break;
    case 0x92:  // Version 2.0
      reference = MFRC522_firmware_referenceV2_0;
      break;
    default:  // Unknown version
      return false; // abort test
  }
  
  bool verified = true;
  // Verify that the results match up to our expectations.
  for(uint8_t i = 0; i < 64; i++) {
    if(result[i] != pgm_read_byte(&(reference[i]))) {
      verified = false;
    }
  }
  
  // 8. Perform a re-init, because PCD does not work after test.
  // Reset does not work as expected.
  // "Auto self-test done" does not work as expected.
  PCD_Init();
  
  // Test process done.
  return verified;
} // End PCD_PerformSelfTest()

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Only soft power down mode is available through software.
 * IMPORTANT NOTE!!!! Calling any other function that uses CommandReg will disable soft power down mode.
 * For more details about power control, refer to the datasheet - page 33 (8.6).
 */
void MFRC522::PCD_SoftPowerDown() {
  byte val = _driver.PCD_ReadRegister(PCD_Register::CommandReg); // Read state of the command register 
  val |= (1 << 4);// set PowerDown bit ( bit 4 ) to 1 
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, val);//write new value to the command register
}

/**
 * Power on the PCD again, after PCD_SoftPowerDown.
 */
void MFRC522::PCD_SoftPowerUp() {
  byte val = _driver.PCD_ReadRegister(PCD_Register::CommandReg); // Read state of the command register.
  val &= ~(1 << 4);// set PowerDown bit ( bit 4 ) to 0.
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, val);//write new value to the command register.
  // wait until PowerDown bit is cleared (this indicates end of wake up procedure).
  const uint32_t timeout = (uint32_t)millis()+500;// create timer for timeout (just in case).
  
  while(millis() <= timeout) { // set timeout to 500 ms 
    val = _driver.PCD_ReadRegister(PCD_Register::CommandReg);// Read state of the command register
    if(!(val & (1 << 4))) { // if powerdown bit is 0 
      break; // wake up procedure is finished 
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_TransceiveData(byte *sendData,    ///< Pointer to the data to transfer to the FIFO.
                                                byte sendLen,    ///< Number of bytes to transfer to the FIFO.
                                                byte *backData,    ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                byte *backLen,    ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                byte *validBits,  ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
                                                byte rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                               ) {
  byte waitIRq = 0x30;    // RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(PCD_Command::PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_CommunicateWithPICC(byte command,    ///< The command to execute. One of the PCD_Command enums.
                                                     byte waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                                     byte *sendData,    ///< Pointer to the data to transfer to the FIFO.
                                                     byte sendLen,    ///< Number of bytes to transfer to the FIFO.
                                                     byte *backData,    ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                     byte *backLen,    ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                     byte *validBits,  ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                                     byte rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                     bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                                    ) {
  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4)+txLastBits;    // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
  
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Idle);      // Stop any active command.
  _driver.PCD_WriteRegister(PCD_Register::ComIrqReg, 0x7F);          // Clear all seven interrupt request bits
  _driver.PCD_WriteRegister(PCD_Register::FIFOLevelReg, 0x80);        // FlushBuffer = 1, FIFO initialization
  _driver.PCD_WriteRegister(PCD_Register::FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
  _driver.PCD_WriteRegister(PCD_Register::BitFramingReg, bitFraming);    // Bit adjustments
  _driver.PCD_WriteRegister(PCD_Register::CommandReg, command);        // Execute the command
  if(command == PCD_Command::PCD_Transceive) {
    PCD_SetRegisterBitMask(PCD_Register::BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
  }
  
  // Wait for the command to complete.
  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86μs.
  // TODO check/modify for other architectures than Arduino Uno 16bit
  uint16_t i;
  for(i = 2000; i > 0; i--) {
    byte n = _driver.PCD_ReadRegister(PCD_Register::ComIrqReg);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if(n & waitIRq) {          // One of the interrupts that signal success has been set.
      break;
    }
    if(n & 0x01) {            // Timer interrupt - nothing received in 25ms
      return StatusCode::STATUS_TIMEOUT;
    }
  }
  // 35.7ms and nothing happened. Communication with the MFRC522 might be down.
  if(i == 0) {
    return StatusCode::STATUS_TIMEOUT;
  }
  
  // Stop now if any errors except collisions were detected.
  byte errorRegValue = _driver.PCD_ReadRegister(PCD_Register::ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if(errorRegValue & 0x13) {   // BufferOvfl ParityErr ProtocolErr
    return StatusCode::STATUS_ERROR;
  }
  
  byte _validBits = 0;
  
  // If the caller wants data back, get it from the MFRC522.
  if(backData && backLen) {
    byte n = _driver.PCD_ReadRegister(PCD_Register::FIFOLevelReg);  // Number of bytes in the FIFO
    if(n > *backLen) {
      return StatusCode::STATUS_NO_ROOM;
    }
    *backLen = n;                      // Number of bytes returned
    _driver.PCD_ReadRegister(PCD_Register::FIFODataReg, n, backData, rxAlign);  // Get received data from FIFO
    _validBits = _driver.PCD_ReadRegister(PCD_Register::ControlReg) & 0x07;    // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if(validBits) {
      *validBits = _validBits;
    }
  }
  
  // Tell about collisions
  if(errorRegValue & 0x08) {    // CollErr
    return StatusCode::STATUS_COLLISION;
  }
  
  // Perform CRC_A validation if requested.
  if(backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if(*backLen == 1 && _validBits == 4) {
      return StatusCode::STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if(*backLen < 2 || _validBits != 0) {
      return StatusCode::STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte                controlBuffer[2];
    MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen-2, &controlBuffer[0]);
    if(status != StatusCode::STATUS_OK) {
      return status;
    }
    if((backData[*backLen-2] != controlBuffer[0]) || (backData[*backLen-1] != controlBuffer[1])) {
      return StatusCode::STATUS_CRC_WRONG;
    }
  }
  
  return StatusCode::STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get StatusCode::STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_RequestA(byte *bufferATQA,  ///< The buffer to store the ATQA (Answer to request) in
                                           byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if StatusCode::STATUS_OK.
                                          ) {
  return PICC_REQA_or_WUPA(PICC_Command::PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get StatusCode::STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_WakeupA(byte *bufferATQA,  ///< The buffer to store the ATQA (Answer to request) in
                                          byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if StatusCode::STATUS_OK.
                                         ) {
  return PICC_REQA_or_WUPA(PICC_Command::PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get StatusCode::STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_REQA_or_WUPA(byte command,    ///< The command to send - PICC_Command::PICC_CMD_REQA or PICC_Command::PICC_CMD_WUPA
                                               byte *bufferATQA,  ///< The buffer to store the ATQA (Answer to request) in
                                               byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if StatusCode::STATUS_OK.
                                              ) {
  byte                validBits;
  MFRC522::StatusCode status;
  
  if(bufferATQA == nullptr || *bufferSize < 2) {  // The ATQA response is 2 bytes long.
    return StatusCode::STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(PCD_Register::CollReg, 0x80);    // ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;                  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status    = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
  if(status != StatusCode::STATUS_OK) {
    return status;
  }
  if(*bufferSize != 2 || validBits != 0) {    // ATQA must be exactly 16 bits.
    return StatusCode::STATUS_ERROR;
  }
  return StatusCode::STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_Select(Uid *uid,      ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
                                         byte validBits    ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
                                        ) {
  bool                uidComplete;
  bool                selectDone;
  bool                useCascadeTag;
  byte                cascadeLevel = 1;
  MFRC522::StatusCode result;
  byte                count;
  byte                checkBit;
  byte                index;
  byte                uidIndex;          // The first index in uid->uidByte[] that is used in the current Cascade Level.
  int8_t              currentLevelKnownBits;    // The number of known UID bits in the current Cascade Level.
  byte                buffer[9];          // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte                bufferUsed;        // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  byte                rxAlign;          // Used in BitFramingReg. Defines the bit position for the first bit received.
  byte                txLastBits;        // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
  byte                *responseBuffer;
  byte                responseLength;
  
  // Description of buffer structure:
  //		Byte 0: SEL 				Indicates the Cascade Level: PICC_Command::PICC_CMD_SEL_CL1, PICC_Command::PICC_CMD_SEL_CL2 or PICC_Command::PICC_CMD_SEL_CL3
  //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
  //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
  //		Byte 3: UID-data
  //		Byte 4: UID-data
  //		Byte 5: UID-data
  //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
  //		Byte 7: CRC_A
  //		Byte 8: CRC_A
  // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
  //		========	=============	=====	=====	=====	=====
  //		 4 bytes		1			uid0	uid1	uid2	uid3
  //		 7 bytes		1			CT		uid0	uid1	uid2
  //						2			uid3	uid4	uid5	uid6
  //		10 bytes		1			CT		uid0	uid1	uid2
  //						2			CT		uid3	uid4	uid5
  //						3			uid6	uid7	uid8	uid9
  
  // Sanity checks
  if(validBits > 80) {
    return StatusCode::STATUS_INVALID;
  }
  
  // Prepare MFRC522
  PCD_ClearRegisterBitMask(PCD_Register::CollReg, 0x80);    // ValuesAfterColl=1 => Bits received after collision are cleared.
  
  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = false;
  while(!uidComplete) {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch(cascadeLevel) {
      case 1:
        buffer[0] = PICC_Command::PICC_CMD_SEL_CL1;
        uidIndex      = 0;
        useCascadeTag = validBits && uid->size > 4;  // When we know that the UID has more than 4 bytes
        break;
      
      case 2:
        buffer[0] = PICC_Command::PICC_CMD_SEL_CL2;
        uidIndex      = 3;
        useCascadeTag = validBits && uid->size > 7;  // When we know that the UID has more than 7 bytes
        break;
      
      case 3:
        buffer[0] = PICC_Command::PICC_CMD_SEL_CL3;
        uidIndex      = 6;
        useCascadeTag = false;            // Never used in CL3.
        break;
      
      default:
        return StatusCode::STATUS_INTERNAL_ERROR;
        break;
    }
    
    // How many UID bits are known in this Cascade Level?
    currentLevelKnownBits = validBits-(8*uidIndex);
    if(currentLevelKnownBits < 0) {
      currentLevelKnownBits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index                 = 2; // destination index in buffer[]
    if(useCascadeTag) {
      buffer[index++] = PICC_Command::PICC_CMD_CT;
    }
    byte bytesToCopy = currentLevelKnownBits/8+(currentLevelKnownBits%8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if(bytesToCopy) {
      byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if(bytesToCopy > maxBytes) {
        bytesToCopy = maxBytes;
      }
      for(count     = 0; count < bytesToCopy; count++) {
        buffer[index++] = uid->uidByte[uidIndex+count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if(useCascadeTag) {
      currentLevelKnownBits += 8;
    }
    
    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = false;
    while(!selectDone) {
      // Find out how many bits and bytes to send and receive.
      if(currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
        //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
        // Calculate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        // Calculate CRC_A
        result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
        if(result != StatusCode::STATUS_OK) {
          return result;
        }
        txLastBits     = 0; // 0 => All 8 bits are valid.
        bufferUsed     = 9;
        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer = &buffer[6];
        responseLength = 3;
      } else { // This is an ANTICOLLISION.
        //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        txLastBits = currentLevelKnownBits%8;
        count      = currentLevelKnownBits/8;  // Number of whole bytes in the UID part.
        index      = 2+count;          // Number of whole bytes: SEL + NVB + UIDs
        buffer[1] = (index << 4)+txLastBits;  // NVB - Number of Valid Bits
        bufferUsed     = index+(txLastBits ? 1 : 0);
        // Store response in the unused part of buffer
        responseBuffer = &buffer[index];
        responseLength = sizeof(buffer)-index;
      }
      
      // Set bit adjustments
      rxAlign = txLastBits;                      // Having a separate variable is overkill. But it makes the next line easier to read.
      _driver.PCD_WriteRegister(PCD_Register::BitFramingReg, (rxAlign << 4)+txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
      
      // Transmit the buffer and receive the response.
      result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
      if(result == StatusCode::STATUS_COLLISION) { // More than one PICC in the field => collision.
        byte valueOfCollReg = _driver.PCD_ReadRegister(PCD_Register::CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
        if(valueOfCollReg & 0x20) { // CollPosNotValid
          return StatusCode::STATUS_COLLISION; // Without a valid collision position we cannot continue
        }
        byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
        if(collisionPos == 0) {
          collisionPos = 32;
        }
        if(collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
          return StatusCode::STATUS_INTERNAL_ERROR;
        }
        // Choose the PICC with the bit set.
        currentLevelKnownBits = collisionPos;
        count                 = currentLevelKnownBits%8; // The bit to modify
        checkBit              = (currentLevelKnownBits-1)%8;
        index                 = 1+(currentLevelKnownBits/8)+(count ? 1 : 0); // First byte is index 0.
        buffer[index] |= (1 << checkBit);
      } else if(result != StatusCode::STATUS_OK) {
        return result;
      } else { // StatusCode::STATUS_OK
        if(currentLevelKnownBits >= 32) { // This was a SELECT.
          selectDone = true; // No more anticollision 
          // We continue below outside the while.
        } else { // This was an ANTICOLLISION.
          // We now have all 32 bits of the UID in this Cascade Level
          currentLevelKnownBits = 32;
          // Run loop again to do the SELECT.
        }
      }
    } // End of while (!selectDone)
    
    // We do not check the CBB - it was constructed by us above.
    
    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index       = (buffer[2] == PICC_Command::PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy = (buffer[2] == PICC_Command::PICC_CMD_CT) ? 3 : 4;
    for(count   = 0; count < bytesToCopy; count++) {
      uid->uidByte[uidIndex+count] = buffer[index++];
    }
    
    // Check response SAK (Select Acknowledge)
    if(responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
      return StatusCode::STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if(result != StatusCode::STATUS_OK) {
      return result;
    }
    if((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
      return StatusCode::STATUS_CRC_WRONG;
    }
    if(responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    } else {
      uidComplete = true;
      uid->sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)
  
  // Set correct uid->size
  uid->size = 3*cascadeLevel+1;
  
  return StatusCode::STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_HaltA() {
  MFRC522::StatusCode result;
  byte                buffer[4];
  
  // Build command buffer
  buffer[0] = PICC_Command::PICC_CMD_HLTA;
  buffer[1] = 0;
  // Calculate CRC_A
  result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  // Send the command.
  // The standard says:
  //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
  //		HLTA command, this response shall be interpreted as 'not acknowledge'.
  // We interpret that this way: Only StatusCode::STATUS_TIMEOUT is a success.
  result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0);
  if(result == StatusCode::STATUS_TIMEOUT) {
    return StatusCode::STATUS_OK;
  }
  if(result == StatusCode::STATUS_OK) { // That is ironically NOT ok in this case ;-)
    return StatusCode::STATUS_ERROR;
  }
  return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise. Probably StatusCode::STATUS_TIMEOUT if you supply the wrong key.
 */
MFRC522::StatusCode MFRC522::PCD_Authenticate(byte command,    ///< PICC_Command::PICC_CMD_MF_AUTH_KEY_A or PICC_Command::PICC_CMD_MF_AUTH_KEY_B
                                              byte blockAddr,  ///< The block number. See numbering in the comments in the .h file.
                                              MIFARE_Key *key,  ///< Pointer to the Crypto1 key to use (6 bytes).
                                              Uid *uid      ///< Pointer to Uid struct. The first 4 bytes of the UID is used.
                                             ) {
  byte waitIRq = 0x10;    // IdleIRq
  
  // Build command buffer
  byte sendData[12];
  sendData[0] = command;
  sendData[1] = blockAddr;
  for(byte i = 0; i < MIFARE_Misc::MF_KEY_SIZE; i++) {  // 6 key bytes
    sendData[2+i] = key->keyByte[i];
  }
  // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
  // section 3.2.5 "MIFARE Classic Authentication".
  // The only missed case is the MF1Sxxxx shortcut activation,
  // but it requires cascade tag (CT) byte, that is not part of uid.
  for(byte i = 0; i < 4; i++) {        // The last 4 bytes of the UID
    sendData[8+i] = uid->uidByte[i+uid->size-4];
  }
  
  // Start the authentication.
  return PCD_CommunicateWithPICC(PCD_Command::PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData));
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void MFRC522::PCD_StopCrypto1() {
  // Clear MFCrypto1On bit
  PCD_ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning StatusCode::STATUS_OK.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Read(byte blockAddr,  ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
                                         byte *buffer,    ///< The buffer to store the data in.
                                         byte *bufferSize  ///< Buffer size, at least 18 bytes. Also number of bytes returned if StatusCode::STATUS_OK.
                                        ) {
  MFRC522::StatusCode result;
  
  // Sanity check
  if(buffer == nullptr || *bufferSize < 18) {
    return StatusCode::STATUS_NO_ROOM;
  }
  
  // Build command buffer
  buffer[0] = PICC_Command::PICC_CMD_MF_READ;
  buffer[1] = blockAddr;
  // Calculate CRC_A
  result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  // Transmit the buffer and receive the response, validate CRC_A.
  return PCD_TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Write(byte blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
                                          byte *buffer,  ///< The 16 bytes to write to the PICC.
                                          byte bufferSize  ///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
                                         ) {
  MFRC522::StatusCode result;
  
  // Sanity check
  if(buffer == nullptr || bufferSize < 16) {
    return StatusCode::STATUS_INVALID;
  }
  
  // Mifare Classic protocol requires two communications to perform a write.
  // Step 1: Tell the PICC we want to write to block blockAddr.
  byte cmdBuffer[2];
  cmdBuffer[0] = PICC_Command::PICC_CMD_MF_WRITE;
  cmdBuffer[1] = blockAddr;
  result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  // Step 2: Transfer the data
  result = PCD_MIFARE_Transceive(buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  return StatusCode::STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 byte page to the active MIFARE Ultralight PICC.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Ultralight_Write(byte page,    ///< The page (2-15) to write to.
                                                     byte *buffer,  ///< The 4 bytes to write to the PICC
                                                     byte bufferSize  ///< Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
                                                    ) {
  MFRC522::StatusCode result;
  
  // Sanity check
  if(buffer == nullptr || bufferSize < 4) {
    return StatusCode::STATUS_INVALID;
  }
  
  // Build command buffer
  byte cmdBuffer[6];
  cmdBuffer[0] = PICC_Command::PICC_CMD_UL_WRITE;
  cmdBuffer[1] = page;
  memcpy(&cmdBuffer[2], buffer, 4);
  
  // Perform the write
  result = PCD_MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  return StatusCode::STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Decrement(byte blockAddr, ///< The block (0-0xff) number.
                                              int32_t delta    ///< This number is subtracted from the value of block blockAddr.
                                             ) {
  return MIFARE_TwoStepHelper(PICC_Command::PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Increment(byte blockAddr, ///< The block (0-0xff) number.
                                              int32_t delta    ///< This number is added to the value of block blockAddr.
                                             ) {
  return MIFARE_TwoStepHelper(PICC_Command::PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Restore(byte blockAddr ///< The block (0-0xff) number.
                                           ) {
  // The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
  // Doing only a single step does not work, so I chose to transfer 0L in step two.
  return MIFARE_TwoStepHelper(PICC_Command::PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_TwoStepHelper(byte command,  ///< The command to use
                                                  byte blockAddr,  ///< The block (0-0xff) number.
                                                  int32_t data    ///< The data to transfer in step 2
                                                 ) {
  MFRC522::StatusCode result;
  byte                cmdBuffer[2]; // We only need room for 2 bytes.
  
  // Step 1: Tell the PICC the command and block address
  cmdBuffer[0] = command;
  cmdBuffer[1] = blockAddr;
  result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  // Step 2: Transfer the data
  result = PCD_MIFARE_Transceive((byte*) & data, 4, true); // Adds CRC_A and accept timeout as success.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  return StatusCode::STATUS_OK;
} // End MIFARE_TwoStepHelper()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Transfer(byte blockAddr ///< The block (0-0xff) number.
                                            ) {
  MFRC522::StatusCode result;
  byte                cmdBuffer[2]; // We only need room for 2 bytes.
  
  // Tell the PICC we want to transfer the result into block blockAddr.
  cmdBuffer[0] = PICC_Command::PICC_CMD_MF_TRANSFER;
  cmdBuffer[1] = blockAddr;
  result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  return StatusCode::STATUS_OK;
} // End MIFARE_Transfer()

/**
 * Helper routine to read the current value from a Value Block.
 * 
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function. 
 * 
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[out]  value       Current value of the Value Block.
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
  */
MFRC522::StatusCode MFRC522::MIFARE_GetValue(byte blockAddr, int32_t *value) {
  MFRC522::StatusCode status;
  byte                buffer[18];
  byte                size = sizeof(buffer);
  
  // Read the block
  status = MIFARE_Read(blockAddr, buffer, &size);
  if(status == StatusCode::STATUS_OK) {
    // Extract the value
    *value = (int32_t(buffer[3]) << 24) | (int32_t(buffer[2]) << 16) | (int32_t(buffer[1]) << 8) | int32_t(buffer[0]);
  }
  return status;
} // End MIFARE_GetValue()

/**
 * Helper routine to write a specific value into a Value Block.
 * 
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function. 
 * 
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[in]   value       New value of the Value Block.
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_SetValue(byte blockAddr, int32_t value) {
  byte buffer[18];
  
  // Translate the int32_t into 4 bytes; repeated 2x in value block
  buffer[0]  = buffer[8]  = (value & 0xFF);
  buffer[1]  = buffer[9]  = (value & 0xFF00) >> 8;
  buffer[2]  = buffer[10] = (value & 0xFF0000) >> 16;
  buffer[3]  = buffer[11] = (value & 0xFF000000) >> 24;
  // Inverse 4 bytes also found in value block
  buffer[4]  = ~buffer[0];
  buffer[5]  = ~buffer[1];
  buffer[6]  = ~buffer[2];
  buffer[7]  = ~buffer[3];
  // Address 2x with inverse address 2x
  buffer[12] = buffer[14] = blockAddr;
  buffer[13] = buffer[15] = ~blockAddr;
  
  // Write the whole data block
  return MIFARE_Write(blockAddr, buffer, 16);
} // End MIFARE_SetValue()

/**
 * Authenticate with a NTAG216.
 * 
 * Only for NTAG216. Authenticate with 32bit password.
 * First implemented by Gargantuanman.
 * 
 * @param[in]   password   password (32bit).
 * @param[in]   pACK       result success???.
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_NTAG216_AUTH(const byte password[4], byte pACK[]) {
  // TODO: Fix cmdBuffer length and rxlength. They really should match.
  //       (Better still, rxlength should not even be necessary.)
  // TODO: Refactor.
  
  MFRC522::StatusCode result;
  byte                cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
  
  cmdBuffer[0] = 0x1B; // Authentication command.
  
  for(byte i = 0; i < 4; i++) {
    cmdBuffer[i+1] = password[i];
  }
  
  result = PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);
  
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  // Transceive the data, store the reply in cmdBuffer[]
  byte waitIRq   = 0x30;  // RxIRq and IdleIRq
//	byte cmdBufferSize	= sizeof(cmdBuffer);
  byte validBits = 0;
  byte rxlength  = 5;
  result = PCD_CommunicateWithPICC(PCD_Command::PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits);
  
  pACK[0] = cmdBuffer[0];
  pACK[1] = cmdBuffer[1];
  
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  
  return StatusCode::STATUS_OK;
} // End PCD_NTAG216_AUTH()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 * 
 * @return StatusCode::STATUS_OK on success, StatusCode::STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_MIFARE_Transceive(byte *sendData,    ///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
                                                   byte sendLen,    ///< Number of bytes in sendData.
                                                   bool acceptTimeout  ///< True => A timeout is also success
                                                  ) {
  MFRC522::StatusCode result;
  byte                cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
  
  // Sanity check
  if(sendData == nullptr || sendLen > 16) {
    return StatusCode::STATUS_INVALID;
  }
  
  // Copy sendData[] to cmdBuffer[] and add CRC_A
  memcpy(cmdBuffer, sendData, sendLen);
  result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  sendLen += 2;
  
  // Transceive the data, store the reply in cmdBuffer[]
  byte waitIRq       = 0x30;    // RxIRq and IdleIRq
  byte cmdBufferSize = sizeof(cmdBuffer);
  byte validBits     = 0;
  result = PCD_CommunicateWithPICC(PCD_Command::PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits);
  if(acceptTimeout && result == StatusCode::STATUS_TIMEOUT) {
    return StatusCode::STATUS_OK;
  }
  if(result != StatusCode::STATUS_OK) {
    return result;
  }
  // The PICC must reply with a 4 bit ACK
  if(cmdBufferSize != 1 || validBits != 4) {
    return StatusCode::STATUS_ERROR;
  }
  if(cmdBuffer[0] != MIFARE_Misc::MF_ACK) {
    return StatusCode::STATUS_MIFARE_NACK;
  }
  return StatusCode::STATUS_OK;
} // End PCD_MIFARE_Transceive()


/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
MFRC522::PICC_Type MFRC522::PICC_GetType(byte sak    ///< The SAK byte returned from PICC_Select().
                                        ) {
  // http://www.nxp.com/documents/application_note/AN10833.pdf 
  // 3.2 Coding of Select Acknowledge (SAK)
  // ignore 8-bit (iso14443 starts with LSBit = bit 1)
  // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
  sak &= 0x7F;
  switch(sak) {
    case 0x04:
      return PICC_Type::PICC_TYPE_NOT_COMPLETE;  // UID not complete
    case 0x09:
      return PICC_Type::PICC_TYPE_MIFARE_MINI;
    case 0x08:
      return PICC_Type::PICC_TYPE_MIFARE_1K;
    case 0x18:
      return PICC_Type::PICC_TYPE_MIFARE_4K;
    case 0x00:
      return PICC_Type::PICC_TYPE_MIFARE_UL;
    case 0x10:
    case 0x11:
      return PICC_Type::PICC_TYPE_MIFARE_PLUS;
    case 0x01:
      return PICC_Type::PICC_TYPE_TNP3XXX;
    case 0x20:
      return PICC_Type::PICC_TYPE_ISO_14443_4;
    case 0x40:
      return PICC_Type::PICC_TYPE_ISO_18092;
    default:
      return PICC_Type::PICC_TYPE_UNKNOWN;
  }
} // End PICC_GetType()


/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MFRC522::MIFARE_CalculateAccessBits(byte accessBitBuffer[3],  ///< Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
                                         const byte g0,        ///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
                                         const byte g1,        ///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
                                         const byte g2,        ///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
                                         const byte g3          ///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
                                        ) const {
  const byte c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
  const byte c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
  const byte c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);
  
  accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
  accessBitBuffer[1] = c1 << 4 | (~c3 & 0xF);
  accessBitBuffer[2] = c3 << 4 | c2;
} // End MIFARE_CalculateAccessBits()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_Command::PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool MFRC522::PICC_IsNewCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  // Reset baud rates
  _driver.PCD_WriteRegister(PCD_Register::TxModeReg, 0x00);
  _driver.PCD_WriteRegister(PCD_Register::RxModeReg, 0x00);
  // Reset ModWidthReg
  _driver.PCD_WriteRegister(PCD_Register::ModWidthReg, 0x26);
  
  MFRC522::StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
  return (result == StatusCode::STATUS_OK || result == StatusCode::STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
bool MFRC522::PICC_ReadCardSerial() {
  MFRC522::StatusCode result = PICC_Select(&uid);
  return (result == StatusCode::STATUS_OK);
} // End PICC_ReadCardSerial()
