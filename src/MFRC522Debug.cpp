/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522Debug.h"

/**
 * Returns a __FlashStringHelper pointer to the PICC type name.
 * 
 * @return const __FlashStringHelper *
 */
const __FlashStringHelper *MFRC522Debug::PICC_GetTypeName(PICC_Type piccType  ///< One of the PICC_Type enums.
                                                         ) {
  switch(piccType) {
    case PICC_Type::PICC_TYPE_ISO_14443_4:
      return F("PICC compliant with ISO/IEC 14443-4");
    case PICC_Type::PICC_TYPE_ISO_18092:
      return F("PICC compliant with ISO/IEC 18092 (NFC)");
    case PICC_Type::PICC_TYPE_MIFARE_MINI:
      return F("MIFARE Mini, 320 bytes");
    case PICC_Type::PICC_TYPE_MIFARE_1K:
      return F("MIFARE 1KB");
    case PICC_Type::PICC_TYPE_MIFARE_4K:
      return F("MIFARE 4KB");
    case PICC_Type::PICC_TYPE_MIFARE_UL:
      return F("MIFARE Ultralight or Ultralight C");
    case PICC_Type::PICC_TYPE_MIFARE_PLUS:
      return F("MIFARE Plus");
    case PICC_Type::PICC_TYPE_MIFARE_DESFIRE:
      return F("MIFARE DESFire");
    case PICC_Type::PICC_TYPE_TNP3XXX:
      return F("MIFARE TNP3XXX");
    case PICC_Type::PICC_TYPE_NOT_COMPLETE:
      return F("SAK indicates UID is not complete.");
    case PICC_Type::PICC_TYPE_UNKNOWN:
    default:
      return F("Unknown type");
  }
} // End PICC_GetTypeName()

/**
 * Returns a __FlashStringHelper pointer to a status code name.
 * 
 * @return const __FlashStringHelper *
 */
const __FlashStringHelper *MFRC522Debug::GetStatusCodeName(StatusCode code  ///< One of the StatusCode enums.
                                                          ) {
  switch(code) {
    case StatusCode::STATUS_OK:
      return F("Success.");
    case StatusCode::STATUS_ERROR:
      return F("Error in communication.");
    case StatusCode::STATUS_COLLISION:
      return F("collision detected.");
    case StatusCode::STATUS_TIMEOUT:
      return F("Timeout in communication.");
    case StatusCode::STATUS_NO_ROOM:
      return F("A buffer is not big enough.");
    case StatusCode::STATUS_INTERNAL_ERROR:
      return F("Internal error in the code. Should not happen.");
    case StatusCode::STATUS_INVALID:
      return F("Invalid argument.");
    case StatusCode::STATUS_CRC_WRONG:
      return F("The CRC_A does not match.");
    case StatusCode::STATUS_MIFARE_NACK:
      return F("A MIFARE PICC responded with NAK.");
    default:
      return F("Unknown error");
  }
} // End GetStatusCodeName()

void MFRC522Debug::PrintUID(Print &logPrint, const MFRC522Constants::Uid &uid) {
  for(byte i = 0; i < uid.size; i++) {
    logPrint.print(uid.uidByte[i] < 0x10 ? " 0" : " ");
    logPrint.print(uid.uidByte[i], HEX);
  }
}

void MFRC522Debug::PrintSelectedUID(MFRC522 &device, Print &logPrint) {
  PrintUID(logPrint, device.uid);
}

/**
 * Dumps debug info about the connected PCD to Serial.
 * Shows all known firmware versions.
 */
void MFRC522Debug::PCD_DumpVersionToSerial(MFRC522 &device, Print &logPrint) {
  // Get the MFRC522 firmware version.
  PCD_Version version = device.PCD_GetVersion();
  
  if(version != PCD_Version::Version_Unknown) {
    logPrint.print(F("Firmware Version: 0x"));
    logPrint.print(version, HEX);
  }
  // Human readable version.
  switch(version) {
    case 0xb2:
      logPrint.println(F(" = FM17522_1"));
      break;
    case 0x88:
      logPrint.println(F(" = FM17522"));
      break;
    case 0x89:
      logPrint.println(F(" = FM17522E"));
      break;
    case 0x90:
      logPrint.println(F(" = v0.0"));
      break;
    case 0x91:
      logPrint.println(F(" = v1.0"));
      break;
    case 0x92:
      logPrint.println(F(" = v2.0"));
      break;
    case 0x12:
      logPrint.println(F(" = counterfeit chip"));
      break;
    default:
      logPrint.println(F(" = (unknown)"));
  }
  // When 0x00 or 0xFF is returned, communication probably failed
  if(version == PCD_Version::Version_Unknown) {
    logPrint.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
  }
} // End PCD_DumpVersionToSerial()

/**
 * Dumps debug info about the selected PICC to Serial.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.  
 */
void MFRC522Debug::PICC_DumpToSerial(MFRC522 &device, Print &logPrint,
                                     Uid *uid  ///< Pointer to Uid struct returned from a successful PICC_Select().
                                    ) {
  MIFARE_Key key;
  
  // Dump UID, SAK and Type
  PICC_DumpDetailsToSerial(device, logPrint, uid);
  
  // Dump contents
  PICC_Type piccType = device.PICC_GetType(uid->sak);
  switch(piccType) {
    case PICC_Type::PICC_TYPE_MIFARE_MINI:
    case PICC_Type::PICC_TYPE_MIFARE_1K:
    case PICC_Type::PICC_TYPE_MIFARE_4K:
      // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
      for(byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
      }
      PICC_DumpMifareClassicToSerial(device, logPrint, uid, piccType, &key);
      break;
    
    case PICC_Type::PICC_TYPE_MIFARE_UL:
      PICC_DumpMifareUltralightToSerial(device, logPrint);
      break;
    
    case PICC_Type::PICC_TYPE_ISO_14443_4:
    case PICC_Type::PICC_TYPE_MIFARE_DESFIRE:
    case PICC_Type::PICC_TYPE_ISO_18092:
    case PICC_Type::PICC_TYPE_MIFARE_PLUS:
    case PICC_Type::PICC_TYPE_TNP3XXX:
      logPrint.println(F("Dumping memory contents not implemented for that PICC type."));
      break;
    
    case PICC_Type::PICC_TYPE_UNKNOWN:
    case PICC_Type::PICC_TYPE_NOT_COMPLETE:
    default:
      break; // No memory dump here
  }
  
  logPrint.println();
  device.PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 */
void MFRC522Debug::PICC_DumpDetailsToSerial(MFRC522 &device, Print &logPrint,
                                            Uid *uid  ///< Pointer to Uid struct returned from a successful PICC_Select().
                                           ) {
  // UID
  logPrint.print(F("Card UID:"));
  for(byte i = 0; i < uid->size; i++) {
    if(uid->uidByte[i] < 0x10)
      logPrint.print(F(" 0"));
    else
      logPrint.print(F(" "));
    logPrint.print(uid->uidByte[i], HEX);
  }
  logPrint.println();
  
  // SAK
  logPrint.print(F("Card SAK: "));
  if(uid->sak < 0x10)
    logPrint.print(F("0"));
  logPrint.println(uid->sak, HEX);
  
  // (suggested) PICC type
  PICC_Type piccType = device.PICC_GetType(uid->sak);
  logPrint.print(F("PICC type: "));
  logPrint.println(PICC_GetTypeName(piccType));
} // End PICC_DumpDetailsToSerial()

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void MFRC522Debug::PICC_DumpMifareClassicToSerial(MFRC522 &device, Print &logPrint,
                                                  Uid *uid,      ///< Pointer to Uid struct returned from a successful PICC_Select().
                                                  PICC_Type piccType,  ///< One of the PICC_Type enums.
                                                  MIFARE_Key *key    ///< Key A used for all sectors.
                                                 ) {
  byte no_of_sectors = 0;
  switch(piccType) {
    case PICC_Type::PICC_TYPE_MIFARE_MINI:
      // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
      no_of_sectors = 5;
      break;
    
    case PICC_Type::PICC_TYPE_MIFARE_1K:
      // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
      no_of_sectors = 16;
      break;
    
    case PICC_Type::PICC_TYPE_MIFARE_4K:
      // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
      no_of_sectors = 40;
      break;
    
    default: // Should not happen. Ignore.
      break;
  }
  
  // Dump sectors, highest address first.
  if(no_of_sectors) {
    logPrint.println(F("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"));
    for(int8_t i = no_of_sectors-1; i >= 0; i--) {
      PICC_DumpMifareClassicSectorToSerial(device, logPrint, uid, key, i);
    }
  }
  device.PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
  device.PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()


/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_Command::PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void MFRC522Debug::PICC_DumpMifareClassicSectorToSerial(MFRC522 &device, Print &logPrint,
                                                        Uid *uid,      ///< Pointer to Uid struct returned from a successful PICC_Select().
                                                        MIFARE_Key *key,  ///< Key A for the sector.
                                                        byte sector      ///< The sector to dump, 0..39.
                                                       ) {
  MFRC522::StatusCode status;
  byte                firstBlock;    // Address of lowest address to dump actually last block dumped)
  byte                no_of_blocks;    // Number of blocks in sector
  bool                isSectorTrailer;  // Set to true while handling the "last" (ie highest address) in the sector.
  
  // The access bits are stored in a peculiar fashion.
  // There are four groups:
  //		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
  //		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
  //		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
  //		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
  // Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
  // The four CX bits are stored together in a nible cx and an inverted nible cx_.
  byte c1, c2, c3;    // Nibbles
  byte c1_, c2_, c3_;    // Inverted nibbles
  bool invertedError;    // True if one of the inverted nibbles did not match
  byte g[4];        // Access bits for each of the four groups.
  byte group;        // 0-3 - active group for access bits
  bool firstInGroup;    // True for the first block dumped in the group
  
  // Determine position and size of sector.
  if(sector < 32) { // Sectors 0..31 has 4 blocks each
    no_of_blocks = 4;
    firstBlock   = sector*no_of_blocks;
  } else if(sector < 40) { // Sectors 32-39 has 16 blocks each
    no_of_blocks = 16;
    firstBlock   = 128+(sector-32)*no_of_blocks;
  } else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
    return;
  }
  
  // Dump blocks, highest address first.
  byte byteCount;
  byte buffer[18];
  byte blockAddr;
  isSectorTrailer = true;
  invertedError   = false;  // Avoid "unused variable" warning.
  for(int8_t blockOffset = no_of_blocks-1; blockOffset >= 0; blockOffset--) {
    blockAddr = firstBlock+blockOffset;
    // Sector number - only on first line
    if(isSectorTrailer) {
      if(sector < 10)
        logPrint.print(F("   ")); // Pad with spaces
      else
        logPrint.print(F("  ")); // Pad with spaces
      logPrint.print(sector);
      logPrint.print(F("   "));
    } else {
      logPrint.print(F("       "));
    }
    // Block number
    if(blockAddr < 10)
      logPrint.print(F("   ")); // Pad with spaces
    else {
      if(blockAddr < 100)
        logPrint.print(F("  ")); // Pad with spaces
      else
        logPrint.print(F(" ")); // Pad with spaces
    }
    logPrint.print(blockAddr);
    logPrint.print(F("  "));
    // Establish encrypted communications before reading the first block
    if(isSectorTrailer) {
      status = device.PCD_Authenticate(PICC_Command::PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
      if(status != StatusCode::STATUS_OK) {
        logPrint.print(F("PCD_Authenticate() failed: "));
        logPrint.println(GetStatusCodeName(status));
        return;
      }
    }
    // Read block
    byteCount = sizeof(buffer);
    status    = device.MIFARE_Read(blockAddr, buffer, &byteCount);
    if(status != StatusCode::STATUS_OK) {
      logPrint.print(F("MIFARE_Read() failed: "));
      logPrint.println(GetStatusCodeName(status));
      continue;
    }
    // Dump data
    for(byte index = 0; index < 16; index++) {
      if(buffer[index] < 0x10)
        logPrint.print(F(" 0"));
      else
        logPrint.print(F(" "));
      logPrint.print(buffer[index], HEX);
      if((index%4) == 3) {
        logPrint.print(F(" "));
      }
    }
    // Parse sector trailer data
    if(isSectorTrailer) {
      c1            = buffer[7] >> 4;
      c2            = buffer[8] & 0xF;
      c3            = buffer[8] >> 4;
      c1_           = buffer[6] & 0xF;
      c2_           = buffer[6] >> 4;
      c3_           = buffer[7] & 0xF;
      invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
      g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
      g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
      g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
      g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
      isSectorTrailer = false;
    }
    
    // Which access group is this block in?
    if(no_of_blocks == 4) {
      group        = blockOffset;
      firstInGroup = true;
    } else {
      group        = blockOffset/5;
      firstInGroup = (group == 3) || (group != (blockOffset+1)/5);
    }
    
    if(firstInGroup) {
      // Print access bits
      logPrint.print(F(" [ "));
      logPrint.print((g[group] >> 2) & 1, DEC);
      logPrint.print(F(" "));
      logPrint.print((g[group] >> 1) & 1, DEC);
      logPrint.print(F(" "));
      logPrint.print((g[group] >> 0) & 1, DEC);
      logPrint.print(F(" ] "));
      if(invertedError) {
        logPrint.print(F(" Inverted access bits did not match! "));
      }
    }
    
    if(group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
      int32_t value = (int32_t(buffer[3]) << 24) | (int32_t(buffer[2]) << 16) | (int32_t(buffer[1]) << 8) | int32_t(buffer[0]);
      logPrint.print(F(" Value=0x"));
      logPrint.print(value, HEX);
      logPrint.print(F(" Adr=0x"));
      logPrint.print(buffer[12], HEX);
    }
    logPrint.println();
  }
  
  return;
} // End PICC_DumpMifareClassicSectorToSerial()

/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void MFRC522Debug::PICC_DumpMifareUltralightToSerial(MFRC522 &device, Print &logPrint) {
  StatusCode status;
  byte       byteCount;
  byte       buffer[18];
  byte       i;
  
  logPrint.println(F("Page  0  1  2  3"));
  // Try the mpages of the original Ultralight. Ultralight C has more pages.
  for(byte page = 0; page < 16; page += 4) { // Read returns data for 4 pages at a time.
    // Read pages
    byteCount = sizeof(buffer);
    status    = device.MIFARE_Read(page, buffer, &byteCount);
    if(status != StatusCode::STATUS_OK) {
      logPrint.print(F("MIFARE_Read() failed: "));
      logPrint.println(GetStatusCodeName(status));
      break;
    }
    // Dump data
    for(byte offset = 0; offset < 4; offset++) {
      i = page+offset;
      if(i < 10)
        logPrint.print(F("  ")); // Pad with spaces
      else
        logPrint.print(F(" ")); // Pad with spaces
      logPrint.print(i);
      logPrint.print(F("  "));
      for(byte index = 0; index < 4; index++) {
        i = 4*offset+index;
        if(buffer[i] < 0x10)
          logPrint.print(F(" 0"));
        else
          logPrint.print(F(" "));
        logPrint.print(buffer[i], HEX);
      }
      logPrint.println();
    }
  }
} // End PICC_DumpMifareUltralightToSerial()
