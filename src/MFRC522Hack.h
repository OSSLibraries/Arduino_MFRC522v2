/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <Arduino.h>
#include <MFRC522v2.h>
#include <MFRC522Debug.h>

class MFRC522Hack {
private:
  using StatusCode = MFRC522Constants::StatusCode;
  using PICC_Command = MFRC522Constants::PICC_Command;
  MFRC522 &_device;
  bool _logErrors;
  Print *_logPrint; // Injected Arduino API could be replaced by void* if required.

public:
  MFRC522Hack(MFRC522 &device, const bool logErrors, Print *logPrint = nullptr) : _device(device), _logPrint(logPrint) {
    _logErrors = logErrors && (logPrint != nullptr);
  };
  
  bool MIFARE_OpenUidBackdoor(void) const;
  
  bool MIFARE_SetUid(const byte *const newUid, const byte uidSize, MFRC522::MIFARE_Key &key, bool withBackdoor) const;
  
  bool MIFARE_UnbrickUidSector(void) const;
};
