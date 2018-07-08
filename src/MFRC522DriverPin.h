/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

/**
 * Must be pure virtual.
 */
class MFRC522DriverPin {
public:
  virtual bool init() = 0;
  
  virtual void high() = 0;
  
  virtual void low() = 0;
};
