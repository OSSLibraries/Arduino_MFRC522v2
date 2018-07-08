/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <Arduino.h>
#include <MFRC522DriverPin.h>


class MFRC522DriverPinSimple : public MFRC522DriverPin {
public:
  MFRC522DriverPinSimple(const uint8_t pin) : _pin{pin} {};
  
  bool init() override {
    pinMode(_pin, OUTPUT);
    return true;
  };
  
  void high() override {
    digitalWrite(_pin, HIGH);
  };
  
  void low() override {
    digitalWrite(_pin, LOW);
  };
protected:
  const uint8_t _pin;
};
