#include <Arduino.h>
#include "MFRC522v2.h"
#include "MFRC522DriverPinSimple.h"
#include "MFRC522Debug.h"
#include "MFRC522DriverUART.h"
#include "MFRC522DriverSPI.h"

// This exampe was tested with an ESP32. 
// It needs the following wiring (which can be fully customized due to the powerful pinmux of the ESP32):
// RC-522 module  |  ESP32
// SDA   (RX)     |  22 (txPin)
// MISO  (TX)     |  23 (rxPin)
// RESET          |  21
//
// Don't forget to modify the RC-522: Disconnect the EA pin (32) from Vcc and wire it to GND.
// See RC522-UART-mod.jpg for reference.

MFRC522DriverUART driver{Serial1}; // Create UART driver.

MFRC522 mfrc522{driver}; // Create MFRC522 instance.

#define RESET_PIN 21

void setup()
{
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);  // power down
    delay(100);
    digitalWrite(RESET_PIN, HIGH);  // Reset by positive edge.
    //--- Start serial
    Serial.begin (115200) ;
#ifdef ESP32
    Serial1.begin(9600, SERIAL_8N1, /* rxPin */ 23, /* txPin */ 22);
#else
    // AVR does not support specifying of the pins when initializing a Serial device.
    Serial1.begin(9600, SERIAL_8N1);
#endif
    //--- Wait for serial (blink led at 10 Hz during waiting)
    while (!Serial) {
        delay (50) ;
    }

    while (!Serial1) {
        delay (50) ;
    }

    delay(1000);

    driver.PCD_ReadRegister(MFRC522Constants::PCD_Register::VersionReg);
    mfrc522.PCD_Init();   // Init MFRC522 board.
    MFRC522Debug::PCD_DumpVersionToSerial(mfrc522, Serial);	// Show details of PCD - MFRC522 Card Reader details.
	
    Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void loop()
{
	Serial.println("Checking for card...");
    if ( mfrc522.PICC_IsNewCardPresent())
    { 
        Serial.println("Card present.");
        if (mfrc522.PICC_ReadCardSerial()) {
            MFRC522Debug::PICC_DumpToSerial(mfrc522, Serial, &(mfrc522.uid));
	    }
    }
    delay(1000);
}