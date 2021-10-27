/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example showing how to read data from more than one PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/OSSLibraries/Arduino_MFRC522v2
 *
 * Example sketch/program showing how to read data from more than one PICC (that is: a RFID Tag or Card) using a
 * MFRC522 based RFID Reader on the Arduino SPI interface.
 *
 * Warning: This may not work! Multiple devices at one SPI are difficult and cause many trouble!! Engineering skill
 *          and knowledge are required!
 *
 * @license Released into the public domain.
 *
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * SPI SS 1    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required **
 * SPI SS 2    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required **
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * Not found? For more see: https://github.com/OSSLibraries/Arduino_MFRC522v2#pin-layout
 */

// Note for i2c:
// With i2c possible if every device has a own unique address.
// Otherwise a i2c multiplexer is required, which requires a own implementation of MFRC522Driver.h. 

#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>

MFRC522DriverPinSimple ss_1_pin(10); // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 2.
MFRC522DriverPinSimple ss_2_pin(8); // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 1.

MFRC522DriverSPI driver_1{ss_1_pin};
MFRC522DriverSPI driver_2{ss_2_pin};

MFRC522 readers[]{driver_1, driver_2};   // Create MFRC522 instance.

/**
 * Initialize.
 */
void setup() {
  Serial.begin(115200);  // Initialize serial communications with the PC for debugging.
  while (!Serial);     // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4).
  
  for (MFRC522 reader : readers) {
    reader.PCD_Init(); // Init each MFRC522 card.
    Serial.print(F("Reader "));
    static uint8_t i = 0;
    i++;
    Serial.print(i);
    Serial.print(F(": "));
    MFRC522Debug::PCD_DumpVersionToSerial(reader, Serial);
  }
}

/**
 * Main loop.
 */
void loop() {
  // Look for new cards.
  for (MFRC522 reader : readers) {
    if (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial()) {
      Serial.print(F("Reader "));
      static uint8_t i = 0;
      i++;
      Serial.print(i);
      
      // Show some details of the PICC (that is: the tag/card).
      Serial.print(F(": Card UID:"));
      MFRC522Debug::PrintUID(Serial, reader.uid);
      Serial.println();
      
      Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = reader.PICC_GetType(reader.uid.sak);
      Serial.println(MFRC522Debug::PICC_GetTypeName(piccType));
      
      // Halt PICC.
      reader.PICC_HaltA();
      // Stop encryption on PCD.
      reader.PCD_StopCrypto1();
    }
  }
}
