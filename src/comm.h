#include <Arduino.h>
#include <LoRa.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include <error.h>

enum Error LoRa_State = UNDEFINED;
/////////////////////////////////////////////////////
///                      BT                       ///
/////////////////////////////////////////////////////


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void initializeBT(){
  SerialBT.begin("endeavourFC"); //Bluetooth device name
}

void sendSerialBT(const String& msg){
    SerialBT.println(msg);
}

/////////////////////////////////////////////////////
///                    LoRa                       ///
/////////////////////////////////////////////////////

#define myWire Wire

const int LoRaCS = 35, LoRaRST = 34, LoRaDIO0 = 38;

void initializeLoRa() {
  Serial.println("Initializing LoRa");
  SerialBT.println("Initializing LoRa");
  Wire.begin();
  LoRa.setPins(LoRaCS, LoRaRST, LoRaDIO0);               //TBD LoRa Connections: RST, DIO0
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setFrequency(433E6);
  for (int i = 0; i < 5; i++) {
    if (LoRa.begin(926E6)) {
      Serial.println("LoRa Initialization Successful!");
      SerialBT.println("LoRa Initialization Successful!");
      LoRa_State = NO_ERROR;
      return;
    }
    delay(1000);
  }
  LoRa_State = RADIO_ERROR;
  // errorState("Starting LoRa failed!");
}

void reinitializeLoRa() {
  Serial.println("Initializing LoRa");
  SerialBT.println("Initializing LoRa");
  Wire.begin();
  LoRa.setPins(LoRaCS, LoRaRST, LoRaDIO0);               //TBD LoRa Connections: RST, DIO0
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setFrequency(433E6);
  if (LoRa.begin(926E6)) {
    Serial.println("LoRa Initialization Successful!");
    SerialBT.println("LoRa Initialization Successful!");
    LoRa_State = NO_ERROR;
    return;
  }
  delay(50);
  LoRa_State = RADIO_ERROR;
  // errorState("Starting LoRa failed!");
}

void sendLoRa(const String& packet) {
  if (LoRa_State != NO_ERROR){
    reinitializeLoRa();
  }
  if (LoRa_State == NO_ERROR){
    LoRa.beginPacket();
    LoRa.println(packet);
    LoRa.endPacket();
  }
}