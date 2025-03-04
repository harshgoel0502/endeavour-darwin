// #include <Arduino.h>
// #include <Adafruit_BMP280.h>
// #include <SparkFun_u-blox_GNSS_v3.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>
// #include <utility/imumaths.h>
// #include <utility/quaternion.h>
// #include <LoRa.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <SPIMemory.h>
// #include <SD.h>
// #include <SparkFun_u-blox_GNSS_v3.h> 
// #include "BluetoothSerial.h"


// const int flashCS = 32;

// SPIFlash flash(flashCS);
// uint32_t flashAddr = 0;
// uint8_t data[8];

// void initializeFlash(){
//   for (int i = 0; i < 5; i++) {
//     if (flash.error()) {
//       Serial.println(flash.error(VERBOSE));
//     }
//     if(flash.begin()){
//       Serial.println("Flash Initialised Successfully!");
//     //   flash.eraseChip();
//       return;
//     }
//     delay(1000);
//   }
// }

// int lastLoggedPkt = 0;

// void dumpSD(){
//     String dmpPkt = "";
//     flashAddr = lastLoggedPkt * 128;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     dmpPkt = dmpPkt + String(flash.readFloat(flashAddr)) + ",";
//     flashAddr += 4;
//     lastLoggedPkt++;
//     Serial.println(dmpPkt);
// }

// void setup(){
//     Serial.begin(115200);
//     SPI.begin();
//     initializeFlash();
// }

// void loop(){
//     dumpSD();
// }