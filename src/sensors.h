#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "ema.h"
#include <utility/imumaths.h>
#include <utility/quaternion.h>
#include <SPI.h>
#include <Wire.h>
#include <SPIMemory.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h> 
#include <comm.h>
#include <states.h>
#include <vector>
#include <error.h>

enum Error IMU_State = UNDEFINED, 
      Baro_State = UNDEFINED,
      SD_State = UNDEFINED, 
      Flash_State = UNDEFINED, 
      GPS_State = UNDEFINED;

/////////////////////////////////////////////////////
///                     GPS                       ///
/////////////////////////////////////////////////////

SFE_UBLOX_GNSS myGNSS;

const int PACKET_SIZE = 128;

#define gnssAddress 0x42 

float gpsLat, gpsLong, gpsAlt, gpsSpd, gpsSats, gpsHdg;

void initializeGPS(){

  SerialBT.println("Initializing GPS");

  for (int i = 0; i < 5; i++) {
    if (myGNSS.begin(Wire, gnssAddress)) {
      myGNSS.setI2COutput(COM_TYPE_UBX); 
      myGNSS.setNavigationFrequency(2); // Produce two solutions per second
      myGNSS.setAutoPVT(true); // Tell the GNSS to output each solution periodically
      SerialBT.println("GPS Initialization Successful!");
      GPS_State = NO_ERROR;
      return;
    }
    delay(1000);
  }  
  GPS_State = GPS_ERROR;
}

void reinitializeGPS(){

  SerialBT.println("Initializing GPS");
  if (myGNSS.begin(Wire, gnssAddress)) {
    myGNSS.setI2COutput(COM_TYPE_UBX); 
    myGNSS.setNavigationFrequency(2); // Produce two solutions per second
    myGNSS.setAutoPVT(true); // Tell the GNSS to output each solution periodically
    SerialBT.println("GPS Initialization Successful!");
    GPS_State = NO_ERROR;
    return;
  }
  delay(50);
  GPS_State = GPS_ERROR;

}

void getGPS(){
  if(GPS_State != NO_ERROR){
    initializeGPS();
  }
  if(GPS_State == NO_ERROR){
    if (myGNSS.getPVT() == true)
    {
      gpsLat = myGNSS.getLatitude();
      gpsLong = myGNSS.getLongitude();
      gpsAlt = myGNSS.getAltitudeMSL();
      gpsSpd = myGNSS.getSpeedAccEst();
      gpsSats = myGNSS.getSIV();
      gpsHdg = myGNSS.getHeadingAccEst();

      // Serial.println(F("----------------------------------"));
      // int32_t lat = myGNSS.getLatitude();
      // Serial.println(F("Lat: "));
      // Serial.print(lat);
      // Serial.println(F(" * 10^-7 degrees"));
      // degreesToCoordinates(lat);

      // int32_t lng = myGNSS.getLongitude();
      // Serial.println(F("Long: "));
      // Serial.print(lng);
      // Serial.println(F(" * 10^-7 degrees"));
      // degreesToCoordinates(lng);

      // int32_t alt = myGNSS.getAltitudeMSL(); 
      // Serial.print(F("Alt: "));
      // Serial.print(alt);
      // Serial.println(F(" mm"));

      // int32_t spd = myGNSS.getGroundSpeed(); 
      // Serial.print(F("Ground Speed: "));
      // Serial.print(spd);
      // Serial.println(F(" mm/s"));

      // int32_t hdg = myGNSS.getHeading(); 
      // Serial.print(F("Heading: "));
      // Serial.print(hdg);
      // Serial.println(F(" * 10^-5 degrees"));

      // uint8_t hour = myGNSS.getHour();
      // uint8_t min = myGNSS.getMinute();
      // uint8_t sec = myGNSS.getSecond();
      // uint16_t ms = myGNSS.getMillisecond();
      // uint32_t timeInMs = timeToMs(hour,min,sec,ms);

      // uint8_t noSats = myGNSS.getSIV();
      // Serial.print(F("Sats used in fix: "));
      // Serial.println(noSats);
      // Serial.println(F("----------------------------------"));
      // Serial.println();
    }
  }
}

/////////////////////////////////////////////////////
///                     BMP                       ///
/////////////////////////////////////////////////////

Adafruit_BMP280 bmp280;
// SimpleKalmanFilter baroPressKalmanFilter(1, 1, 0.01);
const float seaLvlhPa = 1041;
float alt = 0.0, est_alt = 0.0, lastAlt = 0.0, baroTemp = 0.0,
baroPress = 0.0, baroVel = 0.0, lastAltTime = 0.0, baseAlt = 0.0, 
basePress = 0.0, aglAlt = 0.0;


void initializeBMP() {
  Serial.println("Initializing BMP280");
  SerialBT.println("Initializing BMP280");
  for (int i = 0; i < 5; i++) {
    if (bmp280.begin(0x76, 0x58)) {
      bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                          Adafruit_BMP280::SAMPLING_X2, 
                          Adafruit_BMP280::SAMPLING_X16, 
                          Adafruit_BMP280::FILTER_X16, 
                          Adafruit_BMP280::STANDBY_MS_1);

      Serial.println("BMP280 Initialization Successful!");
      SerialBT.println("BMP280 Initialization Successful!");
      Baro_State = NO_ERROR;
      for(int i = 0; i < 20; i++) {
        basePress += bmp280.readPressure()/20;
        baseAlt += bmp280.readAltitude(seaLvlhPa)/20;
      }
      return;
    }
    delay(1000);
    Serial.println("BMP280 not initialized.");
    SerialBT.println("BMP280 not initialized.");
  }
  Baro_State = BARO_ERROR;
}

void reinitializeBMP() {
  Serial.println("Initializing BMP280");
  SerialBT.println("Initializing BMP280");
  if (bmp280.begin(0x76, 0x58)) {
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                        Adafruit_BMP280::SAMPLING_X2, 
                        Adafruit_BMP280::SAMPLING_X16, 
                        Adafruit_BMP280::FILTER_X16, 
                        Adafruit_BMP280::STANDBY_MS_1);

    Serial.println("BMP280 Initialization Successful!");
    SerialBT.println("BMP280 Initialization Successful!");
    for(int i = 0; i < 20; i++) {
      basePress += bmp280.readPressure()/20;
      baseAlt += bmp280.readAltitude(seaLvlhPa)/20;
    }
    Baro_State = NO_ERROR;
    return;
  }
  delay(50);
  Serial.println("BMP280 not initialized.");
  SerialBT.println("BMP280 not initialized.");
  Baro_State = BARO_ERROR;
}

void getBMP() {
  if(Baro_State != NO_ERROR){
    reinitializeBMP();
  }
  if(Baro_State == NO_ERROR){
    alt = bmp280.readAltitude(seaLvlhPa);                   // AMSL Altitude
    aglAlt = alt-(baseAlt);                                 // AGL Altitude
    baroPress = bmp280.readPressure();
    baroTemp = bmp280.readTemperature();
    baroVel = 1000 * (alt - lastAlt) / (millis() - lastAltTime) ;
    // Serial.println(baroVel);
    // Serial.print(lastAlt);
    // Serial.print(",");
    // Serial.println(alt);
    lastAlt = alt;
    lastAltTime = millis();
  }
}

/////////////////////////////////////////////////////
///                     IMU                       ///
/////////////////////////////////////////////////////

float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;     // 9-Axis Variables 
float qX, qY, qZ, qW;                         // DMP Quaternion Variables
float imuTemp;                                // IMU Temperature
float baseAcc = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void initializeIMU(){
  delay(1000);
  for (int i = 0; i < 5; i++) {
    if(bno.begin(OPERATION_MODE_NDOF_FMC_OFF)) {            // Sensor Fusion: On, Fast Magnetic Calibration: OFF
      bno.setExtCrystalUse(true);
      // bno.printSensorDetails();
      Serial.println("BNO055 Initialization Successful!");
      SerialBT.println("BNO055 Initialization Successful!");
      IMU_State = NO_ERROR;
      sensors_event_t accelerometerData;
      for(int i = 0; i < 20; i++) {
        bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        aX = accelerometerData.acceleration.x;
        aY = accelerometerData.acceleration.y;
        aZ = accelerometerData.acceleration.z;
        baseAcc += getAccMagnitude()/20;
      }
      return;
    }
    Serial.println("IMU not Initialized");
    SerialBT.println("IMU not Initialized");
    delay(1000);
  }
  IMU_State = IMU_ERROR;
}

void reinitializeIMU(){
  delay(50);
  if(bno.begin(OPERATION_MODE_NDOF_FMC_OFF)) {            // Sensor Fusion: On, Fast Magnetic Calibration: OFF
    bno.setExtCrystalUse(true);
    // bno.printSensorDetails();
    Serial.println("BNO055 Initialization Successful!");
    SerialBT.println("BNO055 Initialization Successful!");
    IMU_State = NO_ERROR;
    sensors_event_t accelerometerData;
    for(int i = 0; i < 20; i++) {
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      aX = accelerometerData.acceleration.x;
      aY = accelerometerData.acceleration.y;
      aZ = accelerometerData.acceleration.z;
      baseAcc += getAccMagnitude()/20;
    }
    return;
  }
  Serial.println("IMU not Initialized");
  SerialBT.println("IMU not Initialized");
  delay(50);
  IMU_State = IMU_ERROR;
}

void getIMU(){
  if (IMU_State != NO_ERROR){
    reinitializeIMU();
  }
  if (IMU_State == NO_ERROR){
    sensors_event_t angbaroVelData, magnetometerData, accelerometerData;
    bno.getEvent(&angbaroVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    aX = accelerometerData.acceleration.x;
    aY = accelerometerData.acceleration.y;
    aZ = accelerometerData.acceleration.z;
    gX = angbaroVelData.acceleration.x;
    gY = angbaroVelData.acceleration.y;
    gZ = angbaroVelData.acceleration.z;
    mX = magnetometerData.acceleration.x;
    mY = magnetometerData.acceleration.y;
    mZ = magnetometerData.acceleration.z;
    imu::Quaternion quat = bno.getQuat();
    qX = quat.x();
    qY = quat.y();
    qZ= quat.z();
    qW = quat.w();
    imuTemp = bno.getTemp();
  }
}

double getAccMagnitude(){
  return sqrt((aX * aX + aY * aY + aZ * aZ));
}

double getGyroMagnitude(){
  return sqrt((gX * gX + gY * gY + gZ * gZ));
}

double getMagMagnitude(){
  return sqrt((mX * mX + mY * mY + mZ * mZ));
}
/////////////////////////////////////////////////////
///                  Packet                       ///
/////////////////////////////////////////////////////

u_int64_t pktNo = 0;
unsigned long pktTiming = 0;
String packet;
enum State flightState = OFF;

void writeSD(String packet);

void getPacket(){
  getBMP();
  getIMU();
  getGPS();
  packet = String(pktNo) + "," + String(pktTiming = millis()) + "," 
                + String(gX) + "," + String(gY) + "," + String(gZ) + ","
                + String(aX) + "," + String(aY) + "," + String(aZ) + ","
                + String(mX) + "," + String(mY) + "," + String(mZ) + ","
                + String(qX) + "," + String(qY) + "," + String(qZ) + "," + String(qW) + "," 
                + String(imuTemp) + "," 
                + String(baroPress) + "," + String(aglAlt) + "," + String(baroTemp) + "," + String(baroVel) + ","
                + String(gpsLat) + "," + String(gpsLong) + "," + String(gpsAlt) + "," + String(gpsSats) + "," + String(gpsSpd) + "," + String(gpsHdg);
  Serial.println(packet);
  SerialBT.println(String(flightState) + "," + String(aglAlt));
  writeSD(packet);
  sendLoRa(packet);
  sensorDataValues = {
    gX, gY, gZ,
    aX, aY, aZ,
    mX, mY, mZ,
    qX, qY, qZ, qW,
    imuTemp,
    baroPress, aglAlt, baroTemp, baroVel,
    gpsLat, gpsLong, gpsAlt, gpsSats, gpsSpd, gpsHdg
  };
  pktNo++;
}

/////////////////////////////////////////////////////
///                    Flash                      ///
/////////////////////////////////////////////////////

const int flashCS = 32;

SPIFlash flash(flashCS);
uint32_t flashAddr = 0;
uint8_t data[8];

std::array<float, 24> sensorDataValues = {
  gX, gY, gZ,
  aX, aY, aZ,
  mX, mY, mZ,
  qX, qY, qZ, qW,
  imuTemp,
  baroPress, aglAlt, baroTemp, baroVel,
  gpsLat, gpsLong, gpsAlt, gpsSats, gpsSpd, gpsHdg
};

void initializeFlash(){
  Serial.println("Initializing Flash");
  SerialBT.println("Initializing Flash");
  for (int i = 0; i < 5; i++) {
    if (flash.error()) {
      Flash_State = FLASH_ERROR;
      Serial.println(flash.error(VERBOSE));
    }
    if(flash.begin()){
      Serial.println("Flash Initialised Successfully!");
      SerialBT.println("Flash Initialised Successfully!");
      Flash_State = NO_ERROR;
      return;
    }
    Serial.println("Flash not initialized");
    SerialBT.println("Flash not initialized");
    delay(1000);
  }
  Flash_State = FLASH_ERROR;
}

void reinitializeFlash(){
  Serial.println("Initializing Flash");
  SerialBT.println("Initializing Flash");
  if (flash.error()) {
    Flash_State = FLASH_ERROR;
    Serial.println(flash.error(VERBOSE));
  }
  if(flash.begin()){
    Serial.println("Flash Initialised Successfully!");
    SerialBT.println("Flash Initialised Successfully!");
    Flash_State = NO_ERROR;
    return;
  }
  Serial.println("Flash not initialized");
  SerialBT.println("Flash not initialized");
  delay(50);
  Flash_State = FLASH_ERROR;
}

void writeFlash(uint64_t packet_no, unsigned long packet_time_stamp, std::array<float, 24> values, State flight_state){
  uint64_t addr = (int) packet_no * PACKET_SIZE;
  flash.writeFloat(addr, packet_no);
  addr += 4;
  flash.writeFloat(addr, packet_time_stamp);
  addr += 4;
  for(uint8_t value = 0; value < 24; value++){
    flash.writeFloat(addr, values[value]);
    addr += 4;
  }
  flash.writeFloat(addr, flight_state);
  addr += 4;
}



void setFlash(){
  if (Flash_State != NO_ERROR){
    reinitializeFlash();
  }
  if (Flash_State == NO_ERROR){
      writeFlash(pktNo, 
      pktTiming,
      sensorDataValues,
      flightState
    );
  }
}

/////////////////////////////////////////////////////
///                      SD                       ///
/////////////////////////////////////////////////////

String filename = "";   
File myFile;
int sd_count = 0;
bool FL = false;
bool fileclosed = false;
const int SDPin = 33;
float lastLogTime = 0;
int lastLoggedPkt = 0;

void writeSD(String writePkt) {
  if (SD_State != NO_ERROR){
    reinitializeSD();
  }
  if (SD_State == NO_ERROR){
    myFile = SD.open(filename, FILE_APPEND);
    if (myFile) {
      myFile.println(writePkt);
      // SerialBT.println(writePkt);
      myFile.close();
    }
  }
}

boolean loadSDFile() {
  int i = 0;
  boolean file = false;
  while (!file && i < 1024) {
    filename = "/" + (String)i + "FL.csv";
    if (!SD.exists(filename)) {
      Serial.println(filename);
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }
  return file;
}

void initializeSD() {
  Serial.println("Initializing SD");
  SerialBT.println("Initializing SD");
  SPI.begin();
  // SPI.setDataMode(SPI_MODE0);                                                                                                      
  for (int i = 0; i < 5; i++) {
    if (SD.begin(SDPin)) {
      if (loadSDFile()) {
        Serial.println("SD Initialization Successful!");
        SerialBT.println("SD Initialization Successful!");
        SD_State = NO_ERROR;
        return;
      }
    }
    Serial.println("SD not initialized");
    SerialBT.println("SD not initialized");
    delay(1000);
  }
  SD_State = SD_ERROR;
  // errorState("SD initialization failed!");
}

void reinitializeSD() {
  Serial.println("Initializing SD");
  SerialBT.println("Initializing SD");
  SPI.begin();
  // SPI.setDataMode(SPI_MODE0);  
  if (SD.begin(SDPin)) {
    if (loadSDFile()) {
      Serial.println("SD Initialization Successful!");
      SerialBT.println("SD Initialization Successful!");
      SD_State = NO_ERROR;
      return;
    }
  }
  Serial.println("SD not initialized");
  SerialBT.println("SD not initialized");
  delay(50);
  SD_State = SD_ERROR;
  // errorState("SD initialization failed!");
}

void dumpSD(){
  uint8_t count = 0;
  while (count < 5 && SD_State != NO_ERROR){
    reinitializeSD();
  }
  if(SD_State == NO_ERROR){
    if(millis()-lastLogTime>=5000){
      for(int i=0;i<100;i++){
        String dmpPkt = "";
        flashAddr = lastLoggedPkt * 128;
        for(uint16_t j = 0; j < 27; j++){
          dmpPkt += String(flash.readFloat(flashAddr)) + ",";
          flashAddr += 4;
        }
        writeSD(dmpPkt);
        lastLoggedPkt++;
      }
      lastLogTime = millis();
    }
  }
}
