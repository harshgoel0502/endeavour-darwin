#include <Arduino.h>
#include <comm.h>
#include <sensors.h>
#include <states.h>

State Initialise();
State Ready();
State Boosting();
State Coasting();
State Apogee();
State Descent();
State Landing();
void Recovery();


// TODO: Cleanup code, add comments, move thresholds to config file and integrate filters.

/////////////////////////////////////////////////////
///                  Thresholds                   ///
/////////////////////////////////////////////////////
constexpr float launchAltThres = 1.0, launchAccThres = 40.0, coastingAccThreshold = 0.5, 
                boostToCoastTimeThreshold = 10000, coastToApogeeTimeThreshold = 44000, descentVelThres = 1.0;
constexpr float apogeeThres = 0.5;

unsigned long apogeeReachedAt;

int armed = 0;
float launchTime = 0;

State Initialise(){
  flightState = INITIALISE;
  Wire.begin();
  SPI.begin();
  initializeFlash();
  initializeBT();
  while(1){
    // SerialBT.println(".");
    if (SerialBT.available() && !armed) {
      String message = SerialBT.readStringUntil('\r');
      if(message == "CLR"){
        SerialBT.println("Clearing FLASH");
        Serial.println("Clearing FLASH");
        flash.eraseChip();
      }
      else if(message == "ARM"){
        Serial.println("ARMED");
        SerialBT.println("ARMED");
        break;
      }
    }
    delay(50);
  }
  initializeBMP();
  initializeIMU();
  initializeSD();
  initializeGPS();
  initializeLoRa();
  return READY;
}

State Ready(){
  int stateTransitionCount = 0;
  int lastTransitionCount = 0;
  while(1){
    getPacket();
    lastTransitionCount = stateTransitionCount;
    if(IMU_State == NO_ERROR && getAccMagnitude() > baseAcc + launchAccThres){
      stateTransitionCount++;
    }
    if(Baro_State == NO_ERROR && aglAlt>launchAltThres){
      stateTransitionCount++;
    }
    if(stateTransitionCount == lastTransitionCount && stateTransitionCount > 0) stateTransitionCount--;

    if (SerialBT.available()) {
      String message = SerialBT.readStringUntil('\r');
      if(message == "DISARM"){
        ESP.restart();
        return INITIALISE;
      }
    }

    if(stateTransitionCount >= 10){
      launchTime = millis();
      return BOOSTING;
    }
  }
}

State Boosting(){
  int stateTransitionCount = 0;
  float* cachedAcc = (float*) malloc(20 * sizeof(float));
  int cacheCount = 0;
  int storedCount = 0;
  float avgAccChange = 0.0;
  float lastAcc = NULL;
  unsigned long boostingEntered = millis();
  while(1){
    getPacket();

    if(storedCount < 20 && IMU_State == NO_ERROR){
      float acc = getAccMagnitude()/20;
      if(lastAcc != NULL){
        *(cachedAcc+storedCount++) = acc - lastAcc; 
        avgAccChange += acc - lastAcc;
      }
      lastAcc = acc;
    }
    else if(IMU_State == NO_ERROR){
      cacheCount = cacheCount % 20;
      float replacedAcc = *(cachedAcc+cacheCount);
      float acc = getAccMagnitude()/20;
      *(cachedAcc+cacheCount) = acc - lastAcc; 
      avgAccChange += (acc - lastAcc);
      avgAccChange -= replacedAcc;
      lastAcc = acc;
      cacheCount ++;
    }

    if(storedCount == 20 && IMU_State == NO_ERROR){
      if(avgAccChange < coastingAccThreshold){
        stateTransitionCount++;
      }
      else if(stateTransitionCount>0)stateTransitionCount--;
      if(stateTransitionCount >= 10 || millis()-boostingEntered >= boostToCoastTimeThreshold){
        free(cachedAcc);
        return COASTING;
      }
    }
    else if (millis() - boostingEntered >= boostToCoastTimeThreshold){
      free(cachedAcc);
      return COASTING;
    }
  }
}

State Coasting(){
  int stateTransitionCount = 0;
  int lastCount = 0;
  float lastKAlt = 0.0;
  float apogeeAlt = 0.0;
  unsigned long coastingEntered = millis();
  float lowestBaroVel = 0.0;
  float lastBaroPres = 0.0;

  while(1){
    getPacket();
    lastCount = stateTransitionCount;

    // Check if altitude is dropping or increasing
    if(Baro_State == NO_ERROR && aglAlt>apogeeAlt){
      stateTransitionCount = max(0,--stateTransitionCount);
      apogeeAlt = aglAlt;
    }
    else if(Baro_State == NO_ERROR && apogeeAlt-aglAlt>apogeeThres){
      stateTransitionCount++;
    }
    else if(stateTransitionCount>0)stateTransitionCount--;

    // Check if velocity is dropping
    if(Baro_State == NO_ERROR && baroVel < lowestBaroVel && baroVel < 0){
      stateTransitionCount++;
      lowestBaroVel = baroVel;
    }
    else if(stateTransitionCount>0)stateTransitionCount--;

    // Check if pressure is increasing
    if(Baro_State == NO_ERROR && baroPress > lastBaroPres){
      stateTransitionCount++;
    }
    else if(stateTransitionCount>0)stateTransitionCount--;

    lastBaroPres = baroPress;
    

    if(stateTransitionCount >= 10 || millis() - coastingEntered >= coastToApogeeTimeThreshold){
      return APOGEE;
    }
    lastKAlt = aglAlt;
  }
}

State Apogee(){
  getPacket();
  Serial.println("Apogee!");
  sendLoRa("Apogee");
  apogeeReachedAt = millis();
  return DESCENT;
}

//TODO: Implement Drogue and Main Detection Logic
State Descent(){
  int stateTransitionCount = 0;
  std::array<float, 24> lastSensorValues = sensorDataValues;
  while (1)
  {
    getPacket();
    for(int i = 0; i < sensorDataValues.size(); i++){
      if (
        ((i < 14 && IMU_State == NO_ERROR)||
         (i >= 14 && i < 18 && Baro_State == NO_ERROR)||
         (i >= 18 && GPS_State == NO_ERROR)) &&
        sensorDataValues[i] == lastSensorValues[i]){
        stateTransitionCount++;
      }
      else if(stateTransitionCount > 0){
        stateTransitionCount--;
      }
    }
    if(stateTransitionCount >= 200){
      return LANDING;
    }
    lastSensorValues = sensorDataValues;
  }
}

State Landing(){
  int stateTransitionCount = 0;
  float lastKAlt = 0.0;
  u_int32_t lastAltTime = 0;
  std::array<float, 24> lastSensorValues = sensorDataValues;
  
  while(1){
    getPacket();
    if(Baro_State == NO_ERROR){
      if(abs((aglAlt - lastKAlt) / (millis() - lastAltTime)) < descentVelThres){
        stateTransitionCount++;
      }
      else{
        stateTransitionCount = 0;
      }
    }
    else if(stateTransitionCount > 0){
      stateTransitionCount--;
    }
    for(int i = 0; i < sensorDataValues.size(); i++){
      if (
        ((i < 14 && IMU_State == NO_ERROR)||
         (i >= 14 && i < 18 && Baro_State == NO_ERROR)||
         (i >= 18 && GPS_State == NO_ERROR)) &&
        sensorDataValues[i] == lastSensorValues[i]){
        stateTransitionCount++;
      }
      else if(stateTransitionCount > 0){
        stateTransitionCount--;
      }
    }
    
    if(stateTransitionCount >= 100){
      return RECOVERY;
    }

    lastKAlt = aglAlt;
    lastAltTime = millis();
    lastSensorValues = sensorDataValues;
  }
}

void Recovery(){
  while(1){
    getPacket();
  }
}

void setup(){
  Serial.begin(115200);
  flightState = INITIALISE;
  delay(500);
}

void loop() {
  switch(flightState){
    case OFF:
    case INITIALISE:
      flightState = Initialise();
      break;
    case READY:
      flightState = Ready();
      break;
    case BOOSTING:
      flightState = Boosting();
      break;
    case COASTING:
      flightState = Coasting();
      break;
    case APOGEE:
      flightState = Apogee();
      break;
    case DESCENT:
      flightState = Descent();
      break;
    case LANDING:
      flightState = Landing();
      break;
    case RECOVERY:
      Recovery();
      break;
  }
  // if(armed){
  //   flightState = 0;
  //   state0();
  // }
  // getPacket();
  

}

