#include <Drone.h>

//
//This is Drone Support Testing Code:
//
//Usages:
//1.) Serial Write 'I' to test the IMU Getters 
//2.) Serial Write 'A' to test the Autopilot Getters
//3.) Serial write 'G' to test the GPS Getters
//4.) Serial write 'P' to test GPIO Getters
//5.) Serial write 'R' to test the RC Getters
//6.) Serial.print 'C' to test the i2c getters
//7.) Serial.print 'V' to test the Vitals getters
//8.) Serial write 'S' to test the Setters
//

Drone myTrans(serialPort3); 
                            
                            
void setupTransmitter() {
  //setbuttons to inputs
  Serial.begin(57600);  //Start the Serial Serial Port
}


/*
*Functions used for callback support*/

//Satellite Getters
void printLongitude(float Long){
  Serial.print("Long");Serial.println(Long);
}
void printLatitude(float Lat){
  Serial.print("Lat");Serial.println(Lat);
}
void printAltitude(float Alt){
  Serial.print("Alt");Serial.println(Alt);
}
void printSpeed(float Spd){
  Serial.print("Spd");Serial.println(Spd);
}
void printSatelites(int Sat){
  Serial.print("Sat");Serial.println(Sat);
}

//RC Getters
void printThrottle(int16_t throt){
  Serial.print("T");Serial.println(throt);
}
void printAileron(int16_t ail){
  Serial.print("A");Serial.println(ail);
}
void printElevator(int16_t ele){
  Serial.print("E");Serial.println(ele);
}
void printRudder(int16_t rud){
  Serial.print("R");Serial.println(rud);
}

//Autopilot Getters
void printSonarAlt(float Alt){
  Serial.print("Z");Serial.println(Alt);
}
void printSonarX(float XP){
  Serial.print("X");Serial.println(XP);
}
void printSonarY(float YP){
  Serial.print("Y");Serial.println(YP);
}
void printAltitudeBase(float B){
  Serial.print("B");Serial.println(B);
}
void printAltitudeTolerance(float TOL){
  Serial.print("ATOL");Serial.println(TOL);
}
void printAltitudeKp(float KP){
  Serial.print("AKP");Serial.println(KP);
}
void printAltitudeKi(float KI){
  Serial.print("AKI");Serial.println(KI);
}
void printAltitudeReference(float Ref){
  Serial.print("AREF");Serial.println(Ref);
}
void printXKd(float B){
  Serial.print("XB");Serial.println(B);
}
void printXTolerance(float TOL){
  Serial.print("XTOL");Serial.println(TOL);
}
void printXKp(float KP){
  Serial.print("XKP");Serial.println(KP);
}
void printXKi(float KI){
  Serial.print("XKI");Serial.println(KI);
}
void printXReference(float Ref){
  Serial.print("XREF");Serial.println(Ref);
}
void printYKd(float B){
  Serial.print("YB");Serial.println(B);
}
void printYTolerance(float TOL){
  Serial.print("YTOL");Serial.println(TOL);
}
void printYKp(float KP){
  Serial.print("YKP");Serial.println(KP);
}
void printYKi(float KI){
  Serial.print("YKI");Serial.println(KI);
}
void printYReference(float Ref){
  Serial.print("YREF");Serial.println(Ref);
}

//IMU Getters
void printPitch(float P){
  Serial.print("Pit");Serial.println(P);
}
void printRoll(float R){
  Serial.print("Rol");Serial.println(R);
}
void printYaw(float Y){
  Serial.print("Yaw");Serial.println(Y);
}
void printAccelX(float X){
  Serial.print("RawX");Serial.println(X);
}
void printAccelY(float Y){
  Serial.print("RawY");Serial.println(Y);
}
void printAccelZ(float Z){
  Serial.print("RawZ");Serial.println(Z);
}
void printGyroX(float X){
  Serial.print("RawX");Serial.println(X);
}
void printGyroY(float Y){
  Serial.print("RawY");Serial.println(Y);
}
void printGyroZ(float Z){
  Serial.print("RawZ");Serial.println(Z);
}
void printMagX(float X){
  Serial.print("RawX");Serial.println(X);
}
void printMagY(float Y){
  Serial.print("RawY");Serial.println(Y);
}
void printMagZ(float Z){
  Serial.print("RawZ");Serial.println(Z);
}
void printAltitudeBarometer(float b){
  Serial.print("Alt B");Serial.println(b);
}

//gpio getters
void printPinMode(int val){
  Serial.print("Pin val");Serial.println(val);
}

void printPulseIn(int val){
  Serial.print("Pulse in");Serial.println(val);
}

void printAnalogRead(int val){
  Serial.print("A Val");Serial.println(val);
}

void printInterrupt(){
  Serial.print("Interrupt Detected");
}

//i2c getters
void printRead(uint8_t val){
  Serial.print("I2C V");Serial.println(val);
}
void printAvailable(int val){
  Serial.print("I2C Ava");Serial.println(val);
}

//vitals getters
void printVoltage(int vol){
  Serial.print("Volts");Serial.println(vol);
}
void printSignalStrength(int str){
  Serial.print("Strength");Serial.println(str);
}
void printHeartbeat(){
  Serial.println("BEAT");
}
void printHeartbeatLost(){
  Serial.print("LOST");
}


unsigned long manualTimer = 0;
unsigned long previousMillis=0;

void setup() {

  setupTransmitter();

  //Callback Setup
  myTrans.throttleCallback(printThrottle);
  myTrans.aileronCallback(printAileron);
  myTrans.elevatorCallback(printElevator);
  myTrans.sonarAltitudeCallback(printSonarAlt);
  myTrans.sonarXPositionCallback(printSonarX);
  myTrans.sonarYPositionCallback(printSonarY);
  myTrans.AltitudeKpCallback(printAltitudeKp);
  myTrans.AltitudeKiCallback(printAltitudeKi);
  myTrans.AltitudeBaseCallback(printAltitudeBase);
  myTrans.AltitudeReferenceCallback(printAltitudeReference);
  myTrans.AltitudeToleranceCallback(printAltitudeTolerance);
  myTrans.XPositionKpCallback(printXKp);
  myTrans.XPositionKiCallback(printXKi);
  myTrans.XPositionKdCallback(printXKd);
  myTrans.XPositionReferenceCallback(printXReference);
  myTrans.XPositionToleranceCallback(printXTolerance);
  myTrans.YPositionKpCallback(printYKp);
  myTrans.YPositionKiCallback(printYKi);
  myTrans.YPositionKdCallback(printYKd);
  myTrans.YPositionReferenceCallback(printYReference);
  myTrans.YPositionToleranceCallback(printYTolerance);
  myTrans.yawCallback(printYaw);
  myTrans.pitchAngleCallback(printPitch);
  myTrans.rollAngleCallback(printRoll);  
  myTrans.rudderCallback(printRudder);
  myTrans.altitudeCallback(printAltitude);
  myTrans.longitudeCallback(printLongitude);
  myTrans.latitudeCallback(printLatitude);
  myTrans.satelliteCallback(printSatelites);
  myTrans.speedCallback(printSpeed);
  for(int i = 1; i < 9; i++){
    myTrans.digitalReadCallback(printPinMode, i);  
  }
  for(int i = 1; i < 5; i++){
    myTrans.analogReadCallback(printAnalogRead, i);
  }
  myTrans.interruptCallback(printInterrupt,0);
  myTrans.interruptCallback(printInterrupt,1);
    for(int i = 1; i < 9; i++){
    myTrans.digitalReadCallback(printPulseIn, i);  
  }
  myTrans.i2cReadCallback(printRead);
  myTrans.i2cAvailableCallback(printAvailable);
  myTrans.heartbeatLostCallback(printHeartbeatLost);
  myTrans.heartbeatFoundCallback(printHeartbeat);
  myTrans.voltageCallback(printVoltage);
  myTrans.signalStrengthCallback(printSignalStrength);
  myTrans.rawAccelXCallback(printAccelX);
  myTrans.rawAccelYCallback(printAccelY);
  myTrans.rawAccelZCallback(printAccelZ);
  myTrans.rawGyroXCallback(printGyroX);
  myTrans.rawGyroYCallback(printGyroY);
  myTrans.rawGyroZCallback(printGyroZ);
  myTrans.rawMagnetometerXCallback(printMagX);
  myTrans.rawMagnetometerYCallback(printMagY);
  myTrans.rawMagnetometerZCallback(printMagZ);
  myTrans.altitudeBarometerCallback(printAltitudeBarometer);

}

void sendData(int val){

  //
  //Setters Test
  //
  switch(val){
    case 0:
      myTrans.setAltitudeReference(100.0);
      break;
    case 1:
      myTrans.setAltitudeTolerance(50.0);
      break;
    case 2:
      myTrans.setAltitudeKp(100.0);
      break;
    case 3:
      myTrans.setAltitudeKi(50.0);
      break;
    case 4:
      myTrans.setAltitudeBase(25.0);
      break;
    case 5:
      myTrans.setXPositionReference(100.0);
      break;
    case 6:
      myTrans.setXPositionTolerance(50.0);
      break;
    case 7:
      myTrans.setXPositionKp(100.0);
      break;
    case 8:
      myTrans.setXPositionKi(50.0);
      break;
    case 9:
      myTrans.setXPositionKd(25.0);
      break;
    case 10:
      myTrans.setYPositionReference(100.0);
      break;
    case 11:
      myTrans.setYPositionTolerance(50.0);
      break;
    case 12:
      myTrans.setYPositionKp(100.0);
      break;
    case 13:
      myTrans.setYPositionKi(50.0);
      break;
    case 14:
      myTrans.setYPositionKd(50.0);
      break;
    case 15:
      myTrans.setAileron(23);
      break;
    case 16:
      myTrans.setElevator(33);
      break;
    case 17:
      myTrans.setThrottle(43);
      break;
    case 18:
      myTrans.setRudder(53);
      break;
    case 19:
      myTrans.setFlightMode(63);
      break;
    case 20:
      for(int i = 0; i < 8; i++)
        myTrans.digitalWrite(i, 1);
      break;
    case 21:
      for(int i = 0; i < 4; i++)
        myTrans.digitalWrite(i,125);
      break;
    case 22:
      Serial.print("Setting Device Adress");
      myTrans.i2cSetDeviceAddress(33);
      break;
    case 23: 
      Serial.print("i2c begin trans.");
      myTrans.i2cBeginTransmission();
      myTrans.i2cWrite(23);
      myTrans.i2cEndTransmission();
      Serial.print("i2c end transmission");
      break;
    case 24:
      Serial.print("Starting Transmitter Support");
      myTrans.startTransmitterSupport();
      Serial.print("Ending TRansmitter Support");
      myTrans.stopTransmitterSupport();
      break;
  }  
}

long timer = 0;
int counter = 0;
int switcher = 0;

//used to store the variable read from Serial
char incomingByte;

void loop() {

  if(Serial.available() > 0){

    incomingByte = Serial.read(); //check the serial line

    if(incomingByte == 'A' || incomingByte == 'a'){

      myTrans.getAltitudeKp();
      
      myTrans.getAltitudeKi();
      
      myTrans.getAltitudeBase();
      
      myTrans.getAltitudeReference();

      myTrans.getAltitudeTolerance();

      myTrans.getXPositionKp();

      myTrans.getXPositionKi();

      myTrans.getXPositionKd();

      myTrans.getXPositionReference();

      myTrans.getXPositionTolerance();

      myTrans.getYPositionKp();

      myTrans.getYPositionKi();

      myTrans.getYPositionKd();

      myTrans.getYPositionReference();

      myTrans.getYPositionTolerance();

      myTrans.getSonarAltitude();

      myTrans.getSonarYPosition();

      myTrans.getSonarXPosition();

    }else if(incomingByte == 'G' || incomingByte =='g'){

      myTrans.getSatellites();

      myTrans.getSpeed();

      myTrans.getAltitude();

      myTrans.getLatitude();

      myTrans.getLongitude();

    }else if(incomingByte == 'I' || incomingByte == 'i'){

      myTrans.getYaw(); 

      myTrans.getRollAngle();

      myTrans.getPitchAngle();

    }else if(incomingByte == 'R' || incomingByte == 'r'){

      myTrans.getThrottle(); 

      myTrans.getAileron();

      myTrans.getElevator();

      myTrans.getRudder();

      myTrans.getFlightMode();

    }else if(incomingByte == 'V' || incomingByte == 'v'){

      myTrans.getVoltage();

      myTrans.getSignalStrength();

    }else if(incomingByte == 'P' || incomingByte == 'p'){

      for(int i = 1; i < 9; i++){
        Serial.print("Pin: ");Serial.println(i);
        myTrans.digitalRead(i);
      }

      for(int i = 1; i < 5; i++){
        Serial.print("Pin ");Serial.println(i);
        myTrans.analogRead(i);
      }


    }if(incomingByte == 'S' || incomingByte == 's'){

      for(int i = 0; i < 20; ){

        if( (millis() - previousMillis) >= 50){
          sendData(i);
          ++i;
          previousMillis = millis();
        }
      
      }

    }else{
      Serial.println("Incorrect usage: Please refer to the comments at the beginning of this document");
    }

  }

  //process Callbacks
  myTrans.listen();
  
}
