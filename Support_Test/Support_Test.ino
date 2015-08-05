//This Example demonstrates how send the drone 10 feet ahead
#include <Drone.h>

//create an instance of Avant
//0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
//If the Xbee is connected to software serial ports
//Specify the RX, TX port like myTrans(rx, tx)
Drone myTrans(serialPort3); 
                            
                            
void setupTransmitter() {
  //setbuttons to inputs
  myTrans.setFlightMode(10);
  Serial.begin(57600);  //Start the Serial Serial Port
}

void printThrottle(int16_t throt){
  Serial.print("T");Serial.println(throt);
}

void printAileron(int16_t ail){
  Serial.print("A");Serial.println(ail);
}
void printElevator(int16_t ele){
  Serial.print("E");Serial.println(ele);
}

void printSonarAlt(float Alt){
  Serial.print("Z");Serial.println(Alt);
}

void printSonarX(float XP){
  Serial.print("X");Serial.println(XP);
}

void printSonarY(float YP){
  Serial.print("Y");Serial.println(YP);
}

void printAltitudeBase(int16_t B){
  Serial.print("B");Serial.println(B);
}

void printAltitudeTolerance(float TOL){
  Serial.print("ATOL");Serial.println(TOL);
}
void printAltitudeKp(int16_t KP){
  Serial.print("AKP");Serial.println(KP);
}
void printAltitudeKi(int16_t KI){
  Serial.print("AKI");Serial.println(KI);
}
void printAltitudeReference(float Ref){
  Serial.print("AREF");Serial.println(Ref);
}
void printXKd(int16_t B){
  Serial.print("XB");Serial.println(B);
}

void printXTolerance(float TOL){
  Serial.print("XTOL");Serial.println(TOL);
}
void printXKp(int16_t KP){
  Serial.print("XKP");Serial.println(KP);
}
void printXKi(int16_t KI){
  Serial.print("XKI");Serial.println(KI);
}
void printXReference(float Ref){
  Serial.print("XREF");Serial.println(Ref);
}

void printYKd(int16_t B){
  Serial.print("YB");Serial.println(B);
}

void printYTolerance(float TOL){
  Serial.print("YTOL");Serial.println(TOL);
}
void printYKp(int16_t KP){
  Serial.print("YKP");Serial.println(KP);
}
void printYKi(int16_t KI){
  Serial.print("YKI");Serial.println(KI);
}
void printYReference(float Ref){
  Serial.print("YREF");Serial.println(Ref);
}
void printPitch(float P){
  Serial.print("Pit");Serial.println(P);
}
void printRoll(float R){
  Serial.print("Rol");Serial.println(R);
}
void printYaw(float Ya){
  Serial.print("Yaw");Serial.println(Ya);
}


unsigned long manualTimer = 0;
unsigned long previousMillis=0;

void setup() {
  setupTransmitter();
  
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
  
}
void sendShit(int val){
  //
  //Setters Setup
  //
  switch(val){
    case 0:
      myTrans.setAltitudeReference(100.0);
      break;
    case 1:
      myTrans.setAltitudeTolerance(50.0);
      break;
    case 2:
      myTrans.setAltitudeKp(100);
      break;
    case 3:
      myTrans.setAltitudeKi(50);
      break;
    case 4:
      myTrans.setAltitudeBase(25);
      break;
    case 5:
      myTrans.setXPositionReference(100.0);
      break;
    case 6:
      myTrans.setXPositionTolerance(50.0);
      break;
    case 7:
      myTrans.setXPositionKp(100);
      break;
    case 8:
      myTrans.setXPositionKi(50);
      break;
    case 9:
      myTrans.setXPositionKd(25);
      break;
    case 10:
      myTrans.setYPositionReference(100.0);
      break;
    case 11:
      myTrans.setYPositionTolerance(50.0);
      break;
    case 12:
      myTrans.setYPositionKp(100);
      break;
    case 13:
      myTrans.setYPositionKi(50);
      break;
    case 14:
      myTrans.setYPositionKd(50);
      break;
  }  
}

long timer = 0;
int counter = 0;
int switcher = 0;
void loop() {
  
  
  if(counter >= 300){
    
    if(switcher <=14)
      sendShit(switcher);
      
    if(switcher == 0){
      myTrans.getThrottle();
      switcher = 1;
    }else if(switcher == 1){
      myTrans.getAileron();
      switcher = 2;
    }else if(switcher == 2){
      myTrans.getElevator();
      switcher = 3;
    }else if(switcher == 3){
      myTrans.getYaw();
      switcher = 4;
    }else if(switcher == 4){
      myTrans.getPitchAngle();
      switcher = 5;
    }else if(switcher == 5){
      myTrans.getRollAngle();
      switcher = 6;
    }else if(switcher == 6){
      myTrans.getAltitudeKp();
      switcher = 7;
    }else if(switcher == 7){
      myTrans.getAltitudeKi();
      switcher = 8;
    }else if(switcher == 8){
      myTrans.getAltitudeBase();
      switcher = 9;
    }else if(switcher == 9){
      myTrans.getAltitudeReference();
      switcher = 10;
    }else if(switcher == 10){
      myTrans.getAltitudeTolerance();
      switcher = 11;
    }else if(switcher == 11){
      myTrans.getXPositionKp();
      switcher = 12;
    }else if(switcher == 12){
      myTrans.getXPositionKi();
      switcher = 13;
    }else if(switcher == 13){
      myTrans.getXPositionKd();
      switcher = 14;
    }else if(switcher == 14){
      myTrans.getXPositionReference();
      switcher = 15;
    }else if(switcher == 15){
      myTrans.getXPositionTolerance();
      switcher = 16;
    }else if(switcher == 16){
      myTrans.getYPositionKp();
      switcher = 17;
    }else if(switcher == 17){
      myTrans.getYPositionKi();
      switcher = 18;
    }else if(switcher == 18){
      myTrans.getYPositionKd();
      switcher = 19;
    }else if(switcher == 19){
      myTrans.getYPositionReference();
      switcher = 20;
    }else if(switcher == 20){
      myTrans.getYPositionTolerance();
      switcher = 0;
    }
    counter = 0;
  }
  counter++;

  //process Callbacks
  myTrans.listen();
  
}











