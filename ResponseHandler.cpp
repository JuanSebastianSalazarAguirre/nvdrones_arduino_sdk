#include "ResponseHandler.h"

ResponseHandler::ResponseHandler(){};

ResponseHandler::ResponseHandler(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

float ResponseHandler::dataToFloat(byte data[]) {
  union u_tag {
    byte b[4];
    float data_float;
  } u;
  u.b[0] = byte(data[0]);
  u.b[1] = byte(data[1]);
  u.b[2] = byte(data[2]);
  u.b[3] = byte(data[3]);
  return u.data_float;
}

long ResponseHandler::dataToLong(byte data[]) {
  long data_long = 0;
  data_long = data[0] << 8;
  data_long = (data_long + data[1]) << 8;
  data_long = (data_long + data[2]) << 8;
  data_long = data_long + data[3]; 
  return data_long;
}

void ResponseHandler::responseHandler() {
  uint8_t buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
  uint8_t length = 0;
  uint8_t resourceID = 0;
  uint8_t actionID = 0;
  uint16_t datasum = 0;
  uint8_t i = 0;
  while (service->serialAvailable() > 0) {
    if (service->serialRead() == '$') {  //this is the start of a Data Packet
      while((actionID == -1 || resourceID == -1 || length == -1 ) && (i < 250)) {
        uint8_t s = service->serialRead();
        if (length == -1 && s != -1) {
          length = s;         //set the length
        } else if (resourceID == -1 && s != -1) {
          resourceID = s;     //set the resourceID
        } else if (actionID == -1 && s != -1) {
          actionID = s;       //set the actionID
        }
        i++;
      }
      uint8_t data[length+1]; // create a buffer to store the data and checksum info 
      i = 0;    //reset the i to zero to begin a new iteration process
      // TODO: explain `80` better
      for (int iterate = 0; iterate < 80*length; iterate++) { //iterate for certain amount of times based on number of desired bytes
        if (i == length+1) {               //if all desired bytes have been read then break
          break;
        }
        uint8_t s = service->serialRead();
        if (s != -1) {
          data[i] = s;
          i++;
        }
      }
      for(int i=0;i<length;i++){
        datasum = datasum+data[i];
      }
      
      if (data[length] == (length+resourceID+actionID+datasum)%256) {  //check the checksum
        //route the data to the appropriate place here
        switch(resourceID) {
          case 1:
            //Flight_Setup
            Serial.println("got Flight Setup");
            break;
          case 2:
            //Manual_Controls
            break;
          case 3:
            //Mission_Planner
            break;
          case 4:
            //Battery_Status
            break;
          case 5:
            //Pin_Mode
            break;
          case 6:
            //Digital_Write
            break;
          case 7:
            //Analog_Write
            break;
          case 8:
            //Digital_Read
            if(actionID == 1)
              myCallback->digitalRead1(data[0]);
            if(actionID == 2)
              myCallback->digitalRead2(data[0]);
            if(actionID == 3)
              myCallback->digitalRead3(data[0]);
            if(actionID == 4)
              myCallback->digitalRead4(data[0]);
            if(actionID == 5)
              myCallback->digitalRead5(data[0]);
            if(actionID == 6)
              myCallback->digitalRead6(data[0]);
            if(actionID == 7)
              myCallback->digitalRead7(data[0]);
            if(actionID == 8)
              myCallback->digitalRead8(data[0]);
            if(actionID == 9)
              myCallback->digitalRead9(data[0]);
            if(actionID == 10)
              myCallback->digitalRead10(data[0]);
            break;
          case 9:
            //Pose_Data
            if(actionID == 2)
              myCallback->longitude(dataToFloat(data));
            if(actionID == 3)
              myCallback->latitude(dataToFloat(data));
            if(actionID == 4)
              myCallback->altitude(dataToFloat(data));
            if(actionID == 5)
              myCallback->satellite(data[0]);
            if(actionID == 6)
              myCallback->speed(dataToFloat(data));
            if(actionID == 7)
              myCallback->orientation(dataToFloat(data));
            break;
          case 10:
            //SPI_Communication
            break;
          case 11:
            //I2C_Communication
            break;
          case 12:
            //Uart_Communication
            break;
          case 13:
            //Transmission_And_Security
            break;
          case 14:
            //Scripting
            break;
          case 15:
            //SimpleAP
            break;
          case 16:
            if(actionID == 1)
              myCallback->pulseIn1(dataToLong(data));
            if(actionID == 2)
              myCallback->pulseIn2(dataToLong(data));
            if(actionID == 3)
              myCallback->pulseIn3(dataToLong(data));
            if(actionID == 4)
              myCallback->pulseIn4(dataToLong(data));
            if(actionID == 5)
              myCallback->pulseIn5(dataToLong(data));
            if(actionID == 6)
              myCallback->pulseIn6(dataToLong(data));
            if(actionID == 7)
              myCallback->pulseIn7(dataToLong(data));
            if(actionID == 8)
              myCallback->pulseIn8(dataToLong(data));
            if(actionID == 9)
              myCallback->pulseIn9(dataToLong(data));
            if(actionID == 10)
              myCallback->pulseIn10(dataToLong(data));
            break;
          case 17:
            if(actionID == 1)
              myCallback->analogRead1(data[0]);
            if(actionID == 2)
              myCallback->analogRead2(data[0]);
            if(actionID == 3)
              myCallback->analogRead3(data[0]);
            if(actionID == 4)
              myCallback->analogRead4(data[0]);
            break;
      }//end of data packet router
    }//end of checksum
    else
      Serial.println("checksum failed");
    }//end of if Serial available
  }//end of Serial.read
}