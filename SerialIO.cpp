#include "SerialIO.h"

SerialIO::SerialIO() {}

SerialIO::SerialIO(int txPin , int rxPin) {
  softwareSerial = SoftwareSerial(txPin, rxPin);
  softwareSerial.begin(57600);
  selectedSerialPort = swSerialPort;
}

SerialIO::SerialIO(SerialPort serialPort) {
  selectedSerialPort = serialPort;
  switch(serialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        Serial.begin(57600);
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        Serial1.begin(57600);
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        Serial2.begin(57600);
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        Serial3.begin(57600);
      #endif
      break;
    default:
      Serial.println("Error: bad argument to SerialIO constructor");
      break;
  }
}

void SerialIO::write(uint8_t data) {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        Serial.write(data);
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        Serial1.write(data);
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        Serial2.write(data);
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        Serial3.write(data);
      #endif
      break;
    case swSerialPort:
      softwareSerial.write(data);
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}

int16_t SerialIO::read() {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        return Serial.read();
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        return Serial1.read();
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        return Serial2.read();
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        return Serial3.read();
      #endif
      break;
    case swSerialPort:
      return softwareSerial.read();
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
  return -1;
}

// TODO: rename to something more intelligent
int16_t SerialIO::multipleRead(uint16_t count) {
  int16_t result = -1;
  for (uint16_t i=0; i<count; ++i) {
    result = read();
    if (result != -1) break;
  }
  return result;
}

bool SerialIO::available() {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        return Serial.available();
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        return Serial1.available();
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        return Serial2.available();
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        return Serial3.available();
      #endif
      break;
    case swSerialPort:
      return softwareSerial.available();
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
  return false;
}

void SerialIO::sendPacket(uint8_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(1);
  write(resourceID);
  write(actionID);
  write(data);
  write((1+resourceID+actionID+data)%256);
}

void SerialIO::sendPacket(int8_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(1);
  write(byte(resourceID));
  write(byte(actionID));
  write(data);
  // Cast data because the firmware interprets packets as unsigned for calculating checksum.
  write((1+resourceID+actionID+(uint8_t)data)%256);
}

void SerialIO::sendPacket(int16_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(2);
  write(byte(resourceID));
  write(byte(actionID));
  write(highByte(data));
  write(lowByte(data));
  write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
}

void SerialIO::sendPacket(float data, uint8_t resourceID, uint8_t actionID) {
  union u_tag {
    uint8_t b[4];
    float dataFloat;
  } u;
  u.dataFloat = data;

  write('$');
  write(4);
  write(byte(resourceID));
  write(byte(actionID));
  write(u.b[0]);
  write(u.b[1]);
  write(u.b[2]);
  write(u.b[3]);
  write((4+resourceID+actionID+u.b[0]+u.b[1]+u.b[2]+u.b[3])%256);
}

void SerialIO::sendPacket(uint8_t *data, uint8_t length, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(length);
  write(resourceID);
  write(actionID);
  uint8_t sum = length+resourceID+actionID;
  for (uint8_t i=0; i<length; i++) {
    write(data[i]);
    sum += data[i];
  }
  write(sum%256);
}

void SerialIO::print(String data) {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        Serial.print(data);
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        Serial1.print(data);
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        Serial2.print(data);
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        Serial3.print(data);
      #endif
      break;
    case swSerialPort:
      softwareSerial.print(data);
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}
