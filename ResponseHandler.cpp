#include "ResponseHandler.h"
#include "IDs.h"
#include "Utils.h"

//#define NV_DEBUG 1   //uncomment to turn log statments on
#if NV_DEBUG
#define LOG(x,y) Serial.print(x); Serial.println(y);
#else
#define LOG(x,y)
#endif

using namespace Utils;

ResponseHandler::ResponseHandler() {};

ResponseHandler::ResponseHandler(SerialIO *_serialIO, Callback *_callbacks):
serialIO(_serialIO),
callbacks(_callbacks)
{

}

void ResponseHandler::listen() {
  while (serialIO->available() > 0) {
    IncomingPacket p = tryToReadNextPacket();
    if (!p.isValid()) { continue; }

    switch (p.resourceID) {
      case resourceID::rc:
        if (p.actionID == actionID::getAileron) callbacks->aileron((int16_t)p.data[0]);
        else if (p.actionID == actionID::getElevator) callbacks->elevator((int16_t)p.data[0]);
        else if (p.actionID == actionID::getThrottle) callbacks->throttle((int16_t)(int8_t)p.data[0]);
        else if (p.actionID == actionID::getRudder) callbacks->rudder((int16_t)p.data[0]);
        else if (p.actionID == actionID::getFlightMode) callbacks->flightMode((int16_t)p.data[0]); 
        break;
      case resourceID::pose:
        if (p.actionID == actionID::getLatitude) callbacks->latitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getLongitude) callbacks->longitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getAltitude) callbacks->altitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getSatellites) callbacks->satellite((int16_t)p.data[0]);
        else if (p.actionID == actionID::getSpeed) callbacks->speed(dataToFloat(p.data));
        else if (p.actionID == actionID::getYaw) callbacks->orientation(dataToFloat(p.data));
        break;
      case resourceID::i2c:
        if (p.actionID == actionID::readI2c) callbacks->i2cRead(p.data[0]);
        break;
      case resourceID::digitalRead:
        if (p.actionID == 1) callbacks->digitalRead1(p.data);
        if (p.actionID == 2) callbacks->digitalRead2(p.data);
        if (p.actionID == 3) callbacks->digitalRead3(p.data);
        if (p.actionID == 4) callbacks->digitalRead4(p.data);
        if (p.actionID == 5) callbacks->digitalRead5(p.data);
        if (p.actionID == 6) callbacks->digitalRead6(p.data);
        if (p.actionID == 7) callbacks->digitalRead7(p.data);
        if (p.actionID == 8) callbacks->digitalRead8(p.data);
        if (p.actionID == 9) callbacks->digitalRead9(p.data);
        if (p.actionID == 10) callbacks->digitalRead10(p.data);
        break;
      case resourceID::analogRead:
        if (p.actionID == 1) callbacks->analogRead1(p.data);
        if (p.actionID == 2) callbacks->analogRead2(p.data);
        if (p.actionID == 3) callbacks->analogRead3(p.data);
        if (p.actionID == 4) callbacks->analogRead4(p.data);
        break;
      case resourceID::pulseIn:
        if (p.actionID == 1) callbacks->pulseIn1(dataToLong(p.data));
        if (p.actionID == 2) callbacks->pulseIn2(dataToLong(p.data));
        if (p.actionID == 3) callbacks->pulseIn3(dataToLong(p.data));
        if (p.actionID == 4) callbacks->pulseIn4(dataToLong(p.data));
        if (p.actionID == 5) callbacks->pulseIn5(dataToLong(p.data));
        if (p.actionID == 6) callbacks->pulseIn6(dataToLong(p.data));
        if (p.actionID == 7) callbacks->pulseIn7(dataToLong(p.data));
        if (p.actionID == 8) callbacks->pulseIn8(dataToLong(p.data));
        if (p.actionID == 9) callbacks->pulseIn9(dataToLong(p.data));
        if (p.actionID == 10) callbacks->pulseIn10(dataToLong(p.data));
        break;
      case resourceID::interrupt:
        if (p.actionID == actionID::interrupt0) callbacks->interrupt0();
        if (p.actionID == actionID::interrupt1) callbacks->interrupt1();
        break;
      default:
        LOG("Invalid resourceID ", p.resourceID);
        break;
    }
  }
}

IncomingPacket ResponseHandler::tryToReadNextPacket() {
  int maxIterations = 250;
  IncomingPacket errorPacket(0,0,0,0);
  if (serialIO->available() == 0) return errorPacket;
  if (serialIO->read() != '$') return errorPacket;

  int16_t length = serialIO->multipleRead(maxIterations);
  LOG("length: ", length);
  if (length == -1) {
    Serial.println("Error reading length");
    return errorPacket;
  }
  int16_t resourceID = serialIO->multipleRead(maxIterations);
  LOG("resourceID: ", resourceID);
  if (resourceID == -1) {
    Serial.println("Error reading resourceID");
    return errorPacket;
  }
  int16_t actionID = serialIO->multipleRead(maxIterations);
  LOG("actionID: ", actionID);
  if (actionID == -1) {
    Serial.println("Error reading actionID");
    return errorPacket;
  }

  uint8_t data[length];
  for (int i=0; i<length; ++i) {
    data[i] = serialIO->multipleRead(maxIterations);
    LOG("data: ", data[i]);
  }
  // TODO: add error checking.

  int16_t checksum = serialIO->multipleRead(maxIterations);
  if (checksum == -1) { /* TODO: Error handling */ }
  LOG("checksum: ", checksum);

  int16_t calculatedSum = length + actionID + resourceID;
  for (int i=0; i<length; ++i) {
    calculatedSum += data[i];
  }
  LOG("calculated checksum: ", calculatedSum%256);

  if (checksum != calculatedSum%256) {
    Serial.println("Failed checksum");
    return errorPacket;
  }

  IncomingPacket result(resourceID, actionID, data, length);
  LOG("Is packet valid? ", result.isValid());
  return result;
}
