#include "ResponseHandler.h"
#include "IDs.h"
#include "Utils.h"

#if NV_DEBUG
#define LOG(x,y) Serial.print(x); Serial.println(y);
#else
#define LOG(x,y)
#endif

using namespace Utils;

ResponseHandler::ResponseHandler() {};

ResponseHandler::ResponseHandler(SerialIO *serialIO, IncomingPacketReader *incomingPacketReader, Callback *callbacks, Vitals *vitals):
_serialIO(serialIO),
_callbacks(callbacks),
_vitals(vitals),
_incomingPacketReader(incomingPacketReader)
{

}

void ResponseHandler::listen() {
  while (_serialIO->available() > 0) {

    IncomingPacket p = _incomingPacketReader->read();
    
    if (p.isHearbeat()) {
      _vitals->receiveHeartbeat();
      continue;
    }

    if (p.isEmpty()) { continue; }


    switch (p.resourceID) {
      case resourceID::rc:
        if (p.actionID == actionID::getAileron) _callbacks->aileron((int16_t)p.data[0]);
        else if (p.actionID == actionID::getElevator) _callbacks->elevator((int16_t)p.data[0]);
        else if (p.actionID == actionID::getThrottle) _callbacks->throttle((int16_t)(int8_t)p.data[0]);
        else if (p.actionID == actionID::getRudder) _callbacks->rudder((int16_t)p.data[0]);
        else if (p.actionID == actionID::getFlightMode) _callbacks->flightMode((int16_t)p.data[0]);
        break;
      case resourceID::pose:
        if (p.actionID == actionID::getLatitude) _callbacks->latitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getLongitude) _callbacks->longitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getAltitude) _callbacks->altitude(dataToFloat(p.data));
        else if (p.actionID == actionID::getSatellites) _callbacks->satellite((int16_t)p.data[0]);
        else if (p.actionID == actionID::getSpeed) _callbacks->speed(dataToFloat(p.data));
        else if (p.actionID == actionID::getYaw) _callbacks->yaw(dataToFloat(p.data));
        else if (p.actionID == actionID::getPitchAngle) _callbacks->pitchAngle(dataToFloat(p.data));
        else if (p.actionID == actionID::getRollAngle) _callbacks->rollAngle(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawAccelX) _callbacks->rawAccelX(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawAccelY) _callbacks->rawAccelY(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawGyroZ) _callbacks->rawGyroZ(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawGyroX) _callbacks->rawGyroX(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawGyroY) _callbacks->rawGyroY(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawAccelZ) _callbacks->rawAccelZ(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawMagnetometerX) _callbacks->rawMagnetometerX(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawMagnetometerY) _callbacks->rawMagnetometerY(dataToFloat(p.data));
        else if (p.actionID == actionID::getRawMagnetometerZ) _callbacks->rawMagnetometerZ(dataToFloat(p.data));
        else if (p.actionID == actionID::getAltitudeBarometer) _callbacks->altitudeBarometer(dataToFloat(p.data));
        break;
      case resourceID::i2c:
        if (p.actionID == actionID::readI2C) _callbacks->i2cRead(p.data[0]);
        break;
      case resourceID::digitalRead:
        if (p.actionID == 1) _callbacks->digitalRead1(p.data[0]);
        if (p.actionID == 2) _callbacks->digitalRead2(p.data[0]);
        if (p.actionID == 3) _callbacks->digitalRead3(p.data[0]);
        if (p.actionID == 4) _callbacks->digitalRead4(p.data[0]);
        if (p.actionID == 5) _callbacks->digitalRead5(p.data[0]);
        if (p.actionID == 6) _callbacks->digitalRead6(p.data[0]);
        if (p.actionID == 7) _callbacks->digitalRead7(p.data[0]);
        if (p.actionID == 8) _callbacks->digitalRead8(p.data[0]);
        if (p.actionID == 9) _callbacks->digitalRead9(p.data[0]);
        if (p.actionID == 10) _callbacks->digitalRead10(p.data[0]);
        break;
      case resourceID::analogRead:
        if (p.actionID == 1) _callbacks->analogRead1(p.data[0]);
        if (p.actionID == 2) _callbacks->analogRead2(p.data[0]);
        if (p.actionID == 3) _callbacks->analogRead3(p.data[0]);
        if (p.actionID == 4) _callbacks->analogRead4(p.data[0]);
        break;
      case resourceID::pulseIn:
        if (p.actionID == 1) _callbacks->pulseIn1(dataToLong(p.data));
        if (p.actionID == 2) _callbacks->pulseIn2(dataToLong(p.data));
        if (p.actionID == 3) _callbacks->pulseIn3(dataToLong(p.data));
        if (p.actionID == 4) _callbacks->pulseIn4(dataToLong(p.data));
        if (p.actionID == 5) _callbacks->pulseIn5(dataToLong(p.data));
        if (p.actionID == 6) _callbacks->pulseIn6(dataToLong(p.data));
        if (p.actionID == 7) _callbacks->pulseIn7(dataToLong(p.data));
        if (p.actionID == 8) _callbacks->pulseIn8(dataToLong(p.data));
        if (p.actionID == 9) _callbacks->pulseIn9(dataToLong(p.data));
        if (p.actionID == 10) _callbacks->pulseIn10(dataToLong(p.data));
        break;
      case resourceID::interrupt:
        if (p.actionID == actionID::interrupt0) _callbacks->interrupt0();
        if (p.actionID == actionID::interrupt1) _callbacks->interrupt1();
        break;
      case resourceID::vitals:
        if (p.actionID == actionID::getVoltage) _callbacks->voltage(p.data[0]);
        if (p.actionID == actionID::getSignalStrength) _callbacks->signalStrength(p.data[0]);
        break;
      case resourceID::error:
        _callbacks->errorHandler(p.actionID);
        break;
        case resourceID::autopilot:
        if(p.actionID == actionID::manualTakeover) _callbacks->takeover(dataToFloat(p.data)); 
        if(p.actionID == actionID::indoorHover) _callbacks->indoorHover(dataToFloat(p.data));
        if(p.actionID == actionID::outdoorHover)  _callbacks->outdoorHover(dataToFloat(p.data));
        if(p.actionID == actionID::getSonarAltitude) _callbacks->sonarAltitude(dataToFloat(p.data));
        if(p.actionID == actionID::getSonarPositionX) _callbacks->sonarXPosition(dataToFloat(p.data));
        if(p.actionID == actionID::getSonarPositionY) _callbacks->sonarYPosition(dataToFloat(p.data));
        if(p.actionID == actionID::getAltitudeKp) _callbacks->getAltitudeKp(dataToFloat(p.data));
        if(p.actionID == actionID::getAltitudeKi) _callbacks->getAltitudeKi(dataToFloat(p.data));
        if(p.actionID == actionID::getAltitudeBase) _callbacks->getAltitudeBase(dataToFloat(p.data));
        if(p.actionID == actionID::getAltitudeReference) _callbacks->getAltitudeReference(dataToFloat(p.data));
        if(p.actionID == actionID::getAltitudeTolerance) _callbacks->getAltitudeTolerance(dataToFloat(p.data));
        if(p.actionID == actionID::getXPositionKp) _callbacks->getXPositionKp(dataToFloat(p.data));
        if(p.actionID == actionID::getXPositionKi) _callbacks->getXPositionKi(dataToFloat(p.data));
        if(p.actionID == actionID::getXPositionKd) _callbacks->getXPositionKd(dataToFloat(p.data));
        if(p.actionID == actionID::getXPositionReference) _callbacks->getXPositionReference(dataToFloat(p.data));
        if(p.actionID == actionID::getXPositionTolerance) _callbacks->getXPositionTolerance(dataToFloat(p.data));
        if(p.actionID == actionID::getYPositionKp) _callbacks->getYPositionKp(dataToFloat(p.data));
        if(p.actionID == actionID::getYPositionKi) _callbacks->getYPositionKi(dataToFloat(p.data));
        if(p.actionID == actionID::getYPositionKd) _callbacks->getYPositionKd(dataToFloat(p.data));
        if(p.actionID == actionID::getYPositionReference) _callbacks->getYPositionReference(dataToFloat(p.data));
        if(p.actionID == actionID::getYPositionTolerance) _callbacks->getYPositionTolerance(dataToFloat(p.data));
        break;
      default:
        LOG("Invalid resourceID ", p.resourceID);
        break;
    }
  }
}