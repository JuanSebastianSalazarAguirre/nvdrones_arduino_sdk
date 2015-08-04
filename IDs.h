
#ifndef IDs_h
#define IDs_h

namespace resourceID {
  const int16_t flightSetup = 1;
  const int16_t rc = 2;
  const int16_t autopilot = 3;
  const int16_t pinMode = 5;
  const int16_t digitalWrite = 6;
  const int16_t analogWrite = 7;
  const int16_t digitalRead = 8;
  const int16_t pose = 9;
  const int16_t spi = 10;
  const int16_t i2c = 11;
  const int16_t interrupt = 14;
  const int16_t pulseIn = 16;
  const int16_t analogRead = 17;
  const int16_t servo = 18;
  const int16_t vitals = 19;
  const int16_t error = 20;
  const int16_t transmitterSupport = 21;
}

namespace actionID {

  // RC
  const int16_t setRudder = 1;
  const int16_t setThrottle = 2;
  const int16_t setElevator = 3;
  const int16_t setAileron = 4;
  const int16_t setFlightMode = 5;
  const int16_t getAileron = 6;
  const int16_t getElevator = 7;
  const int16_t getThrottle = 8;
  const int16_t getRudder = 9;
  const int16_t getFlightMode = 10;
  const int16_t setAileronElevatorRudderThrottle = 11;
  const int16_t setModeAutopilot = 12;

  //Autopilot
  const int16_t manualTakeover = 47;
  const int16_t indoorHover = 48;
  const int16_t outdoorHover = 49;
  const int16_t getSonarAltitude = 50;
  const int16_t getSonarPositionX = 51;
  const int16_t getSonarPositionY = 52;
  const int16_t setAltitudeKp = 53;
  const int16_t setAltitudeKi = 54;
  const int16_t setAltitudeBase = 55;
  const int16_t getAltitudeKp = 56;
  const int16_t getAltitudeKi = 57;
  const int16_t getAltitudeBase = 58;
  const int16_t setAltitudeReference = 59;
  const int16_t setAltitudeTolerance = 60;
  const int16_t getAltitudeReference = 61;
  const int16_t getAltitudeTolerance = 62;
  const int16_t setXPositionKp = 63;
  const int16_t setXPositionKi = 64;
  const int16_t setXPositionKd = 65;
  const int16_t getXPositionKp = 66;
  const int16_t getXPositionKi = 67;
  const int16_t getXPositionKd = 68;
  const int16_t setXPositionReference = 69; 
  const int16_t getXPositionReference = 70;
  const int16_t setXPositionTolerance = 71; 
  const int16_t getXPositionTolerance = 72;
  const int16_t setYPositionKp = 73; 
  const int16_t setYPositionKi = 74;
  const int16_t setYPositionKd = 75;
  const int16_t getYPositionKp = 76;
  const int16_t getYPositionKi = 77;
  const int16_t getYPositionKd = 78;
  const int16_t setYPositionReference = 79;
  const int16_t getYPositionReference = 80;
  const int16_t setYPositionTolerance = 81;
  const int16_t getYPositionTolerance = 82;

  // Pose
  const int16_t getAllPose = 1;
  const int16_t getLatitude = 2;
  const int16_t getLongitude = 3;
  const int16_t getAltitude = 4;
  const int16_t getSatellites = 5;
  const int16_t getSpeed = 6;
  const int16_t getYaw = 7;
  const int16_t getPitchAngle = 8;
  const int16_t getRollAngle = 9;

  // I2C
  const int16_t setI2CDeviceAddress = 1;
  const int16_t i2cRequestFrom = 2;
  const int16_t beginI2CTransmission = 3;
  const int16_t endI2CTransmission = 4;
  const int16_t writeI2C = 5;
  const int16_t readI2C = 6;
  const int16_t i2cAvailable = 7;

  // Servo
  const int16_t attachServo1 = 1;
  const int16_t writeServo1 = 2;
  const int16_t detachServo1 = 3;
  const int16_t attachServo2 = 4;
  const int16_t writeServo2 = 5;
  const int16_t detachServo2 = 6;
  const int16_t attachServo3 = 7;
  const int16_t writeServo3 = 8;
  const int16_t detachServo3 = 9;

  // Interrupt
  const int16_t interrupt0 = 1;
  const int16_t interrupt1 = 2;

  // Vitals
  const int16_t getVoltage = 1;
  const int16_t getSignalStrength = 2;

  // Error
  const int16_t invalidPacketStartByte = 1;
  const int16_t unableToReadPacketLength = 2;
  const int16_t unableToReadPacketResourceID = 3;
  const int16_t unableToReadPacketActionID = 4;
  const int16_t unableToReadPacketData = 5;
  const int16_t unableToReadPacketChecksum = 6;
  const int16_t failedPacketChecksum = 7;
  const int16_t receivedInvalidResourceID = 8;
  const int16_t receivedInvalidActionIDForManualControls = 9;
  const int16_t receivedInvalidActionIDForPose = 10;
  const int16_t receivedInvalidActionIDForI2C = 11;
  const int16_t receivedInvalidActionIDForServo = 12;
  const int16_t receivedInvalidActionIDForVitals = 13;
  const int16_t heartbeatTimeout = 14;
  const int16_t invalidPinForPinMode = 15;
  const int16_t invalidPinForDigitalWrite = 16;
  const int16_t invalidPinForDigitalRead = 17;
  const int16_t invalidPinForPulseIn = 18;
  const int16_t invalidPinForAnalogRead = 19;
  const int16_t invalidPinForAnalogWrite = 20;
  const int16_t invalidPinForServoAttach = 21;
  const int16_t invalidServoForServoAttach = 22;
  const int16_t invalidServoForServoWrite = 23;
  const int16_t invalidServoForServoDetach = 24;
  const int16_t invalidI2CAddress = 25;
  const int16_t unsetI2CAddress = 27;
  const int16_t gpsError = 28;

  //Transmitter Support
  const int16_t setTransmitterSupportState = 1;
  const int16_t getTransmitterSupportState = 2;
}

#endif // IDs_h
