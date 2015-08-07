
#ifndef NVPose_h
#define NVPose_h

#import "SerialIO.h"
#import "Callback.h"
#import "IncomingPacketReader.h"

class Pose
/**
 
 The pose class allow you to gather from one place all the data related to position. 
 In this case we are talking about GPS location and compass orientation.
 
 */
{
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;
public:
  Pose();
  Pose(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader);

  void getGPSData();
  void getLongitude();
  void getLatitude();
  void getAltitude();
  void getSatellites();
  void getSpeed();
  void getYaw();
  void getPitchAngle();
  void getRollAngle();
  void getRawAccelX();
  void getRawAccelY();
  void getRawAccelZ();
  void getRawGyroX();
  void getRawGyroY();
  void getRawGyroZ();
  void getRawMagnetometerX();
  void getRawMagnetometerY();
  void getRawMagnetometerZ();
  void getAltitudeBarometer();
  void longitudeCallback(void (*cb)(float));
  void latitudeCallback(void (*cb)(float));
  void altitudeCallback(void (*cb)(float));
  void satelliteCallback(void (*cb)(int16_t));
  void speedCallback(void (*cb)(float));
  void yawCallback(void (*cb)(float));
  void pitchAngleCallback(void (*cb)(float));
  void rollAngleCallback(void (*cb)(float));
  void rawAccelXCallback(void (*cb)(float));
  void rawAccelYCallback(void (*cb)(float));
  void rawAccelZCallback(void (*cb)(float));
  void rawGyroXCallback(void (*cb)(float));
  void rawGyroYCallback(void (*cb)(float));
  void rawGyroZCallback(void (*cb)(float));
  void rawMagnetometerXCallback(void (*cb)(float));
  void rawMagnetometerYCallback(void (*cb)(float));
  void rawMagnetometerZCallback(void (*cb)(float));
  void altitudeBarometerCallback(void(*cb)(float));
  float getLatitudeSync();
  float getLongitudeSync();
  float getAltitudeSync();
  int16_t getSatellitesSync();
  float getSpeedSync();
  float getYawSync();
  float getPitchAngleSync();
  float getRollAngleSync();
  float getRawAccelXSync();
  float getRawAccelYSync();
  float getRawAccelZSync();
  float getRawGyroXSync();
  float getRawGyroYSync();
  float getRawGyroZSync();
  float getRawMagnetometerXSync();
  float getRawMagnetometerYSync();
  float getRawMagnetometerZSync();
  float getAltitudeBarometerSync();
};

#endif // NVPose_h
