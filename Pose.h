
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
  void getOrientation();
  void longitudeCallback(void (*cb)(float));
  void latitudeCallback(void (*cb)(float));
  void altitudeCallback(void (*cb)(float));
  void satelliteCallback(void (*cb)(int16_t));
  void speedCallback(void (*cb)(float));
  void orientationCallback(void (*cb)(float));
  float getLatitudeSync();
  float getLongitudeSync();
  float getAltitudeSync();
  int16_t getSatellitesSync();
  float getSpeedSync();
  float getOrientationSync();
};

#endif // NVPose_h
