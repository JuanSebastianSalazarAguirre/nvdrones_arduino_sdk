#ifndef __ArduinoSDK__Pose__
#define __ArduinoSDK__Pose__

#import "SerialIO.h"
#import "Callback.h"
#import "ResponseHandler.h"

class Pose
/**
 
 The pose class allow you to gather from one place all the data related to position. 
 In this case we are talking about GPS location and compass orientation.
 
 */
{
private:
   //\cond
  SerialIO *serialIO;
  Callback *callbacks;
  ResponseHandler *responseHandler;
  //\endcond
public:
  //\cond
  Pose();
  
  Pose(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler);
  //\endcond

  /**
       
  This sends only one message to request all of the POSE data. 
  The Extender will reply with 6 messages containing Long, Lat, number of Sat, Speed, Orientation, and Altitude
       
  */
  void getGPSData();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Longitude
  The response gets handled through the callback function longitudeCallback().
       
  */
  void getLongitude();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Latitude.
  The response gets handled through the callback function latitudeCallback().
       
  */
  void getLatitude();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Altitude.
  The response gets handled through the callback function altitudeCallback().
       
  */
  void getAltitude();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the current number of satellites the vehicles
  GPS is locked onto.
  The response gets handled through the callback function satelliteCallback().
  The more satellites you have the higher the accuracy is.  There needs to be a minimum of 4 satellites to get a lock.  
       
  */
  void getSatellites();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Land Speed given by the GPS.
  The response gets handled through the callback function speedCallback().
       
  */
  void getSpeed();
  
  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Orientation given by the Compass.
  The response gets handled through the callback function orientationCallback().
       
  */
  void getOrientation();
  
  /**
       
  Callback function which passes longitude information to the function that it is passed.
       
  */
  void longitudeCallback(void (*cb)(float));
  
  /**
       
  Callback function which passes latitude information to the function that it is passed.
       
  */
  void latitudeCallback(void (*cb)(float));
  
  /**
       
  Callback function which passes altitude information to the function that it is passed.
       
  */
  void altitudeCallback(void (*cb)(float));
  
  /**
       
  Callback function which passes number of locked satellites to the function that it is passed.
       
  */
  void satelliteCallback(void (*cb)(int16_t));
  
  /**
       
  Callback function which passes speed information to the function that it is passed.
       
  */
  void speedCallback(void (*cb)(float));
  
  /**
       
  Callback function which passes orientation information to the function that it is passed
       
  */
  void orientationCallback(void (*cb)(float));

  /**

  Synchronous version of `getLatitude`.

  */
  float getLatitudeSync();

  /**

  Synchronous version of `getLongitude`.

  */
  float getLongitudeSync();

  /**

  Synchronous version of `getAltitude`.

  */
  float getAltitudeSync();

  /**

  Synchronous version of `getSatellites`.

  */
  int16_t getSatellitesSync();

  /**

  Synchronous version of `getSpeed`.

  */
  float getSpeedSync();

  /**

  Synchronous version of `getOrientation`.

  */
  float getOrientationSync();
};

#endif /* defined __ArduinoSDK__Pose__ */
