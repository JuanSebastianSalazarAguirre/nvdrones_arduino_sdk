#ifndef __ArduinoSDK__Pose__
#define __ArduinoSDK__Pose__

#import "SerialIO.h"
#import "Callback.h"

class Pose
/**
 
 The pose class allow you to gather from one place all the data related to position. 
 In this case we are talking about GPS location and compass orientation.
 
 */
{
  private:
     //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    Pose();
    
    Pose(SerialIO *rcTservice, Callback *callback);
    //\endcond

    /**
         
    This sends only one message to request all of the POSE data. 
    The Extender will reply with 6 messages containing Long, Lat, number of Sat, Speed, Orientation, and Altitude
         
    */
    void getGPSData();
    
    /**
         
    Sends a request to the Extender asking for it to reply with the vehicles current Longitude
    The response gets handled through the callback function longitudeCallbackCallback().
         
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
    void setLongitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes latitude information to the function that it is passed.
         
    */
    void setLatitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes altitude information to the function that it is passed.
         
    */
    void setAltitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes number of locked satellites to the function that it is passed.
         
    */
    void setSatelliteCallback(void (*function)(byte));
    
    /**
         
    Callback function which passes speed information to the function that it is passed.
         
    */
    void setSpeedCallback(void (*function)(float));
    
    /**
         
    Callback function which passes orientation information to the function that it is passed
         
    */
    void setOrientationCallback(void (*function)(float));
};

#endif /* defined __ArduinoSDK__Pose__ */