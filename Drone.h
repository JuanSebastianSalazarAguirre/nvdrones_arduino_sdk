/*

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef NVDrone_h
#define NVDrone_h

#include <inttypes.h>
#include <Stream.h>
#include <Arduino.h>
#include "RC.h"
#include "Autopilot.h"
#include "SerialIO.h"
#include "Callback.h"
#include "Pose.h"
#include "GPIO.h"
#include "I2C.h"
#include "ResponseHandler.h"
#include "Vitals.h"
#include "Transmittersupport.h"

/**

This is the class is for interacting with your drone. Controlling your drone is done
through an instance of this class.

#### For example:

Arming the drone. This is released with API v0.3

\htmlonly
<script src="https://gist.github.com/NVminion/715efb3cd4daa05a7852.js"></script>
\endhtmlonly

*/
class Drone 
{
private:
//\cond
  RC _rc;
  SerialIO _serialIO;
  GPIO _gpio;
  IncomingPacketReader _incomingPacketReader;
  Vitals _vitals;
  ResponseHandler _responseHandler;
  I2C _i2c;
  Callback _callback;
  Pose _pose;
  Autopilot _autopilot;
  TransmitterSupport _transmitterSupport;
//\endcond

public:

  /**

  Creates a new Drone instance.

  */
  Drone();

  /**

  Creates a new Drone instance.

  @param serialPort The serial port for the RF device communicating with the drone.

  */
  Drone(SerialPort serialPort);

  /**

  Creates a new Drone instance.

  @param txPin The transmit pin for the RF device communicating with the drone.
  @param rxPin The receive pin for the RF device communicating with the drone.

  */
  Drone(int txPin, int rxPin);


  /**

  Performs the drone's preflight tasks. For example, initialize ensures the Arduino
  is connected to the drone. This method needs to be called in the setup() function
  of your Arduino sketch.

  */
  void initialize();

  /**

  Turns the drone on/off.

  */
  void arm();

  /**

  Performs the funcationality that needs to be called repeatedly (preferably in each iteration of
  your sketch's loop()). For example, this method allows callback functions to handle incoming data.

  */
  void listen();

  /**
       
  Requests the drone's current longitude. The response gets handled through the
  callback function set using longitudeCallback().
       
  */
  void getLongitude();

  /**

  Synchronous version of getLongitude(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's current longtitude, or -1 if timeout.

  */
  float getLongitudeSync();

  /**
       
  Registers the function to be called when the Arduino receives longitude data from the drone.

  @param cb The function to be called with the drone's longitude.
       
  */
  void longitudeCallback(void (*cb)(float));

  /**
       
  Requests the drone's current latitude. The response gets handled through the
  callback function set using latitudeCallback().
       
  */
  void getLatitude();

  /**

  Synchronous version of getLatitude(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's current latitude, or -1 if timeout.

  */
  float getLatitudeSync();

  /**
       
  Registers the function to be called when the Arduino receives latitude data from the drone.

  @param cb The function to be called with the drone's latitude.
       
  */
  void latitudeCallback(void (*cb)(float));

  /**
       
  Requests the drone's current altitude. The response gets handled through the
  callback function set using altitudeCallback().
       
  */
  void getAltitude();

  /**

  Synchronous version of getAltitude(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's current altitude, or -1 if timeout.

  */
  float getAltitudeSync();

  /**
       
  Registers the function to be called when the Arduino receives altitude data from the drone.

  @param cb The function to be called with the drone's altitude.
       
  */
  void altitudeCallback(void (*cb)(float));

  /**

  Requests the number of satellites the drone's GPS is locked onto. Being locked onto more satellites is
  correlated to higher GPS accuracy. The minimum number of satellites to get GPS data is 4. The response
  gets handled through the callback function set using satellitesCallback().
       
  */
  void getSatellites();

  /**

  Synchronous version of getSatellites(). Waits until a response is received, timing out
  after 1 second.

  @return The number of satellites the drone's GPS is locked onto, or -1 if timeout.

  */
  int getSatellitesSync();

  /**
       
  Registers the function to be called when the Arduino receives the number of connected GPS satellites from the drone.

  @param cb The function to be called with the drone's satellite count.
       
  */
  void satelliteCallback(void (*cb)(int));

  /**
       
  Requests the drone's current speed as calculated by the GPS unit. The response gets handled
  through the callback function set using speedCallback().
       
  */
  void getSpeed();

  /**

  Synchronous version of getSpeed(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's current speed, or -1 if timeout.


  */
  float getSpeedSync();

  /**

  Registers the function to be called when the Arduino receives speed data from the drone.

  @param cb The function to be called with the drone's speed.

  */
  void speedCallback(void (*cb)(float));

  /**

  Requests the drone's current yaw from the onboard compass. The response gets handled
  through the callback function set using yawCallback().

  */
  void getYaw();

  /**

  Synchronous version of getYaw().  Waits until a response is received, timing out
  after 1 second.

  @return The drone's current yaw, or -1 if timeout.

  */
  float getYawSync();

  /**

  Registers the function to be called when the Arduino receives yaw data from the drone.

  @param cb The function to be called with the drone's yaw.

  */
  void yawCallback(void (*cb)(float));

  /**

  Requests the drone's current pitch angle. The response gets handled
  through the callback function set using pitchAngleCallback().

  */
  void getPitchAngle();

  /**

  Synchronous version of getPitchAngle().  Waits until a response is received, timing out
  after 1 second.

  @return The drone's current pitch angle, or -1 if timeout.

  */
  float getPitchAngleSync();

  /**

  Registers the function to be called when the Arduino receives pitch angle data from the drone.

  @param cb The function to be called with the drone's pitch angle.

  */
  void pitchAngleCallback(void (*cb)(float));

  /**

  Requests the drone's current roll angle. The response gets handled
  through the callback function set using rollAngleCallback().

  */
  void getRollAngle();

  /**

  Synchronous version of getRollAngle().  Waits until a response is received, timing out
  after 1 second.

  @return The drone's current roll angle, or -1 if timeout.

  */
  float getRollAngleSync();

  /**

  Registers the function to be called when the Arduino receives roll angle data from the drone.

  @param cb The function to be called with the drone's roll angle.

  */
  void rollAngleCallback(void (*cb)(float));

  /**

  This method calls getLatitude(), getLongitude(), getAltitude(), getSatellites(), getSpeed(), and getYaw().
  The callback for each of these methods -- latitudeCallback(), longitudeCallback(), etc -- need to be set in order
  to handle the incoming GPS data.

  */
  void getGPSData();

  /**

  Requests the drone's current aileron value. The response gets handled through the
  callback function set using aileronCallback().

  */
  void getAileron();

  /**

  Synchronous version of getAileron(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's aileron value, or -1 if timeout.

  */
  int getAileronSync();

  /**

  Registers the function to be called when the Arduino receives aileron data from the drone.

  @param cb The function to be called with the drone's aileron value.

  */
  void aileronCallback(void (*cb)(int));

  /**

  Sets the drone's aileron. Accepts values between -100 and 100.

  @param value The new aileron value.

  */
  void setAileron(int value);

  /**

  Requests the drone's current elevator value. The response gets handled through the
  callback function set using elevatorCallback().

  */
  void getElevator();

  /**

  Synchronous version of getElevator(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's elevator value, or -1 if timeout.

  */
  int getElevatorSync();

  /**

  Registers the function to be called when the Arduino receives the drone's elevator value.

  @param cb The function to be called with the drone's elevator value.

  */
  void elevatorCallback(void (*cb)(int));

  /**

  Sets the drone's elevator. Accepts values between -100 and 100.

  @param value The new elevator value.

  */
  void setElevator(int value);

  /**

  Requests the drone's current rudder value. The response gets handled through the
  callback function set using rudderCallback().

  */
  void getRudder();

  /**

  Synchronous version of getRudder(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's rudder value, or -1 if timeout.

  */
  int getRudderSync();

  /**

  Registers the function to be called when the Arduino receives the drone's rudder value.

  @param cb The function to be called with the drone's rudder value.

  */
  void rudderCallback(void (*cb)(int));

  /**

  Sets the drone's rudder. Accepts values between -100 and 100.

  @param value The new rudder value.

  */
  void setRudder(int value);

  /**

  Requests the drone's current throttle value. The response gets handled through the
  callback function set using throttleCallback().

  */
  void getThrottle();

  /**

  Synchronous version of getThrottle(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's throttle value, or -1 if timeout.

  */
  int getThrottleSync();

  /**

  Registers the function to be called when the Arduino receives the drone's throttle value.

  @param cb The function to be called with the drone's throttle value.

  */
  void throttleCallback(void (*cb)(int));

  /**

  Sets the drone's throttle. Accepts values between -100 and 100.

  @param value The new throttle value.

  */
  void setThrottle(int value);

  /**

  Requests the drone's current flightMode. The response gets handled through the
  callback function set using flightModeCallback().

  */
  void getFlightMode();

  /**

  Synchronous version of getFlightMode(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's flight mode, or -1 if timeout.

  */
  int getFlightModeSync();

  /**

  Registers the function to be called when the Arduino receives the drone's flight mode.

  @param cb The function to be called with the drone's flight mode.

  */
  void flightModeCallback(void (*cb)(int));

  /**

  Sets the drone's flight mode. Accepts values between -100 and 100.

  Different flight controllers handle flight modes differently. We recommend
  setting your flight controller to "manual" (or "atitude" if your flight controller supports
  it). Inspect your flight controller's GUI to see how different flight mode integer values
  correspond to different flight mode settings (manual, atitude, failsafe, loiter, etc).

  @param value The new flight mode value.

  */
  void setFlightMode(int value);

  /**

  Sets the drone's rudder, throttle, elevator, and aileron. All values should be between -100 and 100.

  @param aileron The new aileron value.
  @param elevator The new elevator value.
  @param rudder The new rudder value.
  @param throttle The new throttle value.

  */
  void setAileronElevatorRudderThrottle(int aileron, int elevator, int rudder, int throttle);


/**

  Sets the autopilot Mode
  
  @param value chnges the mode of autopilot
  0 manual override
  1 indoor hover
  2 outdoor hover

  */
  void setAutopilotMode(int value);


  /**
   
  Configures the specified pin on the NVextender to behave either as an input or an output.

  @param pin The pin number. Pins 1 to 8 are available.
  @param mode INPUT or OUTPUT.
   
  */
  void pinMode(int pin, int mode);

  /**

  Requests the value from a digital pin. The response gets handled through the
  callback function set using digitalReadCallback().

  @param pin The pin number. Pins 1 to 8 are available.

  */
  void digitalRead(int pin);

  /**

  Synchronous version of digitalRead(). Waits until a response is received, timing out
  after 1 second.

  @return HIGH or LOW, or -1 if timeout.

  */
  int digitalReadSync(int pin);

  /**

  Registers the function to be called when the Arduino receives digital read data on
  the specified pin from the drone.

  @param cb The function to be called when the Arduino receives digital read data.
  @param pin The pin number. Pins 1 to 8 are available.

  */
  void digitalReadCallback(void (*cb)(int), int pin);

  /**

  Write a HIGH or LOW value to a digital pin on the NVextender.

  @param pin The pin number. Pins 1 to 8 are available.
  @param logicLevel HIGH (true) or LOW (false).

  */
  void digitalWrite(int pin, bool logicLevel);

  /**

  Requests the value from an analog pin. The response gets handled through the
  callback function set using analogReadCallback().

  @param pin The pin number. Pins 1 to 8 are available.

  */
  void analogRead(int pin);

  /**

  Synchronous version of analogRead(). Waits until a response is received, timing out
  after 1 second.

  @param pin The pin number. Pins 1 to 8 are available.
  @return 0 to 1023, or -1 if timeout.

  */
  int analogReadSync(int pin);

  /**
   
  This callback function is executed every time it gets analogRead information.  It passes
  this information to the function specified in the argument.

  @param cb The function to be called when the Arduino receives analog read data.
  @param pin The pin number. Pins 1 to 8 are available.

  */
  void analogReadCallback(void (*cb)(int), int pin);

  /**

  Writes an analog value (PWM wave) to a pin on the NVextender. After a call to analogWrite(),
  the pin will generate a steady square wave of the specified duty cycle until the next call to
  analogWrite() (or a call to digitalRead() or digitalWrite() on the same pin).

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The duty cycle ranging from 0 (always off) and 255 (always on).

  */
  void analogWrite(int pin, int value);

  /**

  Reads a pulse (either HIGH or LOW) on a pin on the NVextender. For example, if the value argument
  is HIGH, pulseIn waits for the pin to the HIGH state, starts timing, then waits for the pin to go
  the LOW state and stops timing. Returns the length of the pulse in microseconds. Gives up and returns
  0 if no pulse starts within 3000 microseconds.

  The response gets handled through the callback function set using pulseInCallback().

  See also pulseIn(int pin, int value, unsigned long timeout);

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.

  */
  void pulseIn(int pin, int value);

  /**

  Reads a pulse (either HIGH or LOW) on a pin on the NVextender. For example, if value is HIGH,
  pulseIn waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and
  stops timing. Returns the length of the pulse in microseconds. Gives up and returns 0 if no
  pulse starts within the specified timeout.

  The response gets handled through thecallback function set using pulseInCallback().

  See also pulseIn(int pin, int value);

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.
  @param timeout The number of microseconds to wait for the pulse to start.

  */
  void pulseIn(int pin, int value, unsigned long timeout);

  /**

  Synchronous version of pulseIn(int pin, int value). Waits up to 1 second to receive a response.

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.
  @return The length of the pulse (in microseconds), 0 if no pulse detected, or -1 if no
  pulse length was received from the drone.

  */
  unsigned long pulseInSync(int pin, int value);

  /**

  Synchronous version of pulseIn(int pin, int value, unsigned long timeout). Waits up to 1 second to
  receive a response.

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.
  @param timeout The number of microseconds to wait for the pulse to start.
  @return The length of the pulse (in microseconds), 0 if no pulse detected, or -1 if no
  pulse length was received from the drone.

  */
  unsigned long pulseInSync(int pin, int value, unsigned long timeout);

  /**

  Registers the function to be called when the Arduino receives pulse data from the drone.

  @param cb The function to be called when the Arduino receives pulse data.
  @param pin The pin number. Pins 1 to 8 are available.
   
  */
  void pulseInCallback(void (*cb)(unsigned long), int pin);

  /**

  Associates a servo to a pin.

  @param servoNumber The servo to attach. Values should be 1 to 3.
  @param pin The pin number to attach the servo to. Pins 1 to 8 are available.

  */
  void attachServo(int servoNumber, int pin);

  /**

  Clear a previously set assocation between a servo and pin.

  @param servoNumber The servo to detach. Values should be 1 to 3.

  */
  void detachServo(int servoNumber);

  /**

  Writes a value to the servo, controlling the shaft accordingly. On a standard servo, this
  will set the angle of the shaft (in degrees), moving the shaft to that orientation. On a
  continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in
  one direction, 180 being full speed in the other, and a value near 90 being no movement).

  This servo should have already been attached via the attachServo() method.

  @param servoNumber The servo to send the angle data to. Values should be 1 to 3.
  @param angle The value to write to the servo, from 0 to 180.

  */
  void writeServo(int servoNumber, int angle);

  /**

  Registers the function to be called when the Arduino is notified by the drone that
  an interrupt occurred.

  @param cb The function to be called when the Arduino is notified of an interrupt.
  @param interrupt The interrupt number. Interrupt 0 and 1 are available.

  */
  void interruptCallback(void (*cb)(void), int interrupt);

  /**

  This sets the address of the I2C device that the NVextender communicates with. This
  address will be continuously used by i2cBeginTransmission() and i2cWireRequest() until it
  is reset to a different value.

  @param address The I2C device's slave address. Available address range from 0 to 127.

  */
  void i2cSetDeviceAddress(int address);

  /**

  Begin a transmission to the I2C slave device address specified by i2cSetDeviceAddress().
  Subsequently, queue bytes for transmission with the i2cWrite() function and transmit them
  by calling i2cEndTransmission().

  */
  void i2cBeginTransmission();

  /**

  Ends a transmission to a slave device that was begun by i2cBeginTransmission() and transmits
  the bytes that were queued by i2cWrite().

  */
  void i2cEndTransmission();

  /**

  Queues bytes for transmission from the NVextender to an I2C slave device. Should be
  called between calls to i2cBeginTransmission() and i2cEndTransmission().

  @param data Adds one byte of information to the queue.

  */
  void i2cWrite(int data);

  /**

  Sends a request to the NVextender to read one byte of information from the I2C buffer.
  The response gets handled through thecallback function set using i2cReadCallback().

  */
  void i2cRead();

  /**

  Synchronous version of i2cRead(). Waits until a response is received, timing out
  after 1 second.

  @return The read byte, or -1 if timeout.

  */
  int i2cReadSync();

  /**

  Sends a request to the NVextender to get the number of bytes available for retrieval
  with i2cRead(). This should be called after a call to i2cRequestFrom().

  */
  void i2cAvailable();

  /**

  Synchronous version of i2cAvailable(). Waits until a response is received, timing out
  after 1 second.

  @return The number of available bytes, or -1 no bytes avaiable or if timeout occurred.

  */
  int i2cAvailableSync();

  /**

  Used by the master to request bytes from a slave device on the NVextender. The bytes may then be
  retrieved with the i2cRead() function.

  @param quantity The number of bytes to request.
      
  */
  void i2cRequestFrom(int quantity);

  /**

  Registers the function to be called when the Arduino receives read I2C data from the drone.

  @param cb The function to be called with the read I2C data.

  */
  void i2cReadCallback(void (*cb)(byte));

  /**

  Registers the function to be called when the Arduino receives a response from an I2C available request
  to the drone.

  @param cb The function to be called with the number of bytes avaible for retrieval with i2cRead().

  */
  void i2cAvailableCallback(void (*cb)(int));

  /**

  Registers the function to be called when the Arduino hasn't received a heartbeat from
  the drone for 5 seconds.

  @param cb The function to be called when the heartbeat is lost.

  */
  void heartbeatLostCallback(void (*cb)(void));

  /**

  Registers the function to be called when the Arduino receives a heartbeat from
  the drone after not receiving any for 5 seconds or more.

  @param cb The function to be called when the heartbeat is found.

  */
  void heartbeatFoundCallback(void (*cb)(void));

  /**

  Requests the drone's current battery voltage. The response gets handled through the
  callback function set using voltageCallback().

  */
  void getVoltage();

  /**

  Synchronous version of getVoltage(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's battery voltage, or -1 if timeout.

  */
  int getVoltageSync();

  /**

  Registers the function to be called when the Arduino receives voltage data from the drone.

  */
  void voltageCallback(void (*cb)(int));

  /**

  Requests the signal strength of the wireless communcation between the drone and sender. The
  signal strength is returned as a value from 0 to 100, 0 meaning no connection, and 100 meaning
  perfect connection.

  The response gets handled through thecallback function set using signalStrengthCallback().

  */
  void getSignalStrength();

  /**

  Synchronous version of getSignalStrength(). Waits until a response is received, timing out
  after 1 second.

  @return The signal strength, or -1 if timeout.

  */
  int getSignalStrengthSync();

  /**

  Registers the function to be called when the Arduino receives signal strength
  data from the drone.

  */
  void signalStrengthCallback(void (*cb)(int));

  /**

  Registers the function to be called when the Arduino receives an error code from the drone.
  The callback function will be called with an error code. Error codes -- invalidPinForPinMode,
  unsetI2CAddress, etc -- are listed in IDs.h.

  */
  void setErrorHandler(void (*cb)(int));

  void startTransmitterSupport(int16_t type);

  void stopTransmitterSupport();

  void getTransmitterType();

  int16_t getTransmitterTypeSync();

  void startTransmitterCalibration();

  void stopTransmitterCalibration();

};

#endif // NVDrone_h
