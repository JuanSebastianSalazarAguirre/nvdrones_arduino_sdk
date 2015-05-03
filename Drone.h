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

#ifndef __ArduinoSDK__Drone__
#define __ArduinoSDK__Drone__

#include <inttypes.h>
#include <Stream.h>
#include <Arduino.h>
#include "RC.h"
#include "SerialIO.h"
#include "Callback.h"
#include "Pose.h"
#include "GPIO.h"
#include "I2C.h"
#include "ResponseHandler.h"
#include "Vitals.h"

class Drone 
{
private:
  RC rc;
  SerialIO serialIO;
  GPIO gpio;
  IncomingPacketReader incomingPacketReader;
  Vitals vitals;
  ResponseHandler responseHandler;
  I2C i2c;
  Callback callback;
  Pose pose;

public:

  /**

  Creates a new Drone instance.

  */
  Drone();

  /**

  Creates a new Drone instance.

  @param serialPort The serial port for wireless communcation with the drone.

  */
  Drone(SerialPort serialPort);

  /**

  Creates a new Drone instance.

  @param txPin The transmit pin for wireless communcation with the drone.
  @param rxPin The receive pin for wireless communcation with the drone.

  */
  Drone(int txPin, int rxPin);


  /**

  Performs the drone's preflight tasks.

  */
  void initialize();

  /**

  Turns the drone on/off.

  */
  void arm();

  /**

  Checks for (and handles) communcation with the drone. Should be called in the loop()
  of your Arduino sketch.

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
       
  Requests the drone's current speed. The response gets handled through the
  callback function set using speedCallback().
       
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

  Requests the drone's current orientation. The response gets handled through the
  callback function set using orientationCallback().

  */
  void getOrientation();

  /**

  Synchronous version of getOrientation().  Waits until a response is received, timing out
  after 1 second.

  @return The drone's current orientation, or -1 if timeout.

  */
  float getOrientationSync();

  /**

  Registers the function to be called when the Arduino receives orientation data from the drone.

  @param cb The function to be called with the drone's orientation.

  */
  void orientationCallback(void (*cb)(float));

  /**

  Helper to call getLatitude(), getLongitude(), getAltitude(), getSatellites(), getSpeed(), and getOrientation().

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

  Synchronous version of getFlightMode(). Waits until a response is received, timing out
  after 1 second.

  @return The drone's flight mode, or -1 if timeout.

  */
  int getFlightModeSync();

  /**

  Requests the drone's current flightMode. The response gets handled through the
  callback function set using flightModeCallback().

  */
  void getFlightMode();

  /**

  Registers the function to be called when the Arduino receives the drone's flight mode.

  @param cb The function to be called with the drone's flight mode.

  */
  void flightModeCallback(void (*cb)(int));

  /**

  Sets the drone's flight mode. Accepts values between -100 and 100.

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
   
  Configures the specified pin on the App Extender to behave either as an input or an output.

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
  void digitalReadCallback(void (*cb)(byte), int pin);

  /**

  Write a HIGH or LOW value to a digital pin on the App Extender.

  @param pin The pin number. Pins 1 to 8 are available.
  @param logicLevel HIGH or LOW.

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
  void analogReadCallback(void (*cb)(byte), int pin);

  /**

  Writes an analog value (PWM wave) to a pin on the App Extender. After a call to analogWrite(),
  the pin will generate a steady square wave of the specified duty cycle until the next call to
  analogWrite() (or a call to digitalRead() or digitalWrite() on the same pin).

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The duty cycle ranging from 0 (always off) and 255 (always on).

  */
  void analogWrite(int pin, int value);

  /**

  Reads a pulse (either HIGH or LOW) on a pin on the App Extender. For example, if value is HIGH,
  pulseIn() waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and
  stops timing. Returns the length of the pulse in microseconds. Gives up and returns 0 if no
  pulse starts within a specified time out.

  The response gets handled through thecallback function set using pulseInCallback().

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.
  @param pin (optional) The number of microseconds to wait for the pulse to start. Default is 3000 microseconds.

  */
  void pulseIn(int pin, int value);
  void pulseIn(int pin, int value, unsigned long timeout);

  /**

  Synchronous version of pulseIn(). Waits up to 1 second to receive a response.

  @param pin The pin number. Pins 1 to 8 are available.
  @param value The type of the pulse to read. Either HIGH or LOW.
  @param pin (optional) The number of microseconds to wait for the pulse to start. Default is 3000 microseconds.
  @return The length of the pulse (in microseconds), 0 if no pulse detected, or -1 if no
  pulse length was received from the drone.

  */
  void pulseInSync(int pin, int value);
  void pulseInSync(int pin, int value, unsigned long timeout);

  /**

  Registers the function to be called when the Arduino receives pulse data from the drone.

  @param cb The function to be called when the Arduino receives pulse data.
  @param pin The pin number. Pins 1 to 8 are available.
   
  */
  void pulseInCallback(void (*cb)(long), int pin);

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

  This sets the address of the I2C device that the App Extender communicates with. This
  address will be continuously used by i2cBeginTransmission() and i2cWireRequest() until it
  is reset to a different value.

  @param address The I2C device's slave address. Available address range from 0 to 127.

  */
  void i2cSetDeviceAddress(int address);

  /**

  Begin a transmission to the I2C slave device address specified by setDeviceAddress().
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

  Queues bytes for transmission from the App Extender to an I2C slave device. Should be
  called between calls to i2cBeginTransmission() and i2cEndTransmission().

  @param data Adds one byte of information to the queue.

  */
  void i2cWrite(int data);

  /**

  Sends a request to the App Extender to read one byte of information from the I2C buffer.
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

  Used by the master to request bytes from a slave device on the App Extender. The bytes may then be
  retrieved with the i2cRead() function.

  @param quantity The number of bytes to request.
      
  */
  void i2cRequestFrom(int quantity);

  /**

  Registers the function to be called when the Arduino receives read I2C data from the drone.

  @param function name of the method

  */
  void i2cReadCallback(void (*cb)(byte));

  /**

  Registers the function to be called when the Arduino hasn't received a heartbeat from
  the drone for 5 seconds.

  */
  void heartbeatLostCallback(void (*cb)(void));

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

  Requests the signal strength of the wireless communcation between the drone and sender.
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

  */
  void setErrorHandler(void (*cb)(int));

};

#endif /* defined __ArduinoSDK__Drone__ */