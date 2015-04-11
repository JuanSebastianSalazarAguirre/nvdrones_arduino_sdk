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
#include "AutoPilot.h"

//\cond
class Drone 
//\endcond
{
    private:
      RC avantRC;
      SerialIO serialIO;
      GPIO avantGPIO;
      ResponseHandler responseHandler;
      I2C avantI2C;
      Callback callback;
      Pose avantPose;
      AutoPilot avantAutoPilot;
    public:
      Drone();
      Drone(SerialPort serialPort);
      Drone(int txPin, int rxPin);
      RC& rc();
      GPIO& gpio();
      I2C& i2c();
      Pose& pose();
      ResponseHandler& avantResponseHandler();
      AutoPilot& autoPilot();
      void arm();
      void initialize();
};

#endif /* defined __ArduinoSDK__Drone__ */