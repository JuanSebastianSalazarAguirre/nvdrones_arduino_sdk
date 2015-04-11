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
#include "AvantRC.h"
#include "SerialIO.h"
#include "Callback.h"
#include "AvantPose.h"
#include "AvantGPIO.h"
#include "AvantI2C.h"  
#include "AvantResponseHandler.h"
#include "AvantAutoPilot.h"

//\cond
class Drone 
//\endcond
{
    private:
      AvantRC avantRC;
      SerialIO serialIO;
      AvantGPIO avantGPIO;
      AvantResponseHandler responseHandler;
      AvantI2C avantI2C;
      Callback callback;
      AvantPose avantPose;
      AvantAutoPilot avantAutoPilot;
    public:
      Drone();
      Drone(SerialPort serialPort);
      Drone(int txPin, int rxPin);
      AvantRC& RC();
      AvantGPIO& GPIO();
      AvantI2C& I2C();
      AvantPose& pose();
      AvantResponseHandler& avantResponseHandler();
      AvantAutoPilot& AutoPilot();
      void arm();
      void initialize();
};

#endif