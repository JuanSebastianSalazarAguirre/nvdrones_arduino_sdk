# Avant SDK for Arduino

Welcome the the Avant SDK for interacting with the Avant Protocol on Arduino-based systems. This project is owned by the [NVdrones](http://nvdrones.com) company, so please visit our website to learn more and how the Avant platform can help you develop software for a variety of drones.

## Before we start

Before you start using our SDK, please take the time and create an account on our developer website by visiting http://developers.nvdrones.com and read our starter guide so you can gain a clear understanding of how to work with our technology, as well as get full access to the documentation.

## Tools needed 

- [Arduino IDE](http://arduino.cc/en/main/software)
- [Account on the developer site](http://developers.nvdrones.com)
- Clone this repository

## How to add the library

- Clone this repository on your local machine, 
- Open the Arduino IDE, 
- Select Sketch > Import Library > Add Library... and point to the directory where you cloned this SDK. 
- Once this is done, go back to Sketch > Import Library but this time select the actual library "Avant_Arduino".

## Write your first Hello World app

After opening the Developer Kit, read the Getting Started guide and add the SDK to the IDE. It is time to write the first app to see if everything works.

```{.ino}
#include <Avant.h>

Avant myTransmitter(1);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Arming");
  myTransmitter.armDrone();
  
  Serial.println("Waiting...");
  delay(10000);
  
  Serial.println("Disarming");
  myTransmitter.disarmDrone();
}
```
Now that you tested everything it is time to get more serious. Visit our [Quick Start App for Arduino](https://github.com/NVdrones/Quick-Start-for-Arduino) which is a more in depth showcase of all the possibilities of API.  

## Connect with us

Since our mission is to simplify your life to empower you to create great solutions using drones. We want to make sure that we stay in connected with each other!

- [Facebook](http://facebook.com/NVdevelopers): check what is going on.
- [Twitter](http://twitter.com/NVdevelopers): ask small questions.
- [Instagram](http://instagram.com/NVdevelopers): look what we do.
- [E-Mail](mailto:developers@NVdrones.com): no big or small question will be unawsered.

# Just so you know

The SDK have a stripped down version of the Arduino [SoftwareSerial Library](http://arduino.cc/en/Reference/softwareSerial) so you don't have to link it every the time you wish to use this SDK.
