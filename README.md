# Arduino SDK

Welcome the the Arduino SDK. This project is owned by the [NVdrones](http://nvdrones.com) company, so please visit our website to learn more about our drone solution.

## Before we start

Before you start using our SDK, please take the time and create an account on our developer website by visiting http://developers.nvdrones.com and read our ‘Get Started’ article, so you can gain a clear understanding of how to work with our technology, as well as get full access to the documentation.

## Tools needed 

- [Arduino IDE](http://arduino.cc/en/main/software)
- [Account on the developer site](http://developers.nvdrones.com)
- Clone this repository
- Include this SDK in your project

## How to add the library

- Clone this repository on your local machine, 
- Open the Arduino IDE, 
- Select Sketch > Import Library > Add Library… and point to the directory where you cloned this SDK. 
- Once this is done, go back to Sketch > Import Library but this time select the actual library “AndroidSDK”.

## Write your first Hello World app

After opening the Developer Kit, read the Getting Started guide and add the SDK to the IDE. It is time to write the first app to see if everything works.

```{.ino}
#include <Drone.h>

// Create an instance of Avant
//
// 0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
//
// If the Xbee is connected to software serial ports
// Specify the RX, TX port like myTrans(rx, tx)
Drone drone(serialPort1); 

void setup() {
  Serial.begin(57600);
	
	//turn the drone on and off every 3 seconds
  delay(3000);               
	
	Serial.println(“Arming 3”);

	//This will turn the drone on and off
  drone.arm();     
}

void loop() {

}
```

## Connect with us

Since our mission is to simplify your life to empower you to create great solutions using drones. We want to make sure that we stay in connected with each other!

- [Facebook](http://facebook.com/NVdevelopers): check what is going on.
- [Twitter](http://twitter.com/NVdevelopers): ask small questions.
- [Instagram](http://instagram.com/NVdevelopers): look what we do.
- [E-Mail](mailto:developers@NVdrones.com): no big or small question will be unanswered.

# Just so you know

The SDK have a stripped down version of the Arduino [SoftwareSerial Library](http://arduino.cc/en/Reference/softwareSerial) so you don’t have to link it every the time you wish to use this SDK.
