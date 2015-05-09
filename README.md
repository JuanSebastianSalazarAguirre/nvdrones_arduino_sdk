# Arduino SDK

Welcome the the Arduino SDK. This project is owned by the [NVdrones](http://nvdrones.com) company, so please visit our website to learn more.

## Before we start

Before you start using our SDK, please take the time and create an account on our developer website by visiting http://developers.nvdrones.com and read our ‘Get Started’ article, so you can gain a clear understanding of how to work with our technology, as well as get full access to the documentation.

## Tools needed 

- [Arduino IDE](http://arduino.cc/en/main/software)
- [Our Developer Kit](https://developers.nvdrones.com/buythedevkit)
- A drone.

## How to add the library

- Clone this repository on your local machine. 
- Open the Arduino IDE.
- Select Sketch > Import Library > Add Library… and point to the directory where you cloned this SDK. 
- Once this is done, go back to Sketch > Import Library but this time select the actual library “ArduinoSDK”.

## Write your first Hello World app

After opening the Developer Kit, read the Getting Started guide and add the SDK to the IDE. It is time to write the first app to see if everything works.

```{.ino}
#include <Drone.h>
 
// Choose serialPort1 if your XBee is connect to your Arduino through
// serial port 1.
Drone drone(serialPort1);
unsigned long time = millis();
 
void setup() {
  // This lets you print back to your Serial terminal.
  Serial.begin(57600);
  
  // This must be in the setup() of your sketch.
  drone.initialize();
  
  // Attach a callback for when the heartbeat is lost.
  drone.heartbeatLostCallback(heartbeatLost);
}
 
void loop() {
  
  // Call arm every 3 seconds without blocking the main loop.
  unsigned long now = millis();
  if (now > time + 3000) {
    Serial.println("Arming");
    drone.arm();  // Turn the drone on/off.
    time = now;
  }
  
  // This must be in the loop() of your sketch to receive data from
  // the drone.
  drone.listen();
}
 
void heartbeatLost() {
  Serial.println("Oh no! Lost connection with the drone.");
}
```

## Connect with us

Since our mission is to simplify your life to empower you to create great solutions using drones. We want to make sure that we stay in connected with each other!

- [Email](mailto:developers@NVdrones.com): no big or small question will be unanswered.
- [Facebook](http://facebook.com/NVdevelopers): check what is going on.
- [Twitter](http://twitter.com/NVdevelopers): ask small questions.
- [Instagram](http://instagram.com/NVdevelopers): look at what we do.

# Just so you know

The SDK has a stripped down version of the Arduino [SoftwareSerial Library](http://arduino.cc/en/Reference/softwareSerial) so you don’t have to link it every the time you wish to use this SDK.
