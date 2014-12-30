Avant SDK for Arduino
=============

Welcom the the avant SDK project for interacting with the Avant Protocol on Arduino based systems. This project is owned by the [NVdrones](http://nvdrones.com) company, so please visit the company website if you would like to know more about why this project is all about and how the Avant platform can help you out develeoping software for a varaity of drones.

Before you start using our SDK, please take the time and create an account on our developer website by visting http://developers.nvdrones.com and read our starter guide so you can get a clear understanding on how to work with our technology and alos get full access to the documentation.

# Tools needed 

- Arduino IDE
- Account on the develoepr site
- Clone this repositiry

# How to add the library

- Cloned this repository on your local machine, 
- please open the Arduino IDE and onec open, 
- select Sketch > Import Library > Add Library... and point to the dirrectory where you cloned this SDK. 
- Once this os done, go back to Sketch > Import Library but this time select the actual library "Avant_Arduino".

# Write your first Hello World app

After you opeend the box, red the Getting Started guide and added the SDK to the IDE itselfe. It is time to write the first app to see if everything works.

```{.ino}
#include <Avant.h>

//
// Create Avant Transmitter which transmits on HardwareSerial1
// 
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

# Connect with us

- [Facebook](http://facebook.com/NVdevelopers)
- [Twitter](http://twitter.com/NVdevelopers)
- [Instagram](http://instagram.com/NVdevelopers)
