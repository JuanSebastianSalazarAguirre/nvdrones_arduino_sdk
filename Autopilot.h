#ifndef NV_AUTOPILOT
#define NV_AUTOPILOT

#include "Callback.h"
#include "SerialIO.h"
#include "IncomingPacketReader.h"

class Autopilot {
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;
public:
  Autopilot();
  Autopilot(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader);

  //chooses which autopilot feature to enable
  void setModeTakeover();
  void setModeIndoorHover();
  void setModeOutdoorHover();
  void setAutopilotMode(int);

  //callbacks
  void indoorHoverCallback(void (*function)(float));
  void outdoorHoverCallback(void (*function)(float));
  void takeoverCallback(void(*function)(float));
};

#endif //NV_AUTOPILOT