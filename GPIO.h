
#ifndef NVGPIO_h
#define NVGPIO_h

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacketReader.h"

class GPIO
{
  private:
    SerialIO *serialIO;
    Callback *callbacks;
    IncomingPacketReader *incomingPacketReader;
  public:
    GPIO();
    GPIO(SerialIO *_serialIO, Callback *_callbacks, IncomingPacketReader *_incomingPacketReader);

    void pinMode(int16_t pin, int16_t logicLevel);
    void digitalWrite(int16_t pin, bool logicLevel);
    void analogWrite(int16_t pin, int16_t value);
    void pulseIn(int16_t pin, int16_t value);
    void pulseIn(int16_t pin, int16_t value, uint32_t timeout);
    uint32_t pulseInSync(int16_t pin, int16_t value);
    uint32_t pulseInSync(int16_t pin, int16_t value, uint32_t timeout);
    void digitalRead(int16_t pin);
    void analogRead(int16_t pin);
    void digitalReadCallback(void (*cb)(int16_t), int16_t pin);
    int16_t digitalReadSync(int16_t pin);
    void pulseInCallback(void (*cb)(uint32_t), int16_t pin);
    void analogReadCallback(void (*cb)(int16_t), int16_t pin);
    int16_t analogReadSync(int16_t pin);
    void interruptCallback(void (*cb)(void), int16_t interrupt);
    void attachServo(int16_t servoNumber, int16_t pin);
    void detachServo(int16_t servoNumber);
    void writeServo(int16_t servoNumber, int16_t data);
};

#endif // NVGPIO_h
