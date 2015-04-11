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

The latest version of this library can always be found at
http://arduiniana.org.
*/

#ifndef __ArduinoSDK__SoftwareSerial__
#define __ArduinoSDK__SoftwareSerial__

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13

#include <Arduino.h>

#define _SS_MAX_RX_BUFF 256 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

//********************************************
//SoftwareSerial Code
//*****************************************
//\cond
class SoftwareSerial : public Stream
//\endcond
{
  private:
    // per object data
    uint8_t _receivePin;
    uint8_t _receiveBitMask;
    volatile uint8_t *_receivePortRegister;
    uint8_t _transmitBitMask;
    volatile uint8_t *_transmitPortRegister;

    uint16_t _rx_delay_centering;
    uint16_t _rx_delay_intrabit;
    uint16_t _rx_delay_stopbit;
    uint16_t _tx_delay;

    uint16_t _buffer_overflow:1;
    uint16_t _inverse_logic:1;

    // static data
    static char _receive_buffer[_SS_MAX_RX_BUFF];
    static volatile uint8_t _receive_buffer_tail;
    static volatile uint8_t _receive_buffer_head;
    static SoftwareSerial *active_object;

    // private methods
    void recv();
    uint8_t rx_pin_read();
    void tx_pin_write(uint8_t pin_state);
    void setTX(uint8_t transmitPin);
    void setRX(uint8_t receivePin);

    // private static method for timing
    static inline void tunedDelay(uint16_t delay);

  public:
    // public methods
    SoftwareSerial(){};
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
    ~SoftwareSerial();
    void begin(long speed);
    bool listen();
    void end();
    bool isListening() { return this == active_object; }
    bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
    int peek();

    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush();

    using Print::write;

    // public only for easy access by interrupt handlers
    static inline void handle_interrupt();
};

#endif /* defined __ArduinoSDK__SoftwareSerial__ */