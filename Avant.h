#ifndef AVANT_H
#define AVANT_H

#include <Arduino.h>
#include <inttypes.h>
#include <Stream.h>
#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class SoftwareSerial : public Stream
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

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round






class TransmitterConfig {
	private:
		byte elevatorPort;
		byte ailronPort;
		byte throttlePort;
		byte rudderPort;
		byte receiverPort;
		byte transmitterPort;
		
	public:
		TransmitterConfig(); // Default Constructor
		~TransmitterConfig(); // Destructor

		byte getElevatorPort() {
			return elevatorPort;
		}
		void setElevatorPort(byte p) {
			elevatorPort = p;
		}

		byte getAilronPort(){
			return ailronPort;
		}
		void setAilronPort(byte p) {
			ailronPort = p;
		}
		
		byte getThrottlePort(){
			return throttlePort;
		}
		void setThrottlePort(byte p) {
			throttlePort = p;
		}

		byte getRudderPort(){
			return rudderPort;
		}
		void setRudderPort(byte p) {
			rudderPort = p;
		}		
		
		byte getTransmitterPort(){
			return transmitterPort;
		}
		void setTransmitterPort(byte p) {
			transmitterPort = p;
		}

		byte getReceiverPort(){
			return receiverPort;
		}
		void setReceiverPort(byte p) {
			receiverPort = p;
		}
};

class Transmitter {
	private:
		int minElevator;
		int maxElevator;
		int minAilron;
		int maxAilron;
		int minThrottle;
		int maxThrottle;
		int minRudder;
		int maxRudder;
		TransmitterConfig configuration;
		void sendData(long data, int resourceID, int actionID);
		
	public:
		Transmitter();
		~Transmitter();
		void configure(TransmitterConfig config);
		void calibrate();
		void transmitData();
};





#endif