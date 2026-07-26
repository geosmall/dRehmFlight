//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3
//
//Target: STM32F4 (NUCLEO_F411RE, NOXE V3)

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code
//STM32: Using SerialRx library for IBus/SBUS protocols only
//
/*
 * Radio Channel Mapping and Failsafe (convention assumed throughout the code):
 *
 * +----------------+----------+----------+----------------------------------------------+
 * | Channel Number | Function | Failsafe | Notes                                        |
 * |   (In Code)    |          |   (us)   |                                              |
 * +----------------+----------+----------+----------------------------------------------+
 * |       1        | Throttle |   1000   | 1000 min, 2000 max                           |
 * |       2        | Roll     |   1500   | 1000 full left, 2000 full right, 1500 center |
 * |       3        | Pitch    |   1500   | 1000 pitch up, 2000 pitch down, 1500 center  |
 * |       4        | Yaw      |   1500   | 2000 full left, 1000 full right, 1500 center |
 * |       5        | Gear     |   2000   | Throttle cut when greater than 1500          |
 * |       6        | Aux1     |   2000   | Free auxiliary channel                       |
 * +----------------+----------+----------+----------------------------------------------+
 *   Failsafe values applied on signal loss (defined in main .ino)
 *
 * If your radio uses a different channel order, re-assign the inputs in
 * updateRadioChannels() below to match these assumed mappings.
 *
 * For IBus (FlySky): endpoint config required to push failsafe value
 * below 885 us threshold for Layer 3 detection.
 */

#include <SerialRx.h>

// SerialRx library objects
HardwareSerial SerialRC(BoardConfig::rc_receiver.instance, BoardConfig::rc_receiver.rx_pin, BoardConfig::rc_receiver.tx_pin);
SerialRx rx;

// Raw channel data - populated by SerialRx adapter
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;

void radioSetup() {
  // CRITICAL: Enable external inverter BEFORE initializing UART
  // F4 lacks hardware RXINV, so the inverter must be set first to avoid
  // receiving garbage during UART initialization
#if defined(USE_SBUS_RX) && !defined(USART_CR2_RXINV) && defined(ARDUINO_OPEN_REVO)
  pinMode(BoardConfig::rc_inverter_pin, OUTPUT);
  digitalWrite(BoardConfig::rc_inverter_pin, HIGH);  // HIGH = SBUS (inverted)
#endif

  // Initialize SerialRx library
  SerialRx::Config config;
  config.serial = &SerialRC;

  #if defined USE_IBUS_RX
    config.rx_protocol = SerialRx::IBUS;
    config.baudrate = 115200;
  #elif defined USE_SBUS_RX
    config.rx_protocol = SerialRx::SBUS;
    config.baudrate = 100000;
    config.invert_rx = true;   // SBUS uses inverted signal
  #elif defined USE_CRSF_RX
    config.rx_protocol = SerialRx::CRSF;
    config.baudrate = 420000;  // CRSF / ELRS standard baud
  #else
    #error No serial RX protocol defined (USE_IBUS_RX, USE_SBUS_RX, or USE_CRSF_RX)
  #endif

  config.timeout_ms = BoardConfig::rc_receiver.timeout_ms;
  config.idle_threshold_us = BoardConfig::rc_receiver.idle_threshold_us;

  if (!rx.begin(config)) {
    Serial.println("ERROR: Radio RX init failed!");
    while (1) { delay(1000); }  // Halt
  }

  Serial.println("Radio RX initialized");
}

void updateRadioChannels() {
  // Adapter: SerialRx → dRehmFlight channel_X_raw variables
  rx.update();

  if (rx.available()) {
    RCMessage msg;
    if (rx.getMessage(&msg)) {
      // Map SerialRx channels to dRehmFlight PWM variables
      // channelToPWM() normalizes to ~1000-2000 µs regardless of protocol:
      //   - SBUS (0-2047) → PWM via iNav formula
      //   - IBus (1000-2000) → passed through unchanged
      channel_1_raw = rx.channelToPWM(msg.channels[0]);  // Throttle
      channel_2_raw = rx.channelToPWM(msg.channels[1]);  // Aileron
      channel_3_raw = rx.channelToPWM(msg.channels[2]);  // Elevator
      channel_4_raw = rx.channelToPWM(msg.channels[3]);  // Rudder
      channel_5_raw = rx.channelToPWM(msg.channels[4]);  // Gear (throttle cut)
      channel_6_raw = rx.channelToPWM(msg.channels[5]);  // Aux1
    }
  }
}

bool radioSignalLost() {
  // Adapter: SerialRx 4-layer failsafe (protocol flag, timeout, range, expiry)
  return rx.isSignalLost();
}

const char* radioSignalStatus() {
  return rx.getSignalStatusString();
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from SerialRx adapter
  unsigned long returnPWM = 0;

  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }

  return returnPWM;
}
