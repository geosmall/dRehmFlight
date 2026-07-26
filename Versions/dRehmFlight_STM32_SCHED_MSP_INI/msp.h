/**
 * @file msp.h
 * @brief MSP V1 definitions for dRehmFlight PWA Configurator
 *
 * Minimal MSP V1 protocol ($M framing, XOR checksum).
 * 9 read-only telemetry commands — all settings via CLI.
 */

#ifndef MSP_H
#define MSP_H

#include <stdint.h>

//========================================================================================================================//
// MSP V1 Frame Constants
//========================================================================================================================//

static constexpr char MSP_PREAMBLE_1       = '$';
static constexpr char MSP_V1_HEADER        = 'M';
static constexpr char MSP_DIR_REQUEST      = '<';
static constexpr char MSP_DIR_RESPONSE     = '>';
static constexpr char MSP_DIR_ERROR        = '!';

static constexpr uint8_t  MSP_MAX_PAYLOAD      = 64;
static constexpr uint32_t MSP_FRAME_TIMEOUT_MS  = 100;

//========================================================================================================================//
// MSP V1 Command Codes (9 read-only + MSP_REBOOT)
//========================================================================================================================//

static constexpr uint8_t MSP_API_VERSION   = 1;
static constexpr uint8_t MSP_FC_VARIANT    = 2;
static constexpr uint8_t MSP_FC_VERSION    = 3;
static constexpr uint8_t MSP_BOARD_INFO    = 4;
static constexpr uint8_t MSP_STATUS        = 101;
static constexpr uint8_t MSP_RAW_IMU       = 102;
static constexpr uint8_t MSP_MOTOR         = 104;
static constexpr uint8_t MSP_RC            = 105;
static constexpr uint8_t MSP_ATTITUDE      = 108;
static constexpr uint8_t MSP_ANALOG        = 110;
static constexpr uint8_t MSP_REBOOT        = 68;   // payload[0] = reboot mode (below)

// MSP_REBOOT payload modes (Betaflight msp.c). Only FIRMWARE/BOOTLOADER_ROM
// (+ BOOTLOADER_FLASH when bootuf2 is present) are actioned; others are ACK-only.
static constexpr uint8_t MSP_REBOOT_FIRMWARE        = 0;
static constexpr uint8_t MSP_REBOOT_BOOTLOADER_ROM  = 1;   // ST system DFU (0483:df11)
static constexpr uint8_t MSP_REBOOT_MSC             = 2;
static constexpr uint8_t MSP_REBOOT_MSC_UTC         = 3;
static constexpr uint8_t MSP_REBOOT_BOOTLOADER_FLASH = 4;  // bootuf2

//========================================================================================================================//
// Sensor Bitmask (MSP_STATUS sensor field)
//========================================================================================================================//

static constexpr uint16_t SENSOR_ACC  = (1 << 0);
static constexpr uint16_t SENSOR_BARO = (1 << 1);
static constexpr uint16_t SENSOR_MAG  = (1 << 2);
static constexpr uint16_t SENSOR_GPS  = (1 << 3);
static constexpr uint16_t SENSOR_GYRO = (1 << 4);

//========================================================================================================================//
// DRHM Identity Constants
//========================================================================================================================//

static constexpr uint8_t MSP_PROTOCOL_VERSION = 0;
static constexpr uint8_t API_VERSION_MAJOR    = 1;
static constexpr uint8_t API_VERSION_MINOR    = 0;
static constexpr uint8_t FC_VERSION_MAJOR     = 1;
static constexpr uint8_t FC_VERSION_MINOR     = 4;
static constexpr uint8_t FC_VERSION_PATCH     = 0;

//========================================================================================================================//
// MSP V1 Parser State Machine
//========================================================================================================================//

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,       // Got '$'
    MSP_HEADER_M,           // Got 'M'
    MSP_HEADER_V1,          // Reading payload_len
    MSP_PAYLOAD_SIZE_V1,    // Reading cmd
    MSP_PAYLOAD_V1,         // Reading payload bytes
    MSP_CHECKSUM_V1,        // Verifying XOR checksum
} mspState_e;

// Parsed MSP message
typedef struct {
    uint8_t cmd;
    uint8_t payloadSize;
    uint8_t payload[MSP_MAX_PAYLOAD];
} mspMessage_t;

// Board identity accessor (defined in mspComm.ino). Lets the CLI stamp the
// compile-time board name without depending on .ino concatenation order.
const char* mspTargetName(void);

#endif // MSP_H
