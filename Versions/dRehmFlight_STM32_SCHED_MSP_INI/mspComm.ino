//========================================================================================================================//
//                                         MSP PROTOCOL HANDLER (V1 only)                                              //
//========================================================================================================================//
//
// MSP V1 protocol for dRehmFlight PWA Configurator.
// 9 read-only command handlers — identity, status, telemetry.
// All settings changes via CLI (not MSP).
//

#include "msp.h"
#include <bootloader.h>   // requestSystemBootloader() (ST system DFU), enterBootloader() (bootuf2)

//========================================================================================================================//
// Board Identification (compile-time from Arduino board selection)
//========================================================================================================================//

#if defined(ARDUINO_OPEN_REVO)
  static const char BOARD_ID[] = "REVO";
  static const char TARGET_NAME[] = "OPEN-REVO";
#elif defined(ARDUINO_BLACKPILL_F411CE)
  static const char BOARD_ID[] = "BP41";
  static const char TARGET_NAME[] = "BLACKPILL-F411";
#elif defined(ARDUINO_NUCLEO_F411RE)
  static const char BOARD_ID[] = "N411";
  static const char TARGET_NAME[] = "NUCLEO-F411RE";
#elif defined(ARDUINO_BKMN_NERO)
  static const char BOARD_ID[] = "NERO";
  static const char TARGET_NAME[] = "BKMN-NERO";
#elif defined(ARDUINO_MATEK_H743VI)
  static const char BOARD_ID[] = "MH74";
  static const char TARGET_NAME[] = "MATEK-H743";
#elif defined(ARDUINO_DEVEBOX_H743)
  static const char BOARD_ID[] = "DH74";
  static const char TARGET_NAME[] = "DEVEBOX-H743";
#elif defined(ARDUINO_BEFH_BETAFPVG473)
  static const char BOARD_ID[] = "BFG4";
  static const char TARGET_NAME[] = "BETAFPV-G473";
#elif defined(ARDUINO_BEFH_BETAFPVF405)
  static const char BOARD_ID[] = "BFF4";
  static const char TARGET_NAME[] = "BETAFPV-F405";
#elif defined(ARDUINO_WEACT_G474CE)
  static const char BOARD_ID[] = "WG47";
  static const char TARGET_NAME[] = "WEACT-G474";
#else
  static const char BOARD_ID[] = "DRHM";
  static const char TARGET_NAME[] = "UNKNOWN";
#endif

// Board identity accessor (declared in msp.h) — used by the CLI provenance header.
const char* mspTargetName(void) { return TARGET_NAME; }

//========================================================================================================================//
// MSP Serial Port & Parser State
//========================================================================================================================//

static Stream* mspSerial = nullptr;
static mspState_e mspState = MSP_IDLE;
static mspMessage_t mspMsg;
static uint8_t mspPayloadIdx = 0;
static uint8_t mspV1PayloadLen = 0;
static uint8_t mspV1Checksum = 0;
static uint32_t mspLastByteTimeMs = 0;

//========================================================================================================================//
// Response Buffer & Helpers
//========================================================================================================================//

static uint8_t mspRespBuf[MSP_MAX_PAYLOAD];

static inline void bufWrite8(uint8_t* buf, uint8_t& idx, uint8_t val) {
    buf[idx++] = val;
}

static inline void bufWrite16(uint8_t* buf, uint8_t& idx, uint16_t val) {
    buf[idx++] = val & 0xFF;
    buf[idx++] = (val >> 8) & 0xFF;
}

static inline void bufWriteS16(uint8_t* buf, uint8_t& idx, int16_t val) {
    bufWrite16(buf, idx, (uint16_t)val);
}

static inline void bufWrite32(uint8_t* buf, uint8_t& idx, uint32_t val) {
    buf[idx++] = val & 0xFF;
    buf[idx++] = (val >> 8) & 0xFF;
    buf[idx++] = (val >> 16) & 0xFF;
    buf[idx++] = (val >> 24) & 0xFF;
}

//========================================================================================================================//
// V1 Response Builder
//========================================================================================================================//

static void mspSendV1Response(uint8_t cmd, const uint8_t* data, uint8_t len) {
    // Drop response if TX buffer can't fit the full frame (BF pattern).
    // Configurator will re-poll within one cycle.
    if (mspSerial->availableForWrite() < (int)(6 + len)) {
        return;
    }
    mspSerial->write(MSP_PREAMBLE_1);
    mspSerial->write(MSP_V1_HEADER);
    mspSerial->write(MSP_DIR_RESPONSE);
    mspSerial->write(len);
    mspSerial->write(cmd);
    uint8_t checksum = len ^ cmd;
    for (uint8_t i = 0; i < len; i++) {
        mspSerial->write(data[i]);
        checksum ^= data[i];
    }
    mspSerial->write(checksum);
}

//========================================================================================================================//
// MSP Message Handlers (9 commands)
//========================================================================================================================//

static void mspHandleMessage(const mspMessage_t& msg) {
    uint8_t idx = 0;

    switch (msg.cmd) {

    //--- Identity (connection handshake) ---

    case MSP_API_VERSION:
        bufWrite8(mspRespBuf, idx, MSP_PROTOCOL_VERSION);
        bufWrite8(mspRespBuf, idx, API_VERSION_MAJOR);
        bufWrite8(mspRespBuf, idx, API_VERSION_MINOR);
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    case MSP_FC_VARIANT:
        mspRespBuf[0] = 'D'; mspRespBuf[1] = 'R';
        mspRespBuf[2] = 'H'; mspRespBuf[3] = 'M';
        mspSendV1Response(msg.cmd, mspRespBuf, 4);
        break;

    case MSP_FC_VERSION:
        bufWrite8(mspRespBuf, idx, FC_VERSION_MAJOR);
        bufWrite8(mspRespBuf, idx, FC_VERSION_MINOR);
        bufWrite8(mspRespBuf, idx, FC_VERSION_PATCH);
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    case MSP_BOARD_INFO: {
        memcpy(&mspRespBuf[idx], BOARD_ID, 4);
        idx += 4;
        bufWrite16(mspRespBuf, idx, 0);          // hardware revision
        bufWrite8(mspRespBuf, idx, 0);            // OSD support
        bufWrite8(mspRespBuf, idx, 0x01);         // comm capabilities (VCP)
        uint8_t nameLen = strlen(TARGET_NAME);
        bufWrite8(mspRespBuf, idx, nameLen);
        memcpy(&mspRespBuf[idx], TARGET_NAME, nameLen);
        idx += nameLen;
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;
    }

    //--- Status ---

    case MSP_STATUS:
        bufWrite16(mspRespBuf, idx, 500);                        // cycleTime µs (2000 Hz)
        bufWrite16(mspRespBuf, idx, averageSystemLoadPercent);   // cpuLoad %
        bufWrite16(mspRespBuf, idx, SENSOR_ACC | SENSOR_GYRO);   // sensors
        bufWrite32(mspRespBuf, idx, armedFly ? 1 : 0);          // flightModes (bit 0 = armed)
        bufWrite8(mspRespBuf, idx, 0);                           // profile
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    //--- Telemetry ---

    case MSP_RAW_IMU:
        bufWriteS16(mspRespBuf, idx, (int16_t)(AccX * 512));
        bufWriteS16(mspRespBuf, idx, (int16_t)(AccY * 512));
        bufWriteS16(mspRespBuf, idx, (int16_t)(AccZ * 512));
        bufWriteS16(mspRespBuf, idx, (int16_t)(GyroX));
        bufWriteS16(mspRespBuf, idx, (int16_t)(GyroY));
        bufWriteS16(mspRespBuf, idx, (int16_t)(GyroZ));
        bufWriteS16(mspRespBuf, idx, (int16_t)(MagX));
        bufWriteS16(mspRespBuf, idx, (int16_t)(MagY));
        bufWriteS16(mspRespBuf, idx, (int16_t)(MagZ));
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    case MSP_MOTOR: {
        // 8 motors, MSP convention (1000-2000 us). m*_command_scaled is the
        // 0..1 value sent to MotorManager; throttle-cut/disarm forces these to
        // 0.0f, which reports 1000 (min) here. Read-only telemetry.
        const float ms[8] = { m1_command_scaled, m2_command_scaled,
                              m3_command_scaled, m4_command_scaled,
                              m5_command_scaled, m6_command_scaled, 0.0f, 0.0f };
        for (int i = 0; i < 8; ++i) {
            bufWrite16(mspRespBuf, idx,
                       (uint16_t)(1000.0f + constrain(ms[i], 0.0f, 1.0f) * 1000.0f));
        }
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;
    }

    case MSP_RC:
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_1_pwm);
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_2_pwm);
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_3_pwm);
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_4_pwm);
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_5_pwm);
        bufWrite16(mspRespBuf, idx, (uint16_t)channel_6_pwm);
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    case MSP_ATTITUDE:
        bufWriteS16(mspRespBuf, idx, (int16_t)(roll_IMU * 10));   // decidegrees
        bufWriteS16(mspRespBuf, idx, (int16_t)(pitch_IMU * 10));
        bufWriteS16(mspRespBuf, idx, (int16_t)(yaw_IMU));         // degrees
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    case MSP_ANALOG:
        bufWrite8(mspRespBuf, idx, 0);       // vbat (no ADC)
        bufWrite16(mspRespBuf, idx, 0);      // mAhDrawn
        bufWrite16(mspRespBuf, idx, 0);      // rssi
        bufWriteS16(mspRespBuf, idx, 0);     // amperage
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        break;

    //--- Reboot / enter bootloader (standard MSP, BF cmd 68) ---

    case MSP_REBOOT: {
        // payload[0] = reboot mode. Safety: refuse while armed. ACK before
        // resetting (host needs the reply before USB drops), then reboot.
        const uint8_t mode = (msg.payloadSize >= 1) ? msg.payload[0]
                                                    : MSP_REBOOT_FIRMWARE;

        if (armedFly) {
            mspSendV1Response(msg.cmd, nullptr, 0);   // refuse: empty ACK, no reboot
            break;
        }

        bool actioned = (mode == MSP_REBOOT_FIRMWARE ||
                         mode == MSP_REBOOT_BOOTLOADER_ROM);
#ifdef BL_BOOTUF2
        actioned = actioned || (mode == MSP_REBOOT_BOOTLOADER_FLASH);
#endif
        if (!actioned) {
            mspSendV1Response(msg.cmd, nullptr, 0);   // unsupported mode: ACK only
            break;
        }

        bufWrite8(mspRespBuf, idx, mode);             // BF echoes the mode
        mspSendV1Response(msg.cmd, mspRespBuf, idx);
        mspSerial->flush();
        delay(50);                                    // let the host read the ACK

        switch (mode) {
        case MSP_REBOOT_BOOTLOADER_ROM:
            requestSystemBootloader();                // .noinit marker + reset; premain hook jumps to ST DFU
            break;
#ifdef BL_BOOTUF2
        case MSP_REBOOT_BOOTLOADER_FLASH:
            enterBootloader();                        // bootuf2 double-tap marker + reset
            break;
#endif
        case MSP_REBOOT_FIRMWARE:
        default:
            NVIC_SystemReset();
            break;
        }
        break;   // unreachable
    }

    //--- Unknown: empty ACK ---

    default:
        mspSendV1Response(msg.cmd, nullptr, 0);
        break;
    }
}

//========================================================================================================================//
// MSP V1 Frame Parser
//========================================================================================================================//

bool mspProcessByte(uint8_t c) {
    uint32_t now = millis();

    // Frame timeout: reset if mid-frame and byte gap exceeds threshold
    if (mspState != MSP_IDLE && (now - mspLastByteTimeMs) > MSP_FRAME_TIMEOUT_MS) {
        mspState = MSP_IDLE;
    }
    mspLastByteTimeMs = now;

    switch (mspState) {

    case MSP_IDLE:
        if (c == MSP_PREAMBLE_1) {
            mspState = MSP_HEADER_START;
            return true;
        }
        return false;

    case MSP_HEADER_START:
        if (c == MSP_V1_HEADER) {
            mspState = MSP_HEADER_M;
            return true;
        }
        mspState = MSP_IDLE;
        return false;

    case MSP_HEADER_M:
        if (c == MSP_DIR_REQUEST) {
            mspState = MSP_HEADER_V1;
            return true;
        }
        mspState = MSP_IDLE;
        return false;

    case MSP_HEADER_V1:
        mspV1PayloadLen = c;
        mspV1Checksum = c;
        mspState = MSP_PAYLOAD_SIZE_V1;
        return true;

    case MSP_PAYLOAD_SIZE_V1:
        mspV1Checksum ^= c;
        mspMsg.cmd = c;
        mspMsg.payloadSize = mspV1PayloadLen;
        mspPayloadIdx = 0;
        if (mspV1PayloadLen > 0) {
            if (mspV1PayloadLen > MSP_MAX_PAYLOAD) {
                mspState = MSP_IDLE;
                return false;
            }
            mspState = MSP_PAYLOAD_V1;
        } else {
            mspState = MSP_CHECKSUM_V1;
        }
        return true;

    case MSP_PAYLOAD_V1:
        mspMsg.payload[mspPayloadIdx++] = c;
        mspV1Checksum ^= c;
        if (mspPayloadIdx >= mspV1PayloadLen) {
            mspState = MSP_CHECKSUM_V1;
        }
        return true;

    case MSP_CHECKSUM_V1:
        mspState = MSP_IDLE;
        if (mspV1Checksum == c) {
            mspHandleMessage(mspMsg);
        }
        return true;

    default:
        mspState = MSP_IDLE;
        return false;
    }
}

//========================================================================================================================//
// MSP Setup
//========================================================================================================================//

void mspSetup() {
    mspSerial = &Serial;
    mspState = MSP_IDLE;
}
