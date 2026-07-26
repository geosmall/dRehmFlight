//========================================================================================================================//
//                                          CLI HANDLER + MODAL SERIAL PROCESSOR                                        //
//========================================================================================================================//
//
// EmbeddedCLI commands for on-demand debug output and parameter tuning.
// Modal serial processor: MSP (default) or CLI (entered via '#').
//
// Session flow:
//   [Connect 115200] → MSP mode (telemetry polling)
//   [Send '#']       → CLI mode (text commands, 100 ms guard)
//   [Type 'exit']    → Reboot (BF/INAV pattern)
//   [Type 'reboot']  → Reboot
//

#include <EmbeddedCLI.h>
#include <bootloader.h>
#include "msp.h"           // mspTargetName(), FC_VERSION_*

// Build provenance: build_id.h is generated automatically by the core prebuild
// hook on every compile. Guarded so an out-of-tree build still compiles.
#if __has_include("build_id.h")
  #include "build_id.h"
#else
  #define BUILD_GIT_SHA      "unknown"
  #define BUILD_GIT_BRANCH   "unknown"
  #define BUILD_UTC_TIME     "unknown"
  #define BUILD_CORE_GIT_SHA "unknown"
#endif

//========================================================================================================================//
// Modal State
//========================================================================================================================//

static bool cliMode = false;
static EmbeddedCLI cli(Serial);

// CLI entry guard — 100 ms quiet period after '#' prevents accidental CLI entry
// from 0x23 bytes in corrupt MSP streams (matches BF/INAV pattern)
static constexpr uint32_t CLI_GUARD_MS = 100;
static uint32_t cliPendingMs = 0;
static bool cliPending = false;

//========================================================================================================================//
// CLI Command Handlers
//========================================================================================================================//

// Command table — single source for both registration and help formatting
struct CmdEntry {
    const char* name;
    const char* help;
    CLICommandHandler handler;
};

static void cmd_help(int argc, char** argv);     // forward declaration
static void cmd_cal(int argc, char** argv);      // forward declaration
static void cmd_save(int argc, char** argv);     // forward declaration (cmd_cal auto-saves, like Betaflight)
static void cmd_diff(int argc, char** argv);     // forward declaration
static void cmd_version(int argc, char** argv);  // forward declaration
static void cmd_comment(int argc, char** argv);  // forward declaration ('#' paste-back no-op)
static void cmd_motor(int argc, char** argv);    // forward declaration (props-off single-motor test)
static void cmd_bb(int argc, char** argv);       // forward declaration (blackbox readout)

static const CmdEntry cmdTable[] = {
    {"help",        "Show commands",           cmd_help},
    {"status",      "Arm state, loop, load",   cmd_status},
    {"version",     "Firmware build provenance", cmd_version},
    {"set",         "Get/set RAM parameter",   cmd_set},
    {"diff",        "Print changed params ('diff all' = every param)", cmd_diff},
    {"save",        "Save RAM to flash",       cmd_save},
    {"motor",       "Single-motor bench test, PROPS OFF ('motor <1-4> <pct>' / 'motor stop')", cmd_motor},
    {"bb",          "Blackbox status ('bb dump' = last-flight CSV)", cmd_bb},
    {"cal",         "Calibrate+save accel (level+still); gyro auto at boot", cmd_cal},
    {"defaults",    "Reset RAM to defaults",   cmd_defaults},
    {"dump",        "Print raw flash config (INI; debug)", cmd_dump},
    {"exit",        "Exit CLI mode",           cmd_exit},
    {"reboot",      "Reboot FC",               cmd_reboot},
#ifdef BL_BOOTUF2
    {"bl",          "Enter UF2 bootloader",    cmd_bl},
#endif
};
static constexpr int CMD_COUNT = sizeof(cmdTable) / sizeof(cmdTable[0]);

static void cmd_help(int argc, char** argv) {
    (void)argc; (void)argv;
    // Find longest command name for alignment
    uint8_t maxLen = 0;
    for (int i = 0; i < CMD_COUNT; i++) {
        uint8_t len = strlen(cmdTable[i].name);
        if (len > maxLen) maxLen = len;
    }
    Serial.println("Commands:");
    for (int i = 0; i < CMD_COUNT; i++) {
        Serial.print("  ");
        Serial.print(cmdTable[i].name);
        uint8_t pad = maxLen - strlen(cmdTable[i].name) + 2;
        for (uint8_t p = 0; p < pad; p++) Serial.print(' ');
        Serial.print("- ");
        Serial.println(cmdTable[i].help);
    }
}

static void cmd_status(int argc, char** argv) {
    (void)argc; (void)argv;
    int loopHz = (dt > 0) ? (int)(1.0f / dt) : 0;
    Serial.printf("Armed: %s  Loop: %d Hz  Load: %d%%\n",
                  armedFly ? "YES" : "NO", loopHz, averageSystemLoadPercent);
}

//========================================================================================================================//
// Parameter Table (25 entries — shared with 'set', 'save', 'defaults')
//========================================================================================================================//

struct ParamEntry {
    const char* name;
    float* ptr;
    float min;
    float max;
    const char* group;
};

static const ParamEntry paramTable[] = {
    // PID gains — angle mode
    {"Kp_roll_angle",  &Kp_roll_angle,  0, 10, "PID Angle"},
    {"Ki_roll_angle",  &Ki_roll_angle,  0, 1,  "PID Angle"},
    {"Kd_roll_angle",  &Kd_roll_angle,  0, 1,  "PID Angle"},
    {"Kp_pitch_angle", &Kp_pitch_angle, 0, 10, "PID Angle"},
    {"Ki_pitch_angle", &Ki_pitch_angle, 0, 1,  "PID Angle"},
    {"Kd_pitch_angle", &Kd_pitch_angle, 0, 1,  "PID Angle"},
    // PID gains — yaw
    {"Kp_yaw",         &Kp_yaw,         0, 10, "PID Yaw"},
    {"Ki_yaw",         &Ki_yaw,         0, 1,  "PID Yaw"},
    {"Kd_yaw",         &Kd_yaw,         0, 1,  "PID Yaw"},
    // PID gains — rate mode
    {"Kp_roll_rate",   &Kp_roll_rate,   0, 10, "PID Rate"},
    {"Ki_roll_rate",   &Ki_roll_rate,   0, 1,  "PID Rate"},
    {"Kd_roll_rate",   &Kd_roll_rate,   0, 1,  "PID Rate"},
    {"Kp_pitch_rate",  &Kp_pitch_rate,  0, 10, "PID Rate"},
    {"Ki_pitch_rate",  &Ki_pitch_rate,  0, 1,  "PID Rate"},
    {"Kd_pitch_rate",  &Kd_pitch_rate,  0, 1,  "PID Rate"},
    // Loop damping
    {"B_loop_roll",    &B_loop_roll,    0, 1,  "Loop Damping"},
    {"B_loop_pitch",   &B_loop_pitch,   0, 1,  "Loop Damping"},
    // Controller limits
    {"i_limit",        &i_limit,        0, 100, "Limits"},
    {"maxRoll",        &maxRoll,        0, 90,  "Limits"},
    {"maxPitch",       &maxPitch,       0, 90,  "Limits"},
    {"maxYaw",         &maxYaw,         0, 500, "Limits"},
    // Throttle curve (Betaflight-style mid/expo + scale limit)
    {"thr_mid",        &thr_mid,            0, 1,  "Throttle"},
    {"thr_expo",       &thr_expo,           0, 1,  "Throttle"},
    {"throttle_limit", &throttle_limit_pct, 0, 1,  "Throttle"},
    // Mixer
    {"yaw_motors_reversed", &yaw_motors_reversed, 0, 1,   "Mixer"},
    {"motor_idle",     &motor_idle,         0, 0.2, "Mixer"},
    // Board alignment (Betaflight align_board_*): per-airframe FC mounting, degrees.
    // Applied on top of the per-target chip alignment; takes effect after save + reboot.
    {"align_board_roll",  &board_align_roll_degrees,  -180, 360, "Board"},
    {"align_board_pitch", &board_align_pitch_degrees, -180, 360, "Board"},
    {"align_board_yaw",   &board_align_yaw_degrees,   -180, 360, "Board"},
    // Filter coefficients
    {"B_madgwick",     &B_madgwick,     0, 1,  "Filters"},
    {"B_accel",        &B_accel,        0, 1,  "Filters"},
    {"B_gyro",         &B_gyro,         0, 1,  "Filters"},
    {"B_mag",          &B_mag,          0, 1,  "Filters"},
    // IMU accel calibration (persistent). Gyro bias is auto-measured at boot, not stored here.
    {"AccErrorX",      &AccErrorX,     -1, 1,  "IMU Cal"},
    {"AccErrorY",      &AccErrorY,     -1, 1,  "IMU Cal"},
    {"AccErrorZ",      &AccErrorZ,     -1, 1,  "IMU Cal"},
};
static constexpr int PARAM_COUNT = sizeof(paramTable) / sizeof(paramTable[0]);

// Defaults captured from global initializers at startup (before flash values overwrite them).
// This keeps default values defined in exactly one place: the main .ino global declarations.
static float paramDefaults[PARAM_COUNT];

//========================================================================================================================//
// Motor output reordering (Betaflight motor_output_reordering) — a CSV permutation param that
// doesn't fit the scalar paramTable, so it is handled as a special case in set / diff / INI.
//========================================================================================================================//
static const char MOTOR_REORDER_KEY[] = "motor_output_reordering";

static void motorReorderIdentity() {
    for (int i = 0; i < 8; i++) motor_reorder[i] = (uint8_t)i;
}

// Validate motor_reorder over the first num_motors entries: it must be a permutation of
// 0..num_motors-1, else reset to identity (fail-safe, mirrors BF validateAndfixMotorOutputReordering).
// A bad remap would otherwise dispatch a motor command to an out-of-range or duplicate output.
static void validateMotorReorder() {
    const int n = BoardConfig::Motor::num_motors;
    bool seen[8] = {false};
    for (int i = 0; i < n; i++) {
        if (motor_reorder[i] >= n || seen[motor_reorder[i]]) { motorReorderIdentity(); return; }
        seen[motor_reorder[i]] = true;
    }
}

// Parse "2,3,0,1" (comma-separated indices) into motor_reorder; unspecified trailing entries
// stay identity. Always validated afterwards.
static void parseMotorReorder(const char* val) {
    motorReorderIdentity();
    int i = 0;
    const char* p = val;
    while (*p && i < 8) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p < '0' || *p > '9') break;
        int v = 0;
        while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; }
        motor_reorder[i++] = (uint8_t)v;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ',') p++;
    }
    validateMotorReorder();
}

// Format motor_reorder (first num_motors entries) as "2,3,0,1" into buf.
static void motorReorderToStr(char* buf) {
    const int n = BoardConfig::Motor::num_motors;
    char* p = buf;
    *p = '\0';
    for (int i = 0; i < n; i++) p += sprintf_(p, "%s%u", i ? "," : "", (unsigned)motor_reorder[i]);
}

//========================================================================================================================//
// PID Persistence — Internal Flash via Shared Header
//========================================================================================================================//
// On boards with BOARD_FLASH_CONFIG_START (flight controllers with UF2 bootloader),
// parameters are stored in a dedicated internal flash region using the same append-log
// format as the bootloader. On boards without it (Nucleos), tuning is RAM-only.

// Parse INI text from buffer and apply matching key=value pairs to paramTable
static void applyIniFromBuffer(const uint8_t *buf, uint32_t size) {
    const char *p = (const char *)buf;
    const char *end = p + size;

    while (p < end) {
        // Skip whitespace
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
        if (p >= end) break;

        // Skip section headers and comments
        if (*p == '[' || *p == ';' || *p == '#') {
            while (p < end && *p != '\n') p++;
            continue;
        }

        // Find key=value
        const char *key_start = p;
        while (p < end && *p != '=' && *p != '\n') p++;
        if (p >= end || *p != '=') continue;
        size_t key_len = (size_t)(p - key_start);
        p++;  // skip '='

        const char *val_start = p;
        while (p < end && *p != '\n' && *p != '\r') p++;

        // Array param (motor_output_reordering) — handled outside the scalar paramTable.
        if (key_len == strlen(MOTOR_REORDER_KEY) &&
            memcmp(key_start, MOTOR_REORDER_KEY, key_len) == 0) {
            char vbuf[48];
            size_t vlen = (size_t)(p - val_start);
            if (vlen >= sizeof(vbuf)) vlen = sizeof(vbuf) - 1;
            memcpy(vbuf, val_start, vlen);
            vbuf[vlen] = '\0';
            parseMotorReorder(vbuf);
            continue;
        }

        // Match key against paramTable
        for (int i = 0; i < PARAM_COUNT; i++) {
            if (strlen(paramTable[i].name) == key_len &&
                memcmp(paramTable[i].name, key_start, key_len) == 0) {
                char vbuf[32];
                size_t vlen = (size_t)(p - val_start);
                if (vlen >= sizeof(vbuf)) vlen = sizeof(vbuf) - 1;
                memcpy(vbuf, val_start, vlen);
                vbuf[vlen] = '\0';
                *paramTable[i].ptr = atof(vbuf);
                break;
            }
        }
    }
}

// Format all parameters as INI text into buffer.
// Returns number of bytes written (not including null terminator).
static uint32_t formatIniText(char *buf, uint32_t buf_size) {
    char *p = buf;
    char *end = buf + buf_size - 1;  // leave room for null
    p += sprintf_(p, "[pid]\n");
    for (int i = 0; i < PARAM_COUNT && p < end - 40; i++) {
        p += sprintf_(p, "%s=%f\n", paramTable[i].name, *paramTable[i].ptr);
    }
    if (p < end - 60) {  // motor_output_reordering (CSV array param, not in paramTable)
        char b[48]; motorReorderToStr(b);
        p += sprintf_(p, "%s=%s\n", MOTOR_REORDER_KEY, b);
    }
    return (uint32_t)(p - buf);
}

#ifdef BOARD_FLASH_CONFIG_START
//========================================================================================================================//
// Per-Family HAL Flash Primitives (required by ini_flash_config.h)
//========================================================================================================================//

#if defined(STM32F4xx) || defined(STM32F7xx)
// F4/F7: 4-byte word programming, sector-based erase
#define INI_FLASH_ALIGN  BOARD_FLASH_CONFIG_ALIGN

static void ini_erase_config(void) {
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector((BOARD_FLASH_CONFIG_START - FLASH_BASE) / 0x4000, FLASH_VOLTAGE_RANGE_3);
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);
    HAL_FLASH_Lock();
}

static bool ini_program(uint32_t addr, const uint8_t *data, uint32_t len) {
    HAL_FLASH_Unlock();
    uint8_t pad_buf[4];
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t remaining = len - i;
        uint32_t word;
        if (remaining >= 4) {
            memcpy(&word, data + i, 4);
        } else {
            memset(pad_buf, 0xFF, 4);
            memcpy(pad_buf, data + i, remaining);
            memcpy(&word, pad_buf, 4);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, (uint64_t)word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);
    }
    HAL_FLASH_Lock();
    return true;
}

#elif defined(STM32G4xx)
// G4: 8-byte doubleword programming, page-based erase
#define INI_FLASH_ALIGN  BOARD_FLASH_CONFIG_ALIGN

static void ini_erase_config(void) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (BOARD_FLASH_CONFIG_START - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = BOARD_FLASH_CONFIG_SIZE / FLASH_PAGE_SIZE;
    uint32_t err = 0;
    HAL_FLASHEx_Erase(&erase, &err);
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);
    HAL_FLASH_Lock();
}

static bool ini_program(uint32_t addr, const uint8_t *data, uint32_t len) {
    HAL_FLASH_Unlock();
    uint8_t pad_buf[8];
    for (uint32_t i = 0; i < len; i += 8) {
        uint32_t remaining = len - i;
        uint64_t dword;
        if (remaining >= 8) {
            memcpy(&dword, data + i, 8);
        } else {
            memset(pad_buf, 0xFF, 8);
            memcpy(pad_buf, data + i, remaining);
            memcpy(&dword, pad_buf, 8);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);
    }
    HAL_FLASH_Lock();
    return true;
}

#elif defined(STM32H7xx)
// H7: 32-byte flash word programming, sector-based erase, D-Cache sync
#define INI_FLASH_ALIGN  BOARD_FLASH_CONFIG_ALIGN
#define INI_CACHE_SYNC(addr, len) \
    do { SCB_InvalidateDCache_by_Addr((void *)(addr), (len)); } while (0)

static void ini_erase_config(void) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {};
    uint32_t bank = (BOARD_FLASH_CONFIG_START >= 0x08100000UL) ? FLASH_BANK_2 : FLASH_BANK_1;
    uint32_t bank_base = (bank == FLASH_BANK_2) ? 0x08100000UL : FLASH_BASE;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = bank;
    erase.Sector = (BOARD_FLASH_CONFIG_START - bank_base) / 0x20000;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t err = 0;
    HAL_FLASHEx_Erase(&erase, &err);

    SCB_InvalidateDCache_by_Addr((void *)BOARD_FLASH_CONFIG_START, BOARD_FLASH_CONFIG_SIZE);
    __DSB();

    HAL_FLASH_Lock();
}

// 32-byte aligned buffer for H7 flash word programming
static uint32_t ini_flash_word[8] __attribute__((aligned(32)));

static bool ini_program(uint32_t addr, const uint8_t *data, uint32_t len) {
    if ((addr & 0x1F) != 0) return false;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < len; i += 32) {
        for (int j = 0; j < 8; j++) {
            int off = i + (j * 4);
            if (off + 4 <= (int)len) {
                memcpy(&ini_flash_word[j], data + off, 4);
            } else if (off < (int)len) {
                ini_flash_word[j] = 0xFFFFFFFF;
                memcpy(&ini_flash_word[j], data + off, len - off);
            } else {
                ini_flash_word[j] = 0xFFFFFFFF;
            }
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i, (uint32_t)ini_flash_word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        uint32_t prog_bank = (addr + i >= 0x08100000UL) ? FLASH_BANK_2 : FLASH_BANK_1;
        FLASH_WaitForLastOperation(HAL_MAX_DELAY, prog_bank);
    }

    SCB_InvalidateDCache_by_Addr((void *)addr, len);
    __DSB();
    HAL_FLASH_Lock();
    return true;
}

#else
#error "BOARD_FLASH_CONFIG_START defined but unsupported STM32 family for INI flash config"
#endif

// Shared append-log scan, read, and write logic
#include <ini_flash_config.h>

#endif // BOARD_FLASH_CONFIG_START

// Initialize and load saved config (called from setup())
void loadConfig() {
    // Snapshot compile-time defaults before flash values overwrite them
    for (int i = 0; i < PARAM_COUNT; i++) {
        paramDefaults[i] = *paramTable[i].ptr;
    }
#ifdef BOARD_FLASH_CONFIG_START
    static uint8_t iniBuf[1024];
    uint32_t iniSize = board_flash_ini_read(iniBuf, sizeof(iniBuf));
    if (iniSize > 0) {
        applyIniFromBuffer(iniBuf, iniSize);
    }
#endif
}

//========================================================================================================================//
// Set / Save / Defaults / Dump Commands
//========================================================================================================================//

/** Find a parameter by name, or -1 if not found */
static int findParam(const char* name) {
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(name, paramTable[i].name) == 0) return i;
    }
    return -1;
}

static void printParam(int i) {
    Serial.print(paramTable[i].name);
    Serial.print(" = ");
    Serial.print(*paramTable[i].ptr, 6);
    Serial.print(" [");
    Serial.print(paramTable[i].min, 6);
    Serial.print(":");
    Serial.print(paramTable[i].max, 6);
    Serial.print("] {");
    Serial.print(paramTable[i].group);
    Serial.println("}");
}

static void cmd_set(int argc, char** argv) {
    if (argc < 2) {
        for (int i = 0; i < PARAM_COUNT; i++) printParam(i);
        char b[48]; motorReorderToStr(b);
        Serial.printf("%s = %s {Motor}\n", MOTOR_REORDER_KEY, b);
        return;
    }
    // Array param (Betaflight motor_output_reordering) — CSV, outside the scalar paramTable.
    if (strcmp(argv[1], MOTOR_REORDER_KEY) == 0) {
        if (argc >= 3) {
            int vi = (strcmp(argv[2], "=") == 0 && argc >= 4) ? 3 : 2;
            parseMotorReorder(argv[vi]);
        }
        char b[48]; motorReorderToStr(b);
        Serial.printf("%s = %s {Motor}\n", MOTOR_REORDER_KEY, b);
        return;
    }
    int idx = findParam(argv[1]);
    if (idx < 0) { Serial.printf("Unknown parameter: %s\n", argv[1]); return; }
    if (argc >= 3) {
        // Tolerate "set name = value": skip a lone '=' token.
        int vi = (strcmp(argv[2], "=") == 0 && argc >= 4) ? 3 : 2;
        // Strict numeric parse — reject garbage rather than silently writing atof()'s 0,
        // which could set a flight-critical param (e.g. yaw_motors_reversed) to a dangerous value.
        char* endp = nullptr;
        float v = (float)strtod(argv[vi], &endp);
        if (endp == argv[vi] || *endp != '\0') {
            Serial.printf("Invalid value '%s' (use: set %s <number>)\n", argv[vi], paramTable[idx].name);
            return;
        }
        if (v < paramTable[idx].min || v > paramTable[idx].max) {
            Serial.print("Out of range [");
            Serial.print(paramTable[idx].min, 6);
            Serial.print(":");
            Serial.print(paramTable[idx].max, 6);
            Serial.println("]");
            return;
        }
        *paramTable[idx].ptr = v;
    }
    printParam(idx);
}

// Props-off single-motor bench test (Betaflight 'motor' command). Drives ONE physical
// motors[] output directly — bypassing the mixer and motor_output_reordering — so each
// output's physical corner and spin direction can be identified visually. Disarm-gated;
// commandMotors() clears it instantly on arm and auto-expires it after 60 s.
static void cmd_motor(int argc, char** argv) {
    if (armedFly) { Serial.println("Refused: disarm first"); return; }
    if (argc >= 2 && strcmp(argv[1], "stop") == 0) {
        motor_test_idx = -1;
        Serial.println("Motor test stopped");
        return;
    }
    if (argc < 3) {
        Serial.println("Usage: motor <1-4> <percent 1-25> | motor stop");
        Serial.println("PROPS OFF. Drives one physical output (bypasses motor_output_reordering).");
        if (motor_test_idx >= 0) {
            Serial.printf("Active: motor %d at %d%%\n", motor_test_idx + 1,
                          (int)(motor_test_value * 100.0f + 0.5f));
        }
        return;
    }
    int n = atoi(argv[1]);
    int pct = atoi(argv[2]);
    if (n < 1 || n > BoardConfig::Motor::num_motors) { Serial.println("Bad motor index"); return; }
    if (pct < 1 || pct > 25) { Serial.println("Percent must be 1-25"); return; }
    motor_test_value = pct / 100.0f;
    motor_test_idx = n - 1;
    motor_test_until_ms = millis() + 60000UL;
    Serial.printf("Motor %d (motors[%d]) at %d%% — PROPS OFF. Auto-stop in 60 s; 'motor stop' to stop.\n",
                  n, n - 1, pct);
}

// In-RAM blackbox readout. 'bb' = status; 'bb dump' = CSV of the frozen last flight,
// oldest record first, terminated with a BB_END sentinel for host-side capture
// (ci/bb_fetch.py). Dump is disarm-gated.
static void cmd_bb(int argc, char** argv) {
    if (argc >= 2 && strcmp(argv[1], "dump") == 0) {
        if (armedFly) { Serial.println("Refused: disarm first"); return; }
        Serial.println("t_ms,roll_ddeg,pitch_ddeg,rdes_ddeg,pdes_ddeg,gx_ddps,gy_ddps,rpid_e4,ppid_e4,m1,m2,m3,m4");
        int start = (bb_head + BB_SAMPLES - bb_count) % BB_SAMPLES;
        for (int i = 0; i < bb_count; i++) {
            const BBRecord& r = bb_buf[(start + i) % BB_SAMPLES];
            //USB-CDC TX silently drops data whenever the host stalls its IN polling
            //for more than USB_CDC_TRANSMIT_TIMEOUT (3 ms): USBSerial::write() gives
            //up mid-buffer (CDC_connected() goes false on the stalled transfer) and
            //a fire-and-forget printf never learns. With the 128-byte TX queue a
            //full-ring dump keeps the queue saturated, so any host hiccup truncates
            //the CSV nondeterministically. Format each row into a buffer and write
            //with explicit retry of the unsent remainder; a row that cannot be
            //delivered within the deadline means the host is really gone -- stop
            //with an explicit BB_ABORT marker rather than dropping rows silently.
            char line[112];
            int len = snprintf(line, sizeof(line),
                               "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u\r\n",
                               (unsigned long)r.t_ms, r.roll_ddeg, r.pitch_ddeg,
                               r.rdes_ddeg, r.pdes_ddeg,
                               r.gx_ddps, r.gy_ddps, r.rpid_e4, r.ppid_e4,
                               r.m[0], r.m[1], r.m[2], r.m[3]);
            if (len >= (int)sizeof(line)) len = sizeof(line) - 1;
            int off = 0;
            uint32_t row_deadline = millis() + 250;
            while (off < len) {
                off += Serial.write((const uint8_t*)line + off, (size_t)(len - off));
                if (off < len) {
                    if (millis() > row_deadline) {
                        Serial.println();
                        Serial.println("BB_ABORT");
                        return;
                    }
                    delay(1);  //let the stalled USB transfer complete, then resend the rest
                }
            }
        }
        Serial.println("BB_END");
        return;
    }
    Serial.printf("Blackbox: %u records (%s), ~%u s at 100 Hz. 'bb dump' for CSV.\n",
                  (unsigned)bb_count, bb_frozen ? "frozen" : "live",
                  (unsigned)(bb_count / 100));
}

static void cmd_cal(int argc, char** argv) {
    (void)argc; (void)argv;
    if (armedFly) { Serial.println("Refused: disarm first"); return; }
    Serial.println("Accel cal: hold the board LEVEL and STILL...");
    calibrateAccel();
    Serial.print("AccErrorX = "); Serial.print(AccErrorX, 4);
    Serial.print("  AccErrorY = "); Serial.print(AccErrorY, 4);
    Serial.print("  AccErrorZ = "); Serial.println(AccErrorZ, 4);
    cmd_save(0, nullptr);  //auto-persist to config flash, like Betaflight's accel cal
}

static void cmd_save(int argc, char** argv) {
    (void)argc; (void)argv;
#ifdef BOARD_FLASH_CONFIG_START
    static char iniBuf[1024];
    uint32_t len = formatIniText(iniBuf, sizeof(iniBuf));
    if (board_flash_ini_write_block((const uint8_t *)iniBuf, len, 0, len, 0, 1)) {
        Serial.println("Saved to config flash");
    } else {
        Serial.println("Save failed");
    }
#else
    Serial.println("No config flash on this board");
#endif
}

static void cmd_defaults(int argc, char** argv) {
    (void)argc; (void)argv;
    for (int i = 0; i < PARAM_COUNT; i++) {
        *paramTable[i].ptr = paramDefaults[i];
    }
    motorReorderIdentity();  // array param isn't in paramTable; reset it too
    Serial.print("Reset ");
    Serial.print(PARAM_COUNT);
    Serial.println(" parameters to compile-time defaults");
}

static void cmd_dump(int argc, char** argv) {
    (void)argc; (void)argv;
#ifdef BOARD_FLASH_CONFIG_START
    static uint8_t buf[1024];
    uint32_t size = board_flash_ini_read(buf, sizeof(buf) - 1);
    if (size == 0) {
        Serial.println("(no config stored)");
        return;
    }
    buf[size] = '\0';
    Serial.println((const char *)buf);
#else
    Serial.println("No config flash on this board");
#endif
}

//========================================================================================================================//
// Provenance: version / diff / diff all
//========================================================================================================================//
// A saved tune must be tied to the exact firmware that produced it and be a complete,
// restorable state. `diff all` emits every param (version-robust — pins absolute values)
// under a build-stamped header; `diff` emits only changed params for quick inspection.
// Lines are `set name = value`, which cmd_set accepts verbatim, so output pastes back.

static void printProvenanceHeader(const char* tag) {
    // '#'-prefixed: treated as a comment by cmd_comment on paste-back (BF convention).
    Serial.print("# ");
    Serial.print(tag);
    Serial.print(" - fw_git: ");
    Serial.print(BUILD_GIT_SHA);
    Serial.print("  core: ");
    Serial.print(BUILD_CORE_GIT_SHA);
    Serial.print("  branch: ");
    Serial.print(BUILD_GIT_BRANCH);
    Serial.print("  commit: ");   //HEAD committer date (reproducible), not build wall-clock
    Serial.println(BUILD_UTC_TIME);
    Serial.print("# board: ");
    Serial.print(mspTargetName());
    Serial.print(" / DRHM ");
    Serial.print(FC_VERSION_MAJOR); Serial.print('.');
    Serial.print(FC_VERSION_MINOR); Serial.print('.');
    Serial.println(FC_VERSION_PATCH);
}

static void cmd_version(int argc, char** argv) {
    (void)argc; (void)argv;
    printProvenanceHeader("dRehmFlight");
}

static void cmd_diff(int argc, char** argv) {
    const bool all = (argc >= 2 && strcmp(argv[1], "all") == 0);
    printProvenanceHeader(all ? "dRehmFlight diff all" : "dRehmFlight diff");
    Serial.println();  // blank line between header and directives (Betaflight style)
    for (int i = 0; i < PARAM_COUNT; i++) {
        const float v = *paramTable[i].ptr;
        // `diff` shows only params that differ from the compile-time default snapshot
        // (paramDefaults[], captured in loadConfig before flash overlay).
        if (!all && fabsf(v - paramDefaults[i]) <= 1e-6f) continue;
        Serial.print("set ");
        Serial.print(paramTable[i].name);
        Serial.print(" = ");
        Serial.println(v, 6);
    }
    // motor_output_reordering (array param): always in `diff all`; in `diff` only if non-identity.
    {
        bool identity = true;
        for (int i = 0; i < BoardConfig::Motor::num_motors; i++)
            if (motor_reorder[i] != i) { identity = false; break; }
        if (all || !identity) {
            char b[48]; motorReorderToStr(b);
            Serial.print("set "); Serial.print(MOTOR_REORDER_KEY);
            Serial.print(" = "); Serial.println(b);
        }
    }
}

// '#'-prefixed lines are comments (the provenance header in a pasted diff). No-op so a
// whole diff block (header + set lines) pastes back cleanly. Registered in cliSetup,
// kept out of cmdTable so it stays off the help list.
static void cmd_comment(int argc, char** argv) {
    (void)argc; (void)argv;
}

//========================================================================================================================//
// Mode Switch Commands
//========================================================================================================================//

static void cmd_exit(int argc, char** argv) {
    (void)argc; (void)argv;
    Serial.println("Exiting CLI...");
    cliMode = false;
}

static void cmd_reboot(int argc, char** argv) {
    (void)argc; (void)argv;
    Serial.println("Rebooting...");
    delay(100);
    NVIC_SystemReset();
}

#ifdef BL_BOOTUF2
static void cmd_bl(int argc, char** argv) {
    (void)argc; (void)argv;
    Serial.println("Entering UF2 bootloader...");
    delay(100);
    enterBootloader();
}
#endif

//========================================================================================================================//
// CLI Setup
//========================================================================================================================//

void cliSetup() {
    for (int i = 0; i < CMD_COUNT; i++) {
        cli.addCommand(cmdTable[i].name, cmdTable[i].help, cmdTable[i].handler);
    }
    // Paste-back comment handler: '#'-prefixed lines (provenance headers) are no-ops.
    // Registered here (not in cmdTable) so it doesn't appear in the help list.
    cli.addCommand("#", "comment", cmd_comment);
}

//========================================================================================================================//
// CLI Enter
//========================================================================================================================//

static void cliEnter() {
    cliMode = true;
    Serial.println("\r\ndRehmFlight CLI");
    Serial.println("Type 'help' for commands, 'exit' to return, 'reboot' to restart\r\n");
    cli.begin("# ");
}

//========================================================================================================================//
// Modal Serial Processor (called by TASK_SERIAL at 100 Hz)
//========================================================================================================================//

void processSerial() {
    if (cliMode) {
        // MSP preamble ('$') while in CLI → configurator expects MSP mode, auto-exit
        if (Serial.available() && Serial.peek() == '$') {
            cliMode = false;
            return;  // Next call processes '$' through MSP parser
        }
        cli.process();
        return;
    }

    // Check pending CLI entry (100 ms guard timer)
    if (cliPending) {
        if (Serial.available()) {
            uint8_t peek = Serial.peek();
            if (peek == '\r' || peek == '\n') {
                Serial.read();  // Consume newline trailing '#' from terminals
                return;         // Keep waiting for guard period
            }
            // Non-whitespace byte — '#' was part of an MSP stream, cancel
            cliPending = false;
            // Fall through to normal MSP processing
        } else if ((millis() - cliPendingMs) > CLI_GUARD_MS) {
            // Quiet period elapsed — safe to enter CLI
            cliPending = false;
            if (!armedFly) {
                cliEnter();
                return;
            }
        } else {
            return;  // Still waiting for guard period
        }
    }

    // MSP mode — process incoming bytes
    while (Serial.available()) {
        uint8_t c = Serial.read();
        bool consumed = mspProcessByte(c);

        // Non-MSP byte when disarmed — start CLI guard timer
        if (!consumed && !armedFly) {
            if (c == '#') {
                cliPendingMs = millis();
                cliPending = true;
                return;
            }
        }
    }
}
