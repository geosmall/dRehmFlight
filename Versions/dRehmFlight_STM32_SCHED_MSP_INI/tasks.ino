//========================================================================================================================//
//                                              SCHEDULER TASK CONFIGURATION                                             //
//========================================================================================================================//

// Forward declarations for task functions
extern "C" void taskFlight(timeUs_t currentTimeUs);
extern "C" void taskRC(timeUs_t currentTimeUs);
extern "C" void taskSerial(timeUs_t currentTimeUs);
extern "C" void taskBlink(timeUs_t currentTimeUs);

//========================================================================================================================//
// Task Configuration Table
//========================================================================================================================//

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = SCHEDULER_TASK_SYSTEM_INIT,

    [TASK_FLIGHT] = {
        .taskName = "FLIGHT",
        .taskFunc = taskFlight,
        .desiredPeriod = TASK_PERIOD_HZ(2000),  // 2000 Hz (500 us) - core flight loop
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_RC] = {
        .taskName = "RC",
        .taskFunc = taskRC,
        .desiredPeriod = TASK_PERIOD_HZ(500),   // 500 Hz (2 ms) - radio input
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),   // 100 Hz (10 ms) - MSP/CLI processor
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_BLINK] = {
        .taskName = "BLINK",
        .taskFunc = taskBlink,
        .desiredPeriod = TASK_PERIOD_HZ(2),     // 2 Hz (500 ms) - LED heartbeat
        .staticPriority = TASK_PRIORITY_LOW,
    },
};

//========================================================================================================================//
// Task Implementations
//========================================================================================================================//

/**
 * TASK_FLIGHT - Core flight control loop at 2000 Hz (REALTIME priority)
 *
 * Contains the time-critical flight path:
 * - IMU read + filtering
 * - Madgwick sensor fusion
 * - PID control
 * - Actuator mixing
 * - Motor/servo output
 * - Safety checks (throttleCut)
 */
extern "C" void taskFlight(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // Calculate dt using scheduler's delta time (critical for Madgwick filter and PID)
    timeDelta_t deltaTime = getTaskDeltaTime((cfTaskId_e)TASK_SELF);
    dt = deltaTime / 1000000.0f;

    // Safety clamp: protect against first iteration (0) and scheduler anomalies
    if (dt <= 0.0f || dt > 0.01f) {
        dt = 0.0005f;  // Default to 500us = 2000 Hz nominal
    }

    // Get arming status (reads channel_5_pwm set by TASK_RC)
    armedStatus();

    // Get vehicle state
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);

    // Compute desired state
    getDesState();

    // PID Controller
    controlANGLE();

    // Actuator mixing and scaling
    controlMixer();
    scaleCommands();

    // CRITICAL SAFETY: Throttle cut check (must be last before motor output)
    throttleCut();

    // Command actuators
    commandMotors();
    commandServos();

    // Blackbox: record flight state at 100 Hz while armed (freezes at disarm)
    bbSample();
}

/**
 * TASK_RC - Radio input at 500 Hz (HIGH priority)
 *
 * Reads RC receiver data and checks for signal loss via SerialRx.
 * 500 Hz is sufficient since IBus/SBUS frames arrive at ~70-150 Hz.
 */
extern "C" void taskRC(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // Get radio commands
    getCommands();

    // SerialRx 4-layer failsafe: protocol flag, timeout, range, expiry
    if (radioSignalLost()) {
        channel_1_pwm = channel_1_fs;
        channel_2_pwm = channel_2_fs;
        channel_3_pwm = channel_3_fs;
        channel_4_pwm = channel_4_fs;
        channel_5_pwm = channel_5_fs;
        channel_6_pwm = channel_6_fs;
    }
}

/**
 * TASK_SERIAL - Modal serial processor at 100 Hz (MEDIUM priority)
 *
 * Processes MSP protocol (default) or CLI text commands.
 * Replaces TASK_TELEMETRY from 1.3a with on-demand debug via CLI.
 */
extern "C" void taskSerial(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    processSerial();
}

/**
 * TASK_BLINK - LED heartbeat at 2 Hz (LOW priority)
 *
 * Simple LED toggle to indicate the scheduler is running.
 */
extern "C" void taskBlink(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    if (!gyroCalibrated) {          //gyro cal in progress (boot/first-arm): solid ON = hold still, not armable
        digitalWrite(ledPin, HIGH);
        return;
    }

    //Init complete + armable: announce with a short rising ESC-beacon chime, one note per
    //2 Hz tick. Spreading across ticks supplies the ~500 ms inter-note spacing Bluejay needs
    //(it plays each tone with interrupts off + wait200ms, so a 2nd beep frame ~100 ms later
    //is missed) without a blocking delay() in a scheduled task. No-op on non-DShot targets.
    //
    //5-note rising sweep (tones 1,2,3,4,5) — ~2.5 s, clearly rising and distinct from Bluejay's
    //own status beeps (f2-short arm-confirm, f4 lost-beacon, f1-f3 fault runs). Edit kChime[]
    //to retune. armedFly gate cancels a partial chime so no orphaned note plays on a later disarm.
    static const uint8_t kChime[] = {1, 2, 3, 4, 5};
    static const uint8_t kChimeLen = (uint8_t)sizeof(kChime);
    static uint8_t chimeStep = 0;
    if (armedFly) chimeStep = kChimeLen;  //arming aborts any in-progress chime
    if (chimeStep < kChimeLen && !armedFly) {
        motors.Beep(kChime[chimeStep]);   //next note of the rising sweep
        chimeStep++;
    }

    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);  //normal heartbeat once calibrated
}
