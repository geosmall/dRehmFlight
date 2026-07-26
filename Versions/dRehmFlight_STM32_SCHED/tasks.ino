//========================================================================================================================//
//                                              SCHEDULER TASK CONFIGURATION                                             //
//========================================================================================================================//

// Forward declarations for task functions
extern "C" void taskFlight(timeUs_t currentTimeUs);
extern "C" void taskRC(timeUs_t currentTimeUs);
extern "C" void taskTelemetry(timeUs_t currentTimeUs);
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

    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(100),   // 100 Hz (10 ms) - debug output
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
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagX, MagY, MagZ, dt);

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
 * TASK_TELEMETRY - Debug output at 100 Hz (MEDIUM priority)
 *
 * Serial print functions for debugging. Only ONE should be active at a time.
 * Rate is now controlled by scheduler (was self-limited via print_counter).
 */
extern "C" void taskTelemetry(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // Uncomment ONE print function for troubleshooting:
    printRadioData();
    //printDesiredState();
    //printGyroData();
    //printAccelData();
    //printMagData();
    //printRollPitchYaw();
    //printPIDoutput();
    //printMotorCommands();
    //printServoCommands();
    //printLoopRate();
    //printSchedulerStats();  // Show scheduler task rates and CPU load
}

/**
 * TASK_BLINK - LED heartbeat at 2 Hz (LOW priority)
 *
 * Simple LED toggle to indicate the scheduler is running.
 */
extern "C" void taskBlink(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
}

//========================================================================================================================//
// Scheduler Helper Functions
//========================================================================================================================//

/**
 * Print scheduler statistics - task rates and CPU load
 */
void printSchedulerStats() {
    cfTaskInfo_t flightInfo, rcInfo, telemetryInfo;
    getTaskInfo(TASK_FLIGHT, &flightInfo);
    getTaskInfo(TASK_RC, &rcInfo);
    getTaskInfo(TASK_TELEMETRY, &telemetryInfo);

    // Calculate actual rates from delta times
    int flightRate = flightInfo.latestDeltaTime > 0 ? 1000000 / flightInfo.latestDeltaTime : 0;
    int rcRate = rcInfo.latestDeltaTime > 0 ? 1000000 / rcInfo.latestDeltaTime : 0;

    Serial.print("CPU:");
    Serial.print(averageSystemLoadPercent);
    Serial.print("% | FLIGHT:");
    Serial.print(flightRate);
    Serial.print("Hz | RC:");
    Serial.print(rcRate);
    Serial.print("Hz | dt:");
    Serial.print(dt * 1000000.0f, 0);
    Serial.println("us");
}
