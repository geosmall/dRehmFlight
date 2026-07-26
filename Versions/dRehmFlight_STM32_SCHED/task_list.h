/**
 * @file task_list.h
 * @brief Task definitions for dRehmFlight_STM32_SCHED
 *
 * Include this file BEFORE <scheduler.h> in your sketch.
 */

#ifndef TASK_LIST_H
#define TASK_LIST_H

// Tell scheduler.h that we're providing the task enum
#define SCHEDULER_TASK_LIST_DEFINED

typedef enum {
    // System task (always required, must be first)
    TASK_SYSTEM = 0,

    //=========================================================================
    // dRehmFlight tasks
    //=========================================================================
    TASK_FLIGHT,      // 2000 Hz REALTIME - core flight loop (IMU, PID, motors)
    TASK_RC,          // 500 Hz HIGH - radio input + failsafe
    TASK_TELEMETRY,   // 100 Hz MEDIUM - debug serial output
    TASK_BLINK,       // 2 Hz LOW - LED heartbeat
    //=========================================================================

    // Task count (always required, must be last)
    TASK_COUNT
} cfTaskId_e;

#endif // TASK_LIST_H
