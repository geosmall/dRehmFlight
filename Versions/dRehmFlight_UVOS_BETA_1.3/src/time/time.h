#pragma once

//This file contains an interface shim for a few Arduino compatibility functions

#include "uvos.h"

/** @brief Arduino-compatible time function.
 *  @return The number of milliseconds from HAL SysTick timer.
 */
inline uint32_t millis(void)
{
  return uvos::System::GetTickHAL(); // SysTick mSec
}

/** @brief Arduino-compatible time function.
 *  @return The number of microseconds from UVOS system timer.
 */
inline uint32_t micros(void)
{
  return uvos::System::GetUs();
}

/** @brief Delay for a specified number of milliseconds.
 *  @param ms Delay in milliseconds.
 */
inline void delay(uint32_t ms)
{
  uvos::System::Delay(ms);
}
