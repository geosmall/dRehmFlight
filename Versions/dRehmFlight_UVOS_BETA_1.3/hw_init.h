#pragma once

#include "uvos.h"
#include "imu.h"  // IMU library (uvos wrapper for uvos icm42688p lib)

namespace Devices {

inline void imu_apply_mounting_matrix(const int32_t matrix[9], int16_t raw[3])
{
  int64_t data_q30[3];
  for (unsigned i = 0; i < 3; i++) {
      data_q30[i] = ((int64_t)matrix[3 * i + 0] * raw[0])
                  + ((int64_t)matrix[3 * i + 1] * raw[1])
                  + ((int64_t)matrix[3 * i + 2] * raw[2]);
  }
  raw[0] = (int16_t)(data_q30[0] >> 30);
  raw[1] = (int16_t)(data_q30[1] >> 30);
  raw[2] = (int16_t)(data_q30[2] >> 30);
}

// Use IMU as alias for uvos::IMU to refer to the library class
// using IMU = uvos::IMU;

// forward declarations
extern uvos::UartHandler& debug_uart;
extern uvos::SpiHandle& spi_imu;
extern uvos::IMU& imu;
extern const int32_t icm_mounting_matrix[9];

bool imu_init();

} // namespace Devices