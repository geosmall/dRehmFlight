#pragma once

#include "uvos.h"

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

class IMU {
public:
    explicit IMU(uvos::SpiHandle& spi_ref);
    void init();
    // other IMU methods
};

// forward declarations
extern uvos::SpiHandle& spi_imu;
extern IMU& imu;
extern int32_t icm_mounting_matrix[9];

void hw_init();

} // namespace Devices