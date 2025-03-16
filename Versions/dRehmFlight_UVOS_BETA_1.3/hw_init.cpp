#include "hw_init.h"

using namespace uvos;

namespace Devices {
/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
int32_t icm_mounting_matrix[9] = { (1 << 30), 0, 0, 0, (1 << 30), 0, 0, 0, (1 << 30) };

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1'000'000

#if defined(ARDUINO_FC_MatekH743)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTC, 15);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTD, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTB, 2);
  constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
  constexpr Pin TX_PIN = Pin(PORTA, 9);
  constexpr Pin RX_PIN = Pin(PORTA, 10);
#elif defined(ARDUINO_NUCLEO_H753ZI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTC, 15);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTD, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTB, 2);
  constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
  constexpr Pin TX_PIN = Pin(PORTD, 8);
  constexpr Pin RX_PIN = Pin(PORTD, 9);
#else // defined(DevEBoxH743VI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTA, 4);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTA, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTA, 0);
  constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
  constexpr Pin TX_PIN = Pin(PORTA, 9);
  constexpr Pin RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_FC_MatekH743 */

constexpr bool off = 0;
constexpr bool on = 1;

//===================================================================================================================//
//                                                 SPI/IMU SETUP                                                     //
//===================================================================================================================//

// Structure to configure the IMU SPI instance using a single lambda.
// The lambda []() constructs and returns the initialized config object.
static const SpiHandle::Config spi_conf = [](){
    // Lambda to initialize 'spi_conf' in one expression.
    SpiHandle::Config conf;
    conf.periph = IMU_SPI_NUM;
    conf.mode = SpiHandle::Config::Mode::MASTER;
    conf.direction = SpiHandle::Config::Direction::TWO_LINES;
    conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;
    // Use preprocessor to choose NSS configuration.
    #ifdef USE_SOFT_NSS
      conf.nss = SpiHandle::Config::NSS::SOFT;
    #else
      conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
    #endif
    conf.pin_config.nss  = IMU_CS_PIN;
    conf.pin_config.sclk = IMU_SCLK_PIN;
    conf.pin_config.miso = IMU_MISO_PIN;
    conf.pin_config.mosi = IMU_MOSI_PIN;
    // Return the fully initialized configuration.
    return conf;
}(); // End of lambda, closing () immediately invokes it.

// "Placement-new" style initialization (preferred)
alignas(uvos::SpiHandle) static uint8_t spi_storage[sizeof(uvos::SpiHandle)];
uvos::SpiHandle& spi_imu = reinterpret_cast<uvos::SpiHandle&>(spi_storage);

alignas(IMU) static uint8_t imu_storage[sizeof(IMU)];
IMU& imu = reinterpret_cast<IMU&>(imu_storage);

void hw_init() {
	new (&spi_imu) uvos::SpiHandle();
	spi_imu.Init(spi_conf);

	new (&imu) IMU(spi_imu);
	imu.init();
}

} // namespace Devices
