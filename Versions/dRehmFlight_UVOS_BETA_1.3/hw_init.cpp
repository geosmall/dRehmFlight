#include "hw_init.h"

// using namespace uvos;

namespace Devices {

/* IMU Mounting Matrix (ICM-42688-P, Q30 Fixed-point)
 * NOTE: In Q30 format, 1.0 is represented as (1 << 30)
 *
 * Vehicle Coordinate Frame:
 *   +X Forward (nose)
 *   +Y Left (left wing)
 *   +Z Upward
 *
 * IMU Coordinate Frame (see datasheet Fig 16, page 54):
 *   +X Forward (opposite pin-1 edge, pins 8-11)
 *   +Y Left (pins 12-14 side)
 *   +Z Upward (top surface)
 *
 * Matrix (identity, no rotation required):
 */
const int32_t icm_mounting_matrix[9] = {
  (1 << 30),  0,          0,
  0,          (1 << 30),  0,
  0,          0,          (1 << 30)
};

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
//                                               Debug Pin SETUP                                                     //
//===================================================================================================================//


#define DEBUG1_PORT UVS_GPIOE
#define DEBUG1_PIN 5
uvs_gpio debug1_pin;

void SetDebugPin1(bool state)
{
  uvs_gpio_write(&debug1_pin, state);
}

#define DEBUG2_PORT UVS_GPIOE
#define DEBUG2_PIN 6
uvs_gpio debug2_pin;

void SetDebugPin2(bool state)
{
  uvs_gpio_write(&debug2_pin, state);
}

void debug_pin_init() {
  debug1_pin.pin.port = DEBUG1_PORT;
  debug1_pin.pin.pin  = DEBUG1_PIN;
  debug1_pin.mode     = UVS_GPIO_MODE_OUTPUT_PP;
  uvs_gpio_init(&debug1_pin);

  debug2_pin.pin.port = DEBUG2_PORT;
  debug2_pin.pin.pin  = DEBUG2_PIN;
  debug2_pin.mode     = UVS_GPIO_MODE_OUTPUT_PP;
  uvs_gpio_init(&debug2_pin);
}

//===================================================================================================================//
//                                              DEBUG UART SETUP                                                     //
//===================================================================================================================//

static const UartHandler::Config debug_uart_conf = [](){
  // Lambda to initialize 'uart_conf' in one expression.
  UartHandler::Config conf;
  conf.periph = UART_NUM;
  conf.mode = UartHandler::Config::Mode::TX;
  conf.pin_config.tx = TX_PIN;
  conf.pin_config.rx = RX_PIN;
  // Return the fully initialized configuration.
  return conf;
}(); // End of lambda, closing () immediately invokes it.

// "Placement-new" style initialization of UART object
alignas(uvos::UartHandler) static uint8_t debug_uart_storage[sizeof(uvos::UartHandler)];
uvos::UartHandler& debug_uart = reinterpret_cast<uvos::UartHandler&>(debug_uart_storage);

void debug_uart_init() {
  // Initialize the UART object
  new (&debug_uart) uvos::UartHandler();
  debug_uart.Init(debug_uart_conf);
}

//===================================================================================================================//
//                                               IMU/SPI SETUP                                                       //
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

// "Placement-new" style initialization of SPI object
alignas(uvos::SpiHandle) static uint8_t spi_storage[sizeof(uvos::SpiHandle)];
uvos::SpiHandle& spi_imu = reinterpret_cast<uvos::SpiHandle&>(spi_storage);

// Initialize uvos::IMU object
alignas(IMU) static uint8_t imu_storage[sizeof(IMU)];
IMU& imu = reinterpret_cast<IMU&>(imu_storage);

bool imu_init() {
	new (&spi_imu) uvos::SpiHandle();
	spi_imu.Init(spi_conf);

  // Initialize the IMU object
  new (&imu) IMU();
  IMU::Result result = imu.Init(spi_imu);
  if (result != IMU::Result::OK) {
    return false;
  }
  // if (imu.Init(spi_imu) != IMU::Result::OK) {
  //   return false;
  // }
  return true;
}

} // namespace Devices
