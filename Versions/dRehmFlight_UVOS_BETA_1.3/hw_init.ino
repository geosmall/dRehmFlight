/** This file contains all necessary code for dRehmFlight initializations
  * to avoid cluttering the main .ino file.
  */

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
  constexpr UartHandler::Config::Peripheral DBG_UART_NUM = UartHandler::Config::Peripheral::USART_1;
  constexpr Pin DBG_TX_PIN = Pin(PORTA, 9);
  constexpr Pin DBG_RX_PIN = Pin(PORTA, 10);
#elif defined(ARDUINO_NUCLEO_H753ZI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTC, 15);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTD, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTB, 2);
  constexpr UartHandler::Config::Peripheral DBG_UART_NUM = UartHandler::Config::Peripheral::USART_3;
  constexpr Pin DBG_TX_PIN = Pin(PORTD, 8);
  constexpr Pin DBG_RX_PIN = Pin(PORTD, 9);
#else // defined(DevEBoxH743VI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTA, 4);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTA, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTA, 0);
  constexpr UartHandler::Config::Peripheral DBG_UART_NUM = UartHandler::Config::Peripheral::USART_1;
  constexpr Pin DBG_TX_PIN = Pin(PORTA, 9);
  constexpr Pin DBG_RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_FC_MatekH743 */

constexpr bool off = 0;
constexpr bool on = 1;

//===================================================================================================================//
//                                              PWM Output SETUP                                                     //
//===================================================================================================================//

//Define servo and ESC outputs
#define NUM_SERVO_OUTPUTS 6
#define NUM_ESC_OUTPUTS 4

//Define PWM output peripheral type alias for readability
using TimPeriph = TimerHandle::Config::Peripheral;

//Define servo outputs (S1-S6)
static PWMOutputChannel servo_outputs[NUM_SERVO_OUTPUTS] = {
  // S1 and S2 on TIM3
  {TimPeriph::TIM_3, TimChannel::CH_3, Pin(PORTB, 0), 0, TimPolarity::HIGH, GPIO_AF2_TIM3},
  {TimPeriph::TIM_3, TimChannel::CH_4, Pin(PORTB, 1), 0, TimPolarity::HIGH, GPIO_AF2_TIM3},
  // S3-S6 on TIM5
  {TimPeriph::TIM_5, TimChannel::CH_1, Pin(PORTA, 0), 0, TimPolarity::HIGH, GPIO_AF2_TIM5},
  {TimPeriph::TIM_5, TimChannel::CH_2, Pin(PORTA, 1), 0, TimPolarity::HIGH, GPIO_AF2_TIM5},
  {TimPeriph::TIM_5, TimChannel::CH_3, Pin(PORTA, 2), 0, TimPolarity::HIGH, GPIO_AF2_TIM5},
  {TimPeriph::TIM_5, TimChannel::CH_4, Pin(PORTA, 3), 0, TimPolarity::HIGH, GPIO_AF2_TIM5},
};

//Define ESC outputs (S7-S10)
static PWMOutputChannel esc_outputs[NUM_ESC_OUTPUTS] = {
  // S7-S10 on TIM4
  {TimPeriph::TIM_4, TimChannel::CH_1, Pin(PORTD, 12), 0, TimPolarity::HIGH, GPIO_AF2_TIM4},
  {TimPeriph::TIM_4, TimChannel::CH_2, Pin(PORTD, 13), 0, TimPolarity::HIGH, GPIO_AF2_TIM4},
  {TimPeriph::TIM_4, TimChannel::CH_3, Pin(PORTD, 14), 0, TimPolarity::HIGH, GPIO_AF2_TIM4},
  {TimPeriph::TIM_4, TimChannel::CH_4, Pin(PORTD, 15), 0, TimPolarity::HIGH, GPIO_AF2_TIM4},
  // {TimPeriph::TIM_15, TimChannel::CH_1, Pin(PORTE, 5), 0, TimPolarity::HIGH, GPIO_AF4_TIM15}, //Used for debug
  // {TimPeriph::TIM_15, TimChannel::CH_2, Pin(PORTE, 6), 0, TimPolarity::HIGH, GPIO_AF4_TIM15}, //Used for debug
};

// Allocate storage for PWMOutput objects with proper alignment
alignas(PWMOutput) static uint8_t servo_pwm_storage[sizeof(PWMOutput)];
alignas(PWMOutput) static uint8_t esc_pwm_storage[sizeof(PWMOutput)];

// Create references to the allocated storage
PWMOutput& Servo_pwm = reinterpret_cast<PWMOutput&>(servo_pwm_storage);
PWMOutput& ESC_pwm = reinterpret_cast<PWMOutput&>(esc_pwm_storage);

// Initialization function, use placement-new to construct objects:
bool pwm_output_init() {
  new (&Servo_pwm) PWMOutput(servo_outputs, NUM_SERVO_OUTPUTS, 50);   // 50 Hz for servos
  new (&ESC_pwm) PWMOutput(esc_outputs, NUM_ESC_OUTPUTS, 500);           // 500 Hz for ESCs

    //Initialize servo PWMOutput object, check for errors
  if (Servo_pwm.Init() != PWMOutput::Result::OK) return false;

  //Initialize servo PWMOutput object, check for errors
  if (ESC_pwm.Init() != PWMOutput::Result::OK) return false;

  return true;
}

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
  conf.periph = DBG_UART_NUM;
  conf.mode = UartHandler::Config::Mode::TX;
  conf.pin_config.tx = DBG_TX_PIN;
  conf.pin_config.rx = DBG_RX_PIN;
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
