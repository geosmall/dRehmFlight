/** This file contains all necessary code for dRehmFlight initializations
  * on the Matek H743 to avoid cluttering the main code.
  */

#if defined(ARDUINO_FC_MatekH743)
  #define IMU_SPI SpiHandle::Config::Peripheral::SPI_1;
  #define IMU_NSS_PIN  Pin(PORTC, 15)
  #define IMU_SCLK_PIN Pin(PORTA, 5)
  #define IMU_MISO_PIN Pin(PORTA, 6)
  #define IMU_MOSI_PIN Pin(PORTD, 7)
  #define SER_RX_UART UartHandler::Config::Peripheral::USART_1
  #define SER_RX_RX_PIN Pin(PORTB, 7)
  #define SER_RX_TX_PIN Pin(PORTB, 6)
#elif defined(ARDUINO_DevEBoxH743VI)
  #define IMU_SPI SpiHandle::Config::Peripheral::SPI_1;
  #define IMU_NSS_PIN  Pin(PORTA, 4)
  #define IMU_SCLK_PIN Pin(PORTA, 5)
  #define IMU_MISO_PIN Pin(PORTA, 6)
  #define IMU_MOSI_PIN Pin(PORTA, 7)
  #define SER_RX_UART UartHandler::Config::Peripheral::USART_1
  #define SER_RX_RX_PIN Pin(PORTB, 7)
  #define SER_RX_TX_PIN Pin(PORTB, 6)
#else
  #error "Please define the board you are using"
#endif /* ARDUINO_FC_MatekH743 */

//Uncomment to use software driven NSS
#define USE_SOFT_NSS

//Requested IMU SPI bus frequency
#define DESIRED_SPI_FREQ 1000000

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

//Instantiate PWMOutput objects
PWMOutput Servo_pwm(servo_outputs, NUM_SERVO_OUTPUTS, 50);  // 50 Hz for servos
PWMOutput ESC_pwm(esc_outputs, NUM_ESC_OUTPUTS, 500);       // 500 Hz for ESCs

//Instantiate PWM_servo objects
PWM_servo servo1(Servo_pwm, 0); // Servo1 on output index 0
PWM_servo servo2(Servo_pwm, 1); // Servo2 on output index 1
PWM_servo servo3(Servo_pwm, 2); // Servo3 on output index 2
PWM_servo servo4(Servo_pwm, 3); // Servo4 on output index 3
PWM_servo servo5(Servo_pwm, 4); // Servo5 on output index 4
PWM_servo servo6(Servo_pwm, 5); // Servo6 on output index 5

// Instantiate PWM_esc objects
PWM_esc esc1(ESC_pwm, 0);
PWM_esc esc2(ESC_pwm, 1);
PWM_esc esc3(ESC_pwm, 2);
PWM_esc esc4(ESC_pwm, 3);

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


void MakekH743_Init() {

  debug1_pin.pin.port = DEBUG1_PORT;
  debug1_pin.pin.pin  = DEBUG1_PIN;
  debug1_pin.mode     = UVS_GPIO_MODE_OUTPUT_PP;
  uvs_gpio_init(&debug1_pin);

  debug2_pin.pin.port = DEBUG2_PORT;
  debug2_pin.pin.pin  = DEBUG2_PIN;
  debug2_pin.mode     = UVS_GPIO_MODE_OUTPUT_PP;
  uvs_gpio_init(&debug2_pin);

  //===================================================================================================================//
  //                                                 PWM OUTPUT SETUP                                                  //
  //===================================================================================================================//

  //Initialize servo PWMOutput object, check for errors
  if (Servo_pwm.Init() != PWMOutput::Result::OK) Error_Handler();

  //Initialize servo PWMOutput object, check for errors
  // if (ESC_pwm.Init() != PWMOutput::Result::OK) Error_Handler();

  //===================================================================================================================//
  //                                                 SPI/IMU SETUP                                                     //
  //===================================================================================================================//

  //Structure to configure the IMU SPI instance
  SpiHandle::Config spi_conf;

  //Configure the ICM-42688P IMU SPI interface (match for Matek_H743 WLITE)
  spi_conf.periph = IMU_SPI;
  spi_conf.mode = SpiHandle::Config::Mode::MASTER;
  spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;
  spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
  spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;

#ifdef USE_SOFT_NSS
  spi_conf.nss = SpiHandle::Config::NSS::SOFT;
#else
  spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif /* USE_SOFT_NSS */

  spi_conf.pin_config.nss  = IMU_NSS_PIN;
  spi_conf.pin_config.sclk = IMU_SCLK_PIN;
  spi_conf.pin_config.miso = IMU_MISO_PIN;
  spi_conf.pin_config.mosi = IMU_MOSI_PIN;

  // spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_32;
  spi_handle.GetBaudHz(spi_conf.periph, DESIRED_SPI_FREQ, spi_conf.baud_prescaler);

  //Initialize the IMU SPI instance
  if (spi_handle.Init(spi_conf) != SpiHandle::Result::OK)
  {
    Error_Handler();
  }

}
