/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_STM32_H
#define __SPI_STM32_H

/* Includes ------------------------------------------------------------------*/
#include "stm32_def.h"
#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

typedef struct spi_stm32_s {
  SPI_HandleTypeDef handle;
  SPI_TypeDef *spi;
  PinName pin_miso;
  PinName pin_mosi;
  PinName pin_sclk;
  PinName pin_ssel;
} spi_stm32_t;


///@brief specifies the SPI speed bus in HZ.
#define SPI_STM32_SPEED_CLOCK_DEFAULT     1000000

#define SPI_STM32_SPEED_CLOCK_DIV2_MHZ    ((uint32_t)2)
#define SPI_STM32_SPEED_CLOCK_DIV4_MHZ    ((uint32_t)4)
#define SPI_STM32_SPEED_CLOCK_DIV8_MHZ    ((uint32_t)8)
#define SPI_STM32_SPEED_CLOCK_DIV16_MHZ   ((uint32_t)16)
#define SPI_STM32_SPEED_CLOCK_DIV32_MHZ   ((uint32_t)32)
#define SPI_STM32_SPEED_CLOCK_DIV64_MHZ   ((uint32_t)64)
#define SPI_STM32_SPEED_CLOCK_DIV128_MHZ  ((uint32_t)128)
#define SPI_STM32_SPEED_CLOCK_DIV256_MHZ  ((uint32_t)256)

///@brief speficies the SPI mode to use
//Mode          Clock Polarity (CPOL)       Clock Phase (CPHA)
//SPI_MODE0             0                         0
//SPI_MODE1             0                         1
//SPI_MODE2             1                         0
//SPI_MODE3             1                         1
//enum definitions coming from SPI.h of SAM
typedef enum {
  SPI_STM32_MODE_0 = 0x00,
  SPI_STM32_MODE_1 = 0x01,
  SPI_STM32_MODE_2 = 0x02,
  SPI_STM32_MODE_3 = 0x03
} spi_stm32_mode_e;

///@brief SPI errors
typedef enum {
  SPI_STM32_OK = 0,
  SPI_STM32_TIMEOUT = 1,
  SPI_STM32_ERROR = 2
} spi_stm32_status_e;

/* Exported functions ------------------------------------------------------- */
void spi_stm32_init(spi_stm32_t *obj, uint32_t speed, spi_stm32_mode_e mode, uint8_t msb);
// void spi_deinit(spi_t *obj);
// spi_status_e spi_send(spi_t *obj, uint8_t *Data, uint16_t len, uint32_t Timeout);
spi_stm32_status_e spi_stm32_transfer(spi_stm32_t *obj, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t len, uint32_t Timeout);
// uint32_t spi_getClkFreq(spi_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_STM32_H */