// SPIdev library collection - Main SPI device class header file
// Abstracts bit and byte SPI R/W functions into a convenient class
// Based on I2Cdev library interface by Jeff Rowberg
// 2020-05-03 by Rafael Carbonell <racarla96@gmail.com>
//
// Changelog:
//      2020-05-?? - initial release

/* ============================================
SPIdev device library code

MIT License

Copyright (c) 2020 racarla96

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===============================================
*/

#ifndef _SPIDEV_H_
#define _SPIDEV_H_

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define SPIDEV_SERIAL_DEBUG

#include "Arduino.h"
#include "spi_stm32.h"

#ifdef __cplusplus
extern "C" {
#endif

#define READ 0B10000000
//#define WRITE 0B00000000 // Write is implicit

#define BUFFERLEN 20

static uint8_t TX_buf[BUFFERLEN];
static uint8_t RX_buf[BUFFERLEN];
static uint16_t readTimeout;
static PinName pinCS;
static bool DataOrder;

void SPIdev_setCS(PinName pin);
void SPIdev_setDataOrder(bool spiMsb);

int8_t SPIdev_readBit(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
int8_t SPIdev_readBitW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
int8_t SPIdev_readBits(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
int8_t SPIdev_readBitsW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
int8_t SPIdev_readByte(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t *data);
int8_t SPIdev_readWord(spi_stm32_t *devAddr, uint8_t regAddr, uint16_t *data);
int8_t SPIdev_readBytes(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t SPIdev_readWords(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

bool SPIdev_writeBit(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool SPIdev_writeBitW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
bool SPIdev_writeBits(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool SPIdev_writeBitsW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
bool SPIdev_writeByte(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t data);
bool SPIdev_writeWord(spi_stm32_t *devAddr, uint8_t regAddr, uint16_t data);
bool SPIdev_writeBytes(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool SPIdev_writeWords(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif // _SPIDEV_H_
