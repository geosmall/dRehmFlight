#include "SPIdev.h"
#include "spi_stm32.h"

/*
// int8_t SPIdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
// int8_t SPIdev_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
// int8_t SPIdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
// int8_t SPIdev_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
// int8_t SPIdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
// int8_t SPIdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
// int8_t SPIdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
// int8_t SPIdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
// bool SPIdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
// bool SPIdev_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
// bool SPIdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
// bool SPIdev_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
// bool SPIdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
// bool SPIdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
// bool SPIdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
// bool SPIdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
// uint16_t SPIdev_readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

Steps:
- Strip out FastWire/TwoWire etc
- Rename 'I2Cdev::' to 'SPIdev_'
- Rename 'uint8_t devAddr' to 'spi_stm32_t *devAddr'
- Rename ', uint16_t timeout' to ''
- Rename ', timeout' to ''
- Add 'SPIdev_' prefix to 'readByte'...etc

*/


static uint8_t tx_byte;
static uint8_t rx_byte;
static uint16_t SPIdev_read_timeout = 1000;  // Timeout duration in HAL ticks


void SPIdev_setCS(PinName pin)
{
  pinCS = pin;
}

void SPIdev_setDataOrder(bool spiMsb)
{
  DataOrder = spiMsb;
}


#ifndef BUFFER_LENGTH
// band-aid fix for platforms without Wire-defined BUFFER_LENGTH (removed from some official implementations)
#define BUFFER_LENGTH 32
#endif

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t SPIdev_readBit(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
  uint8_t b;
  uint8_t count = SPIdev_readByte(devAddr, regAddr, &b);
  *data = b & (1 << bitNum);
  return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t SPIdev_readBitW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data)
{
  uint16_t b;
  uint8_t count = SPIdev_readWord(devAddr, regAddr, &b);
  *data = b & (1 << bitNum);
  return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t SPIdev_readBits(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t count;
  uint8_t b;
  if ((count = SPIdev_readByte(devAddr, regAddr, &b)) != 0) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t SPIdev_readBitsW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data)
{
  // 1101011001101001 read byte
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  //    010           masked
  //           -> 010 shifted
  uint8_t count;
  uint16_t w;
  if ((count = SPIdev_readWord(devAddr, regAddr, &w)) != 0) {
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    w &= mask;
    w >>= (bitStart - length + 1);
    *data = w;
  }
  return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t SPIdev_readByte(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t *data)
{
  return SPIdev_readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t SPIdev_readWord(spi_stm32_t *devAddr, uint8_t regAddr, uint16_t *data)
{
  return SPIdev_readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr SPI device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t SPIdev_readBytes(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{

  int8_t count;

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print("SPI reading ");
  Serial.print(length, DEC);
  Serial.print(" bytes from 0x");
  Serial.print(regAddr, HEX);
  Serial.print("...");
#endif

  // SPI.beginTransaction(settings);

  // // take the slave pin low to select the chip:
  // digitalWrite(slave, LOW);
  digitalWriteFast(pinCS, LOW);


  // SPI.transfer(regAddr | READ); // specify the starting register address
  // specify the starting register address
  tx_byte = (regAddr | READ);
  spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

  tx_byte = 0x00;
  for (count = 0; count < length; count++) {
    // data[count] = SPI.transfer(0x00); // read the data
    spi_stm32_transfer(devAddr, &tx_byte, &data[count], 1, SPIdev_read_timeout); // read the data
#ifdef SPIDEV_SERIAL_DEBUG
    Serial.print(data[count], HEX);
    if (count + 1 < length) Serial.print(" ");
#endif
  }

  // // take the slave pin high to de-select the chip:
  // digitalWrite(slave, HIGH);
  digitalWriteFast(pinCS, HIGH);

  // SPI.endTransaction();

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print(". Done (");
  Serial.print(count, DEC);
  Serial.println(" read).");
#endif

  return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in SPIdev_readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t SPIdev_readWords(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{

  int8_t count;

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print("SPI reading ");
  Serial.print(length, DEC);
  Serial.print(" bytes from 0x");
  Serial.print(regAddr, HEX);
  Serial.print("...");
#endif

  // SPI.beginTransaction(settings);

  // take the CS pin low to select the chip:
  digitalWriteFast(pinCS, LOW);

  // SPI subsystem uses internal buffer that breaks with large data requests
  // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
  // smaller chunks instead of all at once
  bool msb = DataOrder; // starts with MSB, then LSB

  // SPI.transfer(regAddr | READ); // specify the starting register address
  tx_byte = (regAddr | READ);
  spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

  tx_byte = 0x00;
  for (count = 0; count < length; count++) {
    if (msb == MSBFIRST) {
      // first byte is bits 15-8 (MSb=15)
      // data[count] = SPI.transfer(0x00) << 8; // read the data
      spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);
      data[count] = rx_byte << 8;
      msb = LSBFIRST;
    } else {
      // second byte is bits 7-0 (LSb=0)
      // data[count] |= SPI.transfer(0x00); // read the data
      spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);
      data[count] |= rx_byte;
#ifdef SPIDEV_SERIAL_DEBUG
      Serial.print(data[count], HEX);
      if (count + 1 < length) Serial.print(" ");
#endif
      msb = MSBFIRST;
    }
  }

  // take the CS pin high to de-select the chip:
  digitalWriteFast(pinCS, HIGH);

  // SPI.endTransaction();

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print(". Done (");
  Serial.print(count, DEC);
  Serial.println(" read).");
#endif

  return count;

}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeBit(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
  uint8_t b;
  SPIdev_readByte(devAddr, regAddr, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return SPIdev_writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeBitW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
  uint16_t w;
  SPIdev_readWord(devAddr, regAddr, &w);
  w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
  return SPIdev_writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeBits(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;
  if (SPIdev_readByte(devAddr, regAddr, &b) != 0) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return SPIdev_writeByte(devAddr, regAddr, b);
  } else {
    return false;
  }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeBitsW(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
  //              010 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  // 0001110000000000 mask word
  // 1010111110010110 original value (sample)
  // 1010001110010110 original & ~mask
  // 1010101110010110 masked | value
  uint16_t w;
  if (SPIdev_readWord(devAddr, regAddr, &w) != 0) {
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    w &= ~(mask); // zero all important bits in existing word
    w |= data; // combine data with existing word
    return SPIdev_writeWord(devAddr, regAddr, w);
  } else {
    return false;
  }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeByte(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t data)
{
  return SPIdev_writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool SPIdev_writeWord(spi_stm32_t *devAddr, uint8_t regAddr, uint16_t data)
{
  return SPIdev_writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool SPIdev_writeBytes(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print("SPI writing ");
  Serial.print(length, DEC);
  Serial.print(" bytes to 0x");
  Serial.print(regAddr, HEX);
  Serial.print("...");
#endif

  uint8_t status = 0;

  // take the CS pin low to select the chip:
  digitalWriteFast(pinCS, LOW);

  // SPI.transfer(regAddr); // specify the starting register address
  tx_byte = regAddr;
  spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

  for (uint8_t i = 0; i < length; i++) {

#ifdef SPIDEV_SERIAL_DEBUG
    Serial.print(data[i], HEX);
    if (i + 1 < length) Serial.print(" ");
#endif

    // SPI.transfer((uint8_t) data[i]); // send the data
    tx_byte = (uint8_t)data[i];
    spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

  }

  // take the CS pin high to de-select the chip:
  digitalWriteFast(pinCS, HIGH);

  // SPI.endTransaction();

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.println(". Done.");
#endif

  return status == 0;

}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool SPIdev_writeWords(spi_stm32_t *devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.print("SPI writing ");
  Serial.print(length, DEC);
  Serial.print(" words to 0x");
  Serial.print(regAddr, HEX);
  Serial.print("...");
#endif
  uint8_t status = 0;

  // SPI.beginTransaction(settings);

  // take the CS pin low to select the chip:
  digitalWriteFast(pinCS, LOW);

  // SPI.transfer(regAddr); // specify the starting register address
  tx_byte = regAddr;
  spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

  for (uint8_t i = 0; i < length; i++) {

#ifdef SPIDEV_SERIAL_DEBUG
    Serial.print(data[i], HEX);
    if (i + 1 < length) Serial.print(" ");
#endif

    // SPI.transfer((uint8_t)(data[i] >> 8));     // send MSB
    tx_byte = (uint8_t)data[i] >> 8;
    spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);

    // SPI.transfer((uint8_t)data[i]);          // send LSB
    tx_byte = (uint8_t)data[i];
    spi_stm32_transfer(devAddr, &tx_byte, &rx_byte, 1, SPIdev_read_timeout);
  }

  // take the CS pin high to de-select the chip:
  digitalWriteFast(pinCS, HIGH);

  // SPI.endTransaction();

#ifdef SPIDEV_SERIAL_DEBUG
  Serial.println(". Done.");
#endif

  return status == 0;

}