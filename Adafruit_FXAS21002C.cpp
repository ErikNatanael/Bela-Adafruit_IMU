/*!
 * @file Adafruit_FXAS21002C.cpp
 *
 * @mainpage Adafruit FXAS21002C gyroscope sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXAS21002C driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXAS21002C breakout: https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_Sensor">
 * Adafruit_Sensor</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_FXAS21002C.h"

#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    Bridge between Arduino code and bela I2c code
*/
/**************************************************************************/
i2c_char_t Adafruit_FXAS21002C::read8(i2c_char_t reg)
{
  i2c_char_t value[1];

  readRegisters(reg, value, 1);

  return value[0];
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_FXAS21002C class, including assigning
            a unique ID to the gyroscope for logging purposes.

    @param sensorID The unique ID to associate with the gyroscope.
*/
/**************************************************************************/
Adafruit_FXAS21002C::Adafruit_FXAS21002C(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setup the HW

    @param  rng
            The range to set for the gyroscope, based on gyroRange_t

    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::begin(gyroRange_t rng)
{
  /* Enable I2C */
  uint8_t bus = 1;
  uint8_t i2caddr = FXAS21002C_ADDRESS;
  if(initI2C_RW(bus, i2caddr, 0) > 0)
	  return false;

  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  
  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  i2c_char_t answer[1];
  readRegisters(GYRO_REGISTER_WHO_AM_I, answer, 1);
  printf("%i\n", answer[0]);
  if (answer[0] != FXAS21002C_ID)
  {
  	printf("Wrong ID returned from FXAS21002C\n");
    return false;
  }

  /* Set CTRL_REG1 (0x13)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     6  RESET     Reset device on 1                                   0
     5  ST        Self test enabled on 1                              0
   4:2  DR        Output data rate                                  000
                  000 = 800 Hz
                  001 = 400 Hz
                  010 = 200 Hz
                  011 = 100 Hz
                  100 = 50 Hz
                  101 = 25 Hz
                  110 = 12.5 Hz
                  111 = 12.5 Hz
     1  ACTIVE    Standby(0)/Active(1)                                0
     0  READY     Standby(0)/Ready(1)                                 0
  */

  /* Set CTRL_REG0 (0x0D)  Default value 0x00
  =====================================================================
  BIT  Symbol     Description                                   Default
  7:6  BW         cut-off frequency of low-pass filter               00
    5  SPIW       SPI interface mode selection                        0
  4:3  SEL        High-pass filter cutoff frequency selection        00
    2  HPF_EN     High-pass filter enable                             0
  1:0  FS         Full-scale range selection
                  00 = +-2000 dps
                  01 = +-1000 dps
                  10 = +-500 dps
                  11 = +-250 dps
  The bit fields in CTRL_REG0 should be changed only in Standby or Ready modes.
  */

  uint8_t ctrlReg0 = 0x00;

  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      ctrlReg0 = 0x03;
      break;
    case GYRO_RANGE_500DPS:
     ctrlReg0 = 0x02;
      break;
    case GYRO_RANGE_1000DPS:
      ctrlReg0 = 0x01;
      break;
    case GYRO_RANGE_2000DPS:
      ctrlReg0 = 0x00;
      break;
  }

  /* Reset then switch to active mode with 100Hz output */
  writeRegister(GYRO_REGISTER_CTRL_REG1, 0x00);     // Standby
  writeRegister(GYRO_REGISTER_CTRL_REG1, (1<<6));   // Reset
  writeRegister(GYRO_REGISTER_CTRL_REG0, ctrlReg0); // Set sensitivity
  writeRegister(GYRO_REGISTER_CTRL_REG1, 0x0E);     // Active
  usleep(100); // 60 ms + 1/ODR

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event

    @param[out] event
                A reference to the sensors_event_t instances where the
                accelerometer data should be written.

     @return True if the event was successfully read, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::readSensor()
{
  bool readingValid = false;

  /* Clear the raw data placeholder */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  //event->timestamp = millis();

  /* Read 7 bytes from the sensor */
  i2c_char_t buffer[7];
  readRegisters(GYRO_REGISTER_STATUS, buffer, 7);
  
  uint8_t status = buffer[0];
  uint8_t xhi = buffer[1];
  uint8_t xlo = buffer[2];
  uint8_t yhi = buffer[3];
  uint8_t ylo = buffer[4];
  uint8_t zhi = buffer[5];
  uint8_t zlo = buffer[6];
 

  /* Shift values to create properly formed integer */
  raw.x = (int16_t)((xhi << 8) | xlo);
  raw.y = (int16_t)((yhi << 8) | ylo);
  raw.z = (int16_t)((zhi << 8) | zlo);


  /* Compensate values depending on the resolution */
  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      data.x = raw.x * GYRO_SENSITIVITY_250DPS;
      data.y = raw.y * GYRO_SENSITIVITY_250DPS;
      data.z = raw.z * GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      data.x = raw.x * GYRO_SENSITIVITY_500DPS;
      data.y = raw.y * GYRO_SENSITIVITY_500DPS;
      data.z = raw.z * GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_1000DPS:
      data.x = raw.x * GYRO_SENSITIVITY_1000DPS;
      data.y = raw.y * GYRO_SENSITIVITY_1000DPS;
      data.z = raw.z * GYRO_SENSITIVITY_1000DPS;
      break;
    case GYRO_RANGE_2000DPS:
      data.x = raw.x * GYRO_SENSITIVITY_2000DPS;
      data.y = raw.y * GYRO_SENSITIVITY_2000DPS;
      data.z = raw.z * GYRO_SENSITIVITY_2000DPS;
      break;
  }

  /* Convert values to rad/s */
  data.x *= SENSORS_DPS_TO_RADS;
  data.y *= SENSORS_DPS_TO_RADS;
  data.z *= SENSORS_DPS_TO_RADS;

  return true;
}

// Wrap the I2c function to avoid rewriting all the calls to writeRegister
void Adafruit_FXAS21002C::writeRegister(uint8_t reg, uint8_t value) {
	i2c_char_t mess = value;

	writeRegisters(reg, &mess, 1);
}