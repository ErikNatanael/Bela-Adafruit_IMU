/*!
 * @file Adafruit_FXOS8700.cpp
 *
 * @mainpage Adafruit FXOS8700 accel/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXOS8700 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXOS8700 breakout: https://www.adafruit.com/products/3463
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

#include "Adafruit_FXOS8700.h"
#include <cmath>

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB      (0.1F)

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Instantiates a new Adafruit_FXOS8700 class, including assigning
             a unique ID to the accel and magnetometer for logging purposes.

     @param accelSensorID The unique ID to associate with the accelerometer.
     @param magSensorID The unique ID to associate with the magnetometer.
 */
 /**************************************************************************/
Adafruit_FXOS8700::Adafruit_FXOS8700(int32_t accelSensorID, int32_t magSensorID)
{
  _accelSensorID = accelSensorID;
  _magSensorID = magSensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Initializes the hardware, including setting the accelerometer
             range based on fxos8700AccelRange_t

     @param  rng
             The range to set for the accelerometer, based on fxos8700AccelRange_t

     @return True if the device was successfully initialized, otherwise false.
 */
 /**************************************************************************/
bool Adafruit_FXOS8700::begin(fxos8700AccelRange_t rng)
{
  /* Enable I2C */
  uint8_t bus = 1;
  uint8_t i2caddr = FXOS8700_ADDRESS;
  _i2c_address = FXOS8700_ADDRESS;
  if(initI2C_RW(bus, i2caddr, 0) > 0)
	  return false;

  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
 i2c_char_t answer[1];
  readRegisters(FXOS8700_REGISTER_WHO_AM_I, answer, 1);
  printf("%i", answer[0]);
  printf("\n");
  if (answer[0] != FXOS8700_ID)
  {
  	printf("Wrong ID returned from FXOS8700\n");
    return false;
  }
  
  

  /* Set to standby mode (required to make changes to this register) */
  writeRegister(FXOS8700_REGISTER_CTRL_REG1, 0);

  /* Configure the accelerometer */
  switch (_range) {
      case (ACCEL_RANGE_2G):
        writeRegister(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
      break;
      case (ACCEL_RANGE_4G):
        writeRegister(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01);
      break;
      case (ACCEL_RANGE_8G):
        writeRegister(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02);
      break;
  }
  /* High resolution */
  writeRegister(FXOS8700_REGISTER_CTRL_REG2, 0x02);
  /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
  // For 200Hz = 0xD, for 400Hz = 0x5
  // See datasheet table 35
  writeRegister(FXOS8700_REGISTER_CTRL_REG1, 0x15);

  /* Configure the magnetometer */
  /* Hybrid Mode, Over Sampling Rate = 16 */
  writeRegister(FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
  /* Jump to reg 0x33 after reading 0x06 */
  writeRegister(FXOS8700_REGISTER_MCTRL_REG2, 0x20);

  return true;
}


void Adafruit_FXOS8700::calculateVelocity() {
	accel_vel_old.insert(accel_vel_old.begin(), accel_vel);
	if(accel_vel_old.size() > 5) accel_vel_old.pop_back();
	
	float vx = accel_ms2.x - accel_old.x, vy = accel_ms2.y - accel_old.y, vz = accel_ms2.z - accel_old.z;
	// Smooth out using previous values
	/*for(auto const& old_vel: accel_vel_old) {
		vx = (vx + old_vel.x) / 2;
		vy = (vy + old_vel.y) / 2;
		vz = (vz + old_vel.z) / 2;
	}*/
	accel_vel.x = vx;
	accel_vel.y = vy;
	accel_vel.z = vz;
}


/**************************************************************************/
/*!
    @brief  Puts device into/out of standby mode

    @param standby Set this to a non-zero value to enter standy mode.
*/
/**************************************************************************/
/*void Adafruit_FXOS8700::standby(bool standby)
{
  uint8_t reg1 = readRegister8(FXOS8700_REGISTER_CTRL_REG1);
  if (standby) {
    reg1 &= ~(0x01);
  } else {
    reg1 |= (0x01);
  }
  writeRegister(FXOS8700_REGISTER_CTRL_REG1, reg1);

  if (! standby) {
    usleep(100);
  }
}*/

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void Adafruit_FXOS8700::writeRegister(uint8_t reg, uint8_t value) {
	i2c_char_t mess = value;

	writeRegisters(reg, &mess, 1);
}

void Adafruit_FXOS8700::readSensor() {
	//Store old values
	accel_old = accel_ms2;
	
  i2c_char_t buffer[13];
  
  // Read all values
  readRegisters(FXOS8700_REGISTER_STATUS, buffer, 13);
  //printf("Status: %i\n", buffer[0]); // post status
  // 14-bit accelerometer data to 16-bit
  accel_raw.x = (int16_t)((buffer[1] << 8) | buffer[2]) >> 2;
  accel_raw.y = (int16_t)((buffer[3] << 8) | buffer[4]) >> 2;
  accel_raw.z = (int16_t)((buffer[5] << 8) | buffer[6]) >> 2;
  
  //printf("x=%i, y=%i, z=%i\n", accel_raw.x, accel_raw.y, accel_raw.z);
  
  // 16-bit magnetometer data
  mag_raw.x = (buffer[7] << 8) | buffer[8];
  mag_raw.y = (buffer[9] << 8) | buffer[10];
  mag_raw.z = (buffer[11] << 8) | buffer[12];
  
  /* Convert accel values to m/s^2 */
  switch (_range) {
      case (ACCEL_RANGE_2G):
          /*accel_ms2.x = accel_raw.x * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.y = accel_raw.y * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.z = accel_raw.z * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;*/
          accel_ms2.x = accel_raw.x * ACCEL_MG_LSB_2G;
          accel_ms2.y = accel_raw.y * ACCEL_MG_LSB_2G;
          accel_ms2.z = accel_raw.z * ACCEL_MG_LSB_2G;
      break;
      case (ACCEL_RANGE_4G):
          accel_ms2.x = accel_raw.x * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.y = accel_raw.y * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.z = accel_raw.z * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_8G):
          accel_ms2.x = accel_raw.x * ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.y = accel_raw.y * ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accel_ms2.z = accel_raw.z * ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
  }
  
  /* Convert mag values to uTesla */
  mag_uTesla.x = mag_raw.x * MAG_UT_LSB;
  mag_uTesla.y = mag_raw.y * MAG_UT_LSB;
  mag_uTesla.z = mag_raw.z * MAG_UT_LSB;
  
  //rt_printf("ACCEL x=%f, y=%f, z=%f, MAG x=%f, y=%f, z=%f\n", accel_ms2.x, accel_ms2.y, accel_ms2.z, mag_uTesla.x, mag_uTesla.y, mag_uTesla.z);
  //rt_printf("SUM = %f\n", fabs(accel_ms2.x) + fabs(accel_ms2.y) + fabs(accel_ms2.z));
 
	
}

