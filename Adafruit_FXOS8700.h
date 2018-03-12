
// Taken from Adafruit's Arduino library
#ifndef __FXOS8700_H__
#define __FXOS8700_H__

#include <I2c.h>
#include <vector>
#include "Utilities.h"

#define SENSORS_GRAVITY_STANDARD (9.80665F) /**< Earth's gravity in m/s^2 */

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    /** 7-bit I2C address for this sensor */
    #define FXOS8700_ADDRESS           (0x1F)     // 0011111
    /** Device ID for this sensor (used as sanity check during init) */
    #define FXOS8700_ID                (0xC7)     // 1100 0111
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    /*!
        Raw register addresses used to communicate with the sensor.
    */
    typedef enum
    {
      FXOS8700_REGISTER_STATUS          = 0x00, /**< 0x00 */
      FXOS8700_REGISTER_OUT_X_MSB       = 0x01, /**< 0x01 */
      FXOS8700_REGISTER_OUT_X_LSB       = 0x02, /**< 0x02 */
      FXOS8700_REGISTER_OUT_Y_MSB       = 0x03, /**< 0x03 */
      FXOS8700_REGISTER_OUT_Y_LSB       = 0x04, /**< 0x04 */
      FXOS8700_REGISTER_OUT_Z_MSB       = 0x05, /**< 0x05 */
      FXOS8700_REGISTER_OUT_Z_LSB       = 0x06, /**< 0x06 */
      FXOS8700_REGISTER_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
      FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
      FXOS8700_REGISTER_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MSTATUS         = 0x32, /**< 0x32 */
      FXOS8700_REGISTER_MOUT_X_MSB      = 0x33, /**< 0x33 */
      FXOS8700_REGISTER_MOUT_X_LSB      = 0x34, /**< 0x34 */
      FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35, /**< 0x35 */
      FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36, /**< 0x36 */
      FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37, /**< 0x37 */
      FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38, /**< 0x38 */
      FXOS8700_REGISTER_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
    } fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    /*!
        Range settings for the accelerometer sensor.
    */
    typedef enum
    {
      ACCEL_RANGE_2G                    = 0x00, /**< +/- 2g range */
      ACCEL_RANGE_4G                    = 0x01, /**< +/- 4g range */
      ACCEL_RANGE_8G                    = 0x02  /**< +/- 8g range */
    } fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    /*!
        @brief  Raw (integer) values from the gyroscope sensor.
    */
    typedef struct
    {
      int16_t x;    /**< Raw int16_t value from the x axis */
      int16_t y;    /**< Raw int16_t value from the y axis */
      int16_t z;    /**< Raw int16_t value from the z axis */
    } fxos8700RawData_t;
/*=========================================================================*/

	typedef struct
    {
      float x;    /**< Raw int16_t value from the x axis */
      float y;    /**< Raw int16_t value from the y axis */
      float z;    /**< Raw int16_t value from the z axis */
    } fxos8700Data_t;

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the Adafruit FXOS8700 breakout.
*/
/**************************************************************************/
class Adafruit_FXOS8700 : public I2c
{
  public:
    Adafruit_FXOS8700(int32_t accelSensorID = -1, int32_t magSensorID = -1);

    bool begin           ( fxos8700AccelRange_t rng = ACCEL_RANGE_2G );
    void standby         ( bool standby );
    void writeRegister(uint8_t reg, uint8_t value);
    void readSensor();
    void calculateVelocity();

    /*! Raw accelerometer values from last sucsessful sensor read */
    fxos8700RawData_t accel_raw;
    /*! Converted accelerometer data (in m/s^2)  */
    fxos8700Data_t accel_ms2;
    fxos8700Data_t accel_old;
    fxos8700Data_t accel_vel;
    std::vector<fxos8700Data_t> accel_vel_old;
    /*! Raw magnetometer values from last successful sensor read */
    fxos8700RawData_t mag_raw;
    // magnetometer data as uTesla
    fxos8700Data_t mag_uTesla;
    int readI2C() { return 0; } // Unused

  private:

    fxos8700AccelRange_t _range;
    int32_t              _accelSensorID;
    int32_t              _magSensorID;
    int _i2c_address;
    
};

#endif
