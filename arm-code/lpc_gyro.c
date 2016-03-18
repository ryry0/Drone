#include <lpc_gyro.h>


/**************************************************************************/
/*
 * INIT GYRO
 */
/**************************************************************************/
void initGyro(void) {
  //set pwr_mgm to zero
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = PWR_MGM;
  I2CMasterBuffer[2] = 0x00;
  i2cEngine();

  //set sample rate divider to 0x07
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = SMPLRT_DIV;
  I2CMasterBuffer[2] = 0x07;
  i2cEngine();

  //set Digital lpf to 0x1e
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = DLPF_FS;
  I2CMasterBuffer[2] = 0x1E;
  i2cEngine();

  //set interrupt config to 0x0
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = INT_CFG;
  I2CMasterBuffer[2] = 0x00;
  i2cEngine();

} //end gyroInit

/**************************************************************************/
/*
 * CALIBRATE GYRO
 */
/**************************************************************************/
void calibrateGyro(volatile gyro_data_t *gyro_data) {
  int16_t x_sum = 0, y_sum = 0, z_sum = 0;

  gyro_data->x_off = 0;
  gyro_data->y_off = 0;
  gyro_data->z_off = 0;

  for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
    readGyro(gyro_data);

    x_sum += gyro_data->raw_x;
    y_sum += gyro_data->raw_y;
    z_sum += gyro_data->raw_z;
  }

  gyro_data->x_off = x_sum/CALIB_SAMPLES;
  gyro_data->y_off = y_sum/CALIB_SAMPLES;
  gyro_data->z_off = z_sum/CALIB_SAMPLES;
} //end calibrateGyro


/**************************************************************************/
/*
 * READ GYRO
 */
/**************************************************************************/
void readGyro(volatile gyro_data_t *gyro_data) {
  I2CReadLength = 6;
  I2CWriteLength = 2; //inclusive of address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = GYRO_XOUT_H;
  I2CMasterBuffer[2] = ITG3205RD;

  i2cEngine(); //i2cengine blocks until error or finished

  gyro_data->raw_x = ((I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1]) -
    gyro_data->x_off;
  gyro_data->raw_y = ((I2CSlaveBuffer[2] << 8) | I2CSlaveBuffer[3]) -
    gyro_data->y_off;
  gyro_data->raw_z = ((I2CSlaveBuffer[4] << 8) | I2CSlaveBuffer[5]) -
    gyro_data->z_off;

} //end readGyro
