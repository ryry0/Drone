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
  int16_t roll_dot_sum = 0, pitch_dot_sum = 0, yaw_dot_sum = 0;

  gyro_data->roll_dot_off = 0;
  gyro_data->pitch_dot_off = 0;
  gyro_data->yaw_dot_off = 0;

  for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
    readGyro(gyro_data);

    roll_dot_sum += gyro_data->raw_roll_dot;
    pitch_dot_sum += gyro_data->raw_pitch_dot;
    yaw_dot_sum += gyro_data->raw_yaw_dot;
  }

  gyro_data->roll_dot_off = roll_dot_sum/CALIB_SAMPLES;
  gyro_data->pitch_dot_off = pitch_dot_sum/CALIB_SAMPLES;
  gyro_data->yaw_dot_off = yaw_dot_sum/CALIB_SAMPLES;
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

  gyro_data->raw_roll_dot = ((I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1]) -
    gyro_data->roll_dot_off;
  gyro_data->raw_pitch_dot = ((I2CSlaveBuffer[2] << 8) | I2CSlaveBuffer[3]) -
    gyro_data->pitch_dot_off;
  gyro_data->raw_yaw_dot = ((I2CSlaveBuffer[4] << 8) | I2CSlaveBuffer[5]) -
    gyro_data->yaw_dot_off;

} //end readGyro
