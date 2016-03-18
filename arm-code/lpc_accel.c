#include <lpc_accel.h>


/**************************************************************************/
/*
 * INIT ACCEL
 */
/**************************************************************************/

void initAccel(void) {

  //set data format
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ADXL345WR;
  I2CMasterBuffer[1] = DATA_FORMAT;
  I2CMasterBuffer[2] = 0x01;
  i2cEngine();

  //set set power to wakeup
  I2CReadLength = 0;
  I2CWriteLength = 3; //Inclusive of the address
  I2CMasterBuffer[0] = ADXL345WR;
  I2CMasterBuffer[1] = POWER_CTL;
  I2CMasterBuffer[2] = 0x08;
  i2cEngine();

} //initAccel


/**************************************************************************/
/*
 * READ ACCEL
 */
/**************************************************************************/

void readAccel(volatile accel_data_t *accel_data) {
  I2CReadLength = 6;
  I2CWriteLength = 2; //inclusive of address
  I2CMasterBuffer[0] = ADXL345WR;
  I2CMasterBuffer[1] = ACCEL_DATA_X0;
  I2CMasterBuffer[2] = ADXL345RD;

  i2cEngine(); //i2cengine blocks until error or finished

  accel_data->raw_x = ((I2CSlaveBuffer[1] << 8) | I2CSlaveBuffer[0]) -
    accel_data->x_off;
  accel_data->raw_y = ((I2CSlaveBuffer[3] << 8) | I2CSlaveBuffer[2]) -
    accel_data->y_off;
  accel_data->raw_z = ((I2CSlaveBuffer[5] << 8) | I2CSlaveBuffer[4]) -
    accel_data->z_off;

} //readAccel


/**************************************************************************/
/*
 * CALIBRATE ACCEL
 */
/**************************************************************************/

void calibrateAccel(volatile accel_data_t *accel_data) {
  int16_t x_sum = 0, y_sum = 0, z_sum = 0;

  accel_data->x_off = 0;
  accel_data->y_off = 0;
  accel_data->z_off = 0;

  for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
    readAccel(accel_data);

    x_sum += accel_data->raw_x;
    y_sum += accel_data->raw_y;
    z_sum += accel_data->raw_z;
  }

  accel_data->x_off = x_sum/CALIB_SAMPLES;
  accel_data->y_off = y_sum/CALIB_SAMPLES;
  accel_data->z_off = (z_sum/CALIB_SAMPLES) - 256; 
  //assuming 1g on z axis
} //end calibrateAccel

