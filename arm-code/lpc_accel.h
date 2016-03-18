#ifndef LPC_ACCEL_H_
#define LPC_ACCEL_H_
#include <stdint.h>
#include <core/i2c/i2c.h>

//Accelerometer defines
#define ADXL345       0x53
#define ADXL345WR     0xA6
#define ADXL345RD     0xA7
#define DATA_FORMAT   0x01
#define POWER_CTL     0x2D
#define ACCEL_SCALING 0.0039
#define ACCEL_DATA_X0 0x32

#define CALIB_SAMPLES 10

//value in g is (g-rnge / 2^resolution)
typedef struct accel_data_t {
  float roll;
  float pitch;
  float yaw;

  //Accelerometer val in gs
  float xg;
  float yg;
  float zg;

  int16_t x_off;
  int16_t y_off;
  int16_t z_off;

  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
} accel_data_t;

void initAccel(void);
void readAccel(volatile accel_data_t *accel_data);
void calibrateAccel(volatile accel_data_t *accel_data);

#endif
