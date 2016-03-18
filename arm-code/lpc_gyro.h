#ifndef LPC_GYRO_H_
#define LPC_GYRO_H_

#include <stdint.h>
#include <core/i2c/i2c.h>

//Gyroscope defines
#define ITG3205     0x68
#define ITG3205WR   0xD0
#define ITG3205RD   0xD1

#define SMPLRT_DIV    0x15
#define DLPF_FS       0x16
#define INT_CFG       0x17
#define GYRO_XOUT_H   0x1D
#define PWR_MGM       0x3E
#define GYRO_SCALING  14.375
#define CALIB_SAMPLES 10 //max 255

typedef struct gyro_data_t {
  float x; //roll
  float y; //pitch
  float z; //yaw

  int16_t x_off;
  int16_t y_off;
  int16_t z_off;

  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
} gyro_data_t;

void initGyro(void);
void readGyro(volatile gyro_data_t *gyro_data);
void calibrateGyro(volatile gyro_data_t *gyro_data);

#endif
