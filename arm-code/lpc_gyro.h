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
  float roll; //x
  float pitch; //y
  float yaw; //z

  int16_t roll_dot_off;
  int16_t pitch_dot_off;
  int16_t yaw_dot_off;


  union {
    struct {
      int16_t raw_roll_dot;
      int16_t raw_pitch_dot;
      int16_t raw_yaw_dot;
    };
    int16_t angle_dots[3];
  };
} gyro_data_t;

void initGyro(void);
void readGyro(volatile gyro_data_t *gyro_data);
void calibrateGyro(volatile gyro_data_t *gyro_data);

#endif
