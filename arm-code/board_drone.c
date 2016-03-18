#include "projectconfig.h"

#ifdef CFG_BRD_DRONE

#define ADC_LPWRMODE 22
#define AD0 0
#define AD0_CLKDIV 8
#define AD0_START 24
#define AD0_DONE 31

#define P0_7 7
#define P0_8 8
#define P0_16 16
#define P0_17 17

#define SYSCLK_IOCON 7
#define SYSCLK_ADC 13

#define SYSCLK_CT32B0 9
#define SYSCLK_CT32B1 10

#define SYSCLK_CT16B0 7
#define SYSCLK_CT16B1 8

#define PINMODE_CT32B1_MAT0 0xa3
#define PINMODE_AD0 0x02
#define RESET_MR3 10

#define FIL_ALPHA 0.98

#define DELTA_TIME 0.010

#include <string.h> /* strlen */
#include <math.h>

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"
#include "core/adc/adc.h"
#include "core/i2c/i2c.h"
#include "lpc_gyro.h"
#include "lpc_accel.h"

#ifdef CFG_USB
#include "core/usb/usbd.h"
#ifdef CFG_USB_CDC
#include "core/usb/usb_cdc.h"
#endif
#endif

typedef struct copter_t {
  float roll;
  float pitch;
  float yaw;
} copter_t;

uint32_t duty = 1900;
volatile uint32_t tick = 0;

volatile copter_t quad_copter = {0};
volatile gyro_data_t gyro_data = {0};
volatile accel_data_t accel_data = {0};

void boardInit(void);

void _delay_ms (uint16_t ms);
/**************************************************************************/
/*
 * SYSTICK HANDLER
 */
/**************************************************************************/

void SysTick_Handler(void) {
  //time it using oscope
  //LPC_GPIO->NOT[PORT0] |= (1 << P0_8);
  //integrate
  gyro_data.x += (gyro_data.raw_x/GYRO_SCALING) * DELTA_TIME;
  gyro_data.y += (gyro_data.raw_y/GYRO_SCALING) * DELTA_TIME;
  gyro_data.z += (gyro_data.raw_z/GYRO_SCALING) * DELTA_TIME;

  accel_data.xg = accel_data.raw_x * ACCEL_SCALING;
  accel_data.yg = accel_data.raw_y * ACCEL_SCALING;
  accel_data.zg = accel_data.raw_z * ACCEL_SCALING;

  //get position from z facing down (probs need to change)
  accel_data.roll = atan2(accel_data.yg,
      sqrt(accel_data.xg*accel_data.xg + accel_data.zg*accel_data.zg))*
    180/M_PI;

  accel_data.pitch = atan2(-accel_data.xg,
      sqrt(accel_data.yg*accel_data.yg + accel_data.zg*accel_data.zg))*
    180/M_PI;

  //quad_copter.roll = accel_data.roll;
  //quad_copter.pitch = accel_data.pitch;

  //RENAME XYZ to ROLL ETC
  //complementary filter
  quad_copter.roll = FIL_ALPHA * (gyro_data.x) +
    (1-FIL_ALPHA)*accel_data.roll;

  quad_copter.pitch = FIL_ALPHA * (gyro_data.y) +
    (1-FIL_ALPHA)*accel_data.pitch;

  quad_copter.yaw = gyro_data.z;
}

/**************************************************************************/
int main(void) {

  /* configure the HW */
  boardInit();
  initAccel();
  initGyro();

  calibrateAccel(&accel_data);
  calibrateGyro(&gyro_data);

  for (;;) {
    LPC_GPIO->NOT[PORT0] |= (1 << P0_7);

    readGyro(&gyro_data);
    readAccel(&accel_data);

    printf("%f %f %f\n",
        quad_copter.roll, quad_copter.pitch, quad_copter.yaw);

    //clear gyro values
    if (LPC_GPIO->B0[P0_17] == 0) {
      gyro_data.x = 0;
      gyro_data.y = 0;
      gyro_data.z = 0;
    }

    _delay_ms(10);
  }
} //end main

//crappy delay but I need to use systick and counter interrupts
void _delay_ms (uint16_t ms)
{
  uint16_t delay;
  volatile uint32_t i;
  //1ms loop with -Os optimisation
  for (delay = ms; delay >0 ; delay--) {
    for (i=3500; i >0;i--);
  }
}

/**************************************************************************/
/*!
  Board-specific initialisation function
  */
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();

  /**************************************************************************/
  /*
   * GPIO SETUP
   */
  /**************************************************************************/

  GPIOInit();

  LPC_GPIO->DIR[PORT0] |= (1 << P0_7);
  LPC_GPIO->DIR[PORT0] |= (1 << P0_8);
  LPC_GPIO->DIR[PORT0] &= ~(1 << P0_17); //set to inputs
  LPC_GPIO->DIR[PORT0] &= ~(1 << P0_16);

  /**************************************************************************/
  /*
   * USB SETUP
   */
  /**************************************************************************/

  /* Initialise USB */
#ifdef CFG_USB
  _delay_ms(2000);
  usb_init();
#endif


  /**************************************************************************/
  /*
   * ADC SETUP
   */
  /**************************************************************************/

  //LPC_IOCON->TDI_PIO0_11   &= ~0x9F;
  //LPC_IOCON->TDI_PIO0_11  |= 0x02;


  //adcInit();

  /**************************************************************************/
  /*
   * SYSTICK SETUP
   */
  /**************************************************************************/

  // Initialize systick interrupt to 10 ms 100 hz
  SysTick->CTRL = 0x07;
  SysTick->LOAD = 0x000afc7f; //freq  = (system clk * desired interval) - 1

  /**************************************************************************/
  /*
   * I2C SETUP
   */
  /**************************************************************************/

  i2cInit(I2CMASTER);
} //boardInit

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
