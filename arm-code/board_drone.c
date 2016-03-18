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

//Gyroscope defines
#define ITG3205     0x68
#define ITG3205WR   0xD0
#define ITG3205RD   0xD1

#define SMPLRT_DIV  0x15
#define DLPF_FS     0x16
#define INT_CFG     0x17
#define GYRO_XOUT_H 0x1D
#define PWR_MGM     0x3E
#define GYRO_SCALING 14.375

#define DELTA_TIME 0.010

#include <string.h> /* strlen */

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"
#include "core/adc/adc.h"
#include "core/i2c/i2c.h"


#ifdef CFG_USB
#include "core/usb/usbd.h"
#ifdef CFG_USB_CDC
#include "core/usb/usb_cdc.h"
#endif
#endif

typedef struct gyro_data_t {
  float x; //roll
  float y; //pitch
  float z; //yaw

  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
} gyro_data_t;

uint32_t duty = 1900;
volatile uint32_t tick = 0;
volatile gyro_data_t gyro_data = {0};

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
  @brief Board-specific initialisation function
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
}

void gyroInit(void) {
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

void readGyro(gyro_data_t * gyro_data) {
  I2CReadLength = 6;
  I2CWriteLength = 2; //inclusive of address
  I2CMasterBuffer[0] = ITG3205WR;
  I2CMasterBuffer[1] = GYRO_XOUT_H;
  I2CMasterBuffer[2] = ITG3205RD;

  i2cEngine(); //i2cengine blocks until error or finished

  gyro_data->raw_x = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
  gyro_data->raw_y = (I2CSlaveBuffer[2] << 8) | I2CSlaveBuffer[3];
  gyro_data->raw_z = (I2CSlaveBuffer[4] << 8) | I2CSlaveBuffer[5];

}

/**************************************************************************/
/*
 * SYSTICK HANDLER
 */
/**************************************************************************/

void SysTick_Handler(void) {
  //LPC_GPIO->NOT[PORT0] |= (1 << P0_8);
  gyro_data.x += (gyro_data.raw_x/GYRO_SCALING) * DELTA_TIME;
  gyro_data.y += (gyro_data.raw_y/GYRO_SCALING) * DELTA_TIME;
  gyro_data.z += (gyro_data.raw_z/GYRO_SCALING) * DELTA_TIME;
}

/**************************************************************************/
/*!
  @brief Primary (non-RTOS!) entry point for this project.
  */
/**************************************************************************/
int main(void)
{

  /* configure the HW */
  boardInit();
  gyroInit();

  for (;;)
  {
    LPC_GPIO->NOT[PORT0] |= (1 << P0_7);

    readGyro(&gyro_data);
    printf("%f %f %f\n",
        gyro_data.x, gyro_data.y, gyro_data.z);

    if (LPC_GPIO->B0[P0_17] == 0) {
      gyro_data.x = 0;
      gyro_data.y = 0;
      gyro_data.z = 0;
    }
    _delay_ms(10);
  }
}

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
