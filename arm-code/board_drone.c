#include "projectconfig.h"

#ifdef CFG_BRD_DRONE

//Pin Defines
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

#define PINMODE_AD0 0x02
#define RESET_MR3 10

//Filter Defines
#define FIL_ALPHA 0.98
#define DELTA_TIME 0.010

//Serial Defines
#define BAUDRATE 115200
#define SERIAL_DEBUG

//PWM defines
#define PWMEN0 0
#define PWMEN1 1
#define PWMEN2 2

#define PINMODE_CT16B0_MAT0 0x22
#define PINMODE_CT16B1_MAT0 0x21

#define PINMODE_CT32B0_MAT0 0x21
#define PINMODE_CT32B1_MAT0 0xa3

#define PRESCALER 719
#define ZERO_MOTOR_SPEED 1900
#define PERIOD_MATCH 1999

//just assume always get packet of length x
#define PACKET_LENGTH 12

#include <string.h> /* strlen */
#include <math.h>
#include <stdbool.h>

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"
#include "core/adc/adc.h"
#include "core/i2c/i2c.h"
#include "core/uart/uart.h"
#include "lpc_gyro.h"
#include "lpc_accel.h"
#include "lpc_wifi.h"

#ifdef SERIAL_DEBUG
  #ifdef CFG_USB
    #include "core/usb/usbd.h"
    #ifdef CFG_USB_CDC
      #include "core/usb/usb_cdc.h"
    #endif
  #endif
#endif

/**************************************************************************/
/* VARIABLE DEFINITIONS */
/**************************************************************************/
typedef struct copter_t { //current actual values for quadcopter
  float roll;
  float pitch;
  float yaw;
} copter_t;

typedef union copter_setpoints_t { //setpoints for PID algo
  struct {
    float set_roll;
    float set_pitch;
    float set_yaw;
  };
  uint8_t data[12];
} copter_setpoints_t;

typedef enum states_t {OFF, RUNNING} states_t;
typedef enum input_states_t {WAITING, UPDATING} input_states_t;

/**************************************************************************/
/* GLOBAL VARIABLES */
/**************************************************************************/

volatile copter_t quad_copter = {0};
volatile gyro_data_t gyro_data = {0};
volatile accel_data_t accel_data = {0};

/**************************************************************************/
/* FUNCTION PROTOTYPES */
/**************************************************************************/
void boardInit(void);
void _delay_ms (uint16_t ms);

/**************************************************************************/
/* SYSTICK HANDLER */
/**************************************************************************/
void SysTick_Handler(void) {
  //integrate gyro data for angle
  /*
  gyro_data.yaw += (gyro_data.raw_yaw_dot/GYRO_SCALING) * DELTA_TIME;

  accel_data.xg = accel_data.raw_x * ACCEL_SCALING;
  accel_data.yg = accel_data.raw_y * ACCEL_SCALING;
  accel_data.zg = accel_data.raw_z * ACCEL_SCALING;

  //get position from z facing down (probs need to change)
  accel_data.roll = atan2(accel_data.yg, accel_data.zg)* 180/M_PI;

  accel_data.pitch = atan2(-accel_data.xg,
      sqrt(accel_data.yg*accel_data.yg + accel_data.zg*accel_data.zg))*
    180/M_PI;

  //complementary filter
  quad_copter.roll = FIL_ALPHA *
    (quad_copter.roll + ((gyro_data.raw_roll_dot/GYRO_SCALING)*DELTA_TIME)) +
    (1-FIL_ALPHA) * accel_data.roll;

  quad_copter.pitch = FIL_ALPHA *
    (quad_copter.pitch + ((gyro_data.raw_pitch_dot/GYRO_SCALING)*DELTA_TIME)) +
    (1-FIL_ALPHA) * accel_data.pitch;

  quad_copter.yaw = gyro_data.yaw;
  */
} //end SysTick_Handler

/**************************************************************************/
/* MAIN */
/**************************************************************************/
int main(void) {
  states_t state = OFF;
  input_states_t input_state = WAITING;

  int8_t packet_index = 0;
  uint8_t input_byte = 0;
  //int8_t input_buff[PACKET_LENGTH] = {0};
  bool input_updated = false;
  copter_setpoints_t copter_setpoints;

  // configure the HW
  boardInit();
  /*
  initAccel();
  initGyro();
  calibrateAccel(&accel_data);
  calibrateGyro(&gyro_data);
  */

  uartSend((uint8_t *)AT_RESET, strlen(AT_RESET));
  uartSend((uint8_t *)AT_STATION, strlen(AT_STATION));
  uartSend((uint8_t *)AT_CONNECT, strlen(AT_CONNECT));
  _delay_ms(1000);
  uartSend((uint8_t *)AT_MULTI_CONN, strlen(AT_MULTI_CONN));
  _delay_ms(1000);
  uartSend((uint8_t *)AT_CREAT_SERVER, strlen(AT_CREAT_SERVER));
  _delay_ms(1000);

  LPC_GPIO->B0[P0_7] = 1;

  //main loop
  for (;;) {

    switch(state) {
      case OFF:
        /*
           LPC_CT32B0->MR0 = ZERO_MOTOR_SPEED;
           LPC_CT32B1->MR0 = ZERO_MOTOR_SPEED;
           LPC_CT16B0->MR0 = ZERO_MOTOR_SPEED;
           LPC_CT16B1->MR0 = ZERO_MOTOR_SPEED;
           */

        if (LPC_GPIO->B0[P0_17] == 0) {
          while (LPC_GPIO->B0[P0_17] == 0);
          sendWifiCommand(AT_GET_IP);
        }

        if (LPC_GPIO->B0[P0_16] == 0) {
          while (LPC_GPIO->B0[P0_16] == 0);
          state = RUNNING;
        }
        break;

      case RUNNING:
        /*
           readGyro(&gyro_data);
           readAccel(&accel_data);

        //clear gyro values
        if (LPC_GPIO->B0[P0_17] == 0) {
        gyro_data.roll = 0;
        gyro_data.pitch = 0;
        gyro_data.yaw = 0;
        }
        */

        switch(input_state) {
          case WAITING:
            if (input_byte == ':') {
              input_state = UPDATING;
              input_updated = false;
            }
            break;

          case UPDATING:
            if (input_updated) {
              copter_setpoints.data[packet_index++] = input_byte;

              printf("packet index: %d %d\n", packet_index, (int) input_byte);

              if (packet_index >= PACKET_LENGTH) {
                input_state = UPDATING;
                packet_index = 0;

#ifdef SERIAL_DEBUG
                printf("\n%f %f %f\n", copter_setpoints.set_roll,
                    copter_setpoints.set_pitch,
                    copter_setpoints.set_yaw);
#endif
                //update actual setpoints here since it will be used in interrupt
              }
              input_updated = false;
            }
            break;
        }

        break; //end RUNNING

      default:
        break;
    }


    //flush the read buffer
    if (uartRxBufferDataPending()) {
      input_byte = uartRxBufferRead();
      input_updated = true;
#ifdef SERIAL_DEBUG
      printf("%c", (char) input_byte);
#endif
    }

#ifdef SERIAL_DEBUG
    //printf("%f %f %f\n",
    //quad_copter.roll, quad_copter.pitch, quad_copter.yaw);
#endif

  } //end for(;;)
} //end main

/**************************************************************************/
/* DELAY MS */
/**************************************************************************/
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
/* BOARD INIT */
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();

  /**************************************************************************/
  /* GPIO SETUP */
  /**************************************************************************/

  GPIOInit();

  LPC_GPIO->DIR[PORT0] |= (1 << P0_7);
  //LPC_GPIO->DIR[PORT0] |= (1 << P0_8);
  LPC_GPIO->DIR[PORT0] &= ~(1 << P0_17); //set to inputs
  LPC_GPIO->DIR[PORT0] &= ~(1 << P0_16);
  LPC_GPIO->B0[P0_7] = 0; //turn off led until initialized

  /**************************************************************************/
  /* PWM SETUP */
  /**************************************************************************/
  //enable counters and io control block
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << SYSCLK_IOCON)  |
    (1 << SYSCLK_CT32B0) |
    (1 << SYSCLK_CT32B1) |
    (1 << SYSCLK_CT16B0) |
    (1 << SYSCLK_CT16B1);

  //configure pin0 13 as match output
  LPC_IOCON->TDO_PIO0_13  = PINMODE_CT32B1_MAT0;
  //configure pin0 8
  LPC_IOCON->PIO0_8 = PINMODE_CT16B0_MAT0;
  //configure pin0 21
  LPC_IOCON->PIO0_21 = PINMODE_CT16B1_MAT0;
  //configure pin1 24
  LPC_IOCON->PIO1_24 = PINMODE_CT32B0_MAT0;

  LPC_CT32B0->MCR |= (1 << RESET_MR3);
  LPC_CT32B1->MCR |= (1 << RESET_MR3);
  LPC_CT16B0->MCR |= (1 << RESET_MR3);
  LPC_CT16B1->MCR |= (1 << RESET_MR3);

  //PWM freq = (sys_clock/prescaler)/tick_match
  //set the duty cycle
  LPC_CT32B0->MR0 = ZERO_MOTOR_SPEED;
  LPC_CT32B1->MR0 = ZERO_MOTOR_SPEED;
  LPC_CT16B0->MR0 = ZERO_MOTOR_SPEED;
  LPC_CT16B1->MR0 = ZERO_MOTOR_SPEED;

  //set the period -> timer will be reset when it hits this value
  LPC_CT32B0->MR3 = PERIOD_MATCH; //set the pwm freq to 50hz
  LPC_CT32B1->MR3 = PERIOD_MATCH;
  LPC_CT16B0->MR3 = PERIOD_MATCH;
  LPC_CT16B1->MR3 = PERIOD_MATCH;

  //set the prescaler
  LPC_CT32B0->PR = PRESCALER;
  LPC_CT32B1->PR = PRESCALER;
  LPC_CT16B0->PR = PRESCALER;
  LPC_CT16B1->PR = PRESCALER;

  LPC_CT32B0->PWMC |= (1 << PWMEN0) |
    (1 << PWMEN1) |
    (1 << PWMEN2);
  LPC_CT32B1->PWMC |= (1 << PWMEN0) |
    (1 << PWMEN1) |
    (1 << PWMEN2);
  LPC_CT16B0->PWMC |= (1 << PWMEN0) |
    (1 << PWMEN1) |
    (1 << PWMEN2);
  LPC_CT16B1->PWMC |= (1 << PWMEN0) |
    (1 << PWMEN1) |
    (1 << PWMEN2);

  LPC_CT32B0->TCR = 1;
  LPC_CT32B1->TCR = 1;
  LPC_CT16B0->TCR = 1;
  LPC_CT16B1->TCR = 1;

  /**************************************************************************/
  /* USB SETUP */
  /**************************************************************************/

  /* Initialise USB */
#ifdef SERIAL_DEBUG
#ifdef CFG_USB
  _delay_ms(2000);
  usb_init();
#endif
#endif

  /**************************************************************************/
  /* SYSTICK SETUP */
  /**************************************************************************/

  // Initialize systick interrupt to 10 ms 100 hz
  SysTick->CTRL = 0x07;
  SysTick->LOAD = 0x000afc7f; //freq  = (system clk * desired interval) - 1

  /**************************************************************************/
  /* I2C SETUP */
  /**************************************************************************/

  i2cInit(I2CMASTER);

  /**************************************************************************/
  /* UART SETUP */
  /**************************************************************************/

  uartInit(BAUDRATE);
} //boardInit

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
