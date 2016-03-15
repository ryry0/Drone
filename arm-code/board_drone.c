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

#define PWMEN0 0
#define PWMEN1 0
#define PWMEN2 0

#include <string.h> /* strlen */

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"
#include "core/adc/adc.h"

uint32_t duty = 1900;

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

  //LPC_SYSCON->SYSTICKCLKDIV = 0x01; //enable system clock and divide by 1
  SysTick->CTRL = 0x07;
  SysTick->LOAD = 0x00057e3f;
}


/**************************************************************************/
/*
 * SYSTICK HANDLER
 */
/**************************************************************************/

void SysTick_Handler(void) {
  LPC_GPIO->NOT[PORT0] |= (1 << P0_8);
}

/**************************************************************************/
/*!
    @brief Primary (non-RTOS!) entry point for this project.
*/
/**************************************************************************/
int main(void)
{
  uint32_t currentSecond, lastSecond;
  uint32_t adc_result = 0;
  volatile uint32_t regdata = 0;
  currentSecond = lastSecond = 0;

  /* Configure the HW */
  boardInit();

  for (;;)
  {

  }
}

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
