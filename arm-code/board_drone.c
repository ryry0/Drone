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
  delayInit();

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
   * PWM SETUP
   */
  /**************************************************************************/
  //enable counters and io control block
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << SYSCLK_IOCON)  |
    (1 << SYSCLK_CT32B0) |
    (1 << SYSCLK_CT32B1) |
    (1 << SYSCLK_CT16B0) |
    (1 << SYSCLK_CT16B1);

  //configure pin 13 as match output
  LPC_IOCON->TDO_PIO0_13  = PINMODE_CT32B1_MAT0;
  LPC_CT32B1->MCR |= (1 << RESET_MR3);


  //PWM freq = (sys_clock/prescaler)/tick_match
  LPC_CT32B1->MR0 = duty; //set the duty cycle

  //set the period -> timer will be reset when it hits this value
  LPC_CT32B1->MR3 = 1999; //set the pwm freq to 50hz

  //set the prescaler
  LPC_CT32B1->PR = 719;

  LPC_CT32B1->PWMC |= (1 << PWMEN0) |
    (1 << PWMEN1) |
    (1 << PWMEN2);

  LPC_CT32B1->TCR = 1; //enable timer


  /**************************************************************************/
  /*
   * ADC SETUP
   */
  /**************************************************************************/

  LPC_IOCON->TDI_PIO0_11   &= 0xFFFFFF9F;
  LPC_IOCON->TDI_PIO0_11  |= 0x02;

  adcInit();
  /*
  LPC_SYSCON->PDRUNCFG &= ~(1 << ADC_PD); //power up adc
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << SYSCLK_ADC); //send clock to adc

  */

  //LPC_ADC->CR = (1 << ADC_LPWRMODE) | (1 << AD0) | (4 << AD0_CLKDIV);
  /*
  LPC_ADC->CR = (0x01 << 0)                            |
    ((SystemCoreClock / ADC_CLK - 1) << 8) |  // CLKDIV = Fpclk / 1000000 - 1
    (0 << 16)                              |  // BURST = 0, no BURST, software controlled
    (0 << 17)                              |  // CLKS = 0, 11 clocks/10 bits
#if CFG_ADC_MODE_LOWPOWER
    (1 << 22)                              |  // Low-power mode
#endif
#if CFG_ADC_MODE_10BIT
    (1 << 23)                              |  // 10-bit mode
#endif
    (0 << 24)                              |  // START = 0 A/D conversion stops
    (0 << 27);                                // EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion)
    */
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
    /* Blinky (1Hz) */
    currentSecond = delayGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;
      LPC_GPIO->B0[P0_7] = lastSecond % 2;
      //LPC_GPIO->B0[P0_8] = lastSecond % 2;
    }

    /*
    //start ad0 conversion
    LPC_ADC->CR |= (1 << AD0_START);

    //wait until done
    while (regdata < 0x7FFFFFFF) {
      regdata = LPC_ADC->GDR;
    }

    //stop ADC Conversion
    LPC_ADC->CR &= ~(0x07 << AD0_START); //clear the start bits
    LPC_ADC->CR &= 0xF8FFFFFF; //clear the start bits

    //get the result starting at the 4th bit and mask it

    //adc_result = (LPC_ADC->GDR >> 4) & 0xfff;
    */

    if (LPC_GPIO->B0[P0_16] == 0) {
      duty -= 5;
      while (LPC_GPIO->B0[P0_16] == 0);
    }

    if (LPC_GPIO->B0[P0_17] == 0) {
      duty += 5;
      while (LPC_GPIO->B0[P0_17] == 0);
    }

    if (duty < 1800)
      duty = 1800;
    else if (duty > 1900)
      duty = 1900;

    LPC_CT32B1->MR0 = duty;
  }
}

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
