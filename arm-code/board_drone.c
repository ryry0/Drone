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


#ifdef CFG_USB
#include "core/usb/usbd.h"
#ifdef CFG_USB_CDC
#include "core/usb/usb_cdc.h"
#endif
#endif

uint32_t duty = 1900;

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

  LPC_IOCON->TDI_PIO0_11   &= ~0x9F;
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

  /* Initialize systick interrupt */
  SysTick->CTRL = 0x07;
  SysTick->LOAD = 0x00057e3f;
}


/**************************************************************************/
/*
 * SYSTICK HANDLER
 */
/**************************************************************************/

void SysTick_Handler(void) {
  //LPC_GPIO->NOT[PORT0] |= (1 << P0_8);
}

/**************************************************************************/
/*!
  @brief Primary (non-RTOS!) entry point for this project.
  */
/**************************************************************************/
int main(void)
{
  uint32_t currentSecond, lastSecond;
  uint32_t adc_result = 0, regdata = 0;
  currentSecond = lastSecond = 0;

  /* Configure the HW */
  boardInit();

  for (;;)
  {
    LPC_GPIO->NOT[PORT0] |= (1 << P0_7);

    /*
    //start ad0 conversion
    LPC_ADC->CR |= (1 << AD0_START);

    //wait until done
    regdata = LPC_ADC->DR0;
    while (regdata < 0x7FFFFFFF) {
      regdata = LPC_ADC->DR0;
    }

    //stop ADC Conversion
    LPC_ADC->CR &= ~(0x07 << AD0_START); //clear the start bits
    LPC_ADC->CR &= 0xF8FFFFFF; //clear the start bits

    //get the result starting at the 4th bit and mask it

    adc_result = (LPC_ADC->DR0 >> 4) & 0xfff;
    */

    adc_result = adcRead(0);

    if (adc_result > 0x10)
      LPC_GPIO->B0[P0_8] = 1;
    else
      LPC_GPIO->B0[P0_8] = 0;

    printf("ADC: %x\n", adc_result);
    _delay_ms(1000);
  }
}

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
