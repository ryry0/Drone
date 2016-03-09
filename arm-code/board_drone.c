#include "projectconfig.h"

#ifdef CFG_BRD_DRONE

#include <string.h> /* strlen */

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_CDC
    #include "core/usb/usb_cdc.h"
  #endif
#endif

#ifdef CFG_PROTOCOL
  #include "protocol/protocol.h"
#endif

#ifdef CFG_ENABLE_UART
  #include "core/uart/uart.h"
#endif

/**************************************************************************/
/*!
    @brief Board-specific initialisation function
*/
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();
  delayInit();
  GPIOInit();

  #ifdef CFG_PRINTF_UART
    uartInit(CFG_UART_BAUDRATE);
  #endif

  /* Set user LED pin to output and disable it */
  LPC_GPIO->DIR[CFG_LED_PORT] |= (1 << CFG_LED_PIN);
  boardLED(CFG_LED_OFF);

  /* Initialise USB */
  #ifdef CFG_USB
    delay(500);
    usb_init();
  #endif

  /* Turn the user LED on after init to indicate that everything is OK */
  boardLED(CFG_LED_ON);
}

/**************************************************************************/
/*!
    @brief Primary (non-RTOS!) entry point for this project.
*/
/**************************************************************************/
volatile uint32_t test[8] = { 0 };
int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  /* Configure the HW */
  boardInit();

  while (1)
  {
    /* Blinky (1Hz) */
    currentSecond = delayGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;
      boardLED(lastSecond % 2);
    }

    /* Optionally enter high level sleep mode here via WFI */
  }
}

/**************************************************************************/
/*!
    @brief Turns the LED(s) on or off
*/
/**************************************************************************/
void boardLED(uint8_t state)
{
  if (state)
  {
    LPC_GPIO->SET[CFG_LED_PORT] = (1 << CFG_LED_PIN);
  }
  else
  {
    LPC_GPIO->CLR[CFG_LED_PORT] = (1 << CFG_LED_PIN);
  }
}

/**************************************************************************/
/*!
    @brief  Configure the board for low power and enter sleep mode
*/
/**************************************************************************/
void boardSleep(void)
{
  // ToDo!
}

/**************************************************************************/
/*!
    @brief  Restores parts and system peripherals to an appropriate
            state after waking up from sleep mode
*/
/**************************************************************************/
void boardWakeup(void)
{
  // ToDo!
}

#endif /* CFG_BRD_LPCXPRESSO_LPC1347 */
