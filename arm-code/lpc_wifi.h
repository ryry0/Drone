#ifndef LPC_WIFI_H_
#define LPC_WIFI_H_

#include <core/uart/uart.h>
#include <stdint.h>
#include <string.h>

//Wifi Defines
#define AT_RESET        "AT+RST\r\n"
#define AT_ACCEPT_CONNS "AT+CWMODE=3\r\n"
#define AT_STATION      "AT+CWMODE=1\r\n"
#define AT_FIRMWARE     "AT+GMR\r\n"
#define AT_SCAN         "AT+CWLAP\r\n"
#define AT_GET_IP       "AT+CIFSR\r\n"
#define AT_CREAT_SERVER "AT+CIPSERVER=1,5555\r\n"
#define AT_MULTI_CONN   "AT+CIPMUX=1\r\n"

inline void sendCommand(char *command) {
  uartSend((uint8_t *)command, strlen(command));
}

#endif
