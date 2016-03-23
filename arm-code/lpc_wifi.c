#include <lpc_wifi.h>

inline void sendCommand(char *command) {
  uartSend((uint8_t *)command, strlen(command));
}
