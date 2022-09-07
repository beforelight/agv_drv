#include "drv_shell.h"
#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

Shell lshell;
char shellBuffer[512];
FIFO shell_fifo;
signed short userShellWrite(char *data, unsigned short len)
{
  HAL_UART_Transmit(&huart3, (const uint8_t *) data, len, ~0);
  // for (int i = 0; i < len; i++) {
  //   while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET) {
  //   }
  //   huart3.Instance->DR = data[i];
  // }
  return len;
}

void shell_init()
{
  FIFO_Init(&shell_fifo, 64);
  lshell.write = userShellWrite;
  shellInit(&lshell, shellBuffer, sizeof(shellBuffer));
}

int myPrintf(const char *formatString, ...)
{
  static char printBuf[256];
  va_list arg;
  int logLength;
  va_start(arg, formatString);
  logLength = vsnprintf(printBuf, sizeof(printBuf), formatString, arg);
  va_end(arg);
#if SHELL_SUPPORT_END_LINE == 1
  shellWriteEndLine(&lshell, printBuf, logLength);
#else
  HAL_UART_Transmit(&huart3, (const uint8_t *) printBuf, logLength, ~0);
#endif
  return logLength;
}
