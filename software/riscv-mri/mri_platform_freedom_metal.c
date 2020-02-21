#include "platforms.h"
#include <metal/uart.h>

static struct metal_uart *uart0;

void Platform_Init(Token* pParameterTokens)
{
  uart0 = metal_uart_get_device(0);  
}


int Platform_CommReceiveChar(void)
{
  // From porting guide: Platform_CommReceiveChar() | Returns next received character from GDB. This call should block. |
  int c;
  int result;
  do {
    result = metal_uart_getc(uart0, &c);
  } while (!(result == 0 && c != -1));
  return c;
}


int Platform_CommCausedInterrupt(void)
{
  // From porting guide: Platform_CommCausedInterrupt() | Was a character received from GDB the reason for the halt. |
  return 0;
}

int Platform_CommShouldWaitForGdbConnect(void)
{
  // From porting guide: Platform_CommShouldWaitForGdbConnect() | Does MRI need to wait for GDB to connect before progressing?  For example if it needs to wait for first '+' character from GDB to auto detect baud rate. |
  return 0;
}

uint32_t Platform_CommHasReceiveData(void)
{
  // From porting guide: Platform_CommHasReceiveData() | Returns 0 if no data from GDB has already been received and non-zero otherwise. |
  return 0;
}

void Platform_CommClearInterrupt(void)
{
  // From porting guide: Platform_CommClearInterrupt() | Clear any active UART interrupts. |
}

int Platform_CommIsWaitingForGdbToConnect(void)
{
  // From porting guide: Platform_CommIsWaitingForGdbToConnect() | Is MRI currently waiting for GDB to connect? |
  return 0;
}

void Platform_CommWaitForReceiveDataToStop(void)
{
  // From porting guide: Platform_CommWaitForReceiveDataToStop() | Used to throw away received data if MRI has determined that first connection didn't originate from GDB. |
}

void Platform_CommPrepareToWaitForGdbConnection(void)
{
  // From porting guide: Platform_CommPrepareToWaitForGdbConnection() | Typically used to start auto-baud detection. |
}


void Platform_CommSendChar(int c)
{
  // From porting guide: Platform_CommSendChar() | Sends a character to GDB. |
  while (metal_uart_txready(uart0) != 0) {
    /* keep waiting */
  }
  while (metal_uart_putc(uart0, c) != 0) {
    /* try again */
  }
}

uint32_t Platform_GetDeviceMemoryMapXmlSize(void)
{
  return 0;
}


const char* Platform_GetDeviceMemoryMapXml(void)
{
  return NULL;
}


