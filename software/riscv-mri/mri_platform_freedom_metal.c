#include "platforms.h"
#include <metal/uart.h>

static struct metal_uart *uart0;

/* Freedom Metal doesn't expose the RX_EMPTY bit independently of the return status
   when trying to consume a character, so we'll buffer zero or one characters here.  If this
   scheme is problematic, then we could hack Freedom Metal to expose the RX_EMPTY somehow. */
static int uart0_input_lookahead = -1;  /* -1 means "none", otherwise a character value */

static int waiting_for_gdb_to_connect;

void Platform_Init(Token* pParameterTokens)
{
  uart0 = metal_uart_get_device(0);
  uart0_input_lookahead = -1;
  waiting_for_gdb_to_connect = 1;
}


int Platform_CommReceiveChar(void)
{
  // From porting guide: Platform_CommReceiveChar() | Returns next received character from GDB. This call should block. |
  int c;

  while (!Platform_CommHasReceiveData()) {
    /* spin */
  }
  /* When we get here, we are guaranteed to have a character in lookahead.
     Consume and return it */
  c = uart0_input_lookahead;
  uart0_input_lookahead = -1;
  
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
  int result;
  int c;
  
  if (uart0_input_lookahead == -1) {
    /* Try to get one character without blocking */
      result = metal_uart_getc(uart0, &c);
      if (result == 0) {
	uart0_input_lookahead = c;
      }
  }
  return (uart0_input_lookahead != -1);
}

void Platform_CommClearInterrupt(void)
{
  // From porting guide: Platform_CommClearInterrupt() | Clear any active UART interrupts. |
  // For this implementation (the one using Freedom Metal), Freedom Metal is doing this further up the call stack
}

int Platform_CommIsWaitingForGdbToConnect(void)
{
  return waiting_for_gdb_to_connect;
}

void Platform_CommWaitForReceiveDataToStop(void)
{
  // From porting guide: Platform_CommWaitForReceiveDataToStop() | Used to throw away received data if MRI has determined that first connection didn't originate from GDB. |
  while (Platform_CommHasReceiveData()) {
    int c = Platform_CommReceiveChar();
    (void)c;  // bit bucket
  }
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


