#include "platforms.h"
#include "mri_platform_freedom_metal.h"
#include <metal/uart.h>
#include <string.h>

static struct metal_uart *uart0;


/* Freedom Metal doesn't expose the RX_EMPTY bit independently of the return status
   when trying to consume a character, so we have to buffer at least one character,
   and depending on baud rate we might need to buffer more to prevent data loss, but
   for now, we'll just try buffer of depth 1 and if we do need more, then bumping up
   UART_BUF_SIZE may be sufficient */

#define UART_BUF_SIZE 1
typedef struct {
  int num_bytes;
  int rp;
  int wp;
  char buf[UART_BUF_SIZE];
} UART_BUFFER;

void UartBufferInit(UART_BUFFER *p);
int UartBufferEmpty(UART_BUFFER *p);
int UartBufferFull(UART_BUFFER *p);
int UartBufferRead(UART_BUFFER *p);
int UartBufferWrite(UART_BUFFER *p, int ch);

void UartBufferInit(UART_BUFFER *p) {
  p->num_bytes = 0;
  p->rp = 0;
  p->wp = 0;
  memset(p->buf, 0, sizeof(p->buf));
}

int UartBufferEmpty(UART_BUFFER *p) {
  return p->num_bytes == 0;
}

int UartBufferFull(UART_BUFFER *p) {
  return p->num_bytes == sizeof(p->buf);
}

int UartBufferRead(UART_BUFFER *p) {
  if (p->num_bytes == 0) {
    return -1;
  } else {
    int c = p->buf[p->rp];    
    p->rp++;
    if (p->rp == sizeof(p->buf))
      p->rp = 0;
    p->num_bytes--;
    return c;
  }
}

int UartBufferWrite(UART_BUFFER *p, int ch) {
  if (p->num_bytes == sizeof(p->buf))
    return -1;
  
  p->buf[p->wp] = ch;
  p->wp++;
  p->num_bytes++;
  if (p->wp == sizeof(p->buf))
    p->wp = 0;
  return 0;
}


static UART_BUFFER uart_buffer;
static int waiting_for_gdb_to_connect;

static void     clearMemoryFaultFlag(void);
static void     cleanupIfSingleStepping(void);
static void     clearSvcStepFlag(void);

void Platform_Init(Token* pParameterTokens)
{
  uart0 = metal_uart_get_device(0);
  UartBufferInit(&uart_buffer);
  waiting_for_gdb_to_connect = 1;
}


int Platform_CommReceiveChar(void)
{
  // From porting guide: Platform_CommReceiveChar() | Returns next received character from GDB. This call should block. |
  int c;

  while (!Platform_CommHasReceiveData()) {
    /* spin */
  }
  /* When we get here, we are guaranteed to have at least one character in the buffer.
     Consume and return the oldest unconsumed character from the buffer */
  c = UartBufferRead(&uart_buffer);
  
  return c;
}


int Platform_CommCausedInterrupt(void)
{
  // From porting guide: Platform_CommCausedInterrupt() | Was a character received from GDB the reason for the halt. |

  // This implementation assumes that the only external machine mode interrupt that is
  // configured to enter __mriDebugException is from the UART.  If that assumption is not
  // correct for a particular board, this implementation may need refinement.
  RISCV_X_VAL interrupt_mask = (((RISCV_X_VAL)1) << (__riscv_xlen-1));
  RISCV_X_VAL exception_code_mask = interrupt_mask-1;
  RISCV_X_VAL mcause = __mriRiscVState.context.mcause;  

  int is_interrupt = ((mcause & interrupt_mask) != 0);
  int code = (mcause & exception_code_mask);

  return (is_interrupt && code == 11);
}

int Platform_CommShouldWaitForGdbConnect(void)
{
  // From porting guide: Platform_CommShouldWaitForGdbConnect() | Does MRI need to wait for GDB to connect before progressing?  For example if it needs to wait for first '+' character from GDB to auto detect baud rate. |

  // It seems like this needs to be 1, otherwise MRI sends a stop reply packet on the first entry
  //  to __mriDebugException, and that doesn't seem to fit what GDB expects (in the situation where
  //  this software is started first and then GDB connects).  Might need to experiment with this.
  //  Maybe some other issue is leading to that behavior.  But for now, trying this.
  return 1;
}

uint32_t Platform_CommHasReceiveData(void)
{
  // From porting guide: Platform_CommHasReceiveData() | Returns 0 if no data from GDB has already been received and non-zero otherwise. |
  int result;
  int c;
  
  if (UartBufferEmpty(&uart_buffer)) {
    /* Try to slurp in as many immediately available bytes as we can get (and will fit in our buffer) */
    while (!UartBufferFull(&uart_buffer)) {
      result = metal_uart_getc(uart0, &c);
      if (result == 0 && c != -1) {
	UartBufferWrite(&uart_buffer, c);
      } else {
	break;
      }
    }
  }
  return (!UartBufferEmpty(&uart_buffer));
}

void Platform_CommClearInterrupt(void)
{
  /* Freedom Metal will have already done this when it called
     __metal_plic0_claim_interrupt(), via __metal_plic0_handler() */

  /* However, we do need to set the exit flag, otherwise the pre-entry state
     doesn't get restored exactly the same as it was */
    __mriRiscVState.flags |= MRI_RISCV_FLAG_EXITING;  
}

int Platform_CommIsWaitingForGdbToConnect(void)
{
  if (Platform_CommCausedInterrupt()) {
    waiting_for_gdb_to_connect = 0;
  }
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


void Platform_EnteringDebugger(void)
{
    clearMemoryFaultFlag();
    __mriRiscVState.originalPC = __mriRiscVState.context.mepc;
    cleanupIfSingleStepping();
}


static void clearMemoryFaultFlag(void)
{
  __mriRiscVState.flags &= ~MRI_RISCV_FLAG_REENTERED;
}

static void cleanupIfSingleStepping(void)
{
    Platform_DisableSingleStep();
}




int Platform_WasMemoryFaultEncountered(void)
{
    int wasFaultEncountered;

    wasFaultEncountered = (__mriRiscVState.flags & MRI_RISCV_FLAG_REENTERED) != 0 && __mriRiscVState.context.reentered_mcause == 0x5;
    if (wasFaultEncountered)
      clearMemoryFaultFlag();
    
    return wasFaultEncountered;    
}
