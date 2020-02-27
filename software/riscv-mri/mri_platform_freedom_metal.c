#include "platforms.h"
#include "mri_platform_freedom_metal.h"
#include <metal/uart.h>

#define DISABLE_APPARENTLY_ARM_SPECIFIC_CODE 1

static struct metal_uart *uart0;

/* Freedom Metal doesn't expose the RX_EMPTY bit independently of the return status
   when trying to consume a character, so we'll buffer zero or one characters here.  If this
   scheme is problematic, then we could hack Freedom Metal to expose the RX_EMPTY somehow. */
static int uart0_input_lookahead = -1;  /* -1 means "none", otherwise a character value */

static int waiting_for_gdb_to_connect;

static int      shouldRemoveHardwareBreakpointOnSvcHandler(void);
static void     removeHardwareBreakpointOnSvcHandlerIfNeeded(void);
static void     clearMemoryFaultFlag(void);
static void     cleanupIfSingleStepping(void);
static void     clearHardwareBreakpointOnSvcHandler(void);
static void     clearSvcStepFlag(void);
static void     restoreBasePriorityIfNeeded(void);
static void     configureMpuToAccessAllMemoryWithNoCaching(void);
static void enableMPUWithHardAndNMIFaults(void);
static void     configureHighestMpuRegionToAccessAllMemoryWithNoCaching(void);
static void disableMPU(void);
static void     saveOriginalMpuConfiguration(void);

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

  // This implementation assumes that the only external machine mode interrupt that is
  // configured to enter __mriDebugException is from the UART.  If that assumption is not
  // correct for a particular board, this implementation may need refinement.
  RISCV_X_VAL interrupt_mask = (((RISCV_X_VAL)1) << (__riscv_xlen-1));
  RISCV_X_VAL exception_code_mask = interrupt_mask-1;
  int is_interrupt = ((mri_context.mcause & interrupt_mask) != 0);
  int code = (mri_context.mcause & exception_code_mask);

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
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    __mriCortexMState.originalPC = __mriCortexMState.context.PC;    
#else
#endif  
    configureMpuToAccessAllMemoryWithNoCaching();
    cleanupIfSingleStepping();
}


static void clearMemoryFaultFlag(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_FAULT_DURING_DEBUG;
#else
#endif  
}

static void cleanupIfSingleStepping(void)
{
    restoreBasePriorityIfNeeded();
    removeHardwareBreakpointOnSvcHandlerIfNeeded();
    Platform_DisableSingleStep();
}


static void removeHardwareBreakpointOnSvcHandlerIfNeeded(void)
{
    if (shouldRemoveHardwareBreakpointOnSvcHandler())
    {
        clearSvcStepFlag();
        clearHardwareBreakpointOnSvcHandler();
    }
}

static int shouldRemoveHardwareBreakpointOnSvcHandler(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    return __mriCortexMState.flags & CORTEXM_FLAGS_SVC_STEP;  
#else
    return 0;  /* implement, if needed for RISC-V */    
#endif  
}

static void clearHardwareBreakpointOnSvcHandler(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    Platform_ClearHardwareBreakpoint(getNvicVector(SVCall_IRQn) & ~1, 2);
#else
#endif  
}


static void clearSvcStepFlag(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_SVC_STEP;
#else
#endif  
}

static void restoreBasePriorityIfNeeded(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE  
    if (shouldRestoreBasePriority())
    {
        clearRestoreBasePriorityFlag();
        __set_BASEPRI(__mriCortexMState.originalBasePriority);
        __mriCortexMState.originalBasePriority = 0;
    }
#endif    
}

static void configureMpuToAccessAllMemoryWithNoCaching(void)
{
    saveOriginalMpuConfiguration();
    disableMPU();
    configureHighestMpuRegionToAccessAllMemoryWithNoCaching();    
    enableMPUWithHardAndNMIFaults();
}

static void enableMPUWithHardAndNMIFaults(void)
{
    // RESOLVE - implement for RISC-V
}


static void configureHighestMpuRegionToAccessAllMemoryWithNoCaching(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    static const uint32_t regionToStartAtAddress0 = 0U;
    static const uint32_t regionReadWrite = 1  << MPU_RASR_AP_SHIFT;
    static const uint32_t regionSizeAt4GB = 31 << MPU_RASR_SIZE_SHIFT; /* 4GB = 2^(31+1) */
    static const uint32_t regionEnable    = MPU_RASR_ENABLE;
    static const uint32_t regionSizeAndAttributes = regionReadWrite | regionSizeAt4GB | regionEnable;
    
    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(regionToStartAtAddress0);
    setMPURegionAttributeAndSize(regionSizeAndAttributes);
#else
#endif  
}

static void disableMPU(void)
{
    // RESOLVE - implement for RISC-V
}

static void saveOriginalMpuConfiguration(void)
{
#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE
    __mriCortexMState.originalMPUControlValue = getMPUControlValue();
    __mriCortexMState.originalMPURegionNumber = getCurrentMPURegionNumber();
    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    __mriCortexMState.originalMPURegionAddress = getMPURegionAddress();
    __mriCortexMState.originalMPURegionAttributesAndSize = getMPURegionAttributeAndSize();
#else
#endif  
}

int Platform_WasMemoryFaultEncountered(void)
{
    int wasFaultEncountered;

#ifndef DISABLE_APPARENTLY_ARM_SPECIFIC_CODE    
    __DSB();
    wasFaultEncountered = __mriCortexMState.flags & CORTEXM_FLAGS_FAULT_DURING_DEBUG;
#else
    wasFaultEncountered = 0; // implement for RISC-V
#endif    
    clearMemoryFaultFlag();
    
    return wasFaultEncountered;    
}
