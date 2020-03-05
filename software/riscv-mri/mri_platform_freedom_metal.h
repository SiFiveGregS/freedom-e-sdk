#include "riscv.h"
#if 0
#include <stdint.h>

#if __riscv_xlen == 32
typedef uint32_t RISCV_X_VAL;
#else
typedef uint64_t RISCV_X_VAL;
#endif


/* Important!  Do not modify this structure layout without making corresponding changes to
   the assembly code referred to by the label "mri_context" */
typedef struct {
  RISCV_X_VAL flags;
  RISCV_X_VAL x_1_31[31];  // Not including x0!  Its value is fixed at zero always, of course.
  RISCV_X_VAL mepc;
  RISCV_X_VAL mcause;
  RISCV_X_VAL mstatus;  
  RISCV_X_VAL reentered;  
  RISCV_X_VAL reentered_mepc;
  RISCV_X_VAL reentered_mcause;
  RISCV_X_VAL reentered_mstatus;  
} MRI_CONTEXT_RISCV;
#endif

extern MRI_CONTEXT_RISCV mri_context;
