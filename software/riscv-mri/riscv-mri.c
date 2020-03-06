/* Copyright 2019 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <metal/cpu.h>
#include <metal/led.h>
#include <metal/uart.h>
#include <metal/machine.h>

#include "mri.h"
void __mriDebugException(void);  // Not sure why this isn't in an MRI header file (probably an oversight)
void Platform_setMemoryFaultFlag(void);

#define RTC_FREQ	32768

struct metal_cpu *cpu;
struct metal_interrupt *cpu_intr, *tmr_intr;
struct metal_interrupt *uart0_ic;
struct metal_led *led0_red;
int tmr_id, uart0_irq;


void call_mri(int tickle_led) {
    if (tickle_led)
      metal_led_on(led0_red);
    __mriDebugException();
    if (tickle_led)    
      metal_led_off(led0_red);    
}

void uart0_isr (int id, void *data) {
    call_mri(1);
}


void exception_handler_mem_fault(struct metal_cpu *cpu, int ecode) {
  Platform_setMemoryFaultFlag();  
}

int main (void)
{
    int rc;
    struct metal_uart *uart0;
    size_t txcnt;
    int baud_rate;

    __mriInit("");  // For RISC-V, not using any argument string (at this time)

    // Lets get start with getting LEDs and turn only RED ON
    led0_red = metal_led_get_rgb("LD0", "red");
    if (led0_red == NULL) {
        printf("LED red is null.\n");
        return 1;
    }
    metal_led_enable(led0_red);
 
    // Lets get the CPU and and its interrupt
    cpu = metal_cpu_get(metal_cpu_get_current_hartid());
    if (cpu == NULL) {
        printf("CPU null.\n");
        return 2;
    }
    cpu_intr = metal_cpu_interrupt_controller(cpu);
    if (cpu_intr == NULL) {
        printf("CPU interrupt controller is null.\n");
        return 3;
    }
    metal_interrupt_init(cpu_intr);

    // Setup UART 0 and its interrupt
    uart0 = metal_uart_get_device(0);
    metal_uart_set_baud_rate(uart0, 115200);   // 4800 seems to work, 19200 seems to work
    baud_rate = metal_uart_get_baud_rate(uart0);    
    uart0_ic = metal_uart_interrupt_controller(uart0);
    if (uart0_ic == NULL) {
        printf("UART0 interrupt controller is null.\n");
        return 4;
    }
    metal_interrupt_init(uart0_ic);
    uart0_irq = metal_uart_get_interrupt_id(uart0);
    rc = metal_interrupt_register_handler(uart0_ic, uart0_irq, uart0_isr, led0_red);
    if (rc < 0) {
        printf("Uart0 interrupt handler registration failed\n");
        return (rc * -1);
    }

    rc = metal_cpu_exception_register(cpu, 0x5, exception_handler_mem_fault);
    

    // Lets enable the Uart interrupt
    txcnt =  metal_uart_get_transmit_watermark(uart0);
    metal_uart_set_transmit_watermark(uart0, 1);
    // metal_uart_transmit_interrupt_enable(uart0);
    if (metal_interrupt_enable(uart0_ic, uart0_irq) == -1) {
        printf("Uart0 interrupt enable failed\n");
        return 5;
    }
    // Lastly CPU interrupt
    if (metal_interrupt_enable(cpu_intr, 0) == -1) {
        printf("CPU interrupt enable failed\n");
        return 6;
    }

    metal_uart_set_transmit_watermark(uart0, txcnt);
    //    metal_uart_transmit_interrupt_enable(uart0);
    metal_uart_receive_interrupt_enable(uart0);
    //display_instruction();

    while (1) {
        __asm__ volatile("wfi");
    }

    return 0;
}
