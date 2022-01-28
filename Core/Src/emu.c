#include <stdio.h>
#include <emu.h>
#include <cart.h>
#include <cpu.h>
#include <ui.h>
#include <timer.h>
#include <dma.h>
#include <ppu.h>
#include "main.h"
#include "cmsis_os.h"


/* 
  Emu components:

  |Cart|
  |CPU|
  |Address Bus|
  |PPU|
  |Timer|

*/

static emu_context ctx;

emu_context *emu_get_context() {
    return &ctx;
}

void cpu_run(void *p) {
    timer_init();
    cpu_init();
    ppu_init();

    ctx.running = true;
    ctx.paused = false;
    ctx.ticks = 0;

    while(ctx.running) {
        if (ctx.paused) {
            osDelay(10);
            continue;
        }

        if (!cpu_step()) {
            printf("CPU Stopped\n");
            Error_Handler();
        }
    }
    Error_Handler();

}

void emu_run(char *argv) {

    if (!cart_load(argv)) {
        printf("Failed to load ROM file: %s\n", argv);
        Error_Handler();
    }

    printf("Cart loaded..\r\n");

    ui_init();

    u32 prev_frame = 0;

    while(!ctx.die) {
	osDelay(1);
        ui_handle_events();

        if (prev_frame != ppu_get_context()->current_frame) {
            ui_update();
        }

        prev_frame = ppu_get_context()->current_frame;
    }

}

void emu_cycles(int cpu_cycles) {
    
    for (int i=0; i<cpu_cycles; i++) {
        for (int n=0; n<4; n++) {
            ctx.ticks++;
            timer_tick();
            ppu_tick();
        }

        dma_tick();
    }
}
