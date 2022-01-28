#pragma once

#include <common.h>

typedef struct {
    bool paused;
    bool running;
    bool die;
    u64 ticks;
} emu_context;

void cpu_run(void *p);

void emu_run(char *argv) ;

emu_context *emu_get_context();

void emu_cycles(int cpu_cycles);
