#ifndef MINI_RTOS_H
#define MINI_RTOS_H

#include <stdint.h>

typedef enum {
    TASK_READY = 0,
    TASK_DELAY
} task_state_t;

typedef struct {
    void (*task_func)(void);
    uint32_t delay_ticks;   // ???? tick ?
    task_state_t state;
} task_t;

void rtos_init(void);
void rtos_add_task(void (*task_func)(void));
void rtos_start(void);

// ? SysTick_Handler ???
void rtos_tick(void);

// ?????:?????
void task_delay(uint32_t ticks);

#endif
