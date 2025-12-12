#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "drv_dmmotor.h"
#include "pid.h"
#include "EZ_RTOS.H"
#include "tim.h"
// Simplified RTOS in C with task delay support
#include <stdint.h>
#include <stdio.h>

#include "EZ_RTOS.h"

#define MAX_TASKS 8

static task_t task_list[MAX_TASKS];
static uint8_t task_count = 0;
static uint32_t system_ticks = 0;

void rtos_init(void)
{
    task_count = 0;
    system_ticks = 0;

    for (int i = 0; i < MAX_TASKS; i++)
    {
        task_list[i].task_func = 0;
        task_list[i].delay_ticks = 0;
        task_list[i].state = TASK_READY;
    }
}

void rtos_add_task(void (*task_func)(void))
{
    if (task_count < MAX_TASKS)
    {
        task_list[task_count].task_func = task_func;
        task_list[task_count].state = TASK_READY;
        task_list[task_count].delay_ticks = 0;
        task_count++;
    }
}

void rtos_tick(void)
{
    system_ticks++;

    for (int i = 0; i < task_count; i++)
    {
        if (task_list[i].state == TASK_DELAY)
        {
            if (task_list[i].delay_ticks > 0)
            {
                task_list[i].delay_ticks--;
                if (task_list[i].delay_ticks == 0)
                {
                    task_list[i].state = TASK_READY; // ???? ? ??????
                }
            }
        }
    }
}

void task_delay(uint32_t ticks)
{
    // ??????????
    for (int i = 0; i < task_count; i++)
    {
        if (task_list[i].task_func == __builtin_return_address(0))
        {
            task_list[i].delay_ticks = ticks;
            task_list[i].state = TASK_DELAY;
            return;
        }
    }
}

void rtos_start(void)
{
    while (1)
    {
        for (int i = 0; i < task_count; i++)
        {
            if (task_list[i].state == TASK_READY && task_list[i].task_func)
            {
                task_list[i].task_func();  // ????
            }
        }
    }
}


void delay_us(uint32_t us)
{
    // Set timer period for desired delay in microseconds
    __HAL_TIM_SET_AUTORELOAD(&htim3, us - 1); //????????period*?????
    HAL_TIM_Base_Start(&htim3);               // start the timer

    while (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) == RESET)
        ;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE); //???????
    HAL_TIM_Base_Stop(&htim3);                     // Stop the timer
}
