#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

#include "stm32h7xx_hal.h"
typedef enum
{
    PROTECT_MODE,   //????
    REMOTER_MODE,   //????
    AUTO_MODE,    //????
    KEYBOARD_MODE,  //????
    UNPROTECT_MODE,
} ctrl_mode_e;

typedef struct {
    int32_t rise;
    int32_t high;
    uint8_t state;     // 0=等待上升沿  1=等待下降沿
		uint8_t signal_effective;
} IC_t;

void timmer_comm_init(void);
void user_timmer_handle(void);
void ControlMsgHandle(void);
void remoteHandler(void);
void controler_task(void);

#define SPD_TRANSFORM_RATIO 1.5

extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;

#endif
