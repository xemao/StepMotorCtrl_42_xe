#ifndef TB67H450_H
#define TB67H450_H

#include <stdbool.h>
#include <stdint.h>

/* FOC电流矢量控制 */
/* _directionInCount: 电角度 (0-1023)
   _current_mA: 电流值 (mA) */
void TB67H450_SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA);

/* 休眠 */
void TB67H450_Sleep(void);

/* 刹车 */
void TB67H450_Brake(void);

#endif
