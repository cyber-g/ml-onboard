#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stdint.h>
#include "FreeRTOS.h"

typedef struct {
        TickType_t timestamp;
        int activity;
        int16_t xl[3];
} xl_data;

void lsm6dsl_init(int prio, int freq_ms);
void lsm6dsl_read_accel(xl_data *data);

#endif // LSM6DSL_H