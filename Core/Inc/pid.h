#ifndef __PID_H
#define __PID_H

#include <stdint.h>


typedef struct
{
    float kp, ki, kd;
    float out_min, out_max;

    float   integral;
    float   last_meas;
    uint8_t started;

    float error;
    float out;
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float setpoint, float measure, float dt);

#endif
