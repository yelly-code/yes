#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    p->integral  = 0.0f;
    p->last_meas = 0.0f;
    p->started   = 0;
    p->error     = 0.0f;
    p->out       = 0.0f;
}

float PID_Update(PID_t *p, float setpoint, float measure, float dt)
{
    if (dt <= 0.0f) return p->out;

    float err = setpoint - measure;          //  算误差

    p->integral += err * dt;                 //  积分累加
    if (p->ki > 1e-6f)                       //   积分限幅（抗饱和）
    {
        float i_term = p->ki * p->integral;
        if      (i_term > p->out_max) p->integral = p->out_max / p->ki;
        else if (i_term < p->out_min) p->integral = p->out_min / p->ki;
    }

    float d_term = 0.0f;                     // 微分：对测量值求导
    if (p->started)
        d_term = -(measure - p->last_meas) / dt;
    p->last_meas = measure;
    p->started   = 1;

    float out = p->kp * err + p->ki * p->integral + p->kd * d_term;   //

    if (out > p->out_max) out = p->out_max;  // 输出限幅
    if (out < p->out_min) out = p->out_min;

    p->error = err;
    p->out   = out;
    return out;
}
