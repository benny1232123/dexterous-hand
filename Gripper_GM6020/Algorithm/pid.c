#include "pid.h"

/*
    @brief initialise PID parameters
*/
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->outputLimit = outputLimit;
    pid->integralLimit = integralLimit;

    pid->Target = 0.0f;
    pid->Last_measured = 0.0f;
    pid->integral = 0.0f;
}

/*
    @brief change the setpoint
*/
void PID_SetTarget(PID *pid, float Target)
{
    pid->Target = Target;
}

/*
    @brief execute one PID step
*/
float PID_compute(PID *pid, float measured, float dt)
{
    /* proportional error */
    float error = pid->Target - measured;
    /* integral term */
    pid->integral = error * dt + pid->integral;
    /* integral clamp */
    if (pid->integral >= pid->integralLimit) pid->integral = pid->integralLimit;
    if (pid->integral <= -pid->integralLimit) pid->integral = -pid->integralLimit;

    /* local copies */
    float Kp = pid->Kp;
    float Ki = pid->Ki;
    float Kd = pid->Kd;
    float integral = pid->integral;
    float last_measured = pid->Last_measured;

    /* PID output, derivative on measurement */
    float output = Kp * error + Ki * integral + Kd * (last_measured - measured) / dt;
    /* output clamp */
    if (output >= pid->outputLimit) output = pid->outputLimit;
    if (output <= -pid->outputLimit) output = -pid->outputLimit;

    /* store last measured value */
    pid->Last_measured = measured;
    return output;
}

/*
    @brief change PID parameters
*/
void setPID_parameter(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->outputLimit = outputLimit;
    pid->integralLimit = integralLimit;
}

/*
    @brief ResetPID
*/
void ResetPID(PID *pid)
{
    pid->Target = 0.0f;
    pid->Last_measured = 0.0f;
    pid->integral = 0.0f;
}
