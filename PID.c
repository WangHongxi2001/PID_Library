/**
  ******************************************************************************
  * @file    pid.c
  * @author  Hongxi Wong
  * @version V1.0.0
  * @date    2019/12/9
  * @brief   对每一个pid结构体都要先进行函数的连接，再进行初始化
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#include "pid.h"

/***************************PID_param_init******************************/
static void f_PID_param_init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,
    uint16_t period,

    float kp,
    float ki,
    float kd,

    uint8_t improve)
{
    pid->ControlPeriod = period;
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->target = 0;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->iout = 0;

    pid->Improve = improve;
    pid->Output_WindUp = 0;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->output = 0;
}

/**************************PID_param_reset*********************************/
static void f_PID_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/***************************PID_calculate**********************************/
static float f_PID_calculate(PID_TypeDef *pid, float measure)
{
    if (pid->Improve & ErrorHandle) //ErrorHandle
    {
        f_PID_ErrorHandle(pid); // last_xxx = xxx
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->output = 0;
            return -1; //Catch ERROR
        }
    }

    pid->measure = measure;
    pid->err = pid->target - pid->measure;

    if (ABS(pid->err) > pid->DeadBand)
    {
        pid->pout = pid->kp * pid->err;
        pid->iout += pid->ki * pid->err;
        pid->dout = pid->kd * (pid->err - pid->last_err);

        //Trapezoid Intergral
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);

        //Derivative On Measurement
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);

        //Integral limit
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->output = pid->pout + pid->iout + pid->dout; //pid calculate
        if (pid->Improve & OutputFilter)
            pid->output = pid->output * 0.7f + pid->last_output * 0.3f;

        //Output limit
        if (pid->output > pid->MaxOut)
        {
            pid->output = pid->MaxOut;
        }
        if (pid->output < -(pid->MaxOut))
        {
            pid->output = -(pid->MaxOut);
        }
    }
    pid->last_measure = pid->measure;
    pid->last_output = pid->output;
    pid->last_err = pid->err;

    return pid->output;
}

/*****************PID Improvement Function*********************/
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->iout -= pid->ki * pid->err;
    pid->iout += pid->ki * ((pid->err + pid->last_err) * pid->ControlPeriod / 2);
}

static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output;
    temp_Output = pid->pout + pid->iout + pid->dout;
    if (temp_Output > pid->MaxOut)
    {
        if (pid->err * pid->iout > 0) //Integral still increasing
        {

            if (pid->Improve & Trapezoid_Intergral)
                pid->iout -= pid->ki * ((pid->err + pid->last_err) * pid->ControlPeriod / 2);
            else
                pid->iout -= pid->ki * pid->err;
        }
    }
    if (pid->iout > pid->IntegralLimit)
        pid->iout = pid->IntegralLimit;
    if (pid->iout < -pid->IntegralLimit)
        pid->iout = -pid->IntegralLimit;
}

static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->dout = pid->kd * (pid->last_measure - pid->measure);
}

/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    /*ERROR HANDLE*/
    if (pid->target < 100)
        return;

    if ((ABS(pid->output - pid->measure) / pid->output) > 0.9f)
    {
        pid->ERRORHandler.ERRORCount++; //Motor blocked counting
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 150) //Motor blocked over 150times generate ERROR
    {
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*****************PID Initialize Function*********************/
void PID_Init(PID_TypeDef *pid)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset;
    pid->PID_Calc = f_PID_calculate;
}
