/**
  ******************************************************************************
  * @file    pid.c
  * @author  Hongxi Wong
  * @version V1.0.6
  * @date    2019/12/21
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

    float A,
    float B,

    uint8_t improve)
{
    pid->ControlPeriod = period;
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->Target = 0;

    pid->Kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ITerm = 0;

    pid->ScalarA = A;
    pid->ScalarB = B;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**************************PID_param_reset*********************************/
static void f_PID_reset(PID_TypeDef *pid, float Kp, float ki, float kd)
{
    pid->Kp = Kp;
    pid->ki = ki;
    pid->kd = kd;
}

/***************************PID_calculate**********************************/
float PID_Calculate(PID_TypeDef *pid, float measure, float target)
{
    if (pid->Improve & ErrorHandle) //ErrorHandle
    {
        f_PID_ErrorHandle(pid); // last_xxx = xxx
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return -1; //Catch ERROR
        }
    }

    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->ki * pid->Err;
        pid->Dout = pid->kd * (pid->Err - pid->Last_Err);

        //Proportional limit
        f_Proportion_limit(pid);

        //Trapezoid Intergral
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        //Changing Integral Rate
        if (pid->Improve & ChangingIntegralRate)
            f_Changing_Integral_Rate(pid);
        //Integral limit
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);
        //Derivative On Measurement
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout; //pid calculate

        //Output Filter
        if (pid->Improve & OutputFilter)
            f_OutputFilter(pid);

        //Output limit
        if (pid->Output > pid->MaxOut)
        {
            pid->Output = pid->MaxOut;
        }
        if (pid->Output < -(pid->MaxOut))
        {
            pid->Output = -(pid->MaxOut);
        }
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Err = pid->Err;

    return pid->Output;
}

/*****************PID Improvement Function*********************/
static void f_Proportion_limit(PID_TypeDef *pid)
{
    //Proportion limit is insignificant for control process
    //but it enable variable chart to look better
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->ki * ((pid->Err + pid->Last_Err) * pid->ControlPeriod / 2);
}

static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if (pid->Err * pid->Iout > 0) //Integral still increasing
    {
        if (ABS(pid->Err) <= pid->ScalarB)
            return; //Full integral
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) //Integral still increasing
        {
            pid->ITerm = 0;
        }
    }
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->kd * (pid->Last_Measure - pid->Measure);
}

static void f_OutputFilter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * 0.7f + pid->Last_Output * 0.3f;
}

/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    /*ERROR HANDLE*/
    if (pid->Target < 100)
        return;

    if ((ABS(pid->Output - pid->Measure) / pid->Output) > 0.9f)
    {
        pid->ERRORHandler.ERRORCount++; //Motor blocked counting
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000) //Motor blocked over 150times generate ErrOR
    {
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*****************PID Initialize Function*********************/
void PID_Init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,
    uint16_t period,

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    uint8_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset;

    pid->PID_param_init(pid, max_out, intergral_limit, deadband, period,
                        kp, ki, kd, A, B, improve);
}
