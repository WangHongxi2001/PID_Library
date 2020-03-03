/**
  ******************************************************************************
  * @file    pid.c
  * @author  Hongxi Wong
  * @version V1.0.6
  * @date    2019/12/17
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

    float kp,
    float Ki,
    float Kd,

    float Changing_Integral_A,
    float Changing_Integral_B,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**************************PID_param_reset*********************************/
static void f_PID_reset(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    if (pid->Ki == 0)
        pid->Iout = 0;
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
            return 0; //Catch ERROR
        }
    }

    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);

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

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        //Output Filter
        if (pid->Improve & OutputFilter)
            f_OutputFilter(pid);

        //Output limit
        f_Output_limit(pid);

        //Proportional limit
        f_Proportion_limit(pid);
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Err = pid->Err;

    return pid->Output;
}

/*****************PID Improvement Function*********************/
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}

static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        //Integral still increasing
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
        if (pid->Err * pid->Iout > 0)
        {
            //Integral still increasing
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
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}

static void f_OutputFilter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * 0.8f + pid->Last_Output * 0.2f;
}

static void f_Proportion_limit(PID_TypeDef *pid)
{
    //Proportion limit is insignificant for control process
    //but it makes debug chart look better
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

static void f_Output_limit(PID_TypeDef *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.01)
        return;

    if ((ABS(pid->Output - pid->Measure) / pid->Output) > 0.9f)
    {
        //Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*****************PID Initialize Function*********************/
void PID_Init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    uint8_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset;
    pid->PID_param_init(pid, max_out, intergral_limit, deadband,
                        kp, Ki, Kd, A, B, improve);
}
