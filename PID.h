/**
  ******************************************************************************
  * @file	 pid.h
  * @author  Hongxi Wong
  * @version V1.0.5
  * @date    2019/12/19
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef _PID_H
#define _PID_H

#include "stdint.h"

#define ABS(x) ((x > 0) ? x : -x)

typedef enum PID_Improvement
{
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegralRate = 0x20,        //0010 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef enum
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef
{
    float Target;
    float LastNoneZeroTarget;
    float Kp;
    float ki;
    float kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float MaxErr;
    float ScalarA; //For Changing Integral
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        uint16_t maxOut,
        uint16_t integralLimit,
        float deadband,
        uint16_t controlPeriod,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);

    float (*PID_Calc)(
        struct _PID_TypeDef *pid,
        float Measure);
} PID_TypeDef;

static void f_Proportion_limit(PID_TypeDef *pid);
static void f_Trapezoid_Intergral(PID_TypeDef *pid);
static void f_Integral_Limit(PID_TypeDef *pid);
static void f_Derivative_On_Measurement(PID_TypeDef *pid);
static void f_Changing_Integral_Rate(PID_TypeDef *pid);
static void f_OutputFilter(PID_TypeDef *pid);
static void f_PID_ErrorHandle(PID_TypeDef *pid);

void PID_Init(PID_TypeDef *pid);

#endif
