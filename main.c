/**
  ******************************************************************************
  * @file    main.c
  * @author  Hongxi Wong
  * @version V1.0.3
  * @date    2019/12/17
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include "PID.h"

PID_TypeDef PID_Example;
float measure = 0;
float target = 100.0;

int main()
{
    //pid函数连接
    PID_Init(&PID_Example, 9600, 5000, 3, 1, 5, 0.3, 0.3, 100, 100,0.02, ErrorHandle | Integral_Limit | OutputFilter);

    //修改kp ki kd
    PID_Example.PID_reset(&PID_Example, 3, 1, 0);

    //计算 measure为被控对象测量值
    PID_Calculate(&PID_Example, measure, target);

    system("pause");
    return 0;
}