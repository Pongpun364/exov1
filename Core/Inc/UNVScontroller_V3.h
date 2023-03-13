#pragma once

/**
  ******************************************************************************
  * @file    UNVScontroller_V3.c
  * @brief   .
  ******************************************************************************
  * desciption
  *
  * update: 1 October 2021
  * 
  ******************************************************************************
  */

#include "main.h"
#include "math.h"
#include "stdint.h"
    
#define signum(x) ((x > 0) - (x < 0))

typedef struct {
  //<> initial variable <>//
  float Ka,Kr,Kb;
  
  //<> Public variable <>//
  float ref;
  float err;     //@error = reference - feedback
  float err_dot; //@error_dot = differential of error
  
  //<> Private variable <>//
  float r;
  float Int_r;
  float Int_signr;
  float rPrev[2];
  float signrPrev[2];
  float errPrev[2];
  float tStep;

  //<> limit variable <>//
  float Ir_lower;
  float Ir_upper;
  float Isgnr_lower;
  float Isgnr_upper;
  float u_lower;
  float u_upper;
  
  //<> deadzone variable <>//
  float Ir_dzone;
  float Isgnr_dzone;
  float u_dzone;
  //<> velocity limit <>//
  float velLimit;
  //<> filter <>//
  float xPrev;
  float fcut; // normailized cut off frequency [0,1] 

}CTRL_TypeDef;
  
void InitCtrl(CTRL_TypeDef *ctrl, float dt, float gain[], float dzone, float intLimit, float uLimit, float velLimit);
void PosCtrl(CTRL_TypeDef *ctrl, float *u);
void PosCtrlPID(CTRL_TypeDef *ctrl, float *u);
void PosCtrl1stOrder(CTRL_TypeDef *ctrl, float *u);
void CtrlReset(CTRL_TypeDef *ctrl);
float Diff(float curr, float buff[], float dt);
float Integrate(float curr, float buff[], float dt, float UL, float LL, float dZone);
float SatWithDz(float value, float UL, float LL, float dZone);
float Dz(float value, float dZone);
float limitMotorSpeed(float u,float velLimit,float currVel); // do not use this function. it is not working
float lowPassFilter(CTRL_TypeDef *ctrl,float xCurr);
