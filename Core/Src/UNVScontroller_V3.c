/**
  ******************************************************************************
  * @file    UNVScontroller_V3.c
  * @brief   .
  ******************************************************************************
  * desciption
  *
  * update: 1 October 2021
  * change: 1.Calculate error dot by differential of 3 point.
  *         2.Calculate every integral by simpson's 1/3 rule 
  *           with time interval of [h,2h] instead of [0,2h].
  *         3.Include deadzone of current point.
  *         4.Add more parameters in Controller structure.
  *
  *
  ******************************************************************************
*/


#include "UNVScontroller_V3.h"
    
void InitCtrl(CTRL_TypeDef *ctrl, float dt, float gain[], float dzone, float intLimit, float uLimit, float velLimit){
      
  ctrl->Ka = gain[0];
  ctrl->Kr = gain[1];
  ctrl->Kb = gain[2];
  
  ctrl->Ir_dzone    = dzone;
  ctrl->Isgnr_dzone = dzone;
  ctrl->u_dzone     = dzone;
  
  ctrl->Ir_lower    = -intLimit;
  ctrl->Ir_upper    =  intLimit;
  ctrl->Isgnr_lower = -intLimit;
  ctrl->Isgnr_upper =  intLimit;
  ctrl->u_lower     = -uLimit;
  ctrl->u_upper     =  uLimit;
            
  ctrl->tStep = dt;
  ctrl->velLimit=velLimit;
}

void PosCtrl(CTRL_TypeDef *ctrl, float *u)
{
  float signr;
  
  //find edot
  ctrl->err_dot = Diff(ctrl->err, ctrl->errPrev, ctrl->tStep);

  //find r term
  ctrl->r = (ctrl->err_dot + ctrl->Ka*ctrl->err);
  ctrl->r = ctrl->Kr * ctrl->r;
    
  //find integral of r
  ctrl->Int_r += Integrate(ctrl->r, ctrl->rPrev, ctrl->tStep, ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone);
  ctrl->Int_r =  SatWithDz(ctrl->Int_r,ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone); 
  
  //find interal of signum of r
  signr = signum(ctrl->r);
  ctrl->Int_signr +=  Integrate(signr, ctrl->signrPrev, ctrl->tStep, ctrl->Isgnr_upper, ctrl->Isgnr_lower, ctrl->Isgnr_dzone);
  ctrl->Int_signr = SatWithDz(ctrl->Int_signr,ctrl->Isgnr_upper, ctrl->Isgnr_lower, ctrl->Isgnr_dzone);  
  
  //Calculate output of controller    
  *u = ctrl->r + ctrl->Kr *ctrl->Int_r + ctrl->Kb *ctrl->Int_signr;
  *u = SatWithDz(*u,ctrl->u_upper, ctrl->u_lower, ctrl->u_dzone); 
  
}

void PosCtrlPID(CTRL_TypeDef *ctrl, float *u)
{
  
  //find edot
  ctrl->err_dot = Diff(ctrl->err, ctrl->errPrev, ctrl->tStep);

  //PID
  ctrl->r = ctrl->Ka*ctrl->err; //P control
  ctrl->Int_signr = ctrl->Kb * ctrl->err_dot; //D control
    
  //I control
  ctrl->Int_r +=  Integrate(ctrl->r, ctrl->rPrev, ctrl->tStep, ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone);
  ctrl->Int_r = SatWithDz(ctrl->Int_r,ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone); 
  
  //Calculate output of controller      
  *u = ctrl->r + ctrl->Kr * ctrl->Int_r + ctrl->Int_signr;
  *u = SatWithDz(*u,ctrl->u_upper, ctrl->u_lower, ctrl->u_dzone); 
  
}


void PosCtrl1stOrder(CTRL_TypeDef *ctrl, float *u)
{
  //controller for 1st order system
  float signr;
  
  //find r term
  ctrl->r = ctrl->Ka*ctrl->err;
  //ctrl->r = ctrl->Kr*ctrl->r;
    
  //find integral of r
  ctrl->Int_r += Integrate(ctrl->r, ctrl->rPrev, ctrl->tStep, ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone);
  ctrl->Int_r =  SatWithDz(ctrl->Int_r,ctrl->Ir_upper, ctrl->Ir_lower, ctrl->Ir_dzone); 
  
  //find interal of signum of r
  signr = signum(ctrl->r);
  ctrl->Int_signr +=  Integrate(signr, ctrl->signrPrev, ctrl->tStep, ctrl->Isgnr_upper, ctrl->Isgnr_lower, ctrl->Isgnr_dzone);
  ctrl->Int_signr = SatWithDz(ctrl->Int_signr,ctrl->Isgnr_upper, ctrl->Isgnr_lower, ctrl->Isgnr_dzone);  
  
  //Calculate output of controller      
  *u = ctrl->r + ctrl->Kr *ctrl->Int_r + ctrl->Kb *ctrl->Int_signr;
  *u = SatWithDz(*u,ctrl->u_upper, ctrl->u_lower, ctrl->u_dzone); 
  
}

void CtrlReset(CTRL_TypeDef *ctrl)
{
  ctrl->ref = 0.0f;
  ctrl->err = 0.0f;
  ctrl->err_dot = 0.0f;
  
  ctrl->r = 0.0f;
  ctrl->rPrev[0] = 0.0f;
  ctrl->rPrev[1] = 0.0f;
  ctrl->signrPrev[0] = 0.0f;
  ctrl->signrPrev[1] = 0.0f;
  ctrl->errPrev[0] = 0.0f;
  ctrl->errPrev[1] = 0.0f;
  
  ctrl->Int_r = 0.0f;
  ctrl->Int_signr = 0.0f;
  
  ctrl->xPrev=0.0f;
  ctrl->fcut=1.0f;
}

float Diff(float curr, float buff[], float dt)
{
  
  float diffErr;
  
  //find differential for 3 point using first order
  diffErr = (3.0f*curr - 4.0f*buff[0] + buff[1])/2.0f/dt;    
  buff[1]=buff[0];
  buff[0]=curr;
  
  return diffErr;
  
  //  ctrl->err_dot = (3.0f*ctrl->err - 4.0f*ctrl->errPrev[0] + ctrl->errPrev[1])/2/ctrl->tStep;    
  //  ctrl->errPrev[1]=ctrl->errPrev[0];
  //  ctrl->errPrev[0]=ctrl->err;
}

float Integrate(float curr, float buff[], float dt, float UL, float LL, float dZone)
{
  
  float Int;
  
  curr = SatWithDz(curr, INFINITY, -INFINITY, dZone);
  
  //simpson's 1/3 rule   [interval h->2h]
  Int = dt/12.0f *(5.0f*curr + 8.0f*buff[0] - buff[0]);
  buff[1] = buff[0];
  buff[0] = curr;
  
  Int = SatWithDz(Int, UL, LL, 0.0);
  return Int;
  
  //  ctrl->Int_r += ctrl->Kr * ctrl->tStep/12.0f *(5.0f*ctrl->r + 8.0f*ctrl->rPrev[0] - ctrl->rPrev[1]);
  //  ctrl->rPrev[1] = ctrl->rPrev[0];
  //  ctrl->rPrev[0] = ctrl->r;

}

float SatWithDz(float value, float UL, float LL, float dZone)
{
  
  float output;
  if(value<dZone && value>(-dZone)){
    output = 0.0f;  
  }else if(UL>0 && value>UL){
    output = UL;
  }else if(LL<0 && value<LL){
    output = LL;
  }else{
    output = value;
  }
           
  return output;
}

float Dz(float value, float dZone)
{
  float output;
  if(value<dZone && value>(-dZone)){
    output = 0.0f;  
  }
           
  return output;
}

float limitMotorSpeed(float u,float velLimit,float currVel){
// do not use this function. It is not working 
  float uLim, absCurrVel;
  uLim=u;
  absCurrVel=fabs(currVel);
  if (absCurrVel >= 2*velLimit){
    uLim = 0;   
  }  
  else if(absCurrVel > velLimit){
    uLim = u*(1.0f-(absCurrVel-velLimit)/velLimit);
  }
  return uLim;
}

float lowPassFilter(CTRL_TypeDef *ctrl,float xCurr){
//fcut=normalized cut off freq in (0,1)
  float xLowPass=xCurr;
  float xPrev=ctrl->xPrev;
  float fcut=ctrl->fcut;
  if (fcut>=0.0f && fcut <=1.0f){
    xLowPass=(1.0f-fcut)*xPrev + fcut*xCurr;
    ctrl->xPrev=xCurr;
  }
 return xLowPass;
}
