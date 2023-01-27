#include "PMath.h"


//void Integration_Simpson13(float *sum_I, float f_x, float h)
//{
//  static float y[3];
//  y[2] = y[1];
//  y[1] = y[0];
//  y[0] = f_x;
//  
//  *sum_I += 0.3333333333f*h * (y[0] + 4.0f*y[1] + y[2]);
//}

//void Integration_Simpson38(float *sum_I, float f_x, float h)
//{
//  static float y[4];
//  y[3] = y[2];
//  y[2] = y[1];
//  y[1] = y[0];
//  y[0] = f_x;
//  
//  *sum_I += 0.375f*h * (y[0] + 3.0f*y[1] + 3.0f*y[2] + y[3]);
//}


void Integration_Trapezoidal(Typedef_Trapezoidal *I, float f_x, float h)
{
  I->y[1] = I->y[0];
  I->y[0] = f_x;
  
  I->sum_I += 0.5f*h * (I->y[0] + I->y[1]);
}


//void cart2pol_2D(float *vect_xy , float *ouput_rt)
//{
//  float x,y,rho theta;
//  x = vect_xy[0];
//  y = vect_xy[1];
//  rho = sqrt(x*x + y*y);
//  theta = atan2(y,x);
//}
//void pol2cart_2D(float *vect_rt , float *ouput_xy)
//{
//  float x,y,rho theta;
//  x = rho*cos(theta);
//  y = rho*sin(theta);
//  rho = sqrt(x*x + y*y);
//  theta = atan2(y,x);
//}




float Eq_fourier8(Typedef_Fourier *coeff, float ang_rad)
{
  //Fourier Series
  
  float value = coeff->a0 + coeff->a1*cos(ang_rad) + coeff->b1*sin(ang_rad) + \
                            coeff->a2*cos(2.0*ang_rad) + coeff->b2*sin(2.0*ang_rad) + \
                            coeff->a3*cos(3.0*ang_rad) + coeff->b3*sin(3.0*ang_rad) + \
                            coeff->a4*cos(4.0*ang_rad) + coeff->b4*sin(4.0*ang_rad) + \
                            coeff->a5*cos(5.0*ang_rad) + coeff->b5*sin(5.0*ang_rad) + \
                            coeff->a6*cos(6.0*ang_rad) + coeff->b6*sin(6.0*ang_rad) + \
                            coeff->a7*cos(7.0*ang_rad) + coeff->b7*sin(7.0*ang_rad) + \
                            coeff->a8*cos(8.0*ang_rad) + coeff->b8*sin(8.0*ang_rad);
  return value;
}










