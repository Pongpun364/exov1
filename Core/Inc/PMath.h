#ifndef PMATH_H
#define PMATH_H

#ifdef __cplusplus
extern "C"
{
#endif

  
  
#include "main.h"
#include "math.h"
#include "stdint.h"
  
  
  
#ifndef PI
  #define PI   3.14159265358979323846264338327950 
#endif
  
//>>ternary operator<<//
#define min(X, Y)  ( (X<Y)?(X) : (Y) )
#define max(X, Y)  ( (X>Y)?(X) : (Y) )
#define saturationlimit(x,lower,upper) ((x<lower)?(lower) : (x>upper)?(upper) : (x))
#define deadzone(u,LL,UL) ((u<LL)?(u-LL) : (u>UL)?(u-UL) : 0)
#define deadband(u,LL,UL) ((u<LL)?(u) : (u>UL)?(u) : 0) 
#define sign(x)   ((x>0)?(1) : (x<0)?(-1) : 0)
#define signum(x) ((x > 0) - (x < 0))
//#define sign(x)   (x != 0 ? x/abs(x) : 0)

//>>Unit Conversion<<//
#define deg2rad(x)  (0.01745329251994329547 * (x))
#define rad2deg(x)  (57.29577951308232286465 * (x))

#define wrapTo180(lambda) ((lambda+180.0) - 360.0*floor((lambda+180.0)/360.0) - 180.0)
#define wrapTo360(lambda) (lambda - 360.0*floor(lambda/(360.0))

//#define wraoTo2PI(lambda) (lambda - (IIPI)*floor(lambda/(IIPI)))
#define wrapToPI(lambda) ((lambda+PI) - (2.0*PI)*floor((lambda+PI)/(2.0*PI)) - PI)
#define wrapTo2PI(lambda) (lambda - (2.0*PI)*floor(lambda/(2.0*PI)))

  
  
  
#define swapbyte16(rgt) ( ((rgt & 0xFF00U) >> 8) | ((rgt & 0x00FFU) << 8) )
#define swapbyte32(rgt) ( ((rgt & 0xFF000000U) >> 24) | ((rgt & 0x00FF0000U) >> 8) | ((rgt & 0x0000FF00U) << 8) | ((rgt & 0x000000FFU) << 24 ) )


  
  typedef struct{
    float y[2];
    float sum_I;
  }Typedef_Trapezoidal;

void Integration_Trapezoidal(Typedef_Trapezoidal *I, float f_x, float h);


  typedef struct{
    float a0,a1,a2,a3,a4,a5,a6,a7,a8;
    float b1,b2,b3,b4,b5,b6,b7,b8;
  }Typedef_Fourier;
float Eq_fourier8(Typedef_Fourier *coeff, float ang_rad);

//void Integration_Simpson13(float *sum_I, float f_x, float h);
//void Integration_Simpson38(float *sum_I, float f_x, float h);


#ifdef __cplusplus
}
#endif

#endif /*PMATH_H*/
