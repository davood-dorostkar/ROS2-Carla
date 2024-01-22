#ifndef CML_TYPE_H
#define CML_TYPE_H

// typedef float32 fAccel_t                  	    ;
// typedef float32 fAngle_t                  	    ;
// typedef float32 fDistance_t               	    ;
// typedef float32 fTime_t                   	    ;
// typedef float32 fVariance_t               	    ;
// typedef float32 fVelocity_t               	    ;
// typedef float32 fYawRate_t                	    ;
// typedef uint8 percentage_t              	    ;

#include "TM_Global_TypeDefs.h"
typedef struct
{
	float fXDist;
	float fYDist;
} Vector2_f32_t;

typedef struct
{
	signed short nXDist;
	signed short nYDist;
} Vector2_i16_t;

typedef struct
{
	signed long nXDist;
	signed long nYDist;
} Vector2_i32_t;

typedef struct
{
	signed char nXDist;
	signed char nYDist;
} Vector2_i8_t;

typedef struct
{
	signed short s_Real;
	signed short s_Imag;
} t_Complex16;

typedef struct
{
	signed char s_Real;
	signed char s_Imag;
} t_Complex8;

typedef struct
{
	float f_Amplitude;
	float f_Phase;
} t_ComplexPolarf32;

typedef struct
{
	float f_Real;
	float f_Imag;
} t_Complexf32;


#endif

