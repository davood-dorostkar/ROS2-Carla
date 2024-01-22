         

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h" 
#include "TM_CML_Types.h" 
#include "TM_Base_Bayes.h" 
#include "TM_Base_Const.h" 
#include "TM_Base_Emul.h" 
#include "TM_Base_Mat.h" 
#include "TM_Base_Misc.h" 
#include "TM_Base_Trigo.h" 


/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    CML_v_Cartesian2Polar                                */ /*!
  
  @brief           Transforms a complex number from cartesian to polar representation.
  
  @description     Transforms a complex number from cartesian to polar representation.
                   The real and imaginary parts are used to calculate the amplitude 
                   and phase as follows:
                   Let the cartesian cordinates be (x,y) and polar cordinates be (r,ph)
                   then,        
                   r  = sqrt(x*x + y*y)
                   ph = atan(y/x)
  
  @param[in]       cartesian :  complex number cartesian
                                Optimal Range for cartesian.f_Real is [-F_MAX..F_MAX]
                                Optimal Range for cartesian.f_Imag is [-F_MAX..F_MAX]
                                where F_MAX is cube root of max range of float.
  @param[out]      p_polar :  pointer to complex number polar

  @return          void
  


*****************************************************************************/
void CML_v_Cartesian2Polar(const t_Complexf32 cartesian, t_ComplexPolarf32 * const p_polar)
{
  const float32 f_RootArg = BML_Sqr(cartesian.f_Real) + BML_Sqr(cartesian.f_Imag);
  
  if (BML_f_IsNonZero(cartesian.f_Real))
  {
    if (BML_f_IsNonZero(cartesian.f_Imag))
    {
      /* regular calculation, special cases in the case of real or imaginary part = 0 see below */
      p_polar->f_Phase      = ATAN2_(cartesian.f_Imag, cartesian.f_Real);
      p_polar->f_Amplitude  = BML_f_Sqrt(f_RootArg);
    }
    else if (cartesian.f_Real > 0.f)
    {
      /* imag = 0 and real > 0 -> angle = 0 degree*/
      p_polar->f_Phase      = 0.0f;
      p_polar->f_Amplitude  = cartesian.f_Real;
    }
    else
    {
      /* imag = 0 and real < 0 -> angle = 180 degree*/
      p_polar->f_Phase      = BML_f_Pi;
      p_polar->f_Amplitude  = -cartesian.f_Real;
    }
  }
  else if (BML_f_IsZero(cartesian.f_Imag))
  {
    /* real = 0 and imag = 0 -> angle = 0 (convention) */
    p_polar->f_Phase      = 0.0f;
    p_polar->f_Amplitude  = 0.0f;
  }
  else if (cartesian.f_Imag > 0.f)
  {
    /* real = 0 and imag > 0 -> angle = 90 degree*/
    p_polar->f_Phase      = 0.5f*BML_f_Pi;
    p_polar->f_Amplitude  = cartesian.f_Imag;
  }
  else
  {
    /* real = 0 and imag < 0 -> angle = -90 degree*/
    p_polar->f_Phase      = (-0.5f)*BML_f_Pi;
    p_polar->f_Amplitude  = -cartesian.f_Imag;
  }
}

/*****************************************************************************
  Functionname:    CML_v_Polar2Cartesian                                */ /*!
  
  @brief           Transforms a complex number from polar to cartesian representation.
  
  @description     Transforms a complex number from polar to cartesian representation.
                   The amplitude and phase are used to calculate the real and imaginary parts
                   as follows:
                   Let the cartesian cordinates be (x,y) and polar cordinates be (r,ph)
                   then,        
                   x = r * cos(ph)
                   y = r * sin(ph)
  
  @param[in]       polar :  complex number polar
                            Range for polar.f_Amplitude [Full range of float]
                            Range for polar.f_Phase [Full range of float]
  @param[out]      p_cartesian :  complex number cartesian
  
  @return          void
    

*****************************************************************************/
void CML_v_Polar2Cartesian(const t_ComplexPolarf32 polar, 
                           t_Complexf32 * const p_cartesian)
{
  float32 f_Phase = polar.f_Phase,
          f_CosVal,
          f_SinVal;

  /* otherwise COS_ can produce wrong values... */
  while (f_Phase < (-BML_f_Pi))
  {
    f_Phase += (2.f*BML_f_Pi);
  }
  while (f_Phase > BML_f_Pi)
  {
    f_Phase -= (2.f*BML_f_Pi);
  }

  f_CosVal = COS_(f_Phase);
  f_SinVal = SIN_(f_Phase);

  p_cartesian->f_Real = polar.f_Amplitude * f_CosVal;
  p_cartesian->f_Imag = polar.f_Amplitude * f_SinVal;
}

/*************************************************************************

  Functionname:    CML_v_PhaseUnwrapping                            */ /*!
  
  @brief           Does a phase unwrapping for the phases of an array, 
                   can be adapted to array with elevation/monopulse feature 
                   by parameter.

  @description     This function Does a phase unwrapping for the phases of 
                   an array, can be adapted to array with elevation/monopulse 
                   feature by parameter. Here it considers the fact that phase
                   is 2pi periodic. It finds the phase which has the mininmum
                   difference to previous relevent phase.
  
  @param[in, out]  a_values :  complex polar values with phases to be unwrapped 
                            -> complex polar values with unwrapped phases
                               Range for a_values[].f_Amplitude [Full range of float]
                               Range for a_values[].f_Phase [Full range of float]
  @param[in]       u_NofChannels  :  number of the channels
                                     Range can be [Full range of uint32]
  @param[in]       u_NofSubarrays :  number of subarrays, 1u if no elevation 
                                     feature should be considered, so 1u
                                     is the value for default purposes
                                     Range can be [Full range of uint32]

  @return          void
 
*/

/***************************************************************************/
void CML_v_PhaseUnwrapping(t_ComplexPolarf32 a_values[], 
                           const uint32 u_NofChannels,
                           const uint32 u_NofSubarrays)
{
  /* loop counter */
  uint8 u_ChannelCounter;
  
  /* phase of previous channel during the loop over all channels */
  float32 f_PhasePrev;
  
  for (u_ChannelCounter = 1u; u_ChannelCounter < u_NofChannels; u_ChannelCounter++)
  {
    /* phase candidates which are higher and lower than previous relevant phase,
       initialized here to suppress compiler warning, although it is not needed */
    float32 f_PhaseHigh = 0.f;
    float32 f_PhaseLow  = 0.f;
  
    /* phase is 2pi periodic
       => find the phase which has minimum difference to previous relevant phase  */
    if (u_ChannelCounter < u_NofSubarrays)
    {
      /* between this and previous antenna minimize phase difference */
      f_PhasePrev = a_values[u_ChannelCounter-1u].f_Phase;
    }
    else
    {
      /*  for u_NofSubarrays-th antenna to the end, minimize phase 
          difference to antenna u_NofSubarrays before;
          this should provide a more robust phase unwrapping in the case 
          of an elevation feature, where there is a shift at every second phase */
      f_PhasePrev = a_values[u_ChannelCounter-u_NofSubarrays].f_Phase;
    }
  
    if ( f_PhasePrev < a_values[u_ChannelCounter].f_Phase )
    {
      f_PhaseLow = a_values[u_ChannelCounter].f_Phase;
      while (f_PhaseLow > f_PhasePrev)
      {
        f_PhaseHigh = f_PhaseLow;
        f_PhaseLow -= (2.f*BML_f_Pi);
      }
    }
    else
    {
      f_PhaseHigh = a_values[u_ChannelCounter].f_Phase;
      while (f_PhaseHigh < f_PhasePrev)
      {
        f_PhaseLow = f_PhaseHigh;
        f_PhaseHigh += (2.f*BML_f_Pi);
      }
    }
  
    if ( (f_PhaseHigh-f_PhasePrev) < (f_PhasePrev-f_PhaseLow) )
    {
      a_values[u_ChannelCounter].f_Phase = f_PhaseHigh;
    }
    else
    {
      a_values[u_ChannelCounter].f_Phase = f_PhaseLow;
    }
  }
}

/*****************************************************************************
  Functionname :    BML_v_MultiplyComplex                               */ /*!
  
  @brief            Multiplies two complex numbers
  
  @description      Multiplies two complex numbers under the condition which 
                    checks if the first number has to be multiplied by the 
                    second number as it is or by the conjugate of the second 
                    number.
  
  @param[in]        p_Fac1 :  Factor1 
                              Optimal Range for p_Fac1.f_Real is [-F_MAX..F_MAX]
                              Optimal Range for p_Fac1.f_Imag is [-F_MAX..F_MAX]
                              where F_MAX is square root of half the max range of float.
  @param[in]        p_Fac2 :  Factor2
                              Optimal Range for p_Fac2.f_Real is [-F_MAX..F_MAX]
                              Optimal Range for p_Fac2.f_Imag is [-F_MAX..F_MAX]
                              where F_MAX is square root of half the max range of float.
  @param[in]        b_Conj :  Factor2 complex conjugate
                              This value can be either TRUE or FALSE
  @param[out]       p_Prod :  Resulting product
  
  @return           void
  

*****************************************************************************/

void BML_v_MultiplyComplex(const t_Complexf32 *p_Fac1,
                           const t_Complexf32 *p_Fac2,
                           const boolean b_Conj,
                           t_Complexf32 *p_Prod)
{
  const float32 f_RealTmp1 = p_Fac1->f_Real,
                f_RealTmp2 = p_Fac2->f_Real;

  if (b_Conj == TRUE)
  {
    p_Prod->f_Real = (p_Fac1->f_Real*p_Fac2->f_Real) + (p_Fac1->f_Imag*p_Fac2->f_Imag);
    p_Prod->f_Imag = (p_Fac1->f_Imag*f_RealTmp2) - (f_RealTmp1*p_Fac2->f_Imag);
  }
  else
  {
    p_Prod->f_Real = (p_Fac1->f_Real*p_Fac2->f_Real) - (p_Fac1->f_Imag*p_Fac2->f_Imag);
    p_Prod->f_Imag = (p_Fac1->f_Imag*f_RealTmp2) + (f_RealTmp1*p_Fac2->f_Imag);
  }
}


/*****************************************************************************
  Functionname:    BML_v_DivideComplex                                  */ /*!
  
  @brief           complex division
  
  @description     This function divides a complex numerator by a complex 
                   denominator.
  @attention       A division by 0 has to be avoided by user!
  
  @param[in]       p_Num :  Numerator
                            Optimal Range for p_Num.f_Real is [-F_MAX..F_MAX]
                            Optimal Range for p_Num.f_Imag is [-F_MAX..F_MAX]
                            where F_MAX is square root of half the max range of float.
  @param[in]       p_Denom :  Denominator
                              Optimal Range for p_Denom.f_Real is [-F_MAX..F_MAX]
                              Optimal Range for p_Denom.f_Imag is [-F_MAX..F_MAX]
                              where F_MAX is square root of half the max range of float.
  @param[out]      p_Quot :  Resulting quotient
  
  @return          void
  


*****************************************************************************/

void BML_v_DivideComplex(const t_Complexf32 *p_Num, 
                         const t_Complexf32 *p_Denom,
                         t_Complexf32 *p_Quot)
{
  float32 f_Mag2;
  f_Mag2 = (p_Denom->f_Real*p_Denom->f_Real) + (p_Denom->f_Imag*p_Denom->f_Imag);

  p_Quot->f_Real = (p_Num->f_Real*p_Denom->f_Real) + (p_Num->f_Imag*p_Denom->f_Imag);
  p_Quot->f_Real /= f_Mag2;

  p_Quot->f_Imag = (p_Num->f_Imag*p_Denom->f_Real) - (p_Num->f_Real*p_Denom->f_Imag);
  p_Quot->f_Imag /= f_Mag2;
}


/*****************************************************************************
  Functionname:    BML_v_QuadraticEquationComplex                       */ /*!
  
  @brief           calculate the two solutions of z*z + p*z + q = 0
                   according to z1 = 0.5*(-p + sqrt(p*p - 4*q))
                            z2 = 0.5*(-p - sqrt(p*p - 4*q))
               
  @description     The function calculates the solution for the quadratic equation
                   z*z + p*z + q = 0. 
                   The two solutions to the quadratic equation is given by,
                   z1 = 0.5*(-p + sqrt(p*p - 4*q))
                   z2 = 0.5*(-p - sqrt(p*p - 4*q))
  
  @param[in]   p_p :  Complex polynomial coefficient
                          Optimal Range for p_p.f_Real is [-F_MAX..F_MAX]
                          Optimal Range for p_p.f_Imag is [-F_MAX..F_MAX]
                          where F_MAX is square root of half the max range of float.
  @param[in]   p_q :  Complex polynomial coefficient
                          Optimal Range for p_q.f_Real is [-F_MAX..F_MAX]
                          Optimal Range for p_q.f_Imag is [-F_MAX..F_MAX]
                          where F_MAX is one-eighth the max range of float.
                          Note: The value of (p*p - 4*q) for both real and imaginary part 
                          should be greater than or equal to 0
  @param[out]   p_z1 :  Complex solution 1
  @param[out]   p_z2 :  Complex solution 2
  
  @return          void
  


*****************************************************************************/

void BML_v_QuadraticEquationComplex(const t_Complexf32 *p_p, 
                                    const t_Complexf32 *p_q,
                                    t_Complexf32 *p_z1,
                                    t_Complexf32 *p_z2)
{
  t_Complexf32 t_Tmp;
  t_ComplexPolarf32 t_Root;

  BML_v_MultiplyComplex(p_p, p_p, 0u, &t_Tmp);

  t_Tmp.f_Real -= 4.0f*(p_q->f_Real); /* real(p*p - 4*q) */
  t_Tmp.f_Imag -= 4.0f*(p_q->f_Imag); /* imag(p*p - 4*q) */

  /* complex root calculation */
  CML_v_Cartesian2Polar(t_Tmp, &t_Root);

  t_Root.f_Amplitude = BML_f_Sqrt(t_Root.f_Amplitude);
  t_Root.f_Phase *= 0.5f;

  CML_v_Polar2Cartesian(t_Root, &t_Tmp);
  
  p_z1->f_Real = 0.5f*(-p_p->f_Real + t_Tmp.f_Real);
  p_z1->f_Imag = 0.5f*(-p_p->f_Imag + t_Tmp.f_Imag);
  p_z2->f_Real = 0.5f*(-p_p->f_Real - t_Tmp.f_Real);
  p_z2->f_Imag = 0.5f*(-p_p->f_Imag - t_Tmp.f_Imag);
}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"