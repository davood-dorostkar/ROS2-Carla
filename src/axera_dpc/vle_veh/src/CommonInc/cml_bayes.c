         

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h" 
#include "TM_CML_Types.h" 
#include "TM_Base_Bayes.h" 
#include "TM_Base_Const.h" 



/*****************************************************************************
  Functionname:    CML_Bayes2                                           */     
percentage_t CML_Bayes2(percentage_t u_ProbabilityA,
                        percentage_t u_ProbabilityB,
                        const percentage_t a_CPT[CML_BAYES2_CPT_SIZE])
{
  uint32 u_temp;
  uint32 u_p2;
  percentage_t u_ProbabilityNotA, u_ProbabilityNotB;

  /* range check: probability must not be greater than 100% */
  if(u_ProbabilityA > Percentage_max)
  {
    u_ProbabilityA = Percentage_max;
  }
  if(u_ProbabilityB > Percentage_max)
  {
    u_ProbabilityB = Percentage_max;
  }

  u_ProbabilityNotA = Percentage_max - u_ProbabilityA;
  u_ProbabilityNotB = Percentage_max - u_ProbabilityB;

  /* Calculate the probability P(C) by the formula of total probability. */
  /* The events A and B are assumed independent, so P(A, B) = P(A) * P(B) */
  u_temp  =   (uint32) (u_ProbabilityNotA) * (uint32) (u_ProbabilityNotB) * (uint32) a_CPT[0];
  u_temp += ( (uint32) (u_ProbabilityA   ) * (uint32) (u_ProbabilityNotB) * (uint32) a_CPT[1] );
  u_temp += ( (uint32) (u_ProbabilityNotA) * (uint32) (u_ProbabilityB   ) * (uint32) a_CPT[2] );
  u_temp += ( (uint32) (u_ProbabilityA   ) * (uint32) (u_ProbabilityB   ) * (uint32) a_CPT[3] );

  /* fixed point scaling */
  u_p2 = Percentage_max * Percentage_max;
  u_temp += ( u_p2/2u );
  u_temp /= u_p2;
  if(u_temp > Percentage_max)
  {
    u_temp = Percentage_max;
  }

  return (percentage_t)u_temp;
}


/*****************************************************************************
  Functionname:    BML_Bayes3                                           */     
percentage_t BML_Bayes3(percentage_t u_ProbabilityA,
                        percentage_t u_ProbabilityB,
                        percentage_t u_ProbabilityC,
                        const percentage_t a_CPT[BML_BAYES3_CPT_SIZE])
{
  uint32 u_temp;
  percentage_t u_ProbabilityNotC;

  /* range check: probability must not be greater than 100% */
  if(u_ProbabilityC > Percentage_max)
  {
    u_ProbabilityC = Percentage_max;
  }

  u_ProbabilityNotC = Percentage_max - u_ProbabilityC;

  /* Calculate the probability P(D) by the formula of total probability. */
  /* The events A, B, and C are assumed independent, so P(A, B, C) = P(A) * P(B) * P(C) */
  u_temp  = (uint32) CML_Bayes2(u_ProbabilityA,
                                u_ProbabilityB,
                                a_CPT) * (uint32) (u_ProbabilityNotC);
  
  u_temp += (uint32) CML_Bayes2(u_ProbabilityA,
                                u_ProbabilityB,
                                &a_CPT[CML_BAYES2_CPT_SIZE]) * (uint32) (u_ProbabilityC);

  /* fixed point scaling */
  u_temp = (u_temp + (Percentage_max / 2u)) / Percentage_max;
  if(u_temp > Percentage_max)
  {
    u_temp = Percentage_max;
  }

  return (percentage_t)u_temp;
}
 

/*****************************************************************************
  Functionname:    CML_Bayes4                                           */     
percentage_t CML_Bayes4(percentage_t u_ProbabilityA,
                        percentage_t u_ProbabilityB,
                        percentage_t u_ProbabilityC,
                        percentage_t u_ProbabilityD,
                        const percentage_t a_CPT[BML_BAYES4_CPT_SIZE])
{
  uint32 u_temp;
  percentage_t u_ProbabilityNotD;

  /* range check: probability must not be greater than 100% */
  if(u_ProbabilityD > Percentage_max)
  {
    u_ProbabilityD = Percentage_max;
  }

  u_ProbabilityNotD = Percentage_max - u_ProbabilityD;

  /* Calculate the probability P(D) by the formula of total probability. */
  /* The events A, B, C, and D are assumed independent, so P(A, B, C, D) = P(A) * P(B) * P(C) * P(D) */
  u_temp  = (uint32) BML_Bayes3(u_ProbabilityA, 
                                u_ProbabilityB,
                                u_ProbabilityC,
                                a_CPT) * (uint32) (u_ProbabilityNotD);
  
  u_temp += (uint32) BML_Bayes3(u_ProbabilityA,
                                u_ProbabilityB,
                                u_ProbabilityC,
                                &a_CPT[BML_BAYES3_CPT_SIZE]) * (uint32) (u_ProbabilityD);

  /*fixed point scaling*/
  u_temp = (u_temp + (Percentage_max / 2u)) / Percentage_max;
  if(u_temp > Percentage_max)
  {
    u_temp = Percentage_max;
  }

  return (percentage_t)u_temp;
}


/*****************************************************************************
  Functionname:    CML_Bayes5                                           */     
percentage_t CML_Bayes5(percentage_t u_ProbabilityA,
                        percentage_t u_ProbabilityB,
                        percentage_t u_ProbabilityC,
                        percentage_t u_ProbabilityD,
                        percentage_t u_ProbabilityE,
                        const percentage_t a_CPT[BML_BAYES5_CPT_SIZE])
{
  uint32 u_temp;
  percentage_t u_ProbabilityNotE;

  /* range check: probability must not be greater than 100% */
  if(u_ProbabilityE > Percentage_max)
  {
    u_ProbabilityE = Percentage_max;
  }

  u_ProbabilityNotE = Percentage_max - u_ProbabilityE;

  /* Calculate the probability P(E) by the formula of total probability. */
  /* The events A, B, C, D, and E are assumed independent, so P(A, B, C, D, E) = P(A) * P(B) * P(C) * P(D) * P(E) */
  u_temp  = (uint32) CML_Bayes4(u_ProbabilityA,
                                u_ProbabilityB,
                                u_ProbabilityC,
                                u_ProbabilityD,
                                a_CPT) * (uint32) (u_ProbabilityNotE);
  
  u_temp += (uint32) CML_Bayes4(u_ProbabilityA,
                                u_ProbabilityB,
                                u_ProbabilityC,
                                u_ProbabilityD,
                                &a_CPT[BML_BAYES4_CPT_SIZE]) * (uint32) (u_ProbabilityE);

  /* fixed point scaling */
  u_temp = (u_temp + (Percentage_max / 2u)) / Percentage_max;
  if(u_temp > Percentage_max)
  {
    u_temp = Percentage_max;
  }

  return (percentage_t)u_temp;
}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"