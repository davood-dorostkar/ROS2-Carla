         

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h" 
#include "TM_Global_defs.h"
#include "TM_CML_Types.h" 
#include "TM_Base_Cfg.h" 
#include "TM_Base_Const.h" 
#include "TM_Base_Emul.h" 
#include "TM_Base_Mapping.h" 
#include "TM_Base_Mat.h" 
#include "TM_Base_Misc.h" 
#include "TM_Base_Stat.h" 

/*****************************************************************************
  LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/
#define SQRT_OF_2 (1.414213562373095f) /* square root of 2 */

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    BML_v_InitWeightedSample                             */ /*!

  @brief           Initialisation of the weighted sample structure
                   
  @description     This function initializes all elements of the weighted
                   sample structure to zero.
                   
  @param[in]       p_sample : structure to be initialized
                              [Valid pointer to the structure]

  @return          void
                   


*****************************************************************************/
void BML_v_InitWeightedSample(BML_t_WeightedSample *p_sample)
{
  p_sample->dSumme         = 0.0F;
  p_sample->dQuadSumme     = 0.0F;
  p_sample->f_sumOfWeights = 0.0F;
  p_sample->dMittel        = 0.0F;
  p_sample->dStdAbw        = 0.0F;
}

/*****************************************************************************
  Functionname:    BML_v_AddSamplePoint                                 */ /*!

  @brief           Add a sample point to the weighted sample structure
                   
  @description     This function adds a sample point to the weighted sample 
                   structure

  @param[in,out]   p_sample : sample structure to which the new point is added
                              Supported values for p_sample.dSumme [Full range of float]
                              Supported values for p_sample.dQuadSumme [Full range of float]
                              Supported values for p_sample.f_sumOfWeights [Full range of float]
  @param[in]       f_value :  value of the new sample point
                              [Full range of float]
  @param[in]       f_weight : weight of the new sample point, must not be negative
                              [Full range of float]
                              Overflow may occur when one or more input values 
                              are at the defined range extremities.

  @return          none
                   


*****************************************************************************/
void BML_v_AddSamplePoint(BML_t_WeightedSample *p_sample, float32 f_value, float32 f_weight)
{
  if (f_weight >= 0.f)
  {
    p_sample->dSumme         += f_value * f_weight;
    p_sample->dQuadSumme     += SQR(f_value) * f_weight;
    p_sample->f_sumOfWeights += f_weight;
  }
}

/*****************************************************************************
  Functionname:    BML_v_MultiplySampleWithFactor                       */ /*!
  
  @brief           Multiplies the sample with a factor
                   
  @description     This function multiplies the weighted structure sample 
                   with a given factor
                   
  @param[in]       p_sample : sample structure
                              Supported values for p_sample.dSumme [Full range of float]
                              Supported values for p_sample.dQuadSumme [Full range of float]
                              Supported values for p_sample.f_sumOfWeights [Full range of float]
  @param[in]       f_factor : the factor to multiply the values and weights with, 
                              must NOT be negative
                              [Full positive range of float]

  @return          none
                   


*****************************************************************************/
void BML_v_MultiplySampleWithFactor(BML_t_WeightedSample *p_sample, float32 f_factor)
{
  if (f_factor >= 0.f)
  {
    p_sample->dSumme         *= f_factor;
    p_sample->dQuadSumme     *= f_factor;
    p_sample->f_sumOfWeights *= f_factor;
  }
}

/*****************************************************************************
  Functionname:    BML_v_ComputEnvmeanStd                                 */ /*!

  @brief           Compute mean and standard deviation of the sample
                   
  @description     This function computes mean and standard deviation of 
                   the sample.
                   If S is the sum of weighted samples x1, x2, .., xn and N be  
                   the sum of weights of these samples, then mean,
                   M = S/N,
                   Now let QS be the sum of squares of weighted samples, then 
                   standard deviation,
                   SD = sqrt((QS/N) - (M))

  @param[in,out]   p_sample : structure for which mean and std should be 
                              computed
                              Supported values for p_sample.dSumme [Full range of float]
                              Supported values for p_sample.dQuadSumme [Full range of float]
                              Supported values for p_sample.f_sumOfWeights [Full range of float]

  @return          none
                   

*****************************************************************************/
void BML_v_ComputEnvmeanStd(BML_t_WeightedSample *p_sample)
{
  float32 f_dVarianz;

  if (p_sample->f_sumOfWeights > 0.0F)
  {
    p_sample->dMittel = p_sample->dSumme / p_sample->f_sumOfWeights;

    f_dVarianz = ( p_sample->dQuadSumme / p_sample->f_sumOfWeights ) - SQR( p_sample->dMittel );

    if (f_dVarianz > 0.F)
    {
      p_sample->dStdAbw = BML_f_Sqrt( f_dVarianz );
    }
    else
    {
      p_sample->dStdAbw = 0.0F;
    }
  }
  else
  {
    p_sample->dStdAbw    = 0.0F;
  }
}

/*****************************************************************************
  Functionname:    BML_f_CalcGaussErrorFunction                         */ /*!

  @brief           Calculate the Gauss Error Function
                   
  @description     This function calculate the Gauss Error Function
                   Aproximate with a 4th order polynomial, the return value
                   G = ( ( ( (C4 * x^4) - (C3 * x^3) ) - (C2 * x^2) ) + (C1 * x) ) + C0,
                   where the coefficients C0, C1, C2, C3 and C4 are predefined 
                   values.
                   
  @param[in]       f_value : input to the Gauss error function
                             Supported values for f_value [-F_MAX...F_MAX]
                             where F_MAX is the fourth root of the maximum value of float32.

  @return          Gauss error function value
                   
                 
*****************************************************************************/
float32 BML_f_CalcGaussErrorFunction  ( float32 f_value )
{
  float32 f_temp2, f_temp3, f_temp, f_temp4;

  if ( f_value >= CML_f_GaussErrFctMaxX )
  {
    f_temp = 1.0f;
  }
  else
  {
    f_temp2 = f_value * f_value;   /* x^2 */
    f_temp3 = f_temp2 * f_value;   /* x^3 */
    f_temp4 = f_temp2 * f_temp2;   /* x^4 */
    f_temp = ( ( ( (CML_f_GaussErrFctConst4 * f_temp4) - (BML_f_GaussErrFctConst3 * f_temp3) ) - (CML_f_GaussErrFctConst2 * f_temp2) ) + (BML_f_GaussErrFctConst1 * f_value) ) + CML_f_GaussErrFctConst0;

    f_temp = BML_f_Min(f_temp, 1.0f);
  }

  return f_temp;
}

/*****************************************************************************
  Functionname:    BML_f_CalcStdGaussianCDF                             */ /*!

  @brief           Calculate the value of the standard Gaussian CDF
                   
  @description     This function calculate the value of the standard Gaussian
                   cumulative distribution function
                   CDF = 0.5 ( 1 + errorfunction ( ( x - aver ) / (sigma * sqrt(2) ) )
                   
  @param[in]       f_value : input to the CDF
                             [Full range of float]
  @param[in]       f_avg   : mean of the Gaussian distribution
                             [Full range of float]
  @param[in]       f_sigma : standard deviation of the Gaussian distribution
                             [Full range of float]

  @return          standard Gaussian CDF at the given value
                   

*****************************************************************************/
float32 BML_f_CalcStdGaussianCDF ( float32 f_value, float32 f_avg, float32 f_sigma )
{
  float32 f_temp;

  if ( BML_f_Abs( f_sigma ) < CML_f_GaussianCDFMinSigma )
  {
    if ( f_value < f_avg )
    {
      f_temp = 0.0f;
    }
    else
    {
      f_temp = 1.0f;
    }
  }
  else
  { 
        
    f_temp =  ( f_value - f_avg ) / ( f_sigma * SQRT_OF_2 );

    /* check for negative values */
    if ( f_temp < 0.0f )
    {
      f_temp = - BML_f_CalcGaussErrorFunction ( -f_temp );
    }
    else
    {
      f_temp = BML_f_CalcGaussErrorFunction ( f_temp );
    }

    f_temp = BML_f_MultAdd(0.5f,f_temp,0.5f);
  }

  return f_temp;
}



/************************************************************************************/
/****************** Functions for linear Least Square Fit (LSF) *********************/


/*****************************************************************************

  Functionname:    BML_v_Init_LSF                                       */ /*!

  @brief           Initialize the LSF storage structure

  @description     This function initializes all the elements of the LSF
                   storage structure to zero 

  @param[in,out]   p_LSF : LSF storage structure
                           [Valid pointer to the structure]

  @return          void


****************************************************************************/
void BML_v_Init_LSF(BML_t_LeastSquareFit *const p_LSF)
{
  p_LSF->iData_Counter  = 0L;
  p_LSF->fsumX          = 0.0f;
  p_LSF->fsumV          = 0.0f;
  p_LSF->fsumXX         = 0.0f;
  p_LSF->fsumXV         = 0.0f;
  p_LSF->fsumVV         = 0.0f;
}

/*****************************************************************************

  Functionname:    CML_v_InitResults_LSF                                */ /*!

  @brief           Initialize the LSF result structure

  @description     This function initializes all the elements of LSF result
                   structure to zero

  @param[in,out]   p_LSFRes : LSF result structure
                              [Valid pointer to the structure]

  @return          void

****************************************************************************/
void CML_v_InitResults_LSF(BML_t_LeastSquareFitResults *const p_LSFRes)
{
  p_LSFRes->fSlope         = 0.0f;
  p_LSFRes->fCorrelation   = 0.0f;
  p_LSFRes->fYIntersection = 0.0f;
  p_LSFRes->fChi2_Error    = 0.0f;
  p_LSFRes->fmse           = 0.0f;
}



/*****************************************************************************

  Functionname:    BML_v_AddData_LSF                                    */ /*!

  @brief           Add Datapair (x, v) to the LSF storage structure

  @description     This function adds a datapair defined by the given 
                   abscissae and ordinate to the LSF storage structure

  @param[in,out]   p_LSF :       LSF storage structure
                                 Supported values for p_LSF->fsumX [Full range of float32]
                                 Supported values for p_LSF->fsumV [Full range of float32]
                                 Supported values for p_LSF->fsumXX [Full range of float32]
                                 Supported values for p_LSF->fsumXV [Full range of float32]
                                 Supported values for p_LSF->fsumVV [Full range of float32]
                                 Supported values for p_LSF->iData_Counter [Full range of sint32]
  @param[in]       f_abscissae : abscissae of the datapair
                                 Supported values for f_value [-F_MAX...F_MAX]
  @param[in]       f_ordinate :  ordinate of the datapair
                                 Supported values for f_value [-F_MAX...F_MAX]
                                 where F_MAX is the square root of the maximum value of float32.
                                 Overflow may occur when one or more input values 
                                 are at the defined range extremities.

  @return          void

  @pre             -
  @post            -

  @todo            avoid overflow!


****************************************************************************/
void BML_v_AddData_LSF(BML_t_LeastSquareFit *const p_LSF, const float32 f_abscissae, const float32 f_ordinate)
{
  p_LSF->fsumX        += f_abscissae;
  p_LSF->fsumV        += f_ordinate;
  p_LSF->fsumXX       = BML_f_MultAdd(f_abscissae, f_abscissae, p_LSF->fsumXX);
  p_LSF->fsumXV       = BML_f_MultAdd(f_abscissae, f_ordinate, p_LSF->fsumXV);
  p_LSF->fsumVV       = BML_f_MultAdd(f_ordinate, f_ordinate, p_LSF->fsumVV);
  p_LSF->iData_Counter++;
}


/*****************************************************************************

  Functionname:    CML_f_CalculateSlope_LSF                             */ /*!

  @brief           Calculate Slope of the fitting line out of actual LSF 
                   storage structure

  @description     This function calculates the slope of the fitting line
                   out of actual LSF storage structure. The calulation 
                   requires at least two data points, otherwise the function 
                   will return a slope value of zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.

  @return          Slope value of the fitting line



****************************************************************************/
float32 CML_f_CalculateSlope_LSF(BML_t_LeastSquareFit const *const p_LSF)
{
  float32 f_tempDenominator;
  float32 f_DataCounter;
  float32 f_Slope;

  /* you need minimal 2 Datapoints for calulation */
  if(p_LSF->iData_Counter >= 2L)
  {
    f_DataCounter = (float32) (p_LSF->iData_Counter);
    f_tempDenominator = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);

    /* avoid Division by Zero */
    if (fABS(f_tempDenominator) < C_F32_DELTA)
    {
      f_Slope = 0.0f;
    }
    else
    {
      f_Slope = ((f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV)) / f_tempDenominator;
    }
  }
  else
  {
    f_Slope = 0.0f;
  }
 
  return f_Slope;
}



/*****************************************************************************

  Functionname:    CML_f_CalculateCorrelation_LSF                       */ /*!

  @brief           Calculate Correlation of the fitting line out of actual 
                   LSF storage structure

  @description     This function calculates the correlation of the fitting
                   line out of the actual LSF storage structure.The calulation 
                   requires at least two data points, otherwise the function 
                   will return a correlation value of zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.

  @return          Correlation of the fitting line


****************************************************************************/
float32 CML_f_CalculateCorrelation_LSF(BML_t_LeastSquareFit const *const p_LSF)
{
  float32 f_tempNominator;
  float32 f_tempDenominatorX;
  float32 f_tempDenominatorV;
  float32 f_tempDenominatorXV;
  float32 f_DataCounter;
  float32 f_Correlation;

  /* you need minimal 2 Datapoints for calulation */
  if(p_LSF->iData_Counter >= (sint32)2)
  {
    f_DataCounter       = (float32) (p_LSF->iData_Counter);
    f_tempDenominatorX  = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);
    f_tempDenominatorV  = (f_DataCounter * p_LSF->fsumVV) - (p_LSF->fsumV * p_LSF->fsumV);
    f_tempNominator     = (f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV);
    f_tempDenominatorXV = f_tempDenominatorX * f_tempDenominatorV;

    /* avoid Division by Zero */
    if ( f_tempDenominatorXV < C_F32_DELTA )
    {
      f_Correlation = 0.0f;
    }
    else
    {
      f_Correlation = f_tempNominator / BML_f_Sqrt(f_tempDenominatorXV);
    }
  }
  else
  {
    f_Correlation = 0.0f;
  }

  return f_Correlation;
}



/*****************************************************************************

  Functionname:    CML_f_CalculateYIntersection_LSF                     */ /*!

  @brief           Calculate Point of Intersection with Y-Axis of the fitting
                   line out of actual LSF storage structure

  @description     This function calculates the point of intersection with
                   Y-axis of the fitting line out of the actual LSF storage
                   structure.The calulation requires at least two data points,
                   otherwise the function will return a Y-intersection value of 
                   zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.

  @return          Point of Intersection with Y-Axis of the fitting line

  @pre             -
  @post            -


****************************************************************************/
float32 CML_f_CalculateYIntersection_LSF(BML_t_LeastSquareFit const *const p_LSF)
{
  float32 f_tempDenominator;
  float32 f_DataCounter;
  float32 f_YIntersection;

  /* you need minimal 2 Datapoints for calulation */
  if(p_LSF->iData_Counter >= 2L)
  {
    f_DataCounter = (float32) (p_LSF->iData_Counter);
    f_tempDenominator = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);

    /* avoid Division by Zero */
    if (fABS(f_tempDenominator) < C_F32_DELTA)
    {
      f_YIntersection = 0.0f;
    }
    else
    {
      f_YIntersection = ((p_LSF->fsumV * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumXV)) / f_tempDenominator;
    }
  }
  else
  {
    f_YIntersection = 0.0f;
  }

  return f_YIntersection;
}


/*****************************************************************************

  Functionname:    CML_f_CalculateSquareError_LSF                       */ /*!

  @brief           Calculate Least Squared Error of the fitting line out of
                   actual LSF storage structure

  @description     This function calulates the least squared error of the 
                   fitting line out of the actual LSF storage structure.

                   chi^2 = sum_{i=1}^n ( v_i - slope * x_i - YIntersection )^2. 
                   This is not calculated directly, but instead
                   chi^2 = (1-correlation^2)*NVarV  (with NVarV = sumVV - sumV*sumV/n)

                   The calulation requires at least two data points,
                   otherwise the function will return a least squared error
                   value of zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
  @param[in]       f_LSFCorrelation : Correlation of the fitting line
                                      Supported values [Full range of sint32]

  @return          Least Squared Error of the fitting line

  @pre             Calculate actual Correlation!
  @post            -


****************************************************************************/
float32 CML_f_CalculateSquareError_LSF(BML_t_LeastSquareFit const *const p_LSF, const float32 f_LSFCorrelation)
{
  float32 f_tempNVarV;
  float32 f_DataCounter;
  float32 f_Chi2_Error;

  /* you need minimal 2 Datapoints for calulation */
  if(p_LSF->iData_Counter >= 2L)
  {
    f_DataCounter = (float32) (p_LSF->iData_Counter);

    f_tempNVarV  = p_LSF->fsumVV - ((p_LSF->fsumV * p_LSF->fsumV)/f_DataCounter); 

    f_Chi2_Error = (1.f - (f_LSFCorrelation * f_LSFCorrelation)) * f_tempNVarV;
  }
  else
  {
    f_Chi2_Error = 0.0f;
  }

  return f_Chi2_Error;
}


/*****************************************************************************

  Functionname:    CML_f_CalculateMeanSquareError_LSF                   */ /*!

  @brief           Calculate the Mean Square Error of the fitting line out of
                   actual LSF storage structure

  @description     This function calcuates the mean square error of the fitting
                   line out out of the actual LSF storage structure.
                   The calulation requires at least two data points,
                   otherwise the function will return a mean square error
                   value of zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
  @param[in]       f_LSFCorrelation : Correlation of the fitting line
                                      Supported values [Full range of sint32]

  @return          Mean Square Error of actual LSF-Structure

  @pre             Calculate actual Correlation!
  @post            -


****************************************************************************/
float32 CML_f_CalculateMeanSquareError_LSF(BML_t_LeastSquareFit const *const p_LSF, const float32 f_LSFCorrelation)
{
  float32 f_Chi2_Error;
  float32 f_mse;

  if(p_LSF->iData_Counter >= 2L) /* you need minimal 2 Datapoints for calulation */
  {
    f_Chi2_Error = CML_f_CalculateSquareError_LSF(p_LSF, f_LSFCorrelation);

    f_mse = f_Chi2_Error / ((float32)p_LSF->iData_Counter);
  }
  else
  {
    f_mse = 0.0f;
  }

  return f_mse;
}

/*****************************************************************************

  Functionname:    CML_v_CalculateQuality_LSF                           */ /*!

  @brief           Calculate Quality of the fitting line out of actual
                   LSF storage structure

  @description     This function calculate properties describing the Quality 
                   of the fitting line based on the actual LSF storage structure:
                     - Correlation
                     - Mean Square Error
                   
                   The calulation requires at least two data points,
                   otherwise the function will set mean square error
                   and correlation values of zero.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
  @param[out]      f_LSFCorrelation : Correlation of the fitting line
                                      [Full range of float32]
  @param[out]      f_LSFmse : Mean Square Error of the fitting line
                              [Full range of float32]

  @return          void

  @pre             -
  @post            -


****************************************************************************/
void CML_v_CalculateQuality_LSF(BML_t_LeastSquareFit const *const p_LSF, float32 *const f_LSFCorrelation, float32 *const f_LSFmse)
{
  float32 f_tempNominator;
  float32 f_tempDenominatorX;
  float32 f_tempDenominatorV;
  float32 f_tempDenominatorXV;
  float32 f_temp;
  float32 f_DataCounter;

  if(p_LSF->iData_Counter >= 2L) /* you need minimal 2 Datapoints for calulation */
  {
    f_DataCounter = (float32) (p_LSF->iData_Counter);
    f_tempDenominatorX   = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);
    f_tempDenominatorV   = (f_DataCounter * p_LSF->fsumVV) - (p_LSF->fsumV * p_LSF->fsumV);
    f_tempNominator      = (f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV);
    f_tempDenominatorXV  = f_tempDenominatorX * f_tempDenominatorV;
    f_temp               = f_tempDenominatorV / (f_DataCounter*f_DataCounter);

    /* avoid Division by Zero */
    if ( f_tempDenominatorXV < C_F32_DELTA )
    {
      *f_LSFCorrelation = 0.0f;
      *f_LSFmse = f_temp;
    }
    else
    {
      /* calculate Correlation */
      *f_LSFCorrelation = f_tempNominator / BML_f_Sqrt(f_tempDenominatorXV);
      /* calculate mean square error */
      *f_LSFmse = (1.f - ( (*f_LSFCorrelation) * (*f_LSFCorrelation))) * f_temp;
    }
  }
  else
  {
    *f_LSFCorrelation = 0.0f;
    *f_LSFmse = 0.0f;
  }
}



/*****************************************************************************

  Functionname:    CML_v_CalculateAll_LSF                               */ /*!

  @brief           Calculate Fitting results of the fitting line out of actual
                   LSF storage structure

  @description     This function calculate all properties of the fitting line 
                   based on the actual LSF storage structure:
                     - Slope
                     - Y-Intersection
                     - Correlation
                     - Square Error
                     - Mean Square Error
                   The calulation requires at least two data points,
                   otherwise no calculation is done.

  @param[in]       p_LSF : LSF storage structure
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF->iData_Counter [Full range of sint32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
  @param[out]      p_LSFRes : LSF result structure

  @return          void

  @pre             -
  @post            -


****************************************************************************/
void CML_v_CalculateAll_LSF(BML_t_LeastSquareFit const *const p_LSF, BML_t_LeastSquareFitResults *const p_LSFRes)
{
  float32 f_tempNominator;
  float32 f_tempDenominatorX;
  float32 f_tempDenominatorV;
  float32 f_tempDenominatorXV;
  float32 f_tempNVarV;
  float32 f_DataCounter;

  if(p_LSF->iData_Counter >= 2L) /* you need minimal 2 Datapoints for calulation */
  {
    f_DataCounter = (float32) (p_LSF->iData_Counter);
    f_tempDenominatorX   = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);
    f_tempDenominatorV   = (f_DataCounter * p_LSF->fsumVV) - (p_LSF->fsumV * p_LSF->fsumV);
    f_tempNominator      = (f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV);
    f_tempDenominatorXV  = f_tempDenominatorX * f_tempDenominatorV;
    f_tempNVarV          = f_tempDenominatorV / f_DataCounter; /*LSF->fsumVV - (LSF->fsumV * LSF->fsumV)/fData_Counter*/

    /* avoid Division by Zero */
    if ( f_tempDenominatorXV < C_F32_DELTA )
    {
      p_LSFRes->fCorrelation = 0.0f;
      p_LSFRes->fChi2_Error  = f_tempNVarV;
    }
    else
    {
      /* calculate Correlation */
      p_LSFRes->fCorrelation = f_tempNominator / BML_f_Sqrt(f_tempDenominatorXV);
      /* calculate chi^2 */
      p_LSFRes->fChi2_Error = (1.f - (p_LSFRes->fCorrelation * p_LSFRes->fCorrelation)) * f_tempNVarV;
    }

    /* calculate Slope, YIntersection */
    /* avoid Division by Zero */
    if (fABS(f_tempDenominatorX) < C_F32_DELTA)
    {
      p_LSFRes->fSlope         = 0.0f;
      p_LSFRes->fYIntersection = 0.0f;
    }
    else
    {
      p_LSFRes->fSlope         = ((f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV)) / f_tempDenominatorX;
      p_LSFRes->fYIntersection = ((p_LSF->fsumV * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumXV)) / f_tempDenominatorX;
    }

    /* calculate mean square error */
    p_LSFRes->fmse = p_LSFRes->fChi2_Error / f_DataCounter;
  }
}


/*****************************************************************************

  Functionname:    BML_f_Predict_LSF                                    */ /*!

  @brief           Predict y-value on least square fit for given x-value

  @description     This function predict y-value on least square fit for 
                   given x-value.The calculation is done using basic eqaution 
                   for line,
                   y = mx + c, 
                   where,
                   m = slope
                   c = Y intersection 

  @param[in]       f_xValue : x-value for which y-value to be determined
                              Supported values [Full range of float32]
  @param[in,out]   p_LSFResults : LSF result structure
                                  Supported values for p_LSFResults->fSlope [Full range of float32]
                                  Supported values for p_LSFResults->fYIntersection [Full range of float32]
                                  Overflow may occur when one or more input values 
                                  are at the defined range extremities.
  @return          y-value on least square fit for given x-value


****************************************************************************/
float32 BML_f_Predict_LSF(BML_t_LeastSquareFitResults const *const p_LSFResults, float32 f_xValue)
{
  return BML_f_MultAdd(f_xValue,p_LSFResults->fSlope,p_LSFResults->fYIntersection);
}


/*****************************************************************************

  Functionname:    BML_v_Init_LSF_ForgetExponential                     */ /*!

  @brief           Init the LSF storage structure with exponential forget

  @description     This function initializes all the elements of LSF storage 
                   structure with exponential forget to zero.

  @param[in,out]   p_LSF : LSF storage structure with exponential forget
                           [Valid pointer to the structure]

  @return          void

  @pre             -
  @post            -


****************************************************************************/
void BML_v_Init_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential *const p_LSF)
{
  p_LSF->fData_Counter  = 0.0f;
  p_LSF->fsumX          = 0.0f;
  p_LSF->fsumV          = 0.0f;
  p_LSF->fsumXX         = 0.0f;
  p_LSF->fsumXV         = 0.0f;
  p_LSF->fsumVV         = 0.0f;
}


/***********************************************************************
  Functionname:    BML_v_AddData_LSF_ForgetExponential             */ /*!

  @brief           Add data pair (x, v) to the LSF storage structure
                   with exponential forget

  @description     This function adds a data pair (f_abscissa, f_ordinate) 
                   to the LSF storage structure with exponential forget

  @param[in,out]   p_LSF  : LSF storage structure with exponential forget
                            Supported values for p_LSF->fsumX [Full range of float32]
                            Supported values for p_LSF->fsumV [Full range of float32]
                            Supported values for p_LSF->fsumXX [Full range of float32]
                            Supported values for p_LSF->fsumVV [Full range of float32]
                            Supported values for p_LSF->fsumXV [Full range of float32]
                            Supported values for p_LSF-fData_Counter [Full range of float32]
                            Overflow may occur when one or more input values 
                            are at the defined range extremities.
  @param[in]       f_abscissa  : abscissa of the data pair
                                 Supported values [Full range of float32]
  @param[in]       f_ordinate  : ordinate of the data pair
                                 Supported values [Full range of float32]
  @param[in]       f_MemoryWeight : Supported values [Full range of float32]

  @return          void

************************************************************************/

void BML_v_AddData_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential *const p_LSF, const float32 f_abscissa, const float32 f_ordinate, const float32 f_MemoryWeight)
{
  p_LSF->fsumX         *= f_MemoryWeight;
  p_LSF->fsumV         *= f_MemoryWeight;
  p_LSF->fsumXX        *= f_MemoryWeight;
  p_LSF->fsumXV        *= f_MemoryWeight;
  p_LSF->fsumVV        *= f_MemoryWeight;
  p_LSF->fData_Counter *= f_MemoryWeight;

  p_LSF->fsumX  += f_abscissa;
  p_LSF->fsumV  += f_ordinate;
  p_LSF->fsumXX += (f_abscissa*f_abscissa);
  p_LSF->fsumXV += (f_abscissa*f_ordinate);
  p_LSF->fsumVV += (f_ordinate*f_ordinate);
  p_LSF->fData_Counter++;
}


/*****************************************************************************

  Functionname:    BML_v_CalculateAll_LSF_ForgetExponential             */ /*!

  @brief           Calculate Fitting results of the fitting line out of actual
                   LSF storage structure with exponential forget

  @description     This function calculate all properties of the fitting line 
                   based on the actual LSF storage structure with exponential forget:
                     - Slope
                     - Y-Intersection
                     - Correlation
                     - Square Error
                     - Mean Square Error
                   The calulation requires at least two data points,
                   otherwise no calculation is done.

  @param[in]       p_LSF : LSF storage structure with exponential forget
                           Supported values for p_LSF->fsumX [Full range of float32]
                           Supported values for p_LSF->fsumV [Full range of float32]
                           Supported values for p_LSF->fsumXX [Full range of float32]
                           Supported values for p_LSF->fsumVV [Full range of float32]
                           Supported values for p_LSF->fsumXV [Full range of float32]
                           Supported values for p_LSF-fData_Counter [Full range of float32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
  @param[out]      p_LSFRes : LSF result structure

  @return          -

  @pre             -
  @post            -


****************************************************************************/
void BML_v_CalculateAll_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential const *const p_LSF, BML_t_LeastSquareFitResults *const p_LSFRes)
{
  float32 f_tempNominator;
  float32 f_tempDenominatorX;
  float32 f_tempDenominatorV;
  float32 f_tempDenominatorXV;
  float32 f_tempNVarV;
  float32 f_DataCounter;

  if(p_LSF->fData_Counter > 1.f) /* you need minimal 2 data points for calculation */
  {
    f_DataCounter = (float32) (p_LSF->fData_Counter);
    f_tempDenominatorX   = (f_DataCounter * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumX);
    f_tempDenominatorV   = (f_DataCounter * p_LSF->fsumVV) - (p_LSF->fsumV * p_LSF->fsumV);
    f_tempNominator      = (f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV);
    f_tempDenominatorXV  = f_tempDenominatorX * f_tempDenominatorV;
    f_tempNVarV          = f_tempDenominatorV / f_DataCounter;

    /* avoid Division by Zero */
    if ( f_tempDenominatorXV < C_F32_DELTA )
    {
      p_LSFRes->fCorrelation = 0.0f;
      p_LSFRes->fChi2_Error  = f_tempNVarV;
    }
    else
    {
      /* calculate Correlation */
      p_LSFRes->fCorrelation = f_tempNominator / BML_f_Sqrt(f_tempDenominatorXV);
      /* calculate chi^2 */
      p_LSFRes->fChi2_Error = (1.f - (p_LSFRes->fCorrelation * p_LSFRes->fCorrelation)) * f_tempNVarV;
    }

    /* calculate Slope, YIntersection */
    /* avoid Division by Zero */
    if (fABS(f_tempDenominatorX) < C_F32_DELTA)
    {
      p_LSFRes->fSlope         = 0.0f;
      p_LSFRes->fYIntersection = 0.0f;
    }
    else
    {
      p_LSFRes->fSlope         = ((f_DataCounter * p_LSF->fsumXV) - (p_LSF->fsumX * p_LSF->fsumV)) / f_tempDenominatorX;
      p_LSFRes->fYIntersection = ((p_LSF->fsumV * p_LSF->fsumXX) - (p_LSF->fsumX * p_LSF->fsumXV)) / f_tempDenominatorX;
    }

    /* calculate mean square error */
    p_LSFRes->fmse = p_LSFRes->fChi2_Error / f_DataCounter;
  }
}


/*****************************************************************************
  Functionname:    BML_f_CalcQuantile                                   */ /*!

  @brief           Calculates a quantile using the fast quick select algorithm
                   
  @description     This function calculates the quantile defined by the given 
                   position. 
                   For example, if requested for 3rd smallest element
                   in an array, then the function partly sorts the array, finds out
                   the 3rd smallest element and returns this value.
                   
  @param[in,out]   a_valuesDestroyed : pointer onto values from which quantile 
                   should be calculated, 
                   NOTE: array is PARTLY sorted, so if values are needed later, 
                   copy before!
                   Supported values for a_valuesDestroyed[] [Full range of float32]
  @param[in]       u_NofValues : number of values in a_valuesDestroyed
                                 Supported values [Full range of uint32]
  @param[in]       u_NthSmallest: sorted list index of element to be returned, 
                   i.e. 0u for the smallest element, uNofValues-1u for the maximum, 
                   uNofValues/2u for the median
                   Supported values [Full range of uint32]
                   
  @return          quantile
                   
  
*****************************************************************************/
float32 BML_f_CalcQuantile(float32 a_valuesDestroyed[], const uint32 u_NofValues, const uint32 u_NthSmallest)
{
  float32   f_Pivot;

  uint32  u_Left   = 0u,
          u_Right  = u_NofValues - 1u,
          u_Pos    = u_NthSmallest+1u,
          u_Ind;

  while ((u_Left < u_Right) && (u_Pos != u_NthSmallest))
  {
    float32 f_Temp;
    f_Pivot = a_valuesDestroyed[u_NthSmallest];
    
    /* swap */
    f_Temp = a_valuesDestroyed[u_NthSmallest];
    a_valuesDestroyed[u_NthSmallest] = a_valuesDestroyed[u_Right];
    a_valuesDestroyed[u_Right] = f_Temp;
    u_Pos = u_Left;

    for (u_Ind = u_Pos; u_Ind < u_Right; u_Ind++)
    {
      if (a_valuesDestroyed[u_Ind] < f_Pivot)
      {
        /* swap */
        f_Temp = a_valuesDestroyed[u_Ind];
        a_valuesDestroyed[u_Ind] = a_valuesDestroyed[u_Pos];
        a_valuesDestroyed[u_Pos] = f_Temp;
        u_Pos++;
      }
    }
    /* swap */
    f_Temp = a_valuesDestroyed[u_Right];
    a_valuesDestroyed[u_Right] = a_valuesDestroyed[u_Pos];
    a_valuesDestroyed[u_Pos] = f_Temp;

    if (u_Pos < u_NthSmallest)
    {
      u_Left = u_Pos + 1u;
    }
    else if (u_Pos > u_NthSmallest)
    {
          
      
      u_Right = u_Pos - 1u;
    }
    else
    {
      /* do nothing */
    }
  }
  return a_valuesDestroyed[u_NthSmallest];
}


/*****************************************************************************
  Functionname:    BML_f_StdGaussQuantile                               */ /*!

  @brief           Calculates a quantile of the standard Gaussian distribution N(0,1)
                   
  @description     Calculates a quantile of the standard Gaussian distribution N(0,1)
                   by the Beasly-Springer approach, i.e. using a rational
                   approximation of order (3,4)
                   
  @param[in]       f_Prob: the probability for which the quantile should be calculated
                           Supported values are float values in the range [0,..,1]
                   
  @return          the standard Gaussian quantile for the given probability
                   

*****************************************************************************/
float32 BML_f_StdGaussQuantile(float32 f_Prob)
{
  const float32 f_ProbShifted = f_Prob - 0.5f;
  const float32 f_AbsProbShifted = BML_f_Abs(f_ProbShifted);
  const float32 f_ProbShifted2 = f_ProbShifted * f_ProbShifted;
  float32 f_Tmp;
  float32 f_Ret;

  /* use Beasley-Springer approach in [0.08 ; 0.92], outside use simplified linear approximation */
  if (f_AbsProbShifted < BML_f_GaussQuantile_lowerBound)
  {
    f_Ret = f_ProbShifted * 
      ( ( ( ( ( ( (CML_f_GaussQuantile_a3 * f_ProbShifted2) + BML_f_GaussQuantile_a2) * f_ProbShifted2) + CML_f_GaussQuantile_a1) * f_ProbShifted2) + CML_f_GaussQuantile_a0) /
      ( ( ( ( ( ( ( (BML_f_GaussQuantile_b4 * f_ProbShifted2) + CML_f_GaussQuantile_b3) * f_ProbShifted2) + BML_f_GaussQuantile_b2) * f_ProbShifted2) + BML_f_GaussQuantile_b1) * f_ProbShifted2) + BML_f_GaussQuantile_b0) );
  }
  else if (f_AbsProbShifted < CML_f_GaussQuantile_sigma2Bound) /* linear approx to 2 sigma*/
  {
    f_Tmp = (f_AbsProbShifted - BML_f_GaussQuantile_lowerBound) / (CML_f_GaussQuantile_sigma2Bound - BML_f_GaussQuantile_lowerBound);
    f_Ret = (f_Tmp * (CML_f_GaussQuantile_sigma2Value - CML_f_GaussQuantile_lowerValue) ) + CML_f_GaussQuantile_lowerValue;
    if (f_ProbShifted < 0.0f)
    {
      f_Ret = -f_Ret;
    }
  } 
  else if (f_AbsProbShifted < CML_f_GaussQuantile_sigma3Bound) /* linear approx to 3 sigma*/
  {
    f_Tmp = (f_AbsProbShifted - CML_f_GaussQuantile_sigma2Bound) / (CML_f_GaussQuantile_sigma3Bound - CML_f_GaussQuantile_sigma2Bound);
    f_Ret = (f_Tmp * (CML_f_GaussQuantile_sigma3Value - CML_f_GaussQuantile_sigma2Value) ) + CML_f_GaussQuantile_sigma2Value;
    if (f_ProbShifted < 0.0f)
    {
      f_Ret = -f_Ret;
    }
  }
  else /* set to 4 sigma*/
  {
    f_Ret = (f_ProbShifted < 0.0f) ? (-CML_f_GaussQuantile_highValue) : CML_f_GaussQuantile_highValue;
  }

  return f_Ret;
}


/*****************************************************************************
  Functionname:    CML_f_GaussQuantile                                  */ /*!

  @brief           Calculates a quantile of a Gaussian distribution
                   
  @description     This function calculates a quantile of a Gaussian 
                   distribution.
                   
  @param[in]       f_Mean: mean value of the Gaussian distribution
                           Supported values [Full range of float32]
  @param[in]       f_Std:  standard deviation of the Gaussian distribution
                           Supported values [Full range of float32]
  @param[in]       f_Prob: the probability for which the quantile should be calculated
                           Supported values [Full range of float32]
                           Overflow may occur when one or more input values 
                           are at the defined range extremities.
                   
  @return          the Gaussian quantile for the given probability
                   

*****************************************************************************/
float32 CML_f_GaussQuantile(float32 f_Mean, float32 f_Std, float32 f_Prob)
{
  return BML_f_MultAdd(f_Std,BML_f_StdGaussQuantile(f_Prob),f_Mean);
}

/*****************************************************************************

  Functionname:    BML_v_Init_LSF_Quant                                 */ /*!

  @brief           Init the LSF storage structure for quantized LSF

  @description     Initialize the LSF storage structure for quantized 
                   (compressed) LSF. All the elements are set to Zero.
                   The abscissa variable is assumed to take non-negative values,
                   the ordinate variable can also take negative values.

  @param[in,out]   p_LSFQuant : LSF storage structure for quantized LSF
                                [Valid pointer to the structure]

  @return          void

  @pre             -
  @post            -


****************************************************************************/
void BML_v_Init_LSF_Quant(BML_t_LeastSquareFitQuant *const p_LSFQuant)
{
  p_LSFQuant->u_Data_Counter  = 0u;
  p_LSFQuant->u_sumX          = 0u;
  p_LSFQuant->s_sumV          = 0;
  p_LSFQuant->u_sumXX         = 0u;
  p_LSFQuant->s_sumXV         = 0;
  p_LSFQuant->u_sumVV         = 0u;
}

/*****************************************************************************
  
  Functionname:    BML_v_DecodeData_LSF_Quant                           */ /*!

  @brief           Decode the quantized LSF data to reconstructed
                   floating point LSF data
  
  @description     Inverse quantization (reconstruction) stage:
                   Decode the quantized LSF data p_LSFQuant to reconstructed
                   floating point LSF data p_LSFNonQuant using the
                   decoding factors f_decodeX and f_decodeV for the abscissa
                   and ordinate, respectively.

  @attention       Make sure that (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE)
                   if subsequently a new datapair (x_i,v_i) should be added to the statistics

  @param[in]       p_LSFQuant    : LSF storage structure for quantized LSF
                                   Supported values for p_LSFQuant->u_sumX [Full range of uint16]
                                   Supported values for p_LSFQuant->s_sumV [Full range of sint16]
                                   Supported values for p_LSFQuant->u_sumXX [Full range of uint16]
                                   Supported values for p_LSFQuant->s_sumXV [Full range of sint16]
                                   Supported values for p_LSFQuant->u_sumVV [Full range of uint16]
                                   Supported values for p_LSFQuant->u_Data_Counter [Full range of uint16]
                                   Overflow may occur when one or more input values 
                                   are at the defined range extremities.
  @param[in]       f_decodeX     : decoding factor for abscissa of the datapair
                                   [Full range of float32]
  @param[in]       f_decodeV     : decoding factor for ordinate of the datapair
                                   [Full range of float32]
  @param[out]      p_LSFNonQuant : LSF storage structure for reconstructed LSF
                                   [Valid pointer to the structure]

  @return          void

  @pre             Make sure that (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE)
                   if subsequently a new datapair (x_i,v_i) should be added to the statistics
  @post            -


****************************************************************************/
void BML_v_DecodeData_LSF_Quant(BML_t_LeastSquareFitQuant const *const p_LSFQuant, const float32 f_decodeX, const float32 f_decodeV, BML_t_LeastSquareFit *const p_LSFNonQuant)
{
  /* precalculated decoding factor for abscissa and ordinate, respectively, for runtime optimization */
  float32 f_decodeFactorX, f_decodeFactorV;
  
  BML_ASSERT(p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE);
  
  f_decodeFactorX = ( f_decodeX * (1.f/(float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE) ) * (float32)p_LSFQuant->u_Data_Counter;
  f_decodeFactorV = ( f_decodeV * (1.f/(float32)CML_u_SIGNED_16BIT_QUANT_SIZE) ) * (float32)p_LSFQuant->u_Data_Counter;

  /* decode the quantized LSF data to reconstructed floating point LSF data */
  p_LSFNonQuant->fsumX  = p_LSFQuant->u_sumX  * f_decodeFactorX;
  p_LSFNonQuant->fsumV  = p_LSFQuant->s_sumV  * f_decodeFactorV;
  p_LSFNonQuant->fsumXX = p_LSFQuant->u_sumXX * (f_decodeFactorX * f_decodeX);
  p_LSFNonQuant->fsumXV = p_LSFQuant->s_sumXV * (f_decodeFactorV * f_decodeX);
  p_LSFNonQuant->fsumVV = p_LSFQuant->u_sumVV * ( ( (f_decodeV*f_decodeV) * (1.f/(float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE) ) * (float32)p_LSFQuant->u_Data_Counter );
  p_LSFNonQuant->iData_Counter = (sint32)p_LSFQuant->u_Data_Counter;
}

/*****************************************************************************

  Functionname:    CML_v_EncodeData_LSF_Quant                           */ /*!

  @brief           Encode the reconstructed floating point LSF data to
                   quantized LSF data
  
  @description     Forward quantization (classification) stage:
                   Encode the reconstructed floating point LSF data p_LSFNonQuant
                   to quantized LSF data p_LSFQuant using the
                   encoding factors f_encodeX and f_encodeV for the abscissa
                   and ordinate, respectively

  @attention       * To avoid division by zero make sure that the following conditions are fulfilled:
                   ( (BML_f_IsNonZero(f_encodeX)) && (BML_f_IsNonZero(f_encodeV)) &&
                   (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE) )
                   * Ensure correct quantization and avoid integer overflow (see detailed description below),
                   e.g., by setting f_encodeX = max(abs(x_i)) for all i and
                   f_encodeV = max(abs(v_i)) for all i

  @param[in]       p_LSFNonQuant : LSF storage structure for reconstructed LSF
                                   Supported values for p_LSF->fsumX [Full range of float32]
                                   Supported values for p_LSF->fsumV [Full range of float32]
                                   Supported values for p_LSF->fsumXX [Full range of float32]
                                   Supported values for p_LSF->fsumVV [Full range of float32]
                                   Supported values for p_LSF->fsumXV [Full range of float32]
                                   Supported values for p_LSF->iData_Counter [Full range of sint32]
                                   Overflow may occur when one or more input values 
                                   are at the defined range extremities.
  @param[in]       f_encodeX     : encoding factor for abscissa of the datapair
                                   [Full range of float32]
  @param[in]       f_encodeV     : encoding factor for ordinate of the datapair
                                   [Full range of float32]
  @param[out]      p_LSFQuant    : LSF storage structure for quantized LSF
                                   [Valid pointer to the structure]

  @return          void

  @pre             * To avoid division by zero make sure that the following conditions are fulfilled:
                   ( (BML_f_IsNonZero(f_encodeX)) && (BML_f_IsNonZero(f_encodeV)) &&
                   (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE) )
                   * Ensure correct quantization and avoid integer overflow (see detailed description below),
                   e.g., by setting f_encodeX = max(abs(x_i)) for all i and
                   f_encodeV = max(abs(v_i)) for all i

  @post            -


****************************************************************************/
void CML_v_EncodeData_LSF_Quant(BML_t_LeastSquareFit const *const p_LSFNonQuant, const float32 f_encodeX, const float32 f_encodeV, BML_t_LeastSquareFitQuant *const p_LSFQuant)
{
  /* precalculated encoding factor of abscissa and ordinate, respectively, for runtime optimization */
  float32 f_encodeFactorX, f_encodeFactorV;
  /* inverse of encoding factor of abscissa for runtime optimization */
  float32 f_encodeXInverse;
  float32 f_valueClipped;

  /* encode the reconstructed floating point LSF data to quantized LSF data only if it is ensured that no division by zero occurs */
  if ( (BML_f_IsNonZero(f_encodeX)) && (BML_f_IsNonZero(f_encodeV)) && (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE) )
  {
    /* By checking the condition (p_LSFQuant->u_Data_Counter < (uint16)CML_u_UNSIGNED_16BIT_QUANT_SIZE)
    it is ensured that after the increment p_LSFQuant->u_Data_Counter++ the variable p_LSFQuant->u_Data_Counter
    is nonzero */
    p_LSFQuant->u_Data_Counter++;

    f_encodeFactorX = (float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE / ( (float32)p_LSFQuant->u_Data_Counter * f_encodeX );
    f_encodeFactorV = (float32)CML_u_SIGNED_16BIT_QUANT_SIZE   / ( (float32)p_LSFQuant->u_Data_Counter * f_encodeV );

    /* encode the reconstructed floating point LSF data to quantized LSF data */

    /* integer overflow is avoided by the following described facts: */

    /* 0 <= ( p_LSFNonQuant->fsumX / (p_LSFQuant->u_Data_Counter * f_encodeX) ) < 1 */
    f_valueClipped = BML_f_MinMax(0.f, (float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE, p_LSFNonQuant->fsumX  * f_encodeFactorX);
    p_LSFQuant->u_sumX  = (uint16)BML_u_Round2Uint( f_valueClipped );

    /* -1 < ( p_LSFNonQuant->fsumV / (p_LSFQuant->u_Data_Counter * f_encodeV) ) < 1 */
    f_valueClipped = BML_f_MinMax((float32)CML_s_SIGNED_16BIT_LOWER_LIMIT, (float32)CML_u_SIGNED_16BIT_QUANT_SIZE, p_LSFNonQuant->fsumV  * f_encodeFactorV);
    p_LSFQuant->s_sumV  = (sint16)BML_s_Round2Int( f_valueClipped );

    f_encodeXInverse = 1.f / f_encodeX;

    /* 0 <= ( p_LSFNonQuant->fsumXX / (p_LSFQuant->u_Data_Counter * f_encodeX * f_encodeX) ) < 1 */
    f_valueClipped = BML_f_MinMax(0.f, (float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE, p_LSFNonQuant->fsumXX * (f_encodeFactorX * f_encodeXInverse) );
    p_LSFQuant->u_sumXX = (uint16)BML_u_Round2Uint( f_valueClipped );

    /* -1 < ( p_LSFNonQuant->fsumXV / (p_LSFQuant->u_Data_Counter * f_encodeV * f_encodeX) ) < 1 */
    f_valueClipped = BML_f_MinMax((float32)CML_s_SIGNED_16BIT_LOWER_LIMIT, (float32)CML_u_SIGNED_16BIT_QUANT_SIZE, p_LSFNonQuant->fsumXV * (f_encodeFactorV * f_encodeXInverse) );
    p_LSFQuant->s_sumXV = (sint16)BML_s_Round2Int( f_valueClipped );

    /* 0 <= ( p_LSFNonQuant->fsumVV / (p_LSFQuant->u_Data_Counter * f_encodeV * f_encodeV) ) < 1 */
    f_valueClipped = BML_f_MinMax(0.f, (float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE,
      p_LSFNonQuant->fsumVV * ( (float32)CML_u_UNSIGNED_16BIT_QUANT_SIZE / ( (float32)p_LSFQuant->u_Data_Counter * (f_encodeV*f_encodeV) ) ) );
    p_LSFQuant->u_sumVV = (uint16)BML_u_Round2Uint( f_valueClipped );
  }
  else
  {
    BML_ASSERT(FALSE);
  }

}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"