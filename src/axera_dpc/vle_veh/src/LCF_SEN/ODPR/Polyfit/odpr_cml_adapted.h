
/**************************************************************************** */
#include "TM_Global_Types.h"

#include "odpr_cml_mtrx.h"
#include "rtwtypes.h"
//#include "algo_glob.h"
//#include <gmem.h>

#ifndef __ODPR_CML_ADAPTED_H__
#define __ODPR_CML_ADAPTED_H__  // avoid double inclusion

//#define TUE_CML_IsNonZero(value) (CML_f_Abs(value) >= CML_f_AlmostZero) //
// F32_IS_NZERO
//#define TUE_CML_IsNonZero(value) (CML_f_Abs(value) >= LCF_CML_f_AlmostZero) //
// F32_IS_NZERO

//#define CML_f_AlmostZero (1e-15f)
#define LCF_CML_f_AlmostZero (1e-20f)

/*! number of elements in a 2x2 matrix */
#define LCF_MTRX_2X2_NOF_ELEMENTS 4
/*! number of elements in a 3x3 matrix */
#define LCF_MTRX_3X3_NOF_ELEMENTS 9

boolean_T LCF_b_InvertMatrixCramer2(float32 a_res[LCF_MTRX_2X2_NOF_ELEMENTS],
                                    float32 a_in[LCF_MTRX_2X2_NOF_ELEMENTS]);
boolean_T LCF_b_InvertMatrixCramer3(float32 a_res[LCF_MTRX_3X3_NOF_ELEMENTS],
                                    float32 a_in[LCF_MTRX_3X3_NOF_ELEMENTS]);
boolean_T LCF_CML_CalcInvertMatrix_M(ODPR_CML_t_Matrix* p_MatrixA,
                                     float32* p_DataA,
                                     float32* p_DataRes);
void LCF_CML_v_InvertMatrix(ODPR_CML_t_Matrix* p_MatrixRes,
                            ODPR_CML_t_Matrix* p_MatrixA);

#endif
