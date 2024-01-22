

#ifndef RTW_HEADER_ved__ye_private_h_
#define RTW_HEADER_ved__ye_private_h_
#include "ved_consts.h"

/*
 * Generate compile time checks that imported macros for parameters
 * with storage class "Const" are defined
 */
#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__ye_diag_variance(const real32_T rtu_u[16],
                                  rtB_diag_variance_ved__ye *localB);
extern void ved__ye_At(const real32_T rtu_u[4], rtB_At_ved__ye *localB);
extern void ved__ye_eye(rtB_eye_ved__ye *localB);
extern void ved__ye_diag_curve_variance(
    const real32_T rtu_u[4], rtB_diag_curve_variance_ved__ye *localB);
extern void ved__ye_make_A_matrix(real32_T rtu_CycleTime,
                                  rtB_make_A_matrix_ved__ye *localB);

#endif /* RTW_HEADER_ved__ye_private_h_ */
