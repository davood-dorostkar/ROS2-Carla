

#ifndef RTW_HEADER_ved__aye_private_h_
#define RTW_HEADER_ved__aye_private_h_
#include "ved_consts.h"

/* Includes for objects with custom storage classes. */
#include "ved_ext.h"

/*
 * Generate compile time checks that imported macros for parameters
 * with storage class "Const" are defined
 */

/*
 * Generate compile time checks that imported macros for parameters
 * with storage class "VED__Defines" are defined
 */
#ifndef VED_IO_STATE_VALID
#error The variable for the parameter "VED_IO_STATE_VALID" is not defined
#else
#if (VED_IO_STATE_VALID < 0UL) || (VED_IO_STATE_VALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_VALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_LATA
#error The variable for the parameter "VED_SIN_POS_LATA" is not defined
#else
#if (VED_SIN_POS_LATA < 0UL) || (VED_SIN_POS_LATA > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_LATA" is outside of the range 0UL to 4294967295UL
#endif
#endif

#define CALL_EVENT (-1)

/* QAC Fixes */

#endif /* RTW_HEADER_ved__aye_private_h_ */
