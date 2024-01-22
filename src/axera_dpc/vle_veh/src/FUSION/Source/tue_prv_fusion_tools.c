/** \addtogroup tueFusion
 *  \{
 * \file tue_prv_fusion_tools.c
 * \brief Methods that are used all throughout fusion on different occasions
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 */
#include "tue_prv_fusion_tools.h"

#define ONE_BIT_ONE_ONE_BIT_ZERO (0x55555555u)
#define TWO_BITS_ONE_TWO_BITS_ZERO (0x33333333u)
#define FOUR_BITS_ONE_FOUR_BITS_ZERO (0x0F0F0F0Fu)
#define ONE_BIT_ONE_SEVEN_BITS_ZERO (0x01010101u)
#define BITSHIFT_BY_TWO (2u)
#define BITSHIFT_BY_FOUR (4u)
#define BITSHIFT_BY_TWENTYFOUR (24u)

///\name  Fusion Tools

/*********************************************************************
 *
 *  definition of fast approximative square root operator	\todo: Delete
 *later on
 *
 *********************************************************************/
// f32_t RSP_sqrtfast(f32_t const parx)
//{
//   f32_t xux;
//
//   union
//   {
//      f32_t x;
//      s32_t i;
//   } u;
//   u.x = parx;
//   u.i = SQRT_MAGIC_F - (s32_t)((u32_t)u.i >> 1);
//
//   xux = parx * u.x;
//
//   return xux*(1.5f - 0.5f*xux*u.x);
//}

#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Library Function */
uint32 getNumberOfSetBits(uint32 i) {
    // http://stackoverflow.com/questions/109023/how-to-count-the-number-of-set-bits-in-a-32-bit-integer
    // aka variable-precision SWAR algorithm. See here for an explanation:
    // http://stackoverflow.com/questions/22081738/how-variable-precision-swar-algorithm-works

    uint32 u32Result;

    i = i - ((i >> 1u) & ONE_BIT_ONE_ONE_BIT_ZERO);
    i = (i & TWO_BITS_ONE_TWO_BITS_ZERO) +
        ((i >> BITSHIFT_BY_TWO) & TWO_BITS_ONE_TWO_BITS_ZERO);
    u32Result =
        (((i + (i >> BITSHIFT_BY_FOUR)) & FOUR_BITS_ONE_FOUR_BITS_ZERO) *
         ONE_BIT_ONE_SEVEN_BITS_ZERO) >>
        BITSHIFT_BY_TWENTYFOUR;
    return u32Result;
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
