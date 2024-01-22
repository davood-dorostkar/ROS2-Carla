/** \addtogroup DistanceMatrix
 *  \{
 * \file    TueObjFusn_DistMatrix.h
 * \brief  This is a structure definition file for the distance score matrix
 *
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_DISTMATRIX_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_DISTMATRIX_H_

#include "tue_prv_common_types.h" /* standard types (uint16, float32, etc.) */
#include "TueObjFusn_ConfigConstants.h"
#include "FusionCompiler_Cfg.h"
#include "FusionCompiler.h"

/*==================[macros]================================================*/
/** size of the distance matrix; needs to be large enough to accomodata all
 * objects rsp. trackables  */

#define TUE_PRV_DISTSCORE_MAX_ROW_SIZE TUE_PRV_FUSION_TRACKABLE_LIST_SIZE
#define TUE_PRV_DISTSCORE_MAX_COL_SIZE TUE_PRV_FUSION_MAX_INPUT_OBJECTS

#define TUE_PRV_DISTSCORE_CONVERT_F32_SCALE (1024.f)
#define TUE_PRV_DISTSCORE_CONVERT_F16_MAX_FIXED_INT (64.f)

/*==================[typedef]================================================*/
/**
 * The stDistMatrix_t type stores the distances between objects in two lists.
 */
typedef struct stDistMatrix_t_tag {
    uint16
        data[TUE_PRV_DISTSCORE_MAX_ROW_SIZE]
            [TUE_PRV_DISTSCORE_MAX_COL_SIZE]; // data[i][j] is the distance
                                                //  between object i in first
                                                //  object j in second list
    uint16 nRows; /**< number of rows used in data */
    uint16 nCols; /**< number of columns used in data */
} stDistMatrix_t;

#define ObjFusn_START_SEC_CODE

/**
 * @fn      void initDistMatrix(stDistMatrix_t* const pMat, uint16 const
 * u16Rows, uint16 const u16Cols, float32 const f32InitVal)
 *
 * @brief   Inits the distance matrix to the inputted default value
 *
 * @param   [in,out] pMat     stDistMatrix_t* const, distance matrix
 * @param   u16Rows           u16_t const, number of rows
 * @param   u16Cols           u16_t const, number of cols
 * @param   f32InitVal        f32_t const, default value
 *
 * @return  void
 */

void initDistMatrix(CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        spMat,
                    const uint16 u16Rows,
                    const uint16 u16Cols,
                    const float32 f32InitVal);

/**
 * @fn      stDistMatrix_t * const getDistMat(void)
 *
 * @brief   Returns a pointer to the internal distance matrix
 *
 *
 * @return  stDistMatrix_t * const
 */

extern P2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_CODE) getDistMat(void);

/**
 * @fn     uint16 convertFloatToFixedtDistMat(f32_t f32float)
 *
 * @brief   converts floating point to fixed point
 *
 * @param   f32float        f32_t f32float, value in floating point
 *
 * @return  u16_t
 */

uint16 convertFloatToFixedtDistMat(const float32 f32Float);
#define ObjFusn_STOP_SEC_CODE

#ifdef UNITTEST

#define ObjFusn_START_SEC_CODE

/**
 * @fn     float32 convertFixedToFloatDistMat(uint16 u16Num)
 *
 * @brief   converts fixed point to floatign point
 *
 * @param   u16Num        u16_t u16Num, value in fixed point format
 *
 * @return  f32_t        Converted to f32_t
 */
float32 convertFixedToFloatDistMat(const uint16 u16Num);
#define ObjFusn_STOP_SEC_CODE

#endif  // UnitTest

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_DISTMATRIX_H_
