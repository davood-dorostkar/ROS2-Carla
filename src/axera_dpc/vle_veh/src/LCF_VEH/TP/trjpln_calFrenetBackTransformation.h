/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*********************************************************************

PROJECT:        TJA Base Development

MODULE:         Trajectory Planning

Type:           C-Header

CPU:            All

DESCRIPTION:

********************************************************************************

<HISTORY>
#<version>  <modify-date>   <owner>     <comment>
<NEW>
</NEW>
</HISTORY>

*********************************************************************/

#ifndef TRJPLN_CALFRENETBACKTRANSFORMATION_H
#define TRJPLN_CALFRENETBACKTRANSFORMATION_H
#ifdef __cplusplus
extern "C" {
#endif
/* -----------------------------------------------------------------------------

        I N C L U D E S

 * -----------------------------------------------------------------------------
*/

#include "trjpln_calFrenetBackTransformation_TypeDef.h"

/* -----------------------------------------------------------------------------

        D E C L A R A T I O N   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
        \brief Calculate Frenet back transformation (Frenet -> Vehicle).

        \param Inputs Frenet back transformation inputs

        \return
 * -----------------------------------------------------------------------------
*/
TRJPLN_calFBTOutType_t TRJPLN_calFrenetBackTransformation(
    const TRJPLN_calFBTInType_t* Inputs);

/* -----------------------------------------------------------------------------
        \brief Calculate the change of curvature of the trajectory

        \param

        \return
 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FBT_CalcTrajChgOfCurvature(float32 deltaTheta,
                                          float32 Kr,
                                          float32 Kr_p,
                                          float32 d,
                                          float32 d_p,
                                          float32 d_pp,
                                          float32 d_ppp,
                                          float32 Kt,
                                          float32 v_Bahn);

/* -----------------------------------------------------------------------------

        H E L P E R   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
        \brief Avoid zero values

        \param input

        \return

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FBT_AvoidZero(const float32 input);
#ifdef __cplusplus
}
#endif
#endif /* _CALFRENETRUECKTRANSFORMATION_H */
