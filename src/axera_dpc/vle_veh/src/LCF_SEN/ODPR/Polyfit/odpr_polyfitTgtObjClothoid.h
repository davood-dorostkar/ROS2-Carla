/*! \file **********************************************************************

  COMPANY:                   Continental Teves AG & Co. oHG

  CPU:                       CPU-Independent

  COMPONENT:                 TJA (Traffic Jam Assist)

  MODULNAME:                 polyfitTgtObjectClothoid.c

  DESCRIPTION:               implements polyfit for target object trajectory estimation

  CREATION DATE:             $Date: 2019/11/01 18:02:10CET $

  VERSION:                   $Revision: 1.0 $

  CHANGES:                
  ---*/ /*---
                                                                                                                                                                                                                              ****************************************************************************
                                                                                                                                                                                                                              */

/*****************************************************************************
  INCLUDE PROTECTION
*****************************************************************************/
#ifndef FILTER_OBJ_TRAJ_HEADER_
#define FILTER_OBJ_TRAJ_HEADER_

#include "odpr_ext.h"

/*****************************************************************************
  INCLUDES
*****************************************************************************/
//#ifndef PLATFORM_TYPES_H
// typedef unsigned char boolean;
// typedef float float32;
// typedef unsigned char uint8;
//#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
//#ifdef MATH_LOCAL_SFUNCTION /* for SIL mode compatibility. Comment for
// check-in. */
/*! @brief input to the trajectory estimation Kalman filter (provided by the
 * LCOF simulink model)*/
typedef struct {
    boolean bEnable_bool;
    boolean bAddNewSample_bool;
    boolean bReset_bool;
    float32 fObjXPos_met;            // measured object longitudinal position
    float32 fObjYPos_met;            // measured object lateral position
    float32 fObjXPosStdDev_met;      // measured object longitudinal position
    float32 fObjYPosStdDev_met;      // measured object lateral position
    float32 fTimeSinceLastCall_sec;  // passed time since last call to
                                     // filterTgtObjectClothoid - typically the
                                     // cycle time
    float32 fEgoCrv_1pm;             // curvature of the ego vehicle
    float32 fEgoYawRate_rps;         // yaw rate of the ego vehicle
    float32 fEgoVelX_mps;            // velocity of the ego vehicle
    float32 fPredYawRtVar_r2ps2;     // yaw rate variance for prediction
    float32 fPredVelXVar_m2;         // velocity variance for prediction
    float32
        fModelPosYVar_m2;  // model uncertainty with respect to lateral position
    float32
        fModelYawVar_rad2;  // model uncertainty with respect to heading angle
    float32 fModelCrvVar_1pm2;  // model uncertainty with respect to curvature
    float32 fModelCrvChngVar_1pm4;  // model uncertainty with respect to
                                    // curvature change
    float32 fObjMeasPosYVar_m2;     // measurement uncertainty of lateral object
                                    // position
    float32 fObjMeasPosXVar_m2;     // measurement uncertainty of longitudinal
                                    // object position
    float32 fCrvDecay_nu;           // Curvature change decay factor for polyfit
    float32 fCrvChngDecay_nu;       // Curvature change decay factor for polyfit
    float32 fMinHistStartPosX_met;
    float32 fMaxSampleAge_sec;
    uint8 uMinNumValidSamples_nu;
    float32 fMinHistLength_met;
    float32 fMaxGapEgoToHist_met;
    float32 pWeightLastFit_nu;
} FOHTgtObjPFInput_t;

typedef struct {
    uint8 uNumValidSamples_nu;
    float32 fPosX0_met;
    float32 fPosY0_1st_met;
    float32 fPosY0_3rd_met;
    float32 fHeading_1st_rad;
    float32 fHeading_3rd_rad;
    float32 fCrv_1pm;
    float32 fChngOfCrv_1pm2;
    float32 fTrajLength_met;
    boolean bTrajUpdate_bool;
    boolean bTrajInvalid1st_bool;
    boolean bTrajInvalid3rd_bool;
    float32 fMeanDevToTraj_1st_met;
    float32 fMeanDevToTraj_3rd_met;
    float32 fLastStoredPntX_met;
    float32 fLastStoredPntY_met;
    float32 fLastStoredAge_sec;
    float32 fFirstStoredAge_sec;
    float32 fMeanSampleAge_sec;
} FOHTgtObjPFOutput_t;

/*****************************************************************************
  FUNCTION PROTOTYPES (EXTERNAL SCOPE)
*****************************************************************************/

/* ****************************************************************************
Functionname:    polyfitTgtObjectClothoid

@brief           calculates a polynomial fit using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated
                 using the driven delta ego distance and rotated delta ego yaw
angle.
                 The equation matrix X is filled with the x/y sample points and
                 the polynomial coefficient vector is calculated

@param[in]       fObjXPos_met : target object x position

@param[in]       fObjYPos_met : target object y position

@param[out]      vecPolyCoeff : calculated polynomial coefficients

**************************************************************************** */
extern void FOHPolyfitTgtObjClothoid(const FOHTgtObjPFInput_t* input,
                                     FOHTgtObjPFOutput_t* output,
                                     ODPRFOHDebug_t* pFOHDebug);

#endif
