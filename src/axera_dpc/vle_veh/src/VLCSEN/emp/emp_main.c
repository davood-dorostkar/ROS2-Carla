/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
/* EMP Includes */

#include "TM_Math_Cal.h"
//#include "tue_common_libs.h"
#include "vlcSen_common_utils.h"
#include "vlcSen_ext.h"
//#include "vlc_types.h"
#include "vlc_glob_ext.h"

#include "emp_main.h"
#include "emp_calculation.h"
#include "cd_par.h"
//#include "vlc_par.h"
//#include "vlc_sen.h"
#include "emp_ext.h"
#include "frame_sen_custom_types.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
static boolean EMPCalcDistToTraj(const EMPKinEgo_t* pIn_KinEgo,
                                 const EMPObjDesc_t* pIn_Obj,
                                 float32* pfOut_DistToTraj,
                                 float32* pfOut_DistOnTraj);
static boolean EMPCalcVelToTraj(const EMPKinEgo_t* pIn_KinEgo,
                                const EMPObjDesc_t* pIn_Obj,
                                float32* pfOut_VelToTraj,
                                float32* pfOut_VelOnTraj);

/* **************************************************************************** 

  @fn           EMPPredictEgoTraj   */ /*!

                                            @brief        Predict Ego Trajectory
                                          based on given assumption

                                            @param[in]    pIn_KinEgo
                                            @param[in]    ManeuverType
                                            @param[in]    pOut_TrajPred

                                            @return       void

                                          ****************************************************************************
                                          */
extern void EMPPredictEgoTraj(const EMPKinEgo_t* pIn_KinEgo,
                              EMPManeuverType_t ManeuverType,
                              EMPTrajPred_t* pOut_TrajPred) {
    /* Retrieve current Motion States */
    const float32 fCurrEgoVel = pIn_KinEgo->fVel;
    const float32 fCurrEgoAccel = pIn_KinEgo->fAccel;
    const float32 fCurrEgoYawRate = pIn_KinEgo->fYawRate;
    const float32 fCurrALat = fCurrEgoVel * fCurrEgoYawRate;
    const float32 fCurrSlipAngle =
        (fABS(fCurrEgoVel) > C_F32_DELTA)
            ? (((VLC_WHEELBASE_DEFAULT * 0.5f) * fCurrEgoYawRate) / fCurrEgoVel)
            : 0;

    pOut_TrajPred->XofT.fC0 = -(CD_COMMON_EGO_LENGTH * 0.5f);
    pOut_TrajPred->YofT.fC0 = 0;

    switch (ManeuverType) {
        case EMP_MANEUVER_KinematicsUnchanged:
            pOut_TrajPred->XofT.fC2 =
                -0.5f * fCurrEgoVel * fCurrEgoYawRate * SIN_(fCurrSlipAngle);
            pOut_TrajPred->XofT.fC1 = fCurrEgoVel * COS_(fCurrSlipAngle);

            pOut_TrajPred->YofT.fC2 =
                0.5f * fCurrEgoVel * fCurrEgoYawRate * COS_(fCurrSlipAngle);
            pOut_TrajPred->YofT.fC1 = fCurrEgoVel * SIN_(fCurrSlipAngle);
            break;
        case EMP_MANEUVER_KinematicsWithoutAcceleration:
            pOut_TrajPred->XofT.fC2 =
                (0.5f * fCurrEgoAccel * COS_(fCurrSlipAngle)) -
                (0.5f * fCurrALat * SIN_(fCurrSlipAngle));
            pOut_TrajPred->XofT.fC1 = fCurrEgoVel * COS_(fCurrSlipAngle);

            pOut_TrajPred->YofT.fC2 =
                (0.5f * fCurrEgoAccel * SIN_(fCurrSlipAngle)) +
                (0.5f * fCurrALat * COS_(fCurrSlipAngle));
            pOut_TrajPred->YofT.fC1 = fCurrEgoVel * SIN_(fCurrSlipAngle);
            break;
        default:
            /* Should not happen */
            break;
    }

    /* Fill the rest of output Structure */
    pOut_TrajPred->VarXofT.fC0 = EMP_DEFAULT_EGO_VAR_X_C0;
    pOut_TrajPred->VarXofT.fC1 = EMP_DEFAULT_EGO_VAR_X_C1;
    pOut_TrajPred->VarXofT.fC2 = EMP_DEFAULT_EGO_VAR_X_C2;

    pOut_TrajPred->VarYofT.fC0 = EMP_DEFAULT_EGO_VAR_Y_C0;
    pOut_TrajPred->VarYofT.fC1 = EMP_DEFAULT_EGO_VAR_Y_C1;
    pOut_TrajPred->VarYofT.fC2 = EMP_DEFAULT_EGO_VAR_Y_C2;
}

/* **************************************************************************** 

  @fn           EMPPredictObjTraj   */ /*!

                                            @brief        Predict Object
                                          Trajectory based on given assumption

                                            @param[in]    pIn_KinObj
                                            @param[in]    eAssumption
                                            @param[in]    pOut_TrajPred

                                            @return       True if no error
                                          occurred


                                          ****************************************************************************
                                          */
boolean EMPPredictObjTraj(const EMPKinObj_t* pIn_KinObj,
                          EMPManeuverType_t eAssumption,
                          EMPTrajPred_t* pOut_TrajPred) {
    pOut_TrajPred->XofT.fC0 = pIn_KinObj->fPosX;
    pOut_TrajPred->XofT.fC1 = pIn_KinObj->fVelX;
    pOut_TrajPred->XofT.fC2 = 0;

    pOut_TrajPred->YofT.fC0 = pIn_KinObj->fPosY;
    pOut_TrajPred->YofT.fC1 = pIn_KinObj->fVelY;
    pOut_TrajPred->YofT.fC2 = 0;

    /* Fill the rest of output Structure */
    pOut_TrajPred->VarXofT.fC0 = pIn_KinObj->fPosXVar;
    pOut_TrajPred->VarXofT.fC1 = pIn_KinObj->fVelXVar;
    pOut_TrajPred->VarXofT.fC2 = 0;

    pOut_TrajPred->VarYofT.fC0 = pIn_KinObj->fPosYVar;
    pOut_TrajPred->VarYofT.fC1 = pIn_KinObj->fVelYVar;
    pOut_TrajPred->VarYofT.fC2 = 0;

    switch (eAssumption) {
        case EMP_MANEUVER_ComfortBraking: {
            pOut_TrajPred->XofT.fC2 = EMP_OBJ_BRAKE_DECEL_COMFORT;
            break;
        }
        case EMP_MANEUVER_FullBraking: {
            pOut_TrajPred->XofT.fC2 = EMP_OBJ_BRAKE_DECEL_FULL;
            break;
        }
        default:
            break;
    }

    return TRUE;
}

/* **************************************************************************** 

  @fn           EMPCalcCollProbObjObj   */ /*!

                                        @brief        Calculate Collision
                                      Probability of two Objects at a given time

                                        @param[in]    fTime
                                        @param[in]    pIn_Obj1
                                        @param[in]    pIn_Obj2
                                        @param[out]   pOut_CollisionProb

                                        @return       True if no error occurred


                                      ****************************************************************************
                                      */
boolean EMPCalcCollProbObjObj(float32 fTime,
                              const EMPObjPred_t* pIn_Obj1,
                              const EMPObjPred_t* pIn_Obj2,
                              float32* pOut_CollisionProb) {
    boolean bResultOk = TRUE;

    /* Extract Existence Description from Input Objects */
    EMPObjExDesc_t sExDescObj1;
    EMPObjExDesc_t sExDescObj2;

    bResultOk &= EMPCalcObjExDesc(fTime, pIn_Obj1, &sExDescObj1);
    bResultOk &= EMPCalcObjExDesc(fTime, pIn_Obj2, &sExDescObj2);

    bResultOk &=
        EMPCalcCollProbExDesc(&sExDescObj1, &sExDescObj2, pOut_CollisionProb);

    return bResultOk;
}

/* **************************************************************************** 

  @fn           EMPCalcObjToTrajRelation   */ /*!

                                     @brief        Calculates the Relation
                                   between a trajectory and an object

                                     @param[in]    pIn_KinEgo
                                     @param[in]    pIn_Obj
                                     @param[out]   pOut_ObjToTrajRelation

                                     @return       True if no error occurred


                                   ****************************************************************************
                                   */
boolean EMPCalcObjToEgoTrajRelation(
    const EMPKinEgo_t* pIn_KinEgo,
    const EMPObjDesc_t* pIn_Obj,
    EMPObjToTrajRelation_t* pOut_ObjToTrajRelation) {
    boolean bResultDistToTrajOk;
    boolean bResultVelToTrajOk;
    float32 fDistToTraj;
    float32 fDistOnTraj;

    float32 fVelToTraj;
    float32 fVelOnTraj;

    bResultDistToTrajOk =
        EMPCalcDistToTraj(pIn_KinEgo, pIn_Obj, &fDistToTraj, &fDistOnTraj);
    bResultVelToTrajOk =
        EMPCalcVelToTraj(pIn_KinEgo, pIn_Obj, &fVelToTraj, &fVelOnTraj);

    pOut_ObjToTrajRelation->fDistToTraj = fDistToTraj;
    pOut_ObjToTrajRelation->fDistToTrajVar =
        pIn_Obj->Kinematic.fPosYVar; /* As an Approximation */
    pOut_ObjToTrajRelation->fVelocityToTraj = fVelToTraj;
    pOut_ObjToTrajRelation->fVelocityToTrajVar =
        pIn_Obj->Kinematic.fVelYVar; /* As an Approximation */
    pOut_ObjToTrajRelation->fDistOnTraj = fDistOnTraj;
    pOut_ObjToTrajRelation->fDistOnTrajVar =
        pIn_Obj->Kinematic.fPosXVar; /* As an Approximation */
    pOut_ObjToTrajRelation->fVelocityOnTraj = fVelOnTraj;
    pOut_ObjToTrajRelation->fVelocityOnTrajVar =
        pIn_Obj->Kinematic.fVelXVar; /* As an Approximation */

    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/24 changed by
    // guotao 20200513 start
    // set VelocityToTrajectory to zero when AEB deceleration is triggered.
    // Consider yawrate error caused by acute deceleration
    // extern float g_fPreBrakeDecel;
    // if (g_fPreBrakeDecel < -2.0f) {
    //     pOut_ObjToTrajRelation->fVelocityToTraj = 0.f;
    // }
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/24 changed by
    // guotao 20200513 end
    return (bResultDistToTrajOk || bResultVelToTrajOk);
}

/* **************************************************************************** 

  @fn           EMPCalcDistToTraj   */ /*!

                                            @brief        Calculates the
                                          Distance to Trajectory for a given
                                          ObjectDescription

                                            @param[in]    pIn_KinEgo
                                            @param[in]    pIn_Obj
                                            @param[out]   pfOut_DistToTraj
                                            @param[out]   pfOut_DistOnTraj

                                            @return       True if no error
                                          occurred


                                          ****************************************************************************
                                          */
static boolean EMPCalcDistToTraj(const EMPKinEgo_t* pIn_KinEgo,
                                 const EMPObjDesc_t* pIn_Obj,
                                 float32* pfOut_DistToTraj,
                                 float32* pfOut_DistOnTraj) {
    boolean bResultOk = TRUE;

    float32 fMinDistTime, fEuclideanDist;
    EMPPos2D_t fPosEgo, fPosObj;
    EMPTrajPred_t sEgoTrajPred;
    EMPTrajPred_t sObjTrajPred;

    EMPResetTrajPred(&sEgoTrajPred);

    /* Predict Ego Trajectory */
    EMPPredictEgoTraj(pIn_KinEgo, EMP_MANEUVER_KinematicsUnchanged,
                      &sEgoTrajPred);

    /* Fill Obj of current TimeStep */

    EMPResetTrajPred(&sObjTrajPred);
    sObjTrajPred.XofT.fC0 = pIn_Obj->Kinematic.fPosX;
    sObjTrajPred.YofT.fC0 = pIn_Obj->Kinematic.fPosY;

    bResultOk &=
        EMPCalcMinDistTime(&sEgoTrajPred, &sObjTrajPred, &fMinDistTime);

    fEuclideanDist =
        EMPCalcObjObjDistAtTime(fMinDistTime, &sEgoTrajPred, &sObjTrajPred);
    EMPCalcPositionAtTime(fMinDistTime, &sEgoTrajPred, &fPosEgo);
    EMPCalcPositionAtTime(fMinDistTime, &sObjTrajPred, &fPosObj);

    if ((fMinDistTime < EMP_PREDICTION_TIME_EGO_MAX) &&
        (fMinDistTime > C_F32_DELTA)) {
        if (fPosEgo.fY > fPosObj.fY) {
            fEuclideanDist *= -1.f;
        }
        *pfOut_DistToTraj =
            fEuclideanDist; /* DistToTraj is signed to indicate the side */
        *pfOut_DistOnTraj =
            (fMinDistTime * fMinDistTime) * sEgoTrajPred.XofT.fC2 +
            fMinDistTime * sEgoTrajPred.XofT.fC1 + sEgoTrajPred.XofT.fC0;
    } else {
        /* Object is temporally too far away from ego vehicle to calculate a
         * reasonable DistToTraj */
        if (fPosEgo.fY > fPosObj.fY) {
            *pfOut_DistToTraj = -EMP_DIST_TO_TRAJ_MAX;
        } else {
            *pfOut_DistToTraj = EMP_DIST_TO_TRAJ_MAX;
        }
        bResultOk = FALSE;
        /* Object is temporally too far away from ego vehicle to calculate a
         * reasonable DistOnTraj */
        if (fPosEgo.fX > fPosObj.fX) {
            *pfOut_DistOnTraj = -EMP_DIST_ON_TRAJ_MAX;
        } else {
            *pfOut_DistOnTraj = EMP_DIST_ON_TRAJ_MAX;
        }
    }

    return bResultOk;
}

/* **************************************************************************** 

  @fn           EMPCalcVeloToTraj   */ /*!

                                            @brief        Calculates the
                                          Velocity to Trajectory for a given
                                          HistDistToTraj
                                                          calculated by
                                          EMPCalcHistDistToTraj

                                            @param[in]    pIn_KinEgo
                                            @param[in]    pIn_Obj
                                            @param[out]   pfOut_VelToTraj
                                            @param[out]   pfOut_VelOnTraj

                                            @return       True if no error
                                          occurred


                                          ****************************************************************************
                                          */
static boolean EMPCalcVelToTraj(const EMPKinEgo_t* pIn_KinEgo,
                                const EMPObjDesc_t* pIn_Obj,
                                float32* pfOut_VelToTraj,
                                float32* pfOut_VelOnTraj) {
    boolean bResultOk = TRUE;

    EMPTrajPred_t sObjTrajPred;
    float32 fMinDistTime;
    EMPVel2D_t fVelObj;
    EMPTrajPred_t sEgoTrajPred;

    static const BML_t_TrafoMatrix2D
        RotMatrix =  // BML_GetTrafoMatrixByAngle(BML_f_Pi/2.0f);
        {
            ROTATION_, 0, /*cos(Pi/2)*/
            0,            /*Offset x*/
            1.0f,         /*sin(Pi/2)*/
            0             /*Offset x*/
        };

    /* Predict Ego Trajectory */
    EMPResetTrajPred(&sEgoTrajPred);
    EMPPredictEgoTraj(pIn_KinEgo, EMP_MANEUVER_KinematicsUnchanged,
                      &sEgoTrajPred);

    /* Fill Obj of current TimeStep */
    EMPResetTrajPred(&sObjTrajPred);
    sObjTrajPred.XofT.fC0 = pIn_Obj->Kinematic.fPosX;
    sObjTrajPred.YofT.fC0 = pIn_Obj->Kinematic.fPosY;
    fVelObj.fVelX = pIn_Obj->Kinematic.fVelX;
    fVelObj.fVelY = pIn_Obj->Kinematic.fVelY;

    bResultOk &=
        EMPCalcMinDistTime(&sEgoTrajPred, &sObjTrajPred, &fMinDistTime);

    {
        /* calculate ego velocity vector at CurrMinDistTime for orientation of
         * ego coordinate x' axes at that time */
        const float32 fEgoVelAtTime_x =
            sEgoTrajPred.XofT.fC1 + (2.0f * sEgoTrajPred.XofT.fC2);
        const float32 fEgoVelAtTime_y =
            sEgoTrajPred.YofT.fC1 + (2.0f * sEgoTrajPred.YofT.fC2);

        /* calculate orthogonal vector for orientation of ego coordinate y' axes
         * at that time */
        const float32 fEgoVecOrthAtTime_x = (RotMatrix.f00 * fEgoVelAtTime_x) -
                                            (RotMatrix.f10 * fEgoVelAtTime_y);
        const float32 fEgoVecOrthAtTime_y = (RotMatrix.f10 * fEgoVelAtTime_x) +
                                            (RotMatrix.f00 * fEgoVelAtTime_y);

        /* length of y' vector */
        const float32 fEgoVecOrthLength =
            SQRT(SQR(fEgoVecOrthAtTime_x) + SQR(fEgoVecOrthAtTime_y));

        /* check division by zero */
        if (fABS(fEgoVecOrthLength) < C_F32_DELTA) {
            *pfOut_VelToTraj = 0;
            *pfOut_VelOnTraj = 0;
        } else {
            /* projection of object velocity on vector of ego coordinate y' axes
             * or x axes */
            *pfOut_VelToTraj = ((fVelObj.fVelX * fEgoVecOrthAtTime_x) +
                                (fVelObj.fVelY * fEgoVecOrthAtTime_y)) /
                               fEgoVecOrthLength;
            *pfOut_VelOnTraj = ((fVelObj.fVelX * fEgoVecOrthAtTime_y) +
                                (fVelObj.fVelY * fEgoVecOrthAtTime_x)) /
                               fEgoVecOrthLength;
        }
    }
    return bResultOk;
}

/* ***********************************************************************
  @fn            EMPResetObjDesc  */ /*!

                                        @brief         Resets all values of
                                      given EMP Object Description

                                        @param[out]    pIn_Obj

                                        @return        void

                                      ****************************************************************************
                                      */
void EMPResetObjDesc(EMPObjDesc_t* pIn_Obj) {
    pIn_Obj->bExists = FALSE;
    pIn_Obj->Kinematic.fPosX = 0;
    pIn_Obj->Kinematic.fPosY = 0;
    pIn_Obj->Kinematic.fPosXVar = 0;
    pIn_Obj->Kinematic.fPosYVar = 0;
    pIn_Obj->Kinematic.fVelX = 0;
    pIn_Obj->Kinematic.fVelY = 0;
    pIn_Obj->Kinematic.fVelXVar = 0;
    pIn_Obj->Kinematic.fVelYVar = 0;
}

/* ***********************************************************************
  @fn            EMPResetTrajPred  */ /*!

                                       @brief         Resets all values of given
                                     EMP Trajectory Prediction

                                       @param[out]    pIn_TrajPred

                                       @return        void

                                     ****************************************************************************
                                     */
void EMPResetTrajPred(EMPTrajPred_t* pIn_TrajPred) {
    pIn_TrajPred->XofT.fC0 = 0;
    pIn_TrajPred->XofT.fC1 = 0;
    pIn_TrajPred->XofT.fC2 = 0;

    pIn_TrajPred->YofT.fC0 = 0;
    pIn_TrajPred->YofT.fC1 = 0;
    pIn_TrajPred->YofT.fC2 = 0;

    pIn_TrajPred->VarXofT.fC0 = 0;
    pIn_TrajPred->VarXofT.fC1 = 0;
    pIn_TrajPred->VarXofT.fC2 = 0;

    pIn_TrajPred->VarYofT.fC0 = 0;
    pIn_TrajPred->VarYofT.fC1 = 0;
    pIn_TrajPred->VarYofT.fC2 = 0;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */