/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
#include "si_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/
/* predicted distance for the ARS Object CAR */
#define SI_PRED_DIST_FOR_ARSOBJ_CAR (0.9f)
/* predicted displacement To Course Standard for the ARS Object TRUCK */
#define SI_PRED_DIST_FOR_ARSOBJ_TRUCK (1.25f)
/* predicted distance for the object apart from CAR or Truck*/
#define SI_PRED_DIST_FOR_OBJECT_OTHER_THAN_CAR_OR_TRUCK (0.4f)
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

SET_MEMSEC_CONST(SICutOutPredTime)
static const GDBLFunction_t SICutOutPredTime = {

    SI_CUT_OUT_PRED_MAX, SI_CUT_OUT_PRED_MIN,
    ((SI_CUT_OUT_PRED_MIN - SI_CUT_OUT_PRED_MAX) /
     (SI_CUT_OUT_DIST_MAX - SI_CUT_OUT_DIST_MIN)),
    SI_CUT_OUT_PRED_MAX - (((SI_CUT_OUT_PRED_MIN - SI_CUT_OUT_PRED_MAX) /
                            (SI_CUT_OUT_DIST_MAX - SI_CUT_OUT_DIST_MIN)) *
                           SI_CUT_OUT_DIST_MIN)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    SICalcPredDisplToCourseStandard */
void SICalcPredDisplToCourseStandard(fTime_t prediction_time,
                                     Envm_t_CR_Classification object_class,
                                     ObjNumber_t obj_id,
                                     SIPredictedDistance_t* pred_dist) {
    float32 DistanceToTraj;
    float32 DistanceToTrajVar;
    float32 VRelToTraj;
    float32 VRelToTrajVar;
    float32 p_dist_var;
    float32 p_dist_var_fullpred;
    float32 p_dist;
    float32 fVRelToTrajVarFac = SI_VARIANCE_SCALE_FACTOR;

    /* Get actual distance to course and the gradient (Vrel to course) */
    SITrajGetObjToRefDistance(obj_id, &DistanceToTraj, &DistanceToTrajVar);
    SITrajGetObjToRefDistanceGradient(obj_id, &VRelToTraj, &VRelToTrajVar);

    /* Calculate displacement now */
    /* The predicted displacement is the actual position plus the product of
     * gradient and prediction time */
    p_dist = DistanceToTraj + (prediction_time * VRelToTraj);
    p_dist_var = DistanceToTrajVar +
                 (SQR(prediction_time) * fVRelToTrajVarFac * VRelToTrajVar);
    p_dist_var_fullpred =
        DistanceToTrajVar +
        (SQR(SiLaneAssPredTime) * fVRelToTrajVarFac * VRelToTrajVar);

    /*zero crossing?*/
    if ((p_dist * DistanceToTraj) < 0.f) {
        p_dist = 0.f;
    } else {
        p_dist = fABS(p_dist);
    }

    /* respect object width !!!! */
    if (object_class == CR_OBJCLASS_CAR) {
        /* object seems to be a car, so calculate an object width of 1.8m and
         * subtract half of it off calculated distance*/
        p_dist -= SI_PRED_DIST_FOR_ARSOBJ_CAR;
    } else if (object_class == CR_OBJCLASS_TRUCK) {
        p_dist -= SI_PRED_DIST_FOR_ARSOBJ_CAR; /* @todo: FK urspruenglich 1.25,
                        wegen Problemen mit Klassifizierung aber PKW und LKW
                        jetzt gleich,
                        wenn Klassifizierung und Objektbreite plausibel, neu
                        bewerten */
    } else {
        p_dist -= SI_PRED_DIST_FOR_OBJECT_OTHER_THAN_CAR_OR_TRUCK;
    }

    pred_dist->pdist = p_dist;
    pred_dist->pdist_var = p_dist_var;
    pred_dist->pdist_var_fullpred = p_dist_var_fullpred;
}

/*************************************************************************************************************************
  Functionname:    SILimitPredictionBySpeed */
fTime_t SILimitPredictionBySpeed(fTime_t normal_prediction_time) {
    fTime_t limited_prediction_time;
    fVelocity_t vehicle_speed;

    vehicle_speed = EGO_SPEED_X_OBJ_SYNC;

    if (vehicle_speed > SI_MAX_VEHICLE_SPEED_LIMIT_PREDICTION) {
        limited_prediction_time = normal_prediction_time;
    } else {
        limited_prediction_time = SiCutInPotPredTimeLimited;
    }

    return (limited_prediction_time);
}

/*************************************************************************************************************************
  Functionname:    SILimitPredictionTimeDist */
fTime_t SILimitPredictionTimeDist(ObjNumber_t obj_id) {
    fTime_t limited_prediction_time;
    fVelocity_t vehicle_speed;

    vehicle_speed = EGO_SPEED_X_OBJ_SYNC;

    if (fABS(vehicle_speed) > C_F32_DELTA) {
        limited_prediction_time = dGDBmathLineareFunktion(
            &SICutOutPredTime, (OBJ_LONG_DISPLACEMENT(obj_id) / vehicle_speed));
    } else {
        limited_prediction_time = 0.0f;
    }

    return (limited_prediction_time);
}

/*************************************************************************************************************************
  Functionname:    SICalcPredDisplToCutOut */
void SICalcPredDisplToCutOut(fTime_t prediction_time,
                             Envm_t_CR_Classification object_class,
                             ObjNumber_t obj_id,
                             SIPredictedDistance_t* pred_dist) {
    float32 DistanceToTraj;
    float32 DistanceToTrajVar;
    float32 VRelToTraj;
    float32 VRelToTrajVar;
    float32 p_dist_var;
    float32 p_dist_var_fullpred;
    float32 p_dist;
    float32 fVRelToTrajVarFac = SI_VARIANCE_SCALE_FACTOR;

    /* Get actual distance to course and the gradient (Vrel to course) */
    SITrajGetObjToRefDistance(obj_id, &DistanceToTraj, &DistanceToTrajVar);
    SITrajGetObjToRefDistanceGradient(obj_id, &VRelToTraj, &VRelToTrajVar);

    /* Calculate displacement now */
    /* The predicted displacement is the actual position plus the product of
     * gradient and prediction time */
    p_dist = DistanceToTraj + (prediction_time * VRelToTraj);
    p_dist_var = DistanceToTrajVar +
                 (SQR(prediction_time) * fVRelToTrajVarFac * VRelToTrajVar);
    p_dist_var_fullpred =
        DistanceToTrajVar +
        (SQR(SI_CUT_OUT_PRED_MAX) * fVRelToTrajVarFac * VRelToTrajVar);

    p_dist = fABS(p_dist);

    /* respect object width !!!! */
    if (object_class == CR_OBJCLASS_CAR) {
        /* object seems to be a car, so calculate an object width of 1.8m and
         * subtract half of it off calculated distance*/
        p_dist -= SI_PRED_DIST_FOR_ARSOBJ_CAR;
    } else if (object_class == CR_OBJCLASS_TRUCK) {
        p_dist -= SI_PRED_DIST_FOR_ARSOBJ_TRUCK; /* @todo: FK urspruenglich
                        1.25, wegen Problemen mit Klassifizierung aber PKW und
                        LKW jetzt gleich,
                        wenn Klassifizierung und Objektbreite plausibel, neu
                        bewerten */
    } else {
        p_dist -= SI_PRED_DIST_FOR_OBJECT_OTHER_THAN_CAR_OR_TRUCK;
    }

    pred_dist->pdist = p_dist;
    pred_dist->pdist_var = p_dist_var;
    pred_dist->pdist_var_fullpred = p_dist_var_fullpred;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
