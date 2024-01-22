/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * guotao <guotao1@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
/*! @brief       CD_SOLUTION_ARRAY_SIZE
    @general     CD Solution Array size
*/
#define CD_SOLUTION_ARRAY_SIZE (2u)
/*! Local variable for safety offset for TTC */

/*! @brief       CD_CALC_TTC_SAFE_TIME_LONG
    @general     Colission Detection TimeToCollide safe time
*/
#define CD_CALC_TTC_SAFE_TIME_LONG 1.f
/*! @brief       CD_STD_NO_TTC_CALC
    @general     Colission Detection NO Calculation for TimeToCollide
*/
#define CD_STD_NO_TTC_CALC (-1.f)

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION DECLARATIONS
*****************************************************************************/
static boolean CDSolveQuadrEquation(float32 const fA,
                                    float32 const fB,
                                    float32 const fC,
                                    float32 const fDefVal,
                                    float32 Sol[],
                                    uint8* const pSolNum);

static float32 CDSelectMinValue(float32 const fArray[],
                                uint32 ArrayLen,
                                boolean OnlyPositive);

static boolean CDCalculateTTBCore(CDMovement_t const* const pEgoMovement,
                                  CDMovement_t const* const pObjMovement,
                                  fAccel_t const fEgoAccelAssumed,
                                  fTime_t* const pTTCBrake,
                                  fTime_t* const pTTB);

static boolean CDCalculateTTBStandstill(const CDMovement_t* pEgoMovement,
                                        const CDMovement_t* pObjMovement,
                                        fAccel_t fEgoAccelAssumed,
                                        fTime_t fTTC,
                                        fTime_t* pfTTB);

static boolean CDCalculateTTCCore(CDMovement_t const* const pRelMovement,
                                  fTime_t* const pTTC);

static void CDCalculateTTSDynCore(CDMovement_t const* const pEgoMovement,
                                  CDMovement_t const* const pObjMovement,
                                  fAccel_t const fEgoAccelAssumed,
                                  fDistance_t fLateralMinOffset,
                                  fDistance_t const fDistanceToPass,
                                  fTime_t fTime,
                                  fTime_t* const pTTS,
                                  const boolean bUseSteeringChange);

static void CDCalculateAnecLatCore(fDistance_t fRelPositionY,
                                   fDistance_t fLateralOffset,
                                   fTime_t fTime,
                                   fAccel_t* pAnecLat);

static fDistance_t CDCalculateDistance(fDistance_t fDistance,
                                       fVelocity_t fVelocity,
                                       fAccel_t fAcceleration,
                                       fTime_t fTime);

static fVelocity_t CDCalculateVelocity(fVelocity_t fVelocity,
                                       fAccel_t fAcceleration,
                                       fTime_t fTime);

static ObjNumber_t CDCheckClosest(const CDInternalObjectList_t* allObjInternal,
                                  const CDObjectData_t* const pObjectData,
                                  fDistance_t* pFreeSpace,
                                  boolean bSearchLeft,
                                  ObjNumber_t actObj,
                                  ObjNumber_t critObj,
                                  fDistance_t fSafetyOffsetLong);

static void CDCheckTimesLatMovValid(CDInternalObjectList_t* allObjInternal,
                                    const CDObjectData_t* const pObjectData);

static ObjNumber_t CDCheckMostCritObj(
    const CDInternalObjectList_t* allObjInternal,
    const CDObjectData_t* const pObjectData);

static ObjNumber_t CDCheckEscapeObj(
    const CDInternalObjectList_t* allObjInternal,
    const CDObjectData_t* const pObjectData,
    fDistance_t fSafetyOffsetLat,
    fDistance_t fSafetyOffsetLong,
    ObjNumber_t actObj);

static boolean CDCheckEscObjLong(const CDInternalObjectList_t* allObjInternal,
                                 const CDObjectData_t* const pObjectData,
                                 fTime_t TTS,
                                 ObjNumber_t actObj,
                                 ObjNumber_t critObj,
                                 fDistance_t fSafetyOffsetLong);

static fAccel_t CDCalcALat(const fDistance_t fNecLatOffset, float32 fEgoVel);

static void CDCalculateTrajRelation(ObjNumber_t iObjectIndex,
                                    const CDInternalStatus_t* pInternalStatus,
                                    const CDMovement_t* pObjMovement);
static void limitTTCBrake(fTime_t* const pTTCBrake);

/*****************************************************************************
  FUNCTION DEFINITIONS
*****************************************************************************/

/* ****************************************************************************
  @fn              CDInitInternalObjData                      */ /*!
                 @brief           Initialization Function
                 @description     This Function is used to Initialize the CD
               internal Object data Structure CDInternalObject_t
                 @param[in,out]   pInternalObjData
                 @return          [none]
                 @pre             [none]
                 @post            [none]
               ****************************************************************************
               */
void CDInitInternalObjData(CDInternalObject_t* pInternalObjData) {
    pInternalObjData->fVAbsX = 0; /* Only needed for SIL-SIL (always written) */
    pInternalObjData->fAAbsX = 0; /* Only needed for SIL-SIL (always written) */
    // Time to collision (assuming that the vehicle is in course)
    // w/accelerations
    pInternalObjData->TTC = CD_TIME_MAX;
    // Time to collision 2 (assuming that the vehicle is in course) w/o object
    // acceleration
    pInternalObjData->TTC2 = CD_TIME_MAX;
    // Time to collision 3 (assuming that the vehicle is in course w/o
    // accelerations
    pInternalObjData->TTC3 = CD_TIME_MAX;
    // Time to collision 4 (assuming that the vehicle is in course w/o
    // accelerations, object velocity
    pInternalObjData->TTC4 = CD_TIME_MAX;
    pInternalObjData->LongNecAccel =
        0; /* Necessary longitudinal acceleration to avoid a collision */
    pInternalObjData->LatNecAccel =
        0; /* Necessary lateral acceleration to avoid a collision */
    pInternalObjData->TTBPre =
        CD_TIME_MAX; /* time to brake for comfort braking */
    pInternalObjData->TTBAcute =
        CD_TIME_MAX; /* time to brake for emergency braking */
    pInternalObjData->TTSPre =
        CD_TIME_MAX; /* time to steer for comfort steering*/
    pInternalObjData->TTSAcute =
        CD_TIME_MAX; /* time to steer for emergency steering*/
    pInternalObjData->ClosingVelocity =
        0; /* Relative speed at time of collision (not AUTOSAR) */
    pInternalObjData->TTSPreRight =
        CD_TIME_MAX; /* time to steer for comfort steering right */
    pInternalObjData->TTSPreLeft =
        CD_TIME_MAX; /* time to steer for comfort steering left */
    pInternalObjData->TTSAcuteRight =
        CD_TIME_MAX; /* time to steer for emergency steering right*/
    pInternalObjData->TTSAcuteLeft =
        CD_TIME_MAX; /* time to steer for emergency steering left */

    // Necessary lateral acceleration to right side to avoid a collision
    pInternalObjData->LatNecAccelRight = 0;
    // Necessary lateral acceleration to left side to avoid a collision
    pInternalObjData->LatNecAccelLeft = 0;

    pInternalObjData->TrajRelation.fDistToTraj = 0;
    pInternalObjData->TrajRelation.fDistToTrajVar = 0;
    pInternalObjData->TrajRelation.fVelocityToTraj = 0;
    pInternalObjData->TrajRelation.fVelocityToTrajVar = 0;
}

/* **********************************************************************
  @fn            CDSolveQuadrEquation                               */ /*!

     @brief         Solves a quadratic equation

     @description   Solves a quadratic equation equal to f(x) = fA x^2 + fB x +
   fC

     @param[in]     fA Coefficient for x^2
     @param[in]     fB Coefficient for x
     @param[in]     fC Offset coefficient
     @param[in]     fDefVal Default value
     @param[in]     Sol Solution array
     @param[in,out] *pSolNum Number of solutions

     @return        FALSE if error occured

     @pre           [none]

     @post          [none]

   ****************************************************************************
   */
static boolean CDSolveQuadrEquation(float32 const fA,
                                    float32 const fB,
                                    float32 const fC,
                                    float32 const fDefVal,
                                    float32 Sol[],
                                    uint8* const pSolNum) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    float32 fDiscriminant;

    /* pointers are valid */
    if ((Sol != NULL) && (*pSolNum >= CD_SOLUTION_ARRAY_SIZE)) {
        /* set default values */
        Sol[0] = fDefVal;
        Sol[1] = fDefVal;

        /* equation reduces to linear one */
        if (fABS(fA) < C_F32_DELTA) {
            /* equation is only constant */
            if (fABS(fB) < C_F32_DELTA) {
                /* every value is a solution because same equations */
                if (fABS(fC) < C_F32_DELTA) {
                    Sol[0] = 0;
                    *pSolNum = 1;
                    /* no solution because different offsets */
                } else {
                    /* default values have already been set */
                }
                // one solution because quadratic equation reduces to linear
                // equation
            } else {
                Sol[0] = -fC / fB;
                *pSolNum = 1;
            }
            /* solve quadratic equation */
        } else {
            /*general solution of the quadratic equation: sol_0,1=[-fB +-
             * sqrt(fB^2-4*fA*fC)]/(2*fA)*/
            fDiscriminant = SQR(fB) - (4.0f * fA * fC);
            /* double solution because discriminant is zero */
            if (fABS(fDiscriminant) < C_F32_DELTA) {
                Sol[0] = -fB / (2.0f * fA);
                Sol[1] = Sol[0];
            } else {
                /* no solution because discriminant is less than zero */
                if (fDiscriminant < 0) {
                    /* default values have already been set */
                } else {
                    /* two solutions for positive discriminant */
                    /* special case to safeguard vs numerical effects*/
                    if (fABS(fC) < C_F32_DELTA) {
                        Sol[0] = 0;
                        Sol[1] = -fB / fA;
                    } else {
                        Sol[0] = (-fB + SQRT(fDiscriminant)) / (2.0f * fA);
                        Sol[1] = (-fB - SQRT(fDiscriminant)) / (2.0f * fA);
                    }
                }
            }
        }
    } else {
        /* pointer error */
        bReturn = FALSE;
    }

    /* return */
    return bReturn;
}

/* **********************************************************************
  @fn            CDSelectMinValue                                     */ /*!

   @brief         Selects minimum value in an array.

   @description   Selects minimum (positive) value in an array.

   @param[in]     fArray[] Array of floats
   @param[in]     ArrayLen Number of array elements
   @param[in]     OnlyPositive Flag if only positive values should be considered

   @return        Minimum (positive) value from array

   @pre           [none]

   @post          [none]

 **************************************************************************** */
static float32 CDSelectMinValue(float32 const fArray[],
                                uint32 ArrayLen,
                                boolean OnlyPositive) {
    /* local variable(s) */
    float32 fVal = CD_TIME_MAX;
    uint32 ii;

    for (ii = 0u; ii < ArrayLen; ii++) {
        if ((!(fArray[ii] < 0)) || (OnlyPositive != TRUE)) {
            fVal = MIN_FLOAT(fVal, fArray[ii]);
        }
    }

    /* return */
    return fVal;
}

/* **********************************************************************
  @fn            CDCalculateVehStopTime */ /*!

                                 @brief         Calculates the time when vehicle
                               (ego or object) will stop

                                 @description   Calculates the time when the
                               velocity is zero based on current
                                                velocity and acceleration


                                 @param[in]     fVelocity The current vehicle
                               velocity
                                 @param[in]     fAcceleration The current
                               vehicle acceleration
                                 @param[out]    *pfStopTime  The calculated
                               vehicle stop time

                                 @return        FALSE if error occured

                                 @pre           [none]

                                 @post          Value of *pStopTime is in [0,
                               CD_TIME_MAX]

                               ****************************************************************************
                               */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateVehStopTime(fVelocity_t fVelocity,
                               fAccel_t fAcceleration,
                               fTime_t* pfStopTime) {
    /* local variable */
    boolean bReturn = TRUE;

    /* check pointer */
    if (pfStopTime != NULL) {
        *pfStopTime = CD_TIME_MAX;

        /* no acceleration */
        if (fABS(fAcceleration) < C_F32_DELTA) {
            /* no velocity so vehicle already has stopped */
            if (fABS(fVelocity) < C_F32_DELTA) {
                *pfStopTime = 0;
            } else {
                // velocity exists but no acceleration so vehicle will never
                // top
                *pfStopTime = CD_TIME_MAX;
            }
        } else {
            /* acceleration and velocity exist so either vehicle has stopped or
               will stop */
            *pfStopTime = -fVelocity / fAcceleration;
        }

        /* if vehicle already stops/has stopped or if value is too high stop
         * time is maximum value */
        if ((*pfStopTime < 0) || (*pfStopTime > CD_TIME_MAX)) {
            *pfStopTime = CD_TIME_MAX;
        }
    } else {
        /* pointer error */
        bReturn = FALSE;
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateTTCCore */ /*!

                                       @brief        Calculates TTC for ego
                                     vehicle and object

                                       @description  TTC is calculated without
                                     considering possible vehicle stop in
                                                     longitudinal direction.

                                       @param[in]    pRelMovement Relative
                                     movement of vehicles
                                       @param[out]   pTTC Calculated TTC value

                                       @return       FALSE if error has occured
                                       @pre          [none]
                                       @post         [none]

                                     ****************************************************************************
                                     */
static boolean CDCalculateTTCCore(CDMovement_t const* const pRelMovement,
                                  fTime_t* const pTTC) {
    /* local variable(s) */
    boolean bReturn = TRUE;

    fTime_t fTimeSol[CD_SOLUTION_ARRAY_SIZE];

    uint8 TimeSolNum = sizeof(fTimeSol) / sizeof(fTime_t);

    /* set default value for TTC */
    *pTTC = CD_TIME_MAX;

    /* vehicles have already collided, this is not realistic */
    if (pRelMovement->fX < 0) {
        *pTTC = 0;
        bReturn = FALSE;
    } else {
        /* calculate TTC values and select minimum positive value */
        /* Solve equation of motion: 1/2*a*t^2+v*t+s=0 */
        (void)CDSolveQuadrEquation(0.5f * pRelMovement->fAx, pRelMovement->fVx,
                                   pRelMovement->fX, CD_TIME_MAX, fTimeSol,
                                   &TimeSolNum);

        *pTTC = CDSelectMinValue(fTimeSol, TimeSolNum, TRUE);

        /* limit TTC */
        if ((*pTTC < 0) || (*pTTC > CD_TIME_MAX)) {
            *pTTC = CD_TIME_MAX;
        }
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateTTC */ /*!

                                           @brief        Calculates TTC for ego
                                         vehicle and object

                                           @description  TTC is calculated
                                         considering possible vehicle stop in
                                                         longitudinal direction.

                                           @param[in]    pEgoMovement Ego
                                         kinematic values
                                           @param[in]    pObjMovement Object
                                         kinematic values
                                           @param[out]   pfTTC Calculated TTC
                                         value
                                           @return       FALSE if invalid input
                                         values have been set

                                           @pre          [none]

                                           @post         [none]

                                         ****************************************************************************
                                         */
/*this function is part of catalog for common functions and can be used
 * externaly*/

boolean CDCalculateTTC(const CDMovement_t* pEgoMovement,
                       const CDMovement_t* pObjMovement,
                       fTime_t* pfTTC) {
    /* CD_USE_KINEMATIC_STDDEV */
    /* local variable(s) */
    boolean bReturn = TRUE;
    CDMovement_t RelMovement;
    fTime_t fTimeEgoStop;
    fTime_t fTimeObjStop;

    /* calculate relative movement of ego vehicle and object considering safety
     * distance */
    RelMovement.fX = pObjMovement->fX - pEgoMovement->fX;
    RelMovement.fVx = pObjMovement->fVx - pEgoMovement->fVx;
    RelMovement.fAx = pObjMovement->fAx - pEgoMovement->fAx;
    RelMovement.fY = pObjMovement->fY - pEgoMovement->fY;
    RelMovement.fVy = pObjMovement->fVy - pEgoMovement->fVy;
    RelMovement.fAy = pObjMovement->fAy - pEgoMovement->fAy;

    /* calculate TTC */
    (void)CDCalculateTTCCore(&RelMovement, pfTTC);
    /* CD_USE_KINEMATIC_STDDEV */

    /* calculate stop time for vehicles */
    (void)CDCalculateVehStopTime(pEgoMovement->fVx, pEgoMovement->fAx,
                                 &fTimeEgoStop);
    (void)CDCalculateVehStopTime(pObjMovement->fVx, pObjMovement->fAx,
                                 &fTimeObjStop);

    /* ego vehicle stops before object and collision */
    if ((fTimeEgoStop < fTimeObjStop) && (fTimeEgoStop < *pfTTC)) {
        /* if v_obj < 0 - oncoming object */
        if (pObjMovement->fVx < 0) {
            /* set relative movement as if ego has already been standing at
             * final position */
            RelMovement.fX =
                pObjMovement->fX -
                CDCalculateDistance(pEgoMovement->fX, pEgoMovement->fVx,
                                    pEgoMovement->fAx, fTimeEgoStop);
            RelMovement.fVx = pObjMovement->fVx;
            RelMovement.fAx = pObjMovement->fAx;

            (void)CDCalculateTTCCore(&RelMovement, pfTTC);
            /* CD_USE_KINEMATIC_STDDEV */

            /* object stops before collision with standing ego vehicle */
            if (fTimeObjStop < *pfTTC) {
                *pfTTC = CD_TIME_MAX;
            }
        } else {
            /* object is moving same direction */
            *pfTTC = CD_TIME_MAX;
        }
    } else if ((fTimeObjStop < fTimeEgoStop) && (fTimeObjStop < *pfTTC)) {
        /* object stops before ego vehicle and collision */
        /* set relative movement as if object has already been standing at final
         * position */
        RelMovement.fX =
            CDCalculateDistance(pObjMovement->fX, pObjMovement->fVx,
                                pObjMovement->fAx, fTimeObjStop) -
            pEgoMovement->fX;
        RelMovement.fVx = -pEgoMovement->fVx;
        RelMovement.fAx = -pEgoMovement->fAx;

        (void)CDCalculateTTCCore(&RelMovement, pfTTC);
        /* CD_USE_KINEMATIC_STDDEV */

        /* ego vehicle stops before collision with standing object */
        if (fTimeEgoStop < *pfTTC) {
            *pfTTC = CD_TIME_MAX;
        }
    } else {
        /* collision happens first so calculated time is correct */
    }

    /* return */
    return bReturn;
}

/* ****************************************************************************
  @fn              CDCalculateTTC2                      */ /*!
                       @brief           --
                       @param[in]       *pEgoMovement
                       @param[in]       *pObjMovement
                       @param[in,out]   pfTTC
                       @return          Boolean Value from FUnction
                     CDCalculateTTC
                       @pre             [ None ]
                       @post            [ None ]
                     ****************************************************************************
                     */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateTTC2(const CDMovement_t* pEgoMovement,
                        const CDMovement_t* pObjMovement,
                        fTime_t* pfTTC) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    CDMovement_t ObjMovementVariant;

    /* object movement without acceleration */
    ObjMovementVariant = *pObjMovement;
    ObjMovementVariant.fAx = 0;

    bReturn = CDCalculateTTC(pEgoMovement, &ObjMovementVariant, pfTTC);
    /* CD_USE_KINEMATIC_STDDEV */

    /* return */
    return bReturn;
}

/* ****************************************************************************
  @fn              CDCalculateTTC3                      */ /*!
                       @brief           --
                       @param[in]       *pEgoMovement
                       @param[in]       *pObjMovement
                       @param[in,out]   pfTTC
                       @return          Boolean Value from Function
                     CDCalculateTTC
                       @pre             [ None ]
                       @post            [ None ]
                       @author          --
                     ****************************************************************************
                     */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateTTC3(const CDMovement_t* pEgoMovement,
                        const CDMovement_t* pObjMovement,
                        fTime_t* pfTTC) {
    /* local variable(s) */
    boolean bReturn;
    CDMovement_t EgoMovementVariant;
    CDMovement_t ObjMovementVariant;

    /* ego and object movement without acceleration */
    EgoMovementVariant = *pEgoMovement;
    EgoMovementVariant.fAx = 0;
    ObjMovementVariant = *pObjMovement;
    ObjMovementVariant.fAx = 0;

    bReturn = CDCalculateTTC(&EgoMovementVariant, &ObjMovementVariant, pfTTC);
    /* CD_USE_KINEMATIC_STDDEV */

    /* return */
    return bReturn;
}

#if (CD_USE_HYPOTHESES_TTC4)
/* ****************************************************************************
  @fn              CDCalculateTTC4                      */ /*!
                       @brief           --
                       @param[in]      *pEgoMovement
                       @param[in]      *pObjMovement
                       @param[in,out]   pfTTC
                       @return          Boolean Value from Function
                     CDCalculateTTC
                       @pre             [ None ]
                       @post            [ None ]
                       @author          --
                     ****************************************************************************
                     */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateTTC4(const CDMovement_t* pEgoMovement,
                        const CDMovement_t* pObjMovement,
                        fTime_t* pfTTC) {
    /* local variable(s) */
    boolean bReturn;
    CDMovement_t EgoMovementVariant;
    CDMovement_t ObjMovementVariant;

    /* CD_USE_KINEMATIC_STDDEV */

    /* ego and object movement without acceleration and object movement without
     * velocity */
    EgoMovementVariant = *pEgoMovement;
    EgoMovementVariant.fAx = 0;
    ObjMovementVariant = *pObjMovement;
    ObjMovementVariant.fVx = 0;
    ObjMovementVariant.fAx = 0;

    bReturn = CDCalculateTTC(&EgoMovementVariant, &ObjMovementVariant, pfTTC);
#/* CD_USE_KINEMATIC_STDDEV */

    /* return */
    return bReturn;
}
#endif

/*************************************************************************************************************************
  Functionname:    CDCalculateTTBCore */ /*!

                                                                                    @brief         Calculates TTB without considering object standstill.

                                                                                    @description   Calculates TTB without considering object standstill.


                                                                                    @return        FALSE if error has occured

                                                                                    @param[in]     pEgoMovement Ego movement values
                                                                                    @param[in]     pObjMovement Object movement values
                                                                                    @param[in]     fEgoAccelAssumed Assumed ego acceleration when braking
                                                                                    @param[out]    pTTCBrake Time-To-Collision when braking
                                                                                    @param[out]    pTTB Time-To-Brake with assumed acceleration

                                                                                    @pre             [ None ]
                                                                                    @post            [ None ]
                                                                                  *************************************************************************************************************************/
static boolean CDCalculateTTBCore(CDMovement_t const* const pEgoMovement,
                                  CDMovement_t const* const pObjMovement,
                                  fAccel_t const fEgoAccelAssumed,
                                  fTime_t* const pTTCBrake,
                                  fTime_t* const pTTB) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    CDMovement_t RelMovementX;
    fAccel_t RelAccelBraking;
    fAccel_t EgoRelAccel;
    fTime_t TimeEqualVel;
    fDistance_t XRelEqualVel;
    float32 fA;
    float32 fB;
    float32 fC;
    float32 fDiscriminant;
    fTime_t fTTBtemp[CD_SOLUTION_ARRAY_SIZE];
    fTime_t fTTCBraketemp[CD_SOLUTION_ARRAY_SIZE];

    /* set default value for TTB and TTCBrake */
    *pTTB = CD_TIME_MAX;
    *pTTCBrake = CD_TIME_MAX;

    RelMovementX.fX = pObjMovement->fX - pEgoMovement->fX;
    RelMovementX.fVx = pObjMovement->fVx - pEgoMovement->fVx;
    RelAccelBraking = pObjMovement->fAx - fEgoAccelAssumed;

    EgoRelAccel = pEgoMovement->fAx - fEgoAccelAssumed;

    /* ego vehicle already stands and can't drive backwards */
    if (fABS(pEgoMovement->fVx) < C_F32_DELTA) {
        RelMovementX.fVx = pObjMovement->fVx;
        RelAccelBraking = pObjMovement->fAx;
        EgoRelAccel = 0;
    }

    /* DERIVATION 1): ego already brakes with at least assumed acceleration */
    if (EgoRelAccel < C_F32_DELTA) {
        (void)CDCalculateTTC(pEgoMovement, pObjMovement, pTTCBrake);
        /* CD_USE_KINEMATIC_STDDEV */

        *pTTB = (((*pTTCBrake) < CD_TIME_MAX) ? 0 : CD_TIME_MAX);
    } else {
        /* DERIVATION 2): ego with assumed acceleration brakes stronger than
         * before */
        /* DERIVATION 2.1): object brakes with assumed acceleration */
        if (fABS(RelAccelBraking) < C_F32_DELTA) {
            /* time when velocities are equal */
            TimeEqualVel = RelMovementX.fVx / EgoRelAccel;

            /* relative position when velocities are equal */
            XRelEqualVel = CDCalculateDistance(RelMovementX.fX, 0, EgoRelAccel,
                                               TimeEqualVel);
            /* DERIVATION 2.1.1): braking with assumed deceleration is not
             * sufficient to prevent collision */
            if (XRelEqualVel < C_F32_DELTA) {
                *pTTB = 0;

                (void)CDCalculateTTC(pEgoMovement, pObjMovement, pTTCBrake);
                /* CD_USE_KINEMATIC_STDDEV */

            } else {
                /* DERIVATION 2.1.2): braking when velocity is equal prevents
                   collision */
                *pTTB = TimeEqualVel;
                *pTTCBrake = CD_TIME_MAX;
            }
        } else {
            /* DERIVATION 2.2) */
            /* Solutions according to derivation */
            fA = SQR(EgoRelAccel) - (RelAccelBraking * EgoRelAccel);
            fB = -2.0f * RelMovementX.fVx * EgoRelAccel;
            fC = SQR(RelMovementX.fVx) -
                 (2.0f * RelAccelBraking * RelMovementX.fX);
            /* DERIVATION 2.2.1): fA == 0 */
            if (fABS(fA) < C_F32_DELTA) {
                /* DERIVATION 2.2.1.1): fB == 0 */
                if (fABS(fB) < C_F32_DELTA) {
                    /* DERIVATION 2.2.1.1.2 and 2.2.1.1.3) */
                    if (fC >= 0) {
                        /*the possibilities: Ego has either touched the object
                         * or braking cannont avoid the collision*/
                        *pTTB = 0;

                        (void)CDCalculateTTC(pEgoMovement, pObjMovement,
                                             pTTCBrake);

                    } else {
                        /* DERIVATION 2.2.1.1.1) */
                        /* no collision */
                        *pTTB = CD_TIME_MAX;
                        *pTTCBrake = CD_TIME_MAX;
                    }
                } else {
                    /* DERIVATION 2.2.1.2): fB != 0 */
                    *pTTB = -fC / fB;
                    *pTTCBrake =
                        -((RelMovementX.fVx - (EgoRelAccel * (*pTTB))) /
                          RelAccelBraking);
                }
            } else {
                /* DERIVATION 2.2.2): fA != 0 */
                /*general solution of the quadratic equation: sol_0,1=[-fB +-
                 * sqrt(fB^2-4*fA*fC)]/(2*fA)*/
                fDiscriminant =
                    SQR(fB) -
                    (4.0f * fA * fC); /*see solution of quadratic equation*/
                /* DERIVATION 2.2.2.2): fDiscriminant == 0 */
                if (fABS(fDiscriminant) < C_F32_DELTA) {
                    *pTTB = -fB /
                            (2.0f * fA); /*see solution of quadratic equation*/

                    *pTTCBrake = -(RelMovementX.fVx - (EgoRelAccel * (*pTTB))) /
                                 RelAccelBraking;

                } else {
                    /* fDiscriminant != 0 */
                    /* DERIVATION 2.2.2.3) */
                    if (fDiscriminant > 0) {
                        const float32 fSQRTDiscriminant = SQRT(fDiscriminant);
                        fTTBtemp[0] =
                            -(fB + fSQRTDiscriminant) /
                            (2.0f * fA); /*see solution of quadratic equation*/
                        fTTBtemp[1] =
                            -(fB - fSQRTDiscriminant) /
                            (2.0f * fA); /*see solution of quadratic equation*/

                        fTTCBraketemp[0] =
                            -((RelMovementX.fVx - (EgoRelAccel * fTTBtemp[0])) /
                              RelAccelBraking);
                        fTTCBraketemp[1] =
                            -((RelMovementX.fVx - (EgoRelAccel * fTTBtemp[1])) /
                              RelAccelBraking);

                        if ((fTTCBraketemp[0] < *pTTCBrake) &&
                            (fTTBtemp[0] < fTTCBraketemp[0])) {
                            *pTTB = fTTBtemp[0];
                            *pTTCBrake = fTTCBraketemp[0];
                        }
                        if ((fTTCBraketemp[1] < *pTTCBrake) &&
                            (fTTBtemp[1] < fTTCBraketemp[1])) {
                            *pTTB = fTTBtemp[1];
                            *pTTCBrake = fTTCBraketemp[1];
                        }
                    } else {
                        /* DERIVATION 2.2.2.1): fDiscriminant < 0 */
                        if (fC < 0) {
                            *pTTB = CD_TIME_MAX;
                            *pTTCBrake = CD_TIME_MAX;
                        } else {
                            /*fC can't be 0 as in this case fDiscriminant
                             wouldn't be negative*/
                            *pTTB = 0;
                            (void)CDCalculateTTC(pEgoMovement, pObjMovement,
                                                 pTTCBrake);
                        }
                    }
                }
            }
        }
    }

    // limit TTCBrake
    limitTTCBrake(pTTCBrake);

    /* limit TTB */
    if (*pTTB < 0) {
        *pTTB = (((*pTTCBrake) < CD_TIME_MAX) ? 0 : CD_TIME_MAX);
    } else {
        if (*pTTB > CD_TIME_MAX) {
            *pTTB = CD_TIME_MAX;
        }
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateTTBStandstill */ /*!

                                 @brief        Calculates TTB  on objects moving
                               towards Ego such that Ego
                                               is just standing at collision

                                 @description  TTB is calculated for oncoming
                               objects with assumed
                                               longitudinal ego acceleration.
                                               TTB is calculated such that Ego
                               is just standing at collision
                                               position. If Ego is standing
                               without active braking before
                                               the collision, then TTB will be
                               CD_TIME_MAX.
                                               =>Ego stop times are considered.
                                               Obj stop times aren't considered!
                               -> checked before function call

                                 @param[in]    pEgoMovement     Ego kinematic
                               values
                                 @param[in]    pObjMovement     Object kinematic
                               values
                                 @param[in]    fEgoAccelAssumed  Assumed ego
                               acceleration
                                 @param[in]    fTTC              TTC
                                 @param[out]   pfTTB            Calculated TTB

                                 @return       FALSE if error has happened

                                 @pre          [none]

                                 @post         [none]

                               ****************************************************************************
                               */
static boolean CDCalculateTTBStandstill(const CDMovement_t* pEgoMovement,
                                        const CDMovement_t* pObjMovement,
                                        fAccel_t fEgoAccelAssumed,
                                        fTime_t fTTC,
                                        fTime_t* pfTTB) {
    boolean retValue = TRUE;
    CDMovement_t RelMovementX;

    float32 fA;
    float32 fB;
    float32 fC;
    float32 fTTBStandstillSol[CD_SOLUTION_ARRAY_SIZE];
    uint8 uTTBStandstillSolNum = sizeof(fTTBStandstillSol) / sizeof(float32);

    RelMovementX.fX = pObjMovement->fX - pEgoMovement->fX;
    RelMovementX.fVx = pObjMovement->fVx - pEgoMovement->fVx;
    RelMovementX.fAx = pObjMovement->fAx - pEgoMovement->fAx;

    /* equation: xObj - xEgo = fA * TTB^2 + fB *TTB + fC = 0
     * tBrake = -vEgoTTB/aBrake; vEgoTTB = vEgo0 + aEgo0*TTB; vObjTTB = vObj0 +
     * aObj0*TTB; xEgo   = xEgo0 + vEgo0*TTB + 0.5*aEgo0*TTB^2 + vEgoTTB*tBrake
     * + 0.5*aBrake*tBrake^2; xObj   = xObj0 + vObj0*TTB + 0.5*aObj0*TTB^2 +
     * vObjTTB*tBrake + 0.5*aObj0*tBrake^2;
     */
    /*see equation of motion above*/
    fA = (fEgoAccelAssumed * (pEgoMovement->fAx) *
          ((0.5f * (pEgoMovement->fAx)) - (pObjMovement->fAx))) +
         (SQR(fEgoAccelAssumed) * (0.5f * RelMovementX.fAx)) +
         (0.5f * SQR(pEgoMovement->fAx) * (pObjMovement->fAx));
    fB = ((pEgoMovement->fAx) * (pObjMovement->fAx) * (pEgoMovement->fVx)) +
         (SQR(fEgoAccelAssumed) * (RelMovementX.fVx)) +
         (fEgoAccelAssumed * (((pEgoMovement->fAx) * (pEgoMovement->fVx)) -
                              ((pObjMovement->fAx) * (pEgoMovement->fVx)) -
                              ((pEgoMovement->fAx) * (pObjMovement->fVx))));
    /*PRQA S 3121 1*/ /*see equation of motion above*/
    fC = (0.5f * (pObjMovement->fAx) * SQR(pEgoMovement->fVx)) +
         (fEgoAccelAssumed * (pEgoMovement->fVx) *
          ((0.5f * (pEgoMovement->fVx)) - (pObjMovement->fVx))) +
         (SQR(fEgoAccelAssumed) * (RelMovementX.fX));

    (void)CDSolveQuadrEquation(fA, fB, fC, CD_TIME_MAX, fTTBStandstillSol,
                               &uTTBStandstillSolNum);

    if (uTTBStandstillSolNum == CD_SOLUTION_ARRAY_SIZE) {
        /* both solutions == CD_TIME_MAX -> there wouldn't be a collision when
          not considering the standstill cases set TTB to CD_TIME_MAX because
          Ego will stand without braking before the collision*/
        if ((fTTBStandstillSol[0] > (CD_TIME_MAX - C_F32_DELTA)) &&
            (fTTBStandstillSol[1] > (CD_TIME_MAX - C_F32_DELTA))) {
            *pfTTB = CD_TIME_MAX;
            /* 0 <= TTB < CD_TIME_MAX*/
        } else {
            /* is at least one TTB < 0 => choose max(TTB[1],TTB[2]); both TTB >
             * 0 => choose min(TTB[1],TTB[2])*/
            (*pfTTB) =
                ((fTTBStandstillSol[0] < 0) || (fTTBStandstillSol[1] < 0))
                    ? (MAX_FLOAT(fTTBStandstillSol[0], fTTBStandstillSol[1]))
                    : (MIN_FLOAT(fTTBStandstillSol[0], fTTBStandstillSol[1]));
            /* is TTB in interval[0 CD_TIME_MAX], else set TTB to threshold*/
            (*pfTTB) = MINMAX_FLOAT(0, CD_TIME_MAX, *pfTTB);
            /* if TTB is greater than TTC, wrong value has been chosen; braking
             * should have begun in past => set TTB to 0  */
            (*pfTTB) = (*pfTTB < fTTC) ? (*pfTTB) : 0;
        }
    } else if (uTTBStandstillSolNum == 1) {
        (*pfTTB) = MINMAX_FLOAT(0, CD_TIME_MAX, fTTBStandstillSol[0]);
    } else {
        /* impossible case, due to uTTBStandstillSolNum=[1,2] */
        *pfTTB = 0;
    }
    return retValue;
}

/*this function is part of catalog for common functions and can be used
 * externaly*/
/* ***********************************************************************
  @fn           CDCalculateTTBDyn */ /*!

                                        @brief        Calculates TTB for assumed
                                      ego acceleration

                                        @description  TTB is calculated for
                                      current ego and object movement and
                                      assumed
                                                      longitudinal ego
                                      acceleration. Ego or object stop times are
                                      considered.

                                        @param[in]    pEgoMovement     Ego
                                      kinematic values
                                        @param[in]    pObjMovement     Object
                                      kinematic values
                                        @param[in]    fEgoAccelAssumed  Assumed
                                      ego acceleration
                                        @param[out]   pfTTB Calculated TTB
                                        @return       FALSE if error has
                                      happened

                                        @pre          [none]

                                        @post         [none]

                                      ****************************************************************************
                                      */
boolean CDCalculateTTBDyn(const CDMovement_t* pEgoMovement,
                          const CDMovement_t* pObjMovement,
                          fAccel_t fEgoAccelAssumed,
                          fTime_t* pfTTB) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    fTime_t fTTC;
    fTime_t fTTCBrake;
    fTime_t fTimeObjStop;

    /* calculate TTC */
    (void)CDCalculateTTC(pEgoMovement, pObjMovement, &fTTC);

    /* TTC is valid and Ego is moving forward*/
    if ((fTTC < CD_TIME_MAX) && (pEgoMovement->fVx > C_F32_DELTA)) {
        /* calculate stop time for Obj */
        (void)CDCalculateVehStopTime(pObjMovement->fVx, pObjMovement->fAx,
                                     &fTimeObjStop);

        /*object is standing or departing*/
        if (pObjMovement->fVx >= 0) {
            /* calculate TTB */
            (void)CDCalculateTTBCore(pEgoMovement, pObjMovement,
                                     fEgoAccelAssumed, &fTTCBrake, pfTTB);
            if (fTimeObjStop < fTTCBrake) {
                CDMovement_t ObjMovementStopped;
                ObjMovementStopped = *pObjMovement;
                ObjMovementStopped.fX =
                    CDCalculateDistance(pObjMovement->fX, pObjMovement->fVx,
                                        pObjMovement->fAx, fTimeObjStop);
                ObjMovementStopped.fVx = 0;
                ObjMovementStopped.fAx = 0;
                (void)CDCalculateTTBCore(pEgoMovement, &ObjMovementStopped,
                                         fEgoAccelAssumed, &fTTCBrake, pfTTB);
            } else {
                // calculated TTB is valid
            }
            /* object is oncoming */
        } else {
            /* object stops before CD_TIME_MAX (pObjMovement->fAx > 0) */
            if (fTimeObjStop < CD_TIME_MAX) {
                fTime_t fTimeEgoStopBrake;
                fVelocity_t fVelTTB;
                CDMovement_t ObjMovementStopped;
                /* set Obj to stop position and calculate TTB */
                ObjMovementStopped = *pObjMovement;
                ObjMovementStopped.fX =
                    CDCalculateDistance(pObjMovement->fX, pObjMovement->fVx,
                                        pObjMovement->fAx, fTimeObjStop);
                ObjMovementStopped.fVx = 0;
                ObjMovementStopped.fAx = 0;

                (void)CDCalculateTTBCore(pEgoMovement, &ObjMovementStopped,
                                         fEgoAccelAssumed, &fTTCBrake, pfTTB);
                fVelTTB = CDCalculateVelocity(pEgoMovement->fVx,
                                              pEgoMovement->fAx, *pfTTB);
                /* Ego still moving forward at TTB? */
                if (fVelTTB >= 0) {
                    (void)CDCalculateVehStopTime(fVelTTB, fEgoAccelAssumed,
                                                 &fTimeEgoStopBrake);
                    /* will Ego(AccelAssumed) stop before Obj?*/
                    if ((fTimeEgoStopBrake + (*pfTTB)) < fTimeObjStop) {
                        /*Obj won't stop before collision -> Ego standing at
                         * collision*/
                        (void)CDCalculateTTBStandstill(
                            pEgoMovement, pObjMovement, fEgoAccelAssumed, fTTC,
                            pfTTB);
                    } else {
                        /*Obj stops before Ego(Assumed) -> set Obj to stop
                         * position ->calculated TTB is valid*/
                    }
                    /* Ego can't move backwards => Ego standing before collision
                     * without braking*/
                } else {
                    *pfTTB = CD_TIME_MAX;
                }
            } else {
                /*Obj won't stop before collision -> Ego standing at collision*/
                (void)CDCalculateTTBStandstill(pEgoMovement, pObjMovement,
                                               fEgoAccelAssumed, fTTC, pfTTB);
            }
        }
    } else {
        /* Ego standing at collision or TTC is invalid (==CD_TIME_MAX) */
        *pfTTB = CD_TIME_MAX;
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateTTSDynCore */ /*!

                                    @brief        Calculates TTS without
                                  considering object standstill but
                                                  considering lateral movement.

                                    @description  Calculates TTS without
                                  considering object standstill but
                                                  considering lateral movement.

                                    @param[in]    pEgoMovement Ego movement
                                  values
                                    @param[in]    pObjMovement Object movement
                                  values
                                    @param[in]    fEgoAccelAssumed Assumed
                                  lateral acceleration
                                    @param[in]    fLateralMinOffset Wanted
                                  Minimum lateral position offset
                                    @param[in]    fDistanceToPass offset to pass
                                  when cross
                                    @param[in]    fTime Time when object should
                                  be passed (normally TTC)
                                    @param[out]   pTTS Calculated TTS
                                    @param[in]    bUseSteeringChange

                                    @return       void

                                    @pre          [none]

                                    @post         [none]

                                  ****************************************************************************
                                  */
static void CDCalculateTTSDynCore(CDMovement_t const* const pEgoMovement,
                                  CDMovement_t const* const pObjMovement,
                                  fAccel_t const fEgoAccelAssumed,
                                  fDistance_t fLateralMinOffset,
                                  fDistance_t const fDistanceToPass,
                                  fTime_t fTime,
                                  fTime_t* const pTTS,
                                  const boolean bUseSteeringChange) {
    /* local variable(s) */

    CDMovement_t predRelMovement;
    CDMovement_t predObjMovement;
    CDMovement_t ObjMovement;
    CDMovement_t predEgoMovement;
    fTime_t fTimeSavePassX;
    fDistance_t fDistYWhilePass;
    boolean fStopX;
    boolean fStopY;
    const float32 fAbsLongVelObj = fABS(pObjMovement->fVx);
    const float32 fAbsLatVelObj = fABS(pObjMovement->fVy);
    float32 fA;
    float32 fB;
    float32 fC;
    float32 fAccelYMax;
    float32 fSol[CD_SOLUTION_ARRAY_SIZE];
    uint8 SolNum = sizeof(fSol) / sizeof(float32);
    float32 fTotalEgoAccelAssumed = fEgoAccelAssumed;

    /* set default value */
    *pTTS = CD_TIME_MAX;
    fStopX = FALSE;
    fStopY = FALSE;

    /* @Description:
     * - First calculate actual relative movement and predicted movement
     *   at time of collision.
     * - Then calculate the time we need to pass the object path.
     * - Then calculate the object movement while passing the object.
     * - Decide if we will cross the object in front in moving direction.
     * - If that is the case add the lateral displacement on top of the
     *   safety distance. This is the new distance we have to steer in
     *   case of crossing in front of object.
     */

    /*use objectMovement without lateral acceleration*/
    ObjMovement = *pObjMovement;
    ObjMovement.fAy = 0;

    /* calculate relative and predicted movement */
    if (fAbsLongVelObj > fAbsLatVelObj) {
        fStopX = TRUE;
    } else {
        fStopY = TRUE;
    }
    /* use abs movement due to no rel movement in x direction does not mean
     * standstill... */
    CDPredictMovement(&ObjMovement, &predObjMovement, fTime, fStopX, fStopY);
    CDPredictMovement(pEgoMovement, &predEgoMovement, fTime, TRUE, FALSE);

    predRelMovement.fY = predObjMovement.fY - predEgoMovement.fY;
    predRelMovement.fVx = predObjMovement.fVx - predEgoMovement.fVx;

    /* calculate time to pass ( TTP ); in case of no solution for quadratic
     * equation fTimeSavePassX = 0 -> no passing */
    /* Solve equation of motion: 1/2*a*t^2+v*t+s=0 */

    /* reduce artifacts due to noisy measurements */
    if (fABS(predObjMovement.fVy) > CD_TTS_MAX_YVEL_FOR_PASSING) {
        /* to prevent over complex distinction of cases calculate time to pass
         * without acceleration */
        if (predRelMovement.fVx > -C_F32_DELTA) {
            /* No Passing */
            fTimeSavePassX = CD_TTS_MAX_PASSING_TIME;
        } else {
            fTimeSavePassX = -fDistanceToPass / predRelMovement.fVx;
            /*Safeguard against negative passing distances and threshold for max
             * time*/
            fTimeSavePassX =
                MINMAX_FLOAT(0, CD_TTS_MAX_PASSING_TIME, fTimeSavePassX);
        }
    } else {
        fTimeSavePassX = 0;
    }

    fSol[0] = 0;
    fSol[1] = 0;
    SolNum = sizeof(fSol) / sizeof(float32);

    /*
      calculate distance in y-direction while passing object
      meaning ego- front same position as object tail up to
      ego tail same position as object front
      Obj is driving to the right -> fDistYWhilePass is negative and vice versa
    */
    fDistYWhilePass = CDCalculateDistance(0, ObjMovement.fVy, ObjMovement.fAy,
                                          fTimeSavePassX);

    /*
      add distance to safety offset in case
        - object movement and evasion movement point in same direction and
    */
    if (((ObjMovement.fVy < 0) && (fEgoAccelAssumed < 0)) ||
        ((ObjMovement.fVy > 0) && (fEgoAccelAssumed > 0))) {
        /*Ego escape to the right -> fLateralMinOffset is positive and vice
         * versa*/
        fLateralMinOffset = fLateralMinOffset - fDistYWhilePass;
    }

    /* Only calculate TTS if distance between ego vehicle and object is smaller
       than needed lateral offset, otherwise there is no need for (further)
       steering*/
    if (((fLateralMinOffset > 0) && (predRelMovement.fY < fLateralMinOffset)) ||
        ((fLateralMinOffset < 0) && (predRelMovement.fY > fLateralMinOffset))) {
        /* offset the ego acceleration to assumed one */
        if (bUseSteeringChange != FALSE) {
            fTotalEgoAccelAssumed = fEgoAccelAssumed + pEgoMovement->fAy;
            /* limit to physical boundaries */
            fAccelYMax = CML_f_CalculatePolygonValue(
                CD_NUMBER_OF_EMERGENCY_EGO_ACCEL_Y_VALUES,
                CD_EMERGENCY_EGO_ACCEL_Y, pEgoMovement->fVx);
            fTotalEgoAccelAssumed =
                MINMAX_FLOAT(-fAccelYMax, fAccelYMax, fTotalEgoAccelAssumed);
        }

        /* equation of motion: 1/2*fA*t^2+fB*t+fC=0 */

        fA = 0.5f * (pEgoMovement->fAy - fTotalEgoAccelAssumed);
        fB = -(pEgoMovement->fAy - fTotalEgoAccelAssumed) * fTime;
        fC = predObjMovement.fY - (pEgoMovement->fVy * fTime) -
             ((0.5f * fTotalEgoAccelAssumed) * SQR(fTime)) - fLateralMinOffset;

        (void)CDSolveQuadrEquation(fA, fB, fC, 0, fSol, &SolNum);

        // if (SolNum > 1 && ((fSol[0] >= 0 && fSol[1] >= 0) || (fSol[0] <= 0 &&
        // fSol[1] <= 0)))
        //{
        *pTTS = CDSelectMinValue(fSol, SolNum, FALSE);
        //}
        // else
        //{
        // *pTTS = CDSelectMinValue(fSol, SolNum, TRUE);//fix problem while
        // two solutions have different sign, we should use the postive
        // solution. 20200107 changed by guotao
        //}
        /* limit TTS value */
        *pTTS = MINMAX_FLOAT(0, CD_TIME_MAX, *pTTS);
    }

    /* return */
    return;
}

/* ***********************************************************************
  @fn           CDCalculateTTSDyn */ /*!

                                        @brief        Calculates TTS for assumed
                                      lateral ego acceleration and given
                                                      lateral position offset.

                                        @description  Calculates TTS for assumed
                                      lateral ego acceleration and given
                                                      lateral position offset.

                                        @param[in]       pEgoMovement
                                      Ego kinematic values
                                        @param[in]       pObjMovement
                                      Object kinematic values
                                        @param[in]       fEgoAccelAssumed
                                      Assumed ego acceleration
                                        @param[in]       fLateralOffset
                                      Wanted lateral offset
                                        @param[in]       fDistanceToPass:
                                        @param[out]      pfTTS
                                      Selected TTS for steering
                                        @param[in,out]   bUseSteeringChange :

                                        @return       FALSE if error has
                                      happened

                                        @pre          [none]

                                        @post         [none]

                                      ****************************************************************************
                                      */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateTTSDyn(const CDMovement_t* pEgoMovement,
                          const CDMovement_t* pObjMovement,
                          fAccel_t fEgoAccelAssumed,
                          fDistance_t fLateralOffset,
                          fDistance_t fDistanceToPass,
                          fTime_t* pfTTS,
                          const boolean bUseSteeringChange) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    fTime_t fTTC;

    /* calculate TTC */
    (void)CDCalculateTTC(pEgoMovement, pObjMovement, &fTTC);

    /* set default values */
    *pfTTS = CD_TIME_MAX;

    /* TTC is valid */
    if (fTTC < CD_TIME_MAX) {
        /* calculate TTS for steering */
        CDCalculateTTSDynCore(pEgoMovement, pObjMovement, fEgoAccelAssumed,
                              fLateralOffset, fDistanceToPass, fTTC, pfTTS,
                              bUseSteeringChange);
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateAnecLatCore */ /*!

                                   @brief        Core function for calculation
                                 of necessary lateral acceleration

                                   @description  Calculates the necessary
                                 lateral acceleration for the given time
                                                 and position

                                   @param[in]    fRelPositionY   Relative
                                 lateral position
                                   @param[in]    fLateralOffset  Wanted lateral
                                 position offset
                                   @param[in]    fTime           Time for
                                 calculation
                                   @param[out]   *pAnecLat       Necessary
                                 lateral acceleration

                                   @return       void

                                   @pre          [none]

                                   @post         [none]

                                 ****************************************************************************
                                 */
static void CDCalculateAnecLatCore(fDistance_t fRelPositionY,
                                   fDistance_t fLateralOffset,
                                   fTime_t fTime,
                                   fAccel_t* pAnecLat) {
    /* set default value */
    *pAnecLat = 0;

    /* relative position is smaller than requested lateral offset */
    if (fABS(fRelPositionY) < fABS(fLateralOffset)) {
        /* prevent division by zero and negative time values */
        if (fTime >= C_F32_DELTA) {
            *pAnecLat = (2.0f * (fRelPositionY - fLateralOffset)) /
                        SQR(fTime); /*physical constant: s=1/2*a*t^2*/

            /* limit value range */
            *pAnecLat = MINMAX_FLOAT(-CD_NEC_LAT_DECEL_MAX,
                                     CD_NEC_LAT_DECEL_MAX, *pAnecLat);
        }
    }
}

/* ***********************************************************************
  @fn           CDCalculateAnecLat */ /*!

                                       @brief        Calculates necessary
                                     lateral acceleration to prevent collision

                                       @description  Necessary lateral
                                     acceleration is calculated.

                                       @param[in]    *pEgoMovement   Ego
                                     kinematic values
                                       @param[in]    *pObjMovement   Object
                                     kinematic values
                                       @param[in]    fLateralOffset  Wanted
                                     lateral offset beetween ego and object at
                                     TTC
                                       @param[out]   *pfAnecLat       Selected
                                     necessary lateral acceleration

                                       @return       FALSE if error has happened

                                       @pre          Object is valid

                                       @post         [none]

                                       @todo         consider vehicle stop
                                     times, perhaps in CDPredictMovement
                                     function

                                     ****************************************************************************
                                     */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateAnecLat(const CDMovement_t* pEgoMovement,
                           const CDMovement_t* pObjMovement,
                           fDistance_t fLateralOffset,
                           fAccel_t* pfAnecLat) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    fTime_t fTTC;
    fDistance_t fRelDistanceTTCNoEgoAccel;

    /* calculate TTC */
    (void)CDCalculateTTC(pEgoMovement, pObjMovement, &fTTC);

    /* set default value */
    *pfAnecLat = 0;

    /* TTC is valid */
    if (fTTC < CD_TIME_MAX) {
        /* calculate relative distance when ego has no lateral acceleration at
         * time of collsion */
        fRelDistanceTTCNoEgoAccel = CDCalculateDistance(
            pObjMovement->fY - pEgoMovement->fY,
            pObjMovement->fVy - pEgoMovement->fVy, pObjMovement->fAy, fTTC);

        /* calculate AnecLat for passing the object */
        CDCalculateAnecLatCore(fRelDistanceTTCNoEgoAccel, fLateralOffset, fTTC,
                               pfAnecLat);
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn           CDCalculateClosingVelocity */ /*!

                               @brief        Calculates closing velocity for
                             current kinematics

                               @description  Closing velocity is calculated
                             based on current kinematics of
                                             ego vehicle and object

                               @param[in]    *pEgoMovement      Ego kinemtatic
                             values
                               @param[in]    *pObjMovement      Object
                             kinemtatic values
                               @param[out]   *pfClosingVelocity  Calculated
                             closing velocity

                               @return       FALSE if error has happened

                               @pre          [none]

                               @post         Positive closing velocity is set
                             for oncoming object

                             ****************************************************************************
                             */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateClosingVelocity(const CDMovement_t* pEgoMovement,
                                   const CDMovement_t* pObjMovement,
                                   fVelocity_t* pfClosingVelocity) {
    /* local variable(s) */
    boolean bReturn = TRUE;
    fTime_t fTTC;
    fVelocity_t EgoPredictVelocityX;
    fVelocity_t ObjPredictVelocityX;
    fTime_t fTimeEgoStop;
    fTime_t fTimeObjStop;

    /* set default value */
    *pfClosingVelocity = 0;

    /* calculate TTC */
    (void)CDCalculateTTC(pEgoMovement, pObjMovement, &fTTC);

    /* calculate closing velocity if TTC is valid */
    if (fTTC < CD_TIME_MAX) {
        /* calculate stop time for vehicles */
        (void)CDCalculateVehStopTime(pEgoMovement->fVx, pEgoMovement->fAx,
                                     &fTimeEgoStop);
        (void)CDCalculateVehStopTime(pObjMovement->fVx, pObjMovement->fAx,
                                     &fTimeObjStop);

        /* predict ego movement if ego stops before TTC */
        if (fTimeEgoStop < fTTC) {
            EgoPredictVelocityX = 0;
        } else {
            EgoPredictVelocityX =
                CDCalculateVelocity(pEgoMovement->fVx, pEgoMovement->fAx, fTTC);
        }

        /* predict object movement if object stops before TTC */
        if (fTimeObjStop < fTTC) {
            ObjPredictVelocityX = 0;
        } else {
            ObjPredictVelocityX =
                CDCalculateVelocity(pObjMovement->fVx, pObjMovement->fAx, fTTC);
        }

        /* closing velocity should always be positive, i.e. EgoPredictVelocityX
         * >= ObjPredictVelocityX */
        *pfClosingVelocity =
            MAX_FLOAT((EgoPredictVelocityX - ObjPredictVelocityX), 0);
    }

    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn            CDCalculateDistance */ /*!


                                     @brief         Calculates the distance
                                   after a time with a given initial distance,
                                   velocity and acceleration.

                                     @description   Calculates the distance
                                   after a time with a given initial distance,
                                   velocity and acceleration.

                                     @param[in]     fDistance      the initial
                                   distance
                                     @param[in]     fVelocity      the velocity
                                   (relative or absolute)
                                     @param[in]     fAcceleration  the
                                   acceleration (relative or absolute)
                                     @param[in]     fTime          the time

                                     @return        Returns distance after time
                                   using constantly accelerated movement model

                                   ****************************************************************************
                                   */
static fDistance_t CDCalculateDistance(fDistance_t fDistance,
                                       fVelocity_t fVelocity,
                                       fAccel_t fAcceleration,
                                       fTime_t fTime) {
    const fDistance_t fDist =
        fDistance + (fVelocity * fTime) + (0.5f * fAcceleration * SQR(fTime));

    return fDist;
}

/* ***********************************************************************
  @fn            CDCalculateVelocity */ /*!


                                     @brief         Calculates the velocity
                                   after a time with a given velocity and
                                   acceleration.

                                     @description   Calculates the velocity
                                   after a time with a given velocity and
                                   acceleration.

                                     @param[in]     fVelocity      the velocity
                                   (relative or absolute)
                                     @param[in]     fAcceleration  the
                                   acceleration (relative or absolute)
                                     @param[in]     fTime          the time

                                     @return        Returns velocity after time
                                   using constantly accelerated movement model
                                   ****************************************************************************
                                   */
static fVelocity_t CDCalculateVelocity(fVelocity_t fVelocity,
                                       fAccel_t fAcceleration,
                                       fTime_t fTime) {
    const fVelocity_t fSpeed = fVelocity + (fAcceleration * fTime);

    return fSpeed;
}

/* ***********************************************************************
  @fn            CDPredictMovement                                  */ /*!

      @brief         predicts Movement at a certain time

      @description   predicts Movement at a certain time.
                     if speed 0 is reached in one direction the Object is
                     assumed to completely stop if corresponding parameter has
                     been set

      @param[in]     pMovement           the current movement info
      @param[in]     pPredictedMovement  the predicted movement info
      @param[in]     fPredictTime        the prediction time
      @param[in]     bStopXDir           vehicle stops when reaching speed 0
                                         in X direction
      @param[in]     bStopYDir           vehicle stops when reaching speed 0
                                         in Y direction

      @return        none

    ****************************************************************************
    */
void CDPredictMovement(const CDMovement_t* pMovement,
                       CDMovement_t* pPredictedMovement,
                       float32 fPredictTime,
                       boolean bStopXDir,
                       boolean bStopYDir) {
    float32 fTime2Stop = CD_TIME_MAX;

    /* predict velocities */
    pPredictedMovement->fVx =
        CDCalculateVelocity(pMovement->fVx, pMovement->fAx, fPredictTime);
    pPredictedMovement->fVy =
        CDCalculateVelocity(pMovement->fVy, pMovement->fAy, fPredictTime);

    /* stopping possible in x direction */
    if ((bStopXDir == TRUE) &&
        (((pPredictedMovement->fVx <= 0) &&
          (pMovement->fVx > 0)) /* change from moving to oncoming */
         || ((pPredictedMovement->fVx >= 0) && (pMovement->fVx < 0)))) {
        /* change from oncoming to moving */
        /* Condition implies a != 0 */
        /* Calc time to stop; negative fTime2Stop means stop in the past */
        /* fTime2Stop has same sign as fPredictTime due to conditions */
    } else {
        fTime2Stop = -pMovement->fVx / pMovement->fAx;
        bStopXDir = FALSE;
    }

    /* stopping possible in y direction */
    if ((bStopYDir == TRUE) &&
        (((pPredictedMovement->fVy <= 0) &&
          (pMovement->fVy > 0)) /* change from moving right to left */
         || ((pPredictedMovement->fVy >= 0) &&
             (pMovement->fVy < 0))) /* change from moving left to right */
        && (fABS(pMovement->fAy) > C_F32_DELTA)) {
        /* Condition above implies a != 0 but it is checked explicite here
         * because of more safeguarding */
        /* Calc time to stop; negative fTime2Stop means stop in the past */
        /* fTime2Stop has same sign as fPredictTime due to conditions */
        const float32 fTime2StopY = -pMovement->fVy / pMovement->fAy;

        /* select minimum stop time */
        if (bStopXDir == TRUE) {
            fTime2Stop = MIN_FLOAT(fTime2Stop, fTime2StopY);
        } else {
            fTime2Stop = fTime2StopY;
        }
    } else {
        bStopYDir = FALSE;
    }

    /* predict movement with stop */
    if ((bStopXDir == TRUE) || (bStopYDir == TRUE)) {
        pPredictedMovement->fX = CDCalculateDistance(
            pMovement->fX, pMovement->fVx, pMovement->fAx, fTime2Stop);
        pPredictedMovement->fVx = 0;
        pPredictedMovement->fAx = 0;
        pPredictedMovement->fY = CDCalculateDistance(
            pMovement->fY, pMovement->fVy, pMovement->fAy, fTime2Stop);
        pPredictedMovement->fVy = 0;
        pPredictedMovement->fAy = 0;
    } else {
        /* predict movement without stop */
        pPredictedMovement->fX = CDCalculateDistance(
            pMovement->fX, pMovement->fVx, pMovement->fAx, fPredictTime);
        /* pPredictedMovement->fVx has already been set before condition */
        pPredictedMovement->fAx = pMovement->fAx;
        pPredictedMovement->fY = CDCalculateDistance(
            pMovement->fY, pMovement->fVy, pMovement->fAy, fPredictTime);
        /* pPredictedMovement->fVy has already been set before condition */
        pPredictedMovement->fAy = pMovement->fAy;
    }
}

/* ***********************************************************************
  @fn            CDCalculateAnecLongLatency */
boolean CDCalculateAnecLongLatency(
    const CDMovement_t* pEgoMovement,
    const CDMovement_t* pObjMovement,
    const CDAdjSafeDistance_t* pCDAdjSafeDistance,
    fAccel_t* pfAnecLong) {
    /* local variable(s) */
    boolean bReturn = TRUE;

    if ((pEgoMovement != NULL) && (pObjMovement != NULL) &&
        (pCDAdjSafeDistance != NULL) && (pfAnecLong != NULL)) {
        /* local variable(s) */
        CDMovement_t egoMovementWoAcceleration;
        CDMovement_t currObjMovement;
        CDMovement_t predObjMovement;
        CDMovement_t histEgoMovement;
        CDMovement_t predEgoMovement;
        float32 vRelHist;
        float32 aRelHist;
        float32 vRelCurr;
        float32 vRelPred;

        /* don't consider ego acceleration */
        egoMovementWoAcceleration = *pEgoMovement;
        egoMovementWoAcceleration.fAx = 0;

        /* calculate history values for Ego */
        CDPredictMovement(&egoMovementWoAcceleration, &histEgoMovement,
                          -CD_LATENCY_OBJ_ACQUISITION, TRUE, FALSE);
        /* calculate predicted values for Ego */
        CDPredictMovement(&egoMovementWoAcceleration, &predEgoMovement,
                          CD_LATENCY_VLC2BRAKE, TRUE, FALSE);
        /* calculate current values for Object */
        CDPredictMovement(pObjMovement, &currObjMovement,
                          CD_LATENCY_OBJ_ACQUISITION, TRUE, FALSE);
        /* calculate predicted values for Object */
        CDPredictMovement(pObjMovement, &predObjMovement, CD_LATENCY_SYSTEM,
                          TRUE, FALSE);

        vRelHist = pObjMovement->fVx - histEgoMovement.fVx;
        aRelHist = pObjMovement->fAx - histEgoMovement.fAx;
        vRelCurr = currObjMovement.fVx - egoMovementWoAcceleration.fVx;
        vRelPred = predObjMovement.fVx - predEgoMovement.fVx;

        /* Check for crash within prediction time */
        /* Assumption: as prediction time accounts for system latency we shall
        not check for safety distance as this already would have been considered
        in the past. => Consistent mechanism is to check only for crash and in
        all other cases use the predicted values */

        /* Check for minimum distance within CD_LATENCY_SYSTEM */
        if ((aRelHist > C_F32_DELTA) && (vRelHist <= 0) && (vRelPred > 0) &&
            (vRelCurr > 0)) {
            const float32 timeOfMinimum = (-vRelHist) / aRelHist;
            CDMovement_t objMovementMinimum;
            CDMovement_t egoMovementMinimum;
            float32 fMinDist;
            CDPredictMovement(&histEgoMovement, &egoMovementMinimum,
                              timeOfMinimum, TRUE, FALSE);
            CDPredictMovement(pObjMovement, &objMovementMinimum, timeOfMinimum,
                              TRUE, FALSE);
            fMinDist = objMovementMinimum.fX - egoMovementMinimum.fX;
            if ((fMinDist <= 0) || (predObjMovement.fX <=
                                    (predEgoMovement.fX +
                                     pCDAdjSafeDistance->fLongNecRemainDist))) {
                /* Crash case 1: Within Prediction Time the Objects touched */
                /* Crash case 2: After Prediction X Distance is still less than
                 * safety distance */
                *pfAnecLong = CD_NEC_LONG_DECEL_MAX;
            } else {
                /* min dist was reached within Prediction time and min dist is
                 * Ok after Latency => situation is cleared */
                *pfAnecLong = 0;
            }
        } else {
            /* consider distance ego has traveled between data acquisition and
             * VLC call */
            predEgoMovement.fX -= histEgoMovement.fX;
            /* If no crash is found calculate plain Anec Long*/
            predObjMovement.fX -= pCDAdjSafeDistance->fLongNecRemainDist;
            (void)CDCalculateAnecLong(&predEgoMovement, &predObjMovement,
                                      pfAnecLong);
        }
    } else {
        bReturn = FALSE;
    }
    /* return */
    return bReturn;
}

/* ***********************************************************************
  @fn            CDCalculateAnecLong */
/*this function is part of catalog for common functions and can be used
 * externaly*/
boolean CDCalculateAnecLong(const CDMovement_t* pEgoMovement,
                            const CDMovement_t* pObjMovement,
                            fAccel_t* pfAnecLong) {
    /* local variable(s) */
    boolean bReturn = TRUE;

    /* Relative movement = object movement - ego movement */
    /* Note: object movement for stationary objects is assumed to be correctly
     * set to 0 */
    const float32 fVRelX = pObjMovement->fVx - pEgoMovement->fVx;

    /* ego vehicle is standing */
    if (pEgoMovement->fVx < C_F32_DELTA) {
        (*pfAnecLong) = 0;
        /* ego acceleration 0 sufficient to prevent a crash*/
    } else if ((pObjMovement->fAx > -C_F32_DELTA) && (fVRelX > 0)) {
        (*pfAnecLong) = 0;
    } else {
        /* obj X might be reduced by "safety distance"; ego X might be non-zero
         * due to prediction*/
        const float32 fBrakeDist = MAX(pObjMovement->fX - pEgoMovement->fX,
                                       CD_NEC_LONG_VIRTUAL_OBJ_DIST);
        /* safety distance already reached; should not be reached if
         * CD_NEC_LONG_VIRTUAL_OBJ_DIST > 0 */
        if (fBrakeDist < C_F32_DELTA) {
            (*pfAnecLong) = CD_NEC_LONG_DECEL_MAX;
            /* oncoming traffic */
        } else if (pObjMovement->fVx <= -C_F32_DELTA) {
            /* for oncoming traffic calculate time */
            float32 fAnecLongColl = CD_NEC_LONG_DECEL_MAX;
            float32 fAnecLongCollSol[CD_SOLUTION_ARRAY_SIZE];
            uint8 uAnecLongSolNum = sizeof(fAnecLongCollSol) / sizeof(float32);
            /* find neccessary a_Ego_brake = fAnecLongColl to stop ego just
               before the collsion; fBrakeDist > 0 is always true:
               x_Obj(t_brake) - x_Ego(t_brake) = 0 with t_brake = - v_Ego_0 /
               a_Ego_brake solved for a_Ego_brake */

            (void)CDSolveQuadrEquation(
                fBrakeDist, -((pObjMovement->fVx * pEgoMovement->fVx) -
                              (0.5f * (pEgoMovement->fVx * pEgoMovement->fVx))),
                (((0.5f * pObjMovement->fAx) * pEgoMovement->fVx) *
                 pEgoMovement->fVx),
                CD_NEC_LONG_DECEL_MAX, fAnecLongCollSol, &uAnecLongSolNum);
            if (uAnecLongSolNum == CD_SOLUTION_ARRAY_SIZE) {
                fAnecLongColl =
                    MIN_FLOAT(fAnecLongCollSol[0], fAnecLongCollSol[1]);
            } else if (uAnecLongSolNum == 1) {
                fAnecLongColl = fAnecLongCollSol[0];
            } else {
                fAnecLongColl = CD_NEC_LONG_DECEL_MAX;
            }
            /* oncoming traffic is braking */
            if (pObjMovement->fAx > C_F32_DELTA) {
                /* Available braking distance is*/
                /* current distance - Object way to stop */
                /*  pObjMovement->fAx > C_F32_DELTA is assured => second term >
                 * 0*/
                const float32 fStopObjDist =
                    fBrakeDist -
                    (SQR(pObjMovement->fVx) / (2.0f * pObjMovement->fAx));
                if (fStopObjDist < C_F32_DELTA) {
                    (*pfAnecLong) = CD_NEC_LONG_DECEL_MAX;
                } else {
                    /*insert t=-v_x_Ego/a_NecLong in
                     * StopObjDist=1/2*a_NecLong*t^2+v_x_Ego*t*/

                    (*pfAnecLong) =
                        -(SQR(pEgoMovement->fVx) / (2.0f * fStopObjDist));
                }
                /* object accelerates towards us or does not brake */
            } else {
                (*pfAnecLong) = fAnecLongColl;
            }
            /* normal situation: follow and run-up; moving braking stationary
             */
        } else {
            float32 fANecLongStopped = CD_NEC_LONG_DECEL_MAX;
            float32 fANecLongMoving = CD_NEC_LONG_DECEL_MAX;
            /* Scenario 1: Car in Front stops and we stop with min distance
             * behind the car */
            if (pObjMovement->fAx <= -C_F32_DELTA) {
                /* Available braking distance is*/
                /* current distance + Object way to stop */
                /* fBrakeDist > C_F32_DELTA is already assured */
                /*  pObjMovement->fAx <= -C_F32_DELTA is assured => second term
                 * < 0*/
                const float32 fStopObjDist =
                    fBrakeDist -
                    (SQR(pObjMovement->fVx) / (2.0f * pObjMovement->fAx));
                /* Object assumed to be stationary*/
                const float32 fANecLongCandidateStopped =
                    -(SQR(pEgoMovement->fVx) / (2.0f * fStopObjDist));
                /* verify that the candidate value is valid => Object needs to
                 * stop before Ego */
                const float32 fStopEgoTime =
                    -(pEgoMovement->fVx) / fANecLongCandidateStopped;
                const float32 fStopObjTime =
                    -(pObjMovement->fVx) / (pObjMovement->fAx);
                if (fStopEgoTime >= fStopObjTime) {
                    fANecLongStopped = fANecLongCandidateStopped;
                }
                /* By back-substitution the else case leads to vRel < 0 => its
                 * save to leave the value at max accel */
            }
            /* Scenario 2: Car in Front is currently slower => Brake such that
             * we reach same speed at minimal distance */
            /* This case includes the special case of stationary target vehicles
             */
            /* If vRel < 0 the tangent point is in the past */
            if (fVRelX <= 0) {
                fANecLongMoving =
                    pObjMovement->fAx -
                    (SQR(fVRelX) /
                     (2.f * fBrakeDist)); /* insert t=-v/a in s=1/2*a*t^2+v*t */
            }
            /* use the more relaxed of both to rule out the case that scenario 2
             * is in the area of negative velocity */
            (*pfAnecLong) = MAX(fANecLongStopped, fANecLongMoving);
        }
    }

    /* clip value to [Max-Decel, 0.0] */
    (*pfAnecLong) = MINMAX_FLOAT(CD_NEC_LONG_DECEL_MAX, 0, (*pfAnecLong));

    /* return */
    return bReturn;
}

/*************************************************************************************************************************
  Functionname:    CDCalculateTrajRelation */
static void CDCalculateTrajRelation(ObjNumber_t iObjectIndex,
                                    const CDInternalStatus_t* pInternalStatus,
                                    const CDMovement_t* pObjMovement) {
    boolean bResultOk;
    CDInternalObject_t* currentObjInternal =
        &((*pInternalStatus->rgObjInternal)[iObjectIndex]);

    bResultOk = EMPCalcObjToEgoTrajRelation(
        &((*pInternalStatus).sKinEgo), &(currentObjInternal->EMPObjData),
        &(currentObjInternal->TrajRelation));

    if (bResultOk == FALSE) {
        /* Something went wrong with ObjToEgoTrajRelation Calculation */
        currentObjInternal->TrajRelation.fDistToTraj = EMP_DIST_TO_TRAJ_MAX;
        currentObjInternal->TrajRelation.fDistToTrajVar = 0;
        currentObjInternal->TrajRelation.fVelocityToTraj = 0;
        currentObjInternal->TrajRelation.fVelocityToTrajVar = 0;
        currentObjInternal->TrajRelation.fDistOnTraj = 0;
        currentObjInternal->TrajRelation.fDistOnTrajVar = 0;
        currentObjInternal->TrajRelation.fVelocityOnTraj = 0;
        currentObjInternal->TrajRelation.fVelocityOnTrajVar = 0;
    }
    _PARAM_UNUSED(pObjMovement);
}

#if defined(CD_USE_EMP) && (CD_USE_EMP == SWITCH_ON)
/* **********************************************************************
  @fn            CDUpdateEMPData */ /*!

                                        @brief         Updates the History of
                                      the ego and all other objects for EMP

                                        @param[in]     pInputData Pointer to CD
                                      input data
                                        @param[in]     pInternalStatus Pointer
                                      to CD internal status

                                      ****************************************************************************
                                      */
void CDUpdateEMPData(const CDInputData_t* pInputData,
                     CDInternalStatus_t* pInternalStatus) {
    ObjNumber_t iCurrObjID;

    /* Update Ego Kinematic History */
    const float32 fCurrEgoV = pInputData->pEgoData->pEgoDynObjSync->fVelocityX;
    const float32 fCurrEgoA = pInputData->pEgoData->pEgoDynObjSync->fAccelX;
    float32 fCurrYawRate;
    if ((fCurrEgoV < -C_F32_DELTA) || (fCurrEgoV > C_F32_DELTA)) {
        fCurrYawRate =
            pInputData->pEgoData->pEgoDynObjSync->fAccelY / fCurrEgoV;
    } else {
        fCurrYawRate = 0;
    }

    /* Update Ego Geometry */
    pInternalStatus->sEgoGeometry.fWidth = VLC_fEgoVehicleWidth;
    pInternalStatus->sEgoGeometry.fLength = VLC_fEgoVehicleLength;

    (*pInternalStatus).sKinEgo.fVel = fCurrEgoV;
    (*pInternalStatus).sKinEgo.fAccel = fCurrEgoA;
    (*pInternalStatus).sKinEgo.fYawRate = fCurrYawRate;

    /* Update Obj List */

    /* Push History for each object */
    for (iCurrObjID = 0u; iCurrObjID < EMP_NUM_OBJECTS; ++iCurrObjID) {
        CDInternalObject_t* currentObjInternal =
            &((*pInternalStatus->rgObjInternal)[iCurrObjID]);
        const Envm_t_GenObjKinEnvmatics* const pObjKinematic =
            CDGetPointer_Kinematic(pInputData->pObjectData, iCurrObjID);

        if (OBJ_IS_NEW(iCurrObjID)) {
            EMPResetObjDesc(&(currentObjInternal->EMPObjData));
        }

        if (!OBJ_IS_DELETED(iCurrObjID)) {
            const float32 currObjWidth =
                CDGetPointer_Dimensions(pInputData->pObjectData, iCurrObjID)
                    ->fWidth;
            const float32 currObjLength =
                CDGetPointer_Dimensions(pInputData->pObjectData, iCurrObjID)
                    ->fLength;

            /* Retrieve Kinematic */
            const float32 currObjPosX =
                CD_GET_DIST_X(pInputData->pObjectData, iCurrObjID);
            const float32 currObjPosY =
                CD_GET_DIST_Y(pInputData->pObjectData, iCurrObjID);

            const float32 currObjPosYStd =
                CD_OBJ_DISTY_STDDEV(pInputData->pObjectData, iCurrObjID);

            const float32 currObjPosXStd =
                CD_OBJ_DISTX_STDDEV(pInputData->pObjectData, iCurrObjID);
            const float32 currObjVelX = pObjKinematic->fVabsX;
            const float32 currObjVelXStd = pObjKinematic->fVabsXStd;
            const float32 currObjVelY = pObjKinematic->fVabsY;
            const float32 currObjVelYStd = pObjKinematic->fVabsYStd;
            const float32 currObjPosXVar = currObjPosXStd * currObjPosXStd;
            const float32 currObjPosYVar = currObjPosYStd * currObjPosYStd;
            const float32 currObjVelXVar = currObjVelXStd * currObjVelXStd;
            const float32 currObjVelYVar = currObjVelYStd * currObjVelYStd;

            /* Set Existence */
            currentObjInternal->EMPObjData.bExists = TRUE;

            /*This is just a temporary workaround for radar because in the ARS
            object list no variance is available. When EMPObjData.GeometryVar.fX
            or fY will be used in the future you might consider to calculate
            these variances
            based on the shape points variances (like it is done in camera)*/
            currentObjInternal->EMPObjData.Kinematic.fPosX = currObjPosX;
            currentObjInternal->EMPObjData.Kinematic.fPosXVar = currObjPosXVar;
            currentObjInternal->EMPObjData.Kinematic.fPosY = currObjPosY;
            currentObjInternal->EMPObjData.Kinematic.fPosYVar = currObjPosYVar;
            currentObjInternal->EMPObjData.Kinematic.fVelX = currObjVelX;
            currentObjInternal->EMPObjData.Kinematic.fVelXVar = currObjVelXVar;
            currentObjInternal->EMPObjData.Kinematic.fVelY = currObjVelY;
            currentObjInternal->EMPObjData.Kinematic.fVelYVar = currObjVelYVar;
        } else {
            EMPResetObjDesc(&(currentObjInternal->EMPObjData));
        }
    }
}

#endif /* CD_USE_EMP */

/*************************************************************************************************************************
  Functionname:    CDCalcALat */
static fAccel_t CDCalcALat(const fDistance_t fNecLatOffset, float32 fEgoVel) {
    float32 fAy;
    const float32 fOVerlapThreshold =
        SQR(CD_V_LAT_PRE_LIMIT) / (2.0f * CD_A_LAT_PRE_LIMIT);

    if (fABS(fNecLatOffset) > fOVerlapThreshold) {
        float32 fVelFactor = CML_f_CalculatePolygonValue(
            CD_NUMBER_OF_COMFORT_EGO_VEL_FACTOR_Y_VALUES,
            CD_COMFORT_EGO_VEL_FACTOR_Y, fEgoVel);
        /*calculate necessary lateral acceleration by characteristic*/
        fAy = (SQR(CD_V_LAT_PRE_LIMIT) / (2.0f * fABS(fNecLatOffset))) *
              fVelFactor; /*equations of motion: a= v^2 / (2*s) */
        /*set at least to lower acceleration threshold*/
        if (fABS(fAy) < CD_A_LAT_PRE_MIN) {
            fAy = CD_A_LAT_PRE_MIN;
        }
    } else {
        /*take max lateral acceleration when overlap is smaller than
         * fOVerlapThreshold*/
        fAy = CD_A_LAT_PRE_LIMIT;
    }
    return fAy;
}

/* **********************************************************************
  @fn            CDCalculateObjectProperties */ /*!

                            @brief         Calculates all object criticality
                          values

                            @description   Calculates all object information
                          which is needed for
                                           relevant objects

                            @param[in]     pInputData       Pointer to the input
                          data
                            @param[in]     pInternalStatus  Pointer to the
                          internal status
                            @param[in]     pParameters      Pointer to the
                          parameters

                            @return        void

                            @pre           [none]

                            @post          [none]

                          ****************************************************************************
                          */
void CDCalculateObjectProperties(const CDInputData_t* pInputData,
                                 const CDInternalStatus_t* pInternalStatus,
                                 const CDParameters_t* pParameters) {
    ObjNumber_t iObjectIndex;
    CDMovement_t ObjMovement;
    CDMovement_t EgoMovement;
    CDMovement_t predRelMovement;
    fAccel_t fEgoAccelTTBPre = 0;
    fAccel_t fEgoAccelTTBAcute = 0;
    fAccel_t fEgoAccelTTSPreLeft = 0;
    fAccel_t fEgoAccelTTSPreRight = 0;
    fAccel_t fEgoAccelTTSAcute = 0;
    fDistance_t fLateralOffsetMinLeft = 0;
    fDistance_t fLateralOffsetMinRight = 0;
    fDistance_t fLatOffsetLeftWithSafetyDistance = 0;
    fDistance_t fLatOffsetRightWithSafetyDistance = 0;
    fDistance_t fDistanceToPass = 0;

    CDFillEgoMovement(&EgoMovement, pInputData);

    /* for each object calculate relevant information */
    for (iObjectIndex = 0;
         iObjectIndex < pInputData->pObjectData->iNumberOfObjects;
         iObjectIndex++) {
        const Envm_t_GenObjAttributes* const pObjAttribs =
            CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex);
        const CDObjDimension_t* const pObjDimension =
            CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex);
        CDInternalObject_t* const currentObjInternal =
            &((*pInternalStatus->rgObjInternal)[iObjectIndex]);
        CDInitInternalObjData(currentObjInternal);
        if (!OBJ_IS_DELETED(iObjectIndex)) {
            /* Send RTA-Start Event for Collision Detection runtime */
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                     VLCSEN_RTA_CD_CALCULATE_SINGLE_OBJECT_PROP,
                                     (uint8)(iObjectIndex));

            /* calculate wanted lateral offset; relative position is (obj-ego)
             * so positive for passing right of object */
            /*physical constant*/
            fLateralOffsetMinRight =
                (0.5f * VLC_fEgoVehicleWidth) +
                (0.5f *
                 CDGet_Dimension_Width(pInputData->pObjectData, iObjectIndex));
            fLateralOffsetMinLeft = -fLateralOffsetMinRight;
            fLatOffsetRightWithSafetyDistance =
                pParameters->pAdjSafeDistance->fLatNecRemainDist +
                fLateralOffsetMinRight;
            fLatOffsetLeftWithSafetyDistance =
                -fLatOffsetRightWithSafetyDistance;
            /* ISSUE 426523 and 426525 */
            fDistanceToPass =
                pParameters->pAdjSafeDistance->fLongNecRemainDist +
                VLC_fEgoVehicleLength + pObjDimension->fLength;

            /* TODO: Does ObjMovement need to be relative for all functions? */
            CDFillObjectMovement(iObjectIndex, &ObjMovement, pInputData,
                                 currentObjInternal);
            currentObjInternal->fAAbsX = ObjMovement.fAx;
            currentObjInternal->fVAbsX = ObjMovement.fVx;

            /* Send RTA-Start Event for Collision Detection runtime */
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                     VLCSEN_RTA_CD_CALCULATE_TTC,
                                     (uint8)(iObjectIndex));

            CDCalculateTrajRelation(iObjectIndex, pInternalStatus,
                                    &ObjMovement);

            (void)CDCalculateTTC(&EgoMovement, &ObjMovement,
                                 &(currentObjInternal->TTC));
            (void)CDCalculateTTC2(&EgoMovement, &ObjMovement,
                                  &(currentObjInternal->TTC2));
            (void)CDCalculateTTC3(&EgoMovement, &ObjMovement,
                                  &(currentObjInternal->TTC3));
            (void)CDCalculateTTC4(&EgoMovement, &ObjMovement,
                                  &(currentObjInternal->TTC4));

            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                     VLCSEN_RTA_CD_CALCULATE_TTC,
                                     (uint8)(iObjectIndex));

            /* Send RTA-Start Event for Collision Detection runtime */
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                     VLCSEN_RTA_CD_CALCULATE_LONG,
                                     (uint8)(iObjectIndex));

            (void)CDCalculateAnecLongLatency(
                &EgoMovement, &ObjMovement, pParameters->pAdjSafeDistance,
                &(currentObjInternal->LongNecAccel));

            fEgoAccelTTBPre = CML_f_CalculatePolygonValue(
                CD_NUMBER_OF_COMFORT_EGO_ACCEL_X_VALUES, CD_COMFORT_EGO_ACCEL_X,
                EgoMovement.fVx);

            (void)CDCalculateTTBDyn(&EgoMovement, &ObjMovement, fEgoAccelTTBPre,
                                    &(currentObjInternal->TTBPre));

            fEgoAccelTTBAcute = CD_EMERGENCY_BRAKE_ACCEL;

            (void)CDCalculateTTBDyn(&EgoMovement, &ObjMovement,
                                    fEgoAccelTTBAcute,
                                    &(currentObjInternal->TTBAcute));

            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                     VLCSEN_RTA_CD_CALCULATE_LONG,
                                     (uint8)(iObjectIndex));

            /* Send RTA-Start Event for Collision Detection runtime */
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                     VLCSEN_RTA_CD_CALCULATE_LAT,
                                     (uint8)(iObjectIndex));

            (void)CDCalculateAnecLat(&EgoMovement, &ObjMovement,
                                     fLatOffsetLeftWithSafetyDistance,
                                     &(currentObjInternal->LatNecAccelLeft));
            (void)CDCalculateAnecLat(&EgoMovement, &ObjMovement,
                                     fLatOffsetRightWithSafetyDistance,
                                     &(currentObjInternal->LatNecAccelRight));
            /* select AnecLat and keep sign of acceleration */
            if (fABS(currentObjInternal->LatNecAccelLeft) >
                fABS(currentObjInternal->LatNecAccelRight)) {
                currentObjInternal->LatNecAccel =
                    currentObjInternal->LatNecAccelRight;
            } else {
                currentObjInternal->LatNecAccel =
                    currentObjInternal->LatNecAccelLeft;
            }

            (void)CDCalculateClosingVelocity(
                &EgoMovement, &ObjMovement,
                &(currentObjInternal->ClosingVelocity));

            /*relative position at TTC without object-acceleration due to
             * noise*/
            /* physical constant from equation of motion: s(t)=1/2*a*t^2+v*t+s0
             */
            predRelMovement.fY =
                ObjMovement.fY +
                (((ObjMovement.fVy - EgoMovement.fVy) *
                  (currentObjInternal->TTC)) -
                 (0.5f * EgoMovement.fAy * SQR(currentObjInternal->TTC)));

            /* TTS Pre*/
            fEgoAccelTTSPreLeft = CDCalcALat(
                MAX_FLOAT(predRelMovement.fY - fLateralOffsetMinLeft, 0),
                EgoMovement.fVx);
            fEgoAccelTTSPreRight = -CDCalcALat(
                MIN_FLOAT(predRelMovement.fY - fLateralOffsetMinRight, 0),
                EgoMovement.fVx);

            (void)CDCalculateTTSDyn(&EgoMovement, &ObjMovement,
                                    fEgoAccelTTSPreLeft, fLateralOffsetMinLeft,
                                    fDistanceToPass,
                                    &(currentObjInternal->TTSPreLeft), TRUE);
            (void)CDCalculateTTSDyn(&EgoMovement, &ObjMovement,
                                    fEgoAccelTTSPreRight,
                                    fLateralOffsetMinRight, fDistanceToPass,
                                    &(currentObjInternal->TTSPreRight), TRUE);

            currentObjInternal->TTSPre =
                MAX_FLOAT(currentObjInternal->TTSPreLeft,
                          currentObjInternal->TTSPreRight);

            /* TTS Acute*/
            fEgoAccelTTSAcute = CML_f_CalculatePolygonValue(
                CD_NUMBER_OF_EMERGENCY_EGO_ACCEL_Y_VALUES,
                CD_EMERGENCY_EGO_ACCEL_Y, EgoMovement.fVx);
            (void)CDCalculateTTSDyn(&EgoMovement, &ObjMovement,
                                    fEgoAccelTTSAcute, fLateralOffsetMinLeft,
                                    fDistanceToPass,
                                    &(currentObjInternal->TTSAcuteLeft), FALSE);
            (void)CDCalculateTTSDyn(
                &EgoMovement, &ObjMovement, -fEgoAccelTTSAcute,
                fLateralOffsetMinRight, fDistanceToPass,
                &(currentObjInternal->TTSAcuteRight), FALSE);

            currentObjInternal->TTSAcute =
                MAX_FLOAT(currentObjInternal->TTSAcuteLeft,
                          currentObjInternal->TTSAcuteRight);

            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                     VLCSEN_RTA_CD_CALCULATE_LAT,
                                     (uint8)(iObjectIndex));

            /* Send RTA-Start Event for Collision Detection runtime */
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                     VLCSEN_RTA_CD_CALCULATE_ASSIGN,
                                     (uint8)(iObjectIndex));

            /* assign track probability */
            CDAssignTrackProbability(iObjectIndex, pInputData, pInternalStatus);
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                     VLCSEN_RTA_CD_CALCULATE_SINGLE_OBJECT_PROP,
                                     (uint8)(iObjectIndex));
            VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                     VLCSEN_RTA_CD_CALCULATE_ASSIGN,
                                     (uint8)(iObjectIndex));
        } /* !OBJ_IS_DELETED(iObjectIndex) */

        /* Send RTA-Start Event for Collision Detection runtime */
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                 VLCSEN_RTA_CD_CALCULATE_COMMON,
                                 (uint8)(iObjectIndex));

        /* Do not save DynProb for COD-Objects, as the Data is only an educated
         * guess */
        /*add information for oncoming objects*/
        if (pObjAttribs->eDynamicProperty ==
            (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING) {
            currentObjInternal->HypothesisHist.WasOncomming = 1u;
        } else if ((currentObjInternal->HypothesisHist.WasOncomming == 1) &&
                   (pObjAttribs->eDynamicProperty ==
                    (Envm_t_GenObjDynamicProperty)
                        Envm_GEN_OBJECT_DYN_PROPERTY_MOVING)) {
            currentObjInternal->HypothesisHist.WasOncomming = 0u;
        } else {
            // no reaction!
        }

        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_CALCULATE_COMMON,
                                 (uint8)(iObjectIndex));
    }
    /* occupy recognition on time TTS ( evasion possibility ) */
    CDCheckTimesLatMovValid(&(*pInternalStatus->rgObjInternal),
                            pInputData->pObjectData);
}

/* ***********************************************************************
  @fn            CDFillEgoMovement */ /*!

                                       @brief         Set CDFillEgoMovement

                                       @description

                                       @param[out]    pEgoMovement Pointer to
                                     ego movement data
                                       @param[out]    pInputData   Pointer to
                                     input data

                                       @return        void

                                       @pre           [none]

                                       @post          [none]

                                     ****************************************************************************
                                     */
void CDFillEgoMovement(CDMovement_t* pEgoMovement,
                       const CDInputData_t* pInputData) {
    /* usage of undelayed ego signals for the prediction of the ego movement */
    pEgoMovement->fX = 0; /* ego position X is defined as zero*/
    pEgoMovement->fY = 0; /* ego position Y is defined as zero*/
    pEgoMovement->fVx = pInputData->pEgoData->pEgoDynRaw->fVelocityX;
    pEgoMovement->fVy = pInputData->pEgoData->pEgoDynRaw->fVelocityY;
    pEgoMovement->fAy = pInputData->pEgoData->pEgoDynRaw->fAccelY;
    pEgoMovement->fAx = MIN_FLOAT(0, pInputData->pEgoData->pEgoDynRaw->fAccelX);
}

/* **********************************************************************
  @fn            CDFillObjectMovement */ /*!


                                   @brief         Set Object-Movement

                                   @description

                                   @param[in]     iObjectIndex Object index
                                   @param[in]     pObjMovement Pointer to
                                 movement of the object
                                   @param[out]    pInputData   Pointer to input
                                 data
                                   @param[in]     pCDInternalObject
                                 _PARAM_UNUSED

                                   @return        void

                                   @pre           [none]

                                   @post          [none]

                                 ****************************************************************************
                                 */
void CDFillObjectMovement(ObjNumber_t iObjectIndex,
                          CDMovement_t* pObjMovement,
                          const CDInputData_t* pInputData,
                          const CDInternalObject_t* pCDInternalObject) {
    const Envm_t_GenObjKinEnvmatics* const pObjKinematic =
        CDGetPointer_Kinematic(pInputData->pObjectData, iObjectIndex);
    const Envm_t_GenObjAttributes* const pObjAttribs =
        CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex);
    const float32 fObjDistX =
        CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex);
    const float32 fObjDistY =
        CD_GET_DIST_Y(pInputData->pObjectData, iObjectIndex);
    _PARAM_UNUSED(pCDInternalObject);

    /* calculate Object movement */
    pObjMovement->fX = fObjDistX;
    pObjMovement->fY = fObjDistY;
    /* fill deviations if wanted */

    /* prevent artefacts in ANec-LongCalculation*/
    if (((pObjAttribs->eDynamicProperty ==
          (Envm_t_GenObjDynamicProperty)
              Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
         (pObjAttribs->eDynamicProperty ==
          (Envm_t_GenObjDynamicProperty)
              Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED)) &&
        (pObjKinematic->fVabsX <= 0) &&
        (pObjAttribs->eClassification !=
         (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_PEDESTRIAN)) {
        /* stationary objects do not move */
        pObjMovement->fVx = 0;
        pObjMovement->fVy = 0;
        pObjMovement->fAx = 0;
        pObjMovement->fAy = 0;
    } else {
        /* Absolute speed with yaw correction calculated from relative speeds*/
        pObjMovement->fVx = (pObjKinematic->fVrelX +
                             pInputData->pEgoData->pEgoDynObjSync->fVelocityX) -
                            (fObjDistY * EGO_YAW_RATE_OBJ_SYNC);
        pObjMovement->fVy = (pObjKinematic->fVrelY +
                             pInputData->pEgoData->pEgoDynObjSync->fVelocityY) +
                            (fObjDistX * EGO_YAW_RATE_OBJ_SYNC);
        pObjMovement->fAx = pObjKinematic->fArelX +
                            pInputData->pEgoData->pEgoDynObjSync->fAccelX;
        pObjMovement->fAy = pObjKinematic->fArelY +
                            pInputData->pEgoData->pEgoDynObjSync->fAccelY;

        /* Moving Objects => correct Speed to more secure value if sensor
         * measurement is incorrect*/
        if (((pObjAttribs->eDynamicProperty ==
              Envm_GEN_OBJECT_DYN_PROPERTY_MOVING) ||
             ((pObjAttribs->eDynamicProperty ==
               Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN) &&
              ((pObjAttribs->eClassification !=
                (Envm_t_GenObjClassification)
                    Envm_GEN_OBJECT_CLASS_PEDESTRIAN) &&
               (pObjAttribs->eClassification !=
                (Envm_t_GenObjClassification)
                    Envm_GEN_OBJECT_CLASS_BICYCLE)))) &&
            (pObjMovement->fVx < 0)) {
            pObjMovement->fVx = 0;
            if (pObjMovement->fAx < 0) {
                pObjMovement->fAx = 0;
            }
        }

        /* Artificially restrict a for oncoming PEDs as not to destroy
         Anec calculation (rework if concept for Anec for oncomings is
         clarified)*/
        if ((pObjKinematic->fVabsX <= 0) &&
            (pObjAttribs->eClassification ==
             (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_PEDESTRIAN)) {
            pObjMovement->fAx = 0;
        }
    }
    /* Apply offset to long. object position used for system test to get earlier
       reaction during during run-up tests */
    pObjMovement->fX = pObjMovement->fX + CD_LONG_OFFSET_SIMU;
    /* Apply offset to lateral object position used for system test to trigger
       reaction on adjacent lane object */
    pObjMovement->fY = pObjMovement->fY + CD_LAT_OFFSET_SIMU;
}

    /* *********************************************************************
  @fn            CDCheckTimesLatMovValid */ /*!

                                   @brief         checks if there are possible
                                 objects around the host
                                                  at time from TTS = 0 until TTC
                                 = 0 for each side

                                   @description   predicts if the area around
                                 the relevant object is free at
                                                  time from TTS = 0 to TTC = 0
                                 for each side. In this case the
                                                  TTS for the corresponding side
                                 will be overtaken for critical
                                                  object

                                   @param[in]     allObjInternal Pointer to all
                                 internal objects
                                   @param[in]     pObjectData Pointer to CD
                                 objects

                                   @return        [none]

                                   @pre           [none]

                                   @post          [none]

                                 ****************************************************************************
                                 */
#if (CD_USE_TTS_OCCUPY_RECOGNITION)
static void CDCheckTimesLatMovValid(CDInternalObjectList_t* allObjInternal,
                                    const CDObjectData_t* const pObjectData) {
    ObjNumber_t critObj;
    ObjNumber_t escapeObjL;
    ObjNumber_t escapeObjR;
    fDistance_t fSafetyOffsetLat;
    fDistance_t fSafetyOffsetLong;

    escapeObjL = 0;
    escapeObjR = 0;
    // critObj       = 0;
    fSafetyOffsetLat = VLC_fEgoVehicleWidth * 0.5f;   /*physical constant*/
    fSafetyOffsetLong = VLC_fEgoVehicleLength * 0.5f; /*physical constant*/
    /*
    fLateralOffsetRight = pParameters->pAdjSafeDistance->fLatNecRemainDist +
    (0.5f * CD_COMMON_EGO_WIDTH) + ( 0.5f * pObjGeometry->fWidth);
    */

    /* find critical object */
    critObj = CDCheckMostCritObj((const CDInternalObjectList_t*)allObjInternal,
                                 pObjectData);

    /* only search for satelite objects and recalc  TTS ( and AnecLat ) in case
     * critical object really exists */
    if (!OBJ_IS_DELETED(critObj)) {
        /* search for satelite objects left and right*/
        escapeObjL = CDCheckEscapeObj(
            (const CDInternalObjectList_t*)allObjInternal, pObjectData,
            fSafetyOffsetLat, fSafetyOffsetLong, critObj);
        escapeObjR = CDCheckEscapeObj(
            (const CDInternalObjectList_t*)allObjInternal, pObjectData,
            -fSafetyOffsetLat, fSafetyOffsetLong, critObj);

        /* take over TTS ( and AnecLat ) from crit obj */
        if ((escapeObjL != critObj) &&
            ((*allObjInternal)[escapeObjL].TTSPreLeft <
             ((*allObjInternal)[critObj].TTSPreLeft))) {
            (*allObjInternal)[critObj].TTSPreLeft =
                (*allObjInternal)[escapeObjL].TTSPreLeft;
            (*allObjInternal)[critObj].TTSAcuteLeft =
                (*allObjInternal)[escapeObjL].TTSAcuteLeft;
            (*allObjInternal)[critObj].LatNecAccelLeft =
                (*allObjInternal)[escapeObjL].LatNecAccelLeft;
        }
        if ((escapeObjR != critObj) &&
            ((*allObjInternal)[escapeObjR].TTSPreRight <
             ((*allObjInternal)[critObj].TTSPreRight))) {
            (*allObjInternal)[critObj].TTSPreRight =
                (*allObjInternal)[escapeObjR].TTSPreRight;
            (*allObjInternal)[critObj].TTSAcuteRight =
                (*allObjInternal)[escapeObjR].TTSAcuteRight;
            (*allObjInternal)[critObj].LatNecAccelRight =
                (*allObjInternal)[escapeObjR].LatNecAccelRight;
        }

        /* check for new TTS */
        (*allObjInternal)[critObj].TTSPre =
            MAX_FLOAT((*allObjInternal)[critObj].TTSPreLeft,
                      (*allObjInternal)[critObj].TTSPreRight);
        (*allObjInternal)[critObj].TTSAcute =
            MAX_FLOAT((*allObjInternal)[critObj].TTSAcuteLeft,
                      (*allObjInternal)[critObj].TTSAcuteRight);
    }

    /* return */
    return;
}

/* ***********************************************************************
  @fn            CDCheckMostCritObj */ /*!

                                      @brief         Searches for most critical
                                    object for ego vehicle

                                      @description   This function searches for
                                    the most critical object by checking
                                                     the TTS for each object
                                    (left for objects on left side and vice
                                                     reversa).
                                                     This search is based on the
                                    suggestion that the lowest TTS acute
                                                     represents the most
                                    critical object

                                      @param[in]     allObjInternal Pointer to
                                    all internal objects
                                      @param[in]     pObjectData Pointer to CD
                                    objects

                                      @return        critObj meaning the found
                                    most critical object

                                      @pre           [none]

                                      @post          [none]

                                    ****************************************************************************
                                    */
static ObjNumber_t CDCheckMostCritObj(
    const CDInternalObjectList_t* allObjInternal,
    const CDObjectData_t* const pObjectData) {
    ObjNumber_t iCheckObj;
    ObjNumber_t critObj;
    fTime_t checkLowestTTS;
    fTime_t proofLowestTTS;

    critObj = 0;
    checkLowestTTS = 0;
    proofLowestTTS = CD_TIME_MAX;

    /* search for most critical object */
    for (iCheckObj = 0; iCheckObj < pObjectData->iNumberOfObjects;
         iCheckObj++) {
        /* quality of object to check shall have a minimum value (also to
         * exclude non-filled objects */
        if (CD_EBA_OBJ_QUALITY(pObjectData, iCheckObj) >=
            CD_COMMON_MIN_OBJ_QUALITY) {
            checkLowestTTS = (*allObjInternal)[iCheckObj].TTSAcute;
            /* search for lowest TTS acute but bigger than zero (not
             * calculated?) <-- check this! */
            if ((checkLowestTTS < proofLowestTTS) &&
                (checkLowestTTS > C_F32_DELTA)) {
                /* actual object TTS is lower than the one to proof so check
                 * object is for now the most critical */
                proofLowestTTS = checkLowestTTS;
                critObj = iCheckObj;
            }
        }
    }
    /* return */
    return critObj;
}

/* ***********************************************************************
  @fn            CDCheckEscapeObj */ /*!

                                        @brief         Searches for next
                                      critical object with respect to the
                                      handled
                                                       over object

                                        @description   This function searches
                                      for the next critical object via function
                                                       "CDCheckClosest". This
                                      search gets aborted in case the function
                                                       returns the actual object
                                      or between the 2 check objects is
                                                       enough space to drive
                                      through.

                                        @param[in]     allObjInternal Pointer to
                                      all internal objects
                                        @param[in]     pObjectData Pointer to CD
                                      objects
                                        @param[in]     fSafetyOffsetLat lateral
                                      safety offset [half the ego vehicle width]
                                        @param[in]     fSafetyOffsetLong
                                      longitudinal safety offset [half the ego
                                      vehicle length]
                                        @param[in]     actObj actual object for
                                      which we want to check the next lateral
                                      object

                                        @param[out]    -

                                        @return        escapeObj meaning the
                                      found most left/right object with respect
                                      to
                                                       handled over object

                                        @pre           [none]

                                        @post          [none]

                                      ****************************************************************************
                                      */
static ObjNumber_t CDCheckEscapeObj(
    const CDInternalObjectList_t* allObjInternal,
    const CDObjectData_t* const pObjectData,
    fDistance_t fSafetyOffsetLat,
    fDistance_t fSafetyOffsetLong,
    ObjNumber_t actObj) {
    boolean bSearchLeft;

    ObjNumber_t nextObj;
    ObjNumber_t escapeObj;
    uint8 iControl;
    fDistance_t freeSpace;

    nextObj = 0;
    escapeObj = 0;
    freeSpace = 0;

    /*
    by checking the lateral safety offset we decide whether to check left or
    right side
      - bigger zero means left side
      - smaller or equal means right side
    */
    if (fSafetyOffsetLat > 0) {
        bSearchLeft = TRUE;
    } else {
        bSearchLeft = FALSE;
    }

    escapeObj = actObj;
    /* for first instance check the next object with relation to handled over
     * object */
    nextObj = CDCheckClosest(allObjInternal, pObjectData, &(freeSpace),
                             bSearchLeft, escapeObj, actObj, fSafetyOffsetLong);

    /* search for up to 5 more objects the next left/right object that could
     * become dangerous for our ego vehicle */
    for (iControl = 0u; iControl < 5u; iControl++) {
        /*
        abort the loop incase there is no more left/right object (here shown as
        returned object is same as handled over one) or there is enough space to
        drive between the 2 objects
        */
        if ((freeSpace > (VLC_fEgoVehicleWidth + fABS(fSafetyOffsetLat))) ||
            (nextObj == actObj)) {
            break;
        }
        /* here escape object is set to "last" checked object cause if we find
         * no more object we want to return the last found */
        escapeObj = nextObj;
        nextObj =
            CDCheckClosest(allObjInternal, pObjectData, &(freeSpace),
                           bSearchLeft, nextObj, actObj, fSafetyOffsetLong);
    }
    /* return */
    return escapeObj;
}

#endif
/* ***********************************************************************
  @fn            CDCheckClosest */ /*!

                                          @brief         Searches for object
                                        with lowest lateral displacement on one
                                        side

                                          @description   This function searches
                                        for the lateral closest object on one
                                        side
                                                         and returns the found
                                        object. Also proofes the longitudinal
                                                         displacement via
                                        function "CDCheckEscObjLong".
                                                         Only return object in
                                        case it is critical in longitudinal
                                        direction
                                                         AND in lateral
                                        direction AND it is longitudinal close
                                        to ego vehicle
                                                         (here the object is
                                        only marked as critical in case it has a
                                        TTC
                                                         that is only a little
                                        bigger (1sec) than the one of the
                                        critical
                                                         object cause in other
                                        cases the object is that far away that
                                        it is
                                                         not critical)

                                          @param[in]     allObjInternal Pointer
                                        to all internal objects
                                          @param[in]     pObjectData Pointer to
                                        CD objects
                                          @param[in]     pFreeSpace pointer to
                                        handle over the space between critical
                                        object
                                                         and actual object
                                          @param[in]     bSearchLeft boolean
                                        variable that describes if we seacht on
                                        left or
                                                         right side of ego
                                        vehicle
                                          @param[in]     actObj actual object
                                        for which we want to check the next
                                        lateral object
                                          @param[in]     critObj critical object
                                        from which we want to check lateral
                                        distance and
                                                         (only to handle over
                                        into sub-function no real use here...)
                                                         compare distances and
                                        times
                                          @param[in]     fSafetyOffsetLong
                                        longitudinal safety offset [half the ego
                                        vehicle length]

                                          @param[out]    -

                                          @return        closestObj returns the
                                        closest lateral object

                                          @pre           [none]

                                          @post          [none]

                                        ****************************************************************************
                                        */
static ObjNumber_t CDCheckClosest(const CDInternalObjectList_t* allObjInternal,
                                  const CDObjectData_t* const pObjectData,
                                  fDistance_t* pFreeSpace,
                                  boolean bSearchLeft,
                                  ObjNumber_t actObj,
                                  ObjNumber_t critObj,
                                  fDistance_t fSafetyOffsetLong) {
    fDistance_t compDist;
    ObjNumber_t iCheckObj;
    ObjNumber_t closestObj;
    boolean bLongCrit;
    fTime_t TTS2Check;

    closestObj = actObj;
    compDist = 100.f;

    for (iCheckObj = 0; iCheckObj < pObjectData->iNumberOfObjects;
         iCheckObj++) {
        if (((CD_EBA_OBJ_QUALITY(pObjectData, iCheckObj) >=
              CD_COMMON_MIN_OBJ_QUALITY) &&
             (pObjectData->pGenObjList->aObject[iCheckObj]
                  .Attributes.eClassification != Envm_GEN_OBJECT_CLASS_POINT) &&
             (pObjectData->pGenObjList->aObject[iCheckObj]
                  .Attributes.eClassification !=
              Envm_GEN_OBJECT_CLASS_UNCLASSIFIED)) ||
            ((CD_EBA_OBJ_QUALITY(pObjectData, iCheckObj) >=
              CD_COMMON_MIN_OBJ_QUALITY_POINT) &&
             (pObjectData->pGenObjList->aObject[iCheckObj]
                  .Attributes.eClassification ==
              Envm_GEN_OBJECT_CLASS_POINT))) {
            /* decide whether to check left or right side and chose
             * corresponding TTS */
            if (bSearchLeft == TRUE) {
                TTS2Check = (*allObjInternal)[iCheckObj].TTSAcuteLeft;
            } else {
                TTS2Check = (*allObjInternal)[iCheckObj].TTSAcuteRight;
            }
            /* check for longitudinal criticallity */
            bLongCrit =
                CDCheckEscObjLong(allObjInternal, pObjectData, TTS2Check,
                                  iCheckObj, critObj, fSafetyOffsetLong);

            if ((iCheckObj != actObj) &&
                (((bSearchLeft == TRUE) && (bLongCrit == TRUE) &&
                  (pObjectData->pGenObjList->aObject[iCheckObj]
                       .Kinematic.fDistY < compDist) &&
                  (pObjectData->pGenObjList->aObject[iCheckObj]
                       .Kinematic.fDistY >
                   pObjectData->pGenObjList->aObject[actObj]
                       .Kinematic.fDistY)) ||
                 ((bSearchLeft != TRUE) && (bLongCrit == TRUE) &&
                  (pObjectData->pGenObjList->aObject[iCheckObj]
                       .Kinematic.fDistY > -compDist) &&
                  (pObjectData->pGenObjList->aObject[iCheckObj]
                       .Kinematic.fDistY <
                   pObjectData->pGenObjList->aObject[actObj]
                       .Kinematic.fDistY))) &&
                ((*allObjInternal)[iCheckObj].TTC <
                 ((*allObjInternal)[actObj].TTC +
                  CD_CALC_TTC_SAFE_TIME_LONG))) {
                /*
                - select actual object to go for the next object
                - select new lateral distance for next round
                - calculate free space between found object and actual object
                */
                closestObj = iCheckObj;
                compDist = pObjectData->pGenObjList->aObject[iCheckObj]
                               .Kinematic.fDistY;
                *pFreeSpace = fABS(
                    compDist -
                    pObjectData->pGenObjList->aObject[actObj].Kinematic.fDistY);
            }
        }
    }
    /* return */
    return closestObj;
}

/* ***********************************************************************
  @fn            CDCheckEscObjLong */ /*!

                                       @brief         checks if object is
                                     longitudinal critical for critical object

                                       @description   This fucntion checks if
                                     there is a longitudinal criticallity
                                                      from actual checked object
                                     with respect to handled over
                                                      critical object.
                                                      This happens by checking
                                     the predicted distance between checked
                                                      object and ego vehicle at
                                     time TTS and TTC.

                                       @param[in]     allObjInternal Pointer to
                                     all internal objects
                                       @param[in]     pObjectData Pointer to CD
                                     objects
                                       @param[in]     TTS from object to check
                                     (and side to check)
                                       @param[in]     actObj actual object to
                                     check
                                       @param[in]     critObj critical object
                                     from ego vehicle
                                       @param[in]     fSafetyOffsetLong safety
                                     longitudinal offset [half ego vehicle
                                     length]

                                       @param[out]    -

                                       @return        bIsCrit means TRUE in case
                                     object is longitudinal critical and
                                                                    FALSE in
                                     case object is NOT longitudinal critical

                                       @pre           [none]

                                       @post          [none]

                                     ****************************************************************************
                                     */
static boolean CDCheckEscObjLong(const CDInternalObjectList_t* allObjInternal,
                                 const CDObjectData_t* const pObjectData,
                                 fTime_t TTS,
                                 ObjNumber_t actObj,
                                 ObjNumber_t critObj,
                                 fDistance_t fSafetyOffsetLong) {
    boolean bIsCrit;
    fDistance_t fDistTTC;
    fDistance_t fDistTTS;
    fDistance_t Dx;
    float32 Vx;
    float32 Ax;
    fTime_t TTC;

    const Envm_t_GenObjKinEnvmatics* const pObjectKinematics =
        CDGetPointer_Kinematic(pObjectData, actObj);
    const float32 fObjDistX = CD_GET_DIST_X(pObjectData, actObj);

    bIsCrit = FALSE;

    /* variables we need for predicted distance calculation and TTC from crit
     * object */
    Dx = fObjDistX;
    Vx = pObjectKinematics->fVrelX;
    Ax = pObjectKinematics->fArelX;
    TTC = (*allObjInternal)[critObj].TTC;

    /* calculated longitudinal distances for time TTS and TTC */
    fDistTTC = CDCalculateDistance(Dx, Vx, Ax, TTC);
    fDistTTS = CDCalculateDistance(Dx, Vx, Ax, TTS);

    /*
    proof at both times TTS and TTC if distance is smaller than ego length +
    safety offset
    -> means collide (without gap or "driving through" proof by object falls
    behind us before TTS)
    */
    if ((fDistTTC < (VLC_fEgoVehicleLength + fSafetyOffsetLong)) ||
        (fDistTTS < (VLC_fEgoVehicleLength + fSafetyOffsetLong))) {
        bIsCrit = TRUE;
    }
    return bIsCrit;
}

static void limitTTCBrake(fTime_t* const pTTCBrake) {
    //  limit TTCBrake
    if (*pTTCBrake < 0) {
        *pTTCBrake = 0;
    }
    if (*pTTCBrake > CD_TIME_MAX) {
        *pTTCBrake = CD_TIME_MAX;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */