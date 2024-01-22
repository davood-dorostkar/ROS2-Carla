/*
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"
#include "ved.h"
#ifndef _BML_EXT_INCLUDED
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* general WSP definitions */
#define WHS_VEH_SPEED_MIN \
    VED__CURVE_V_MIN /* Minimum Vego to calculate the WSP curve */

/* Filter time mean sample offset factor */
#define WHS_MEAN_SMPL_FT \
    (float32)(0.15F) /* Filter time, filtered sample interval mean */
#define WHS_MEAN_SMPL_DIFF_THR1 \
    (float32)(0.0002F) /* Threshold one to tune up the overtake factor */
#define WHS_MEAN_SMPL_MIN_FACTOR \
    (float32)(7.5F) /* Min Factor if over Thrshold one */
#define WHS_MEAN_SMPL_DIFF_THR2 \
    (float32)(0.0004F) /* Threshold two to tune up the overtake factor */
#define WHS_MEAN_SMPL_MAX_FACTOR \
    (float32)(15.0F) /* Max Factor if over Thrshold one */

/* Bestimmung const Fahrt (ohne Schlenker) */
#define WHS_DELTA_DIST_GRAD_MAX                                               \
    (float32)(1.0F / 1500.F) /* Maximum curve gradient. RDZ for const driving \
                              */
#define WHS_DELTA_DIST_MIN \
    (float32)(             \
        15.F) /* Minimum distance with low gradient for offset compensation */

/* Bestimmung Geradeausfahrt (Straight) */
#define WHS_STRAIGHT_CMAX_QLT_HIGH \
    (float32)(                     \
        1.0F /                     \
        3500.F) /* curve threshold for straight driving with high quality   */
#define WHS_STRAIGHT_CMAX_QLT_LOW \
    (float32)(1.0F /              \
              2000.F) /* Grenzkruemmung geradeaus bei niedriger Spurguete */
#define WHS_STRAIGHT_QLT_HIGH \
    (float32)(0.80F) /* >= Hohe Spurguete ; < niedrige Spurguete */
#define WHS_STRAIGHT_QLT_LOW \
    (float32)(0.05F) /* Minimale Guete fuer Geradeausermittlung */

/* Offsetbestimmung */
#define WHS_OFFS_VSPEED_MIN \
    (float32)(              \
        40.F /              \
        C_KMH_MS) /* Minimale Eigengeschwindigkeit fuer Offsetberechnung */
#define WHS_OFFS_VSPEED_MAX \
    (float32)(              \
        220.F /             \
        C_KMH_MS) /* Maximale Eigengeschwindigkeit fuer Offsetberechnung */
#define WHS_OFFS_SINT_VOLUME_MAX \
    (sint32)(50) /* Max Anzahl der Werte im Sampleintervall                 */
#define WHS_OFFS_DEV_MAX \
    (float32)(0.0015F) /* Maximale Standardabweichung Sampleintervall */
#define WHS_OFFS_MEAN_MAX \
    (float32)(0.0247F) /* Maximale Verhaeltnis Abweichung links/rechts */
#define WHS_OFFS_V_INT_VOL_MAX \
    (float32)(10000.F) /* Maximale Anzahl pro GeschwindigkeitsIntervall */
#define WHS_OFFS_V_INT_RED_FACT \
    (float32)(9.F / 10.F) /* Maximale Anzahl pro GeschwindigkeitsIntervall */
#define WHS_OFFS_INIT_VOLUME \
    (float32)(               \
        1000.F) /* Masimale Anzahl Samples fuer alle Geschwindigkeitsber   */

/* defaults for externally provided whs curve */
#define VED__TRACK_EX_WSP_VAR (0.85F) /* default variance */

/*****************************************************************************
  MACROS
*****************************************************************************/
#define WHS_GET_ME (&VED_WhsGlobData)
#define WHS_GET_MIF_DATA (VED_WhsGlobData.Io.mif)

#define WHS_GET_DATA (&VED_WhsGlobData.Sensor)
#define WHS_GET_OFFS (&VED_WhsGlobData.Offset)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(VED_WhsGlobData)
static VED_WhsData_t
    VED_WhsGlobData; /*!< @VADDR: 0x20014000 @VNAME: VED_Whs  @ALLOW: ved__priv
                        @cycleid: ved__cycle_id*/
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_WhsCommonInit(VED_WhsData_t *pWhsdata);
static void VED_WhsInitWspOffsData(VED_WhsOffsData_t *WspOffsData);

static void VED_WhsTrackStraight(VED_CrvStatus_t *Status,
                                 const VED_OutCurve_t *TrackExWsp);
static void VED_WhsAxleCalcRatioLeftRight(VED_WhsAxisData_t *Axle);
static void VED_WhsCalcOffsetAxle(VED_WhsAxleOffs_t *Offset,
                                  float32 Ratio,
                                  sint32 SpeedRange,
                                  VED_CrvStatus_t TrackStatus,
                                  float32 DeltaDist);

static sint32 VED_WhsVehSpeedRange(float32 VehicleSpeed);
static void VED_WhsSetOffIntLastRange(VED_WhsAxleOffs_t *Offset,
                                      sint32 SpeedRange,
                                      sint32 LastSpeedRange);
static float32 VED_WhsCalcMeanFactor(VED_WhsAxleOffs_t *Offset,
                                     sint32 SpeedRange);

static void VED_WhsInitAxleData(VED_WhsAxisData_t *Axle);
static void VED_WhsInitWspData(VED_WhsSenData_t *WspData);
static void VED_WhsInitWspAxleOffs(VED_WhsAxleOffs_t *Offset);

/* **********************************************************************
  @fn               VED_WhsExec */ /*!

  @brief            Determine operating sequence for vehicle dynamics observer

  @description      Inits WHS module during INIT

  @param[in]        input
  @param[out]       mif
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_WhsExec(const VED_InputData_t *input, VED_ModIf_t *mif) {
    VED_WhsData_t *pWhsdata = WHS_GET_ME;

    pWhsdata->Io.in = input;
    pWhsdata->Io.mif = mif;

    /* Distinguish between different operating states */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                            input->Frame.CtrlMode)) {
        /*<--- Execution path for normal operating mode --->*/

        VED_WhsSenData_t *WspData;      /* RDZ general date                */
        VED_WhsOffsData_t *WspOffsData; /* offset data                     */
        float32 VehicleSpeed;           /* corrected vehicle velocity      */
        VED_OutCurve_t TrackExWsp;      /* overall curve of vehicle        */

        /* get pointer to general wheel speed data */
        WspData = WHS_GET_DATA;

        /* get pointer to offset data */
        WspOffsData = WHS_GET_OFFS;

        /* get curve calculated without wheel speeds */
        TrackExWsp.Gradient = WHS_GET_MIF_DATA->Curve.Gradient;
        TrackExWsp.Variance = VED__TRACK_EX_WSP_VAR;
        TrackExWsp.Curve = WHS_GET_MIF_DATA->Curve.Curve;

        /* get ego velocity */
        VehicleSpeed = WHS_GET_MIF_DATA->LongMot.VehVelocityCorr;

        /* get wheel speeds from module interface */
        WspData->WspFornt.WspLeft = WHS_GET_MIF_DATA->LongMot.FLWhlVelo;
        WspData->WspFornt.WspRight = WHS_GET_MIF_DATA->LongMot.FRWhlVelo;
        if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL, input->Signals.State) ==
             VED_IO_STATE_VALID) &&
            (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR, input->Signals.State) ==
             VED_IO_STATE_VALID)) {
            WspData->WspFornt.WspValid = TRUE;
        } else {
            WspData->WspFornt.WspValid = FALSE;
        }

        WspData->WspRear.WspLeft = WHS_GET_MIF_DATA->LongMot.RLWhlVelo;
        WspData->WspRear.WspRight = WHS_GET_MIF_DATA->LongMot.RRWhlVelo;
        if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL, input->Signals.State) ==
             VED_IO_STATE_VALID) &&
            (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR, input->Signals.State) ==
             VED_IO_STATE_VALID)) {
            WspData->WspRear.WspValid = TRUE;
        } else {
            WspData->WspRear.WspValid = FALSE;
        }

        /* detect driving straight ahead for offset calculation */
        VED_WhsTrackStraight(&WspOffsData->ExWspStatus, &TrackExWsp);

        if (TrackExWsp.Variance >= C_F32_DELTA) {
            /* check gradient for offset calculation */
            VED_CalcDistStblGrad(WHS_DELTA_DIST_GRAD_MAX, TrackExWsp.Gradient,
                                 &WspOffsData->GradAbsOld,
                                 &WspOffsData->WspDeltaDist, VehicleSpeed);
        } else {
            WspOffsData->WspDeltaDist = (float32)0.F;
        }

        /* save last Speed Range */
        WspOffsData->LastSpeedRange = WspOffsData->SpeedRange;

        /* determine speed range for offset calculation */
        WspOffsData->SpeedRange = VED_WhsVehSpeedRange(VehicleSpeed);

        /* check if next SpeedRange was reached */
        if (WspOffsData->SpeedRange != WspOffsData->LastSpeedRange) {
            /* Set front offset to interval mean of last speed range */
            VED_WhsSetOffIntLastRange(&WspOffsData->OffsFront,
                                      WspOffsData->SpeedRange,
                                      WspOffsData->LastSpeedRange);
            /* Set rear offset to interval mean of last speed range */
            VED_WhsSetOffIntLastRange(&WspOffsData->OffsRear,
                                      WspOffsData->SpeedRange,
                                      WspOffsData->LastSpeedRange);
        }

        /* geradeziehen der WSP Spur bei Veigen nahe Null */
        if (VehicleSpeed >= WHS_VEH_SPEED_MIN) {
            /* check if wheel speeds front are valid */
            if (WspData->WspFornt.WspValid == TRUE) {
                /* calculate ratio left/right */
                VED_WhsAxleCalcRatioLeftRight(&WspData->WspFornt);

                /* calculate offset, curvature and quality of wheel speeds front
                 */
                VED_WhsCalcOffsetAxle(
                    &WspOffsData->OffsFront, WspData->WspFornt.Ratio,
                    WspOffsData->SpeedRange, WspOffsData->ExWspStatus,
                    WspOffsData->WspDeltaDist);
            }

            /* check if wheel speeds rear are valid */
            if (WspData->WspRear.WspValid == TRUE) {
                /* calculate ratio left/right */
                VED_WhsAxleCalcRatioLeftRight(&WspData->WspRear);

                /* calculate offset, curvature and quality of wheel speeds rear
                 */
                VED_WhsCalcOffsetAxle(
                    &WspOffsData->OffsRear, WspData->WspRear.Ratio,
                    WspOffsData->SpeedRange, WspOffsData->ExWspStatus,
                    WspOffsData->WspDeltaDist);
            }
        }
    } else {
        /*<--- Execution path for initialization mode  --->*/
        VED_WhsCommonInit(pWhsdata);
    }

    return;
}

/* **********************************************************************
  @fn               VED_WhsInit */ /*!

  @brief            initialize module data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
void VED_WhsInit(void) {
    VED_WhsData_t *pWhsdata = WHS_GET_ME;

    VED_WhsInitWspOffsData(&pWhsdata->Offset);
    VED_WhsCommonInit(pWhsdata);

    return;
}

/* **********************************************************************
  @fn                     VED_WhsGetPrivateData */ /*!
  @brief                  Access to general wheel speed data 

  @description            Provides access to module internal data by
                          providing the pointer to that data

  @param[in]              -
  @param[out]             -
  @return                 *VED_WhsSenData_t pointer

  @pre                    -
  @post                   -
**************************************************************************** */
VED_WhsData_t *VED_WhsGetPrivateData(void) { return (WHS_GET_ME); }

/* **********************************************************************
  @fn                     VED_WhsGetWspAquaplaning */ /*!
  @brief                  Access to aquaplaning BOOL from Wsp 

  @description            see short description

  @param[in]              -
  @param[out]             -
  @return                 boolean aquaplaning dectection flag

  @pre                    -
  @post                   -
**************************************************************************** */
boolean VED_WhsGetWspAquaplaning(void) {
    VED_WhsSenData_t *WspData = WHS_GET_DATA;

    return (WspData->Aquaplaning);
}

/* **********************************************************************
  @fn                     VED_WhsGetOffsData */ /*!
  @brief                  Access to wheel speed offset data 

  @description            Provides access to module internal data by
                          providing a pointer to that data

  @param[in]              -
  @param[out]             -
  @return                 *VED_WhsOffsData_t

  @pre                    -
  @post                   -

**************************************************************************** */
const VED_WhsOffsData_t *VED_WhsGetOffsData(void) { return (WHS_GET_OFFS); }

/* **********************************************************************
  @fn                     VED_WhsOffset */ /*!
  @brief                  Return the offset data used by the autocode

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 *ved__whs_offset_t

  @pre                    -
  @post                   -

  **************************************************************************** */
ved__whs_offset_t VED_WhsOffset(void) {
    float32 fRatio;
    float32 fDeviation;
    const VED_WhsOffsData_t *pWhsOffsData = WHS_GET_OFFS;
    ved__whs_offset_t ved__whs_offset;

    /* velocity range */
    ved__whs_offset.SpeedRange = pWhsOffsData->SpeedRange;

    /* front axle offset ratio */
    fRatio = pWhsOffsData->OffsFront.Interval[pWhsOffsData->SpeedRange].Mean;
    fDeviation = pWhsOffsData->OffsFront.Interval[pWhsOffsData->SpeedRange].Dev;

    if (TUE_CML_IsZero(fRatio)) {
        ved__whs_offset.offset_ratio_front = 1.0F;
        ved__whs_offset.offset_ratio_front_dev = 1.0F;
    } else {
        ved__whs_offset.offset_ratio_front = fRatio;
        ved__whs_offset.offset_ratio_front_dev = fDeviation;
    }

    /* rear axle offset ratio */
    fRatio = pWhsOffsData->OffsRear.Interval[pWhsOffsData->SpeedRange].Mean;
    fDeviation = pWhsOffsData->OffsRear.Interval[pWhsOffsData->SpeedRange].Dev;

    if (TUE_CML_IsZero(fRatio)) {
        ved__whs_offset.offset_ratio_rear = 1.0F;
        ved__whs_offset.offset_ratio_rear_dev = 1.0F;
    } else {
        ved__whs_offset.offset_ratio_rear = fRatio;
        ved__whs_offset.offset_ratio_rear_dev = fDeviation;
    }

    return ved__whs_offset;
}

/* **********************************************************************
  @fn                     VED_WhsInitAxleData */ /*!
  @brief                  Initialises the wheel speed data of one axle

  @description            see brief description

  @param[in]              Axle data
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsInitAxleData(VED_WhsAxisData_t *Axle) {
    Axle->Ratio = (float32)1.F;
    Axle->WspLeft = 0.F;
    Axle->WspRight = 0.F;
    Axle->WspLeftFilt1 = 0.F;
    Axle->WspRightFilt1 = 0.F;
    Axle->WspLeftFilt2 = 0.F;
    Axle->WspRightFilt2 = 0.F;
    Axle->WspValid = FALSE;

    return;
}

/* **********************************************************************
  @fn                     VED_WhsInitWspData */ /*!
  @brief                  Initialises the wheel speed data 

  @description            see brief description

  @param[in]              WspData
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsInitWspData(VED_WhsSenData_t *WspData) {
    VED_WhsInitAxleData(&WspData->WspFornt);
    VED_WhsInitAxleData(&WspData->WspRear);

    WspData->Aquaplaning = FALSE;
    WspData->CurveOld = (float32)0.F;
    WspData->WspFrontRaw.WspLeft = 0.F;
    WspData->WspFrontRaw.WspRight = 0.F;
    WspData->WspRearRaw.WspLeft = 0.F;
    WspData->WspRearRaw.WspRight = 0.F;

#ifdef ZAWSIM32
    WspData->FilterTime = (float32)0.F;
#endif
}

/* **********************************************************************
  @fn                     VED_WhsInitWspAxleOffs */ /*!
  @brief                  Initialises the offset data of one axle

  @description            see brief description

  @param[in]              Axle offset data
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsInitWspAxleOffs(VED_WhsAxleOffs_t *Offset) {
    sint32 ii;

    for (ii = 0; ii < WHS_SPEEED_RANGE_VOLUME; ii++) {
        VED_StatIntervalInit(&Offset->Interval[ii]);
    }

    VED_StatIntervalInit(&Offset->SampleInterval);
    Offset->IntervalRangeVolume = (sint32)0;

    return;
}

/* **********************************************************************
  @fn                     VED_WhsCommonInit */ /*!
  @brief                  Initialises the wheel speed data without the offset data

  @description            Calls VED_WhsInitWspData()
                          Any additional initialisation can be done here

  @param[in]              pWhsdata
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsCommonInit(VED_WhsData_t *pWhsdata) {
    VED_WhsInitWspData(&pWhsdata->Sensor);

    return;
}

/* **********************************************************************
  @fn                     VED_WhsInitWspOffsData */ /*!
  @brief                  Initialisation of complete offset data

  @description            All offset data structures of both axles
                          but not the wheel speed measurement data

  @param[in]              WspOffsData
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsInitWspOffsData(VED_WhsOffsData_t *WspOffsData) {
    VED_WhsInitWspAxleOffs(&WspOffsData->OffsFront);
    VED_WhsInitWspAxleOffs(&WspOffsData->OffsRear);
    WspOffsData->ExWspStatus = (VED_CrvStatus_t)VED__CRV_NOTOK;
    WspOffsData->SpeedRange = (sint32)0;
    WspOffsData->LastSpeedRange = (sint32)0;
    WspOffsData->WspDeltaDist = (float32)0.F;
    WspOffsData->GradAbsOld = (float32)0.F;

    return;
}

/* **********************************************************************
  @fn                     VED_WhsVehSpeedRange */ /*!
  @brief                  Calculates vehicle velocity range / offset interval

  @description

  @param[in]              VehicleSpeed
  @param[out]             -
  @return                 sint32 (velocity range)

  @pre                    -
  @post                   -

**************************************************************************** */
static sint32 VED_WhsVehSpeedRange(float32 VehicleSpeed) {
    sint32 Range;
    sint32 Divisor;

    Divisor = (((sint32)(WHS_OFFS_VSPEED_MAX - WHS_OFFS_VSPEED_MIN)) /
               (WHS_SPEEED_RANGE_VOLUME));
    Range = ((sint32)(VehicleSpeed - WHS_OFFS_VSPEED_MIN) / Divisor);

    Range = TUE_CML_MinMax(0, (WHS_SPEEED_RANGE_VOLUME - 1), Range);

    return (Range);
}

/* **********************************************************************
  @fn                     VED_WHSSetOffIntLastRange */ /*!
  @brief                  Sets Offset intervall to last speed range mean

  @description            Sets offset of one vehicle axle to mean value of last speed range
                          Used if speed range changes
                          Parameter SpeedRange is always within array limits,
                          checked when calculated in VED_WhsVehSpeedRange()
              
  @param[in]              SpeedRange (speed range)
  @param[in]              Offset
  @param[in]              LastSpeedRange
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsSetOffIntLastRange(VED_WhsAxleOffs_t *Offset,
                                      sint32 SpeedRange,
                                      sint32 LastSpeedRange) {
    if ((Offset->Interval[LastSpeedRange].Volume > 0.0F) &&
        (Offset->Interval[SpeedRange].Volume <= WHS_OFFS_INIT_VOLUME)) {
        Offset->Interval[SpeedRange].Sum =
            (Offset->Interval[LastSpeedRange].Sum /
             Offset->Interval[LastSpeedRange].Volume) *
            Offset->Interval[SpeedRange].Volume;
        Offset->Interval[SpeedRange].SqSum =
            (Offset->Interval[LastSpeedRange].SqSum /
             Offset->Interval[LastSpeedRange].Volume) *
            Offset->Interval[SpeedRange].Volume;
    }

    /* calculate offset mean value */
    VED_StatIntervalMeanDev(&Offset->Interval[SpeedRange]);
}

/* **********************************************************************
  @fn                     VED_WhsCalcMeanFactor */ /*!
  @brief                  Calculate the Mean tune factor

  @description            If the difference between filtered mean sample interval
                          and the current speed range interval is high,
                          then the factor is set to a high value

  @param[in]              Offset
  @param[in]              SpeedRange
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static float32 VED_WhsCalcMeanFactor(VED_WhsAxleOffs_t *Offset,
                                     sint32 SpeedRange) {
    float32 Factor;
    float32 Difference;

    /* filter mean of sample interval */
    Offset->SampleIntFiltMean =
        VED_FilterCycleTime(Offset->SampleInterval.Mean,
                            Offset->SampleIntFiltMean, WHS_MEAN_SMPL_FT);

    /* check delta between filtered mean of sample interval with current speed
     * range intervall*/
    Difference =
        fABS(Offset->SampleIntFiltMean - Offset->Interval[SpeedRange].Mean);

    if ((Difference >= WHS_MEAN_SMPL_DIFF_THR1) &&
        (Difference < WHS_MEAN_SMPL_DIFF_THR2)) {
        /* linear function for the factor */
        Factor = ((WHS_MEAN_SMPL_MAX_FACTOR - WHS_MEAN_SMPL_MIN_FACTOR) /
                  (WHS_MEAN_SMPL_DIFF_THR2 - WHS_MEAN_SMPL_DIFF_THR1)) *
                 Difference;
    } else {
        if (Difference >= WHS_MEAN_SMPL_DIFF_THR2) {
            Factor = WHS_MEAN_SMPL_MAX_FACTOR;
        } else {
            Factor = 1.0F;
        }
    }

    return Factor;
}

/* ***********************************************************************
  @fn                     VED_WhsCalcOffsetAxle */ /*!
  @brief                  Calculates offset of one axle

  @description            Offset is calculated if 
                          - car is driving straight
                          - driving straight for a minimum distance
                          - vehicle speed is in a defined speed range
                          to calculate the offset
                          At startup, all intervals are set filled

  @param[in]              Ratio       (ratio left / right)
  @param[in]              SpeedRange  (velocity range)
  @param[in]              TrackStatus (info straight ahead driving)
  @param[in]              DeltaDist   (distance with constant driving)
  @param[out]             Offset
  @return                 void

  @pre                    wheel speed ratio must have already been calculated
  @post                   -

**************************************************************************** */
static void VED_WhsCalcOffsetAxle(VED_WhsAxleOffs_t *Offset,
                                  float32 Ratio,
                                  sint32 SpeedRange,
                                  VED_CrvStatus_t TrackStatus,
                                  float32 DeltaDist) {
    float32 VehicleSpeed; /* corrected ego velocity */

    /* get ego velocity */
    VehicleSpeed = WHS_GET_MIF_DATA->LongMot.VehVelocityCorr;

    /* check if offset compensation can be done */
    if ((TrackStatus ==
         (VED_CrvStatus_t)VED__CRV_OK)       /* driving straight ahead? */
        && (DeltaDist >= WHS_DELTA_DIST_MIN) /* minimum driven distance with low
                                                wheel curvature reached? */
        &&
        (VehicleSpeed >=
         WHS_OFFS_VSPEED_MIN) /* v ego not too small (noise too big because
                                 number of ticks too low or time too long)? */
        && (VehicleSpeed <=
            WHS_OFFS_VSPEED_MAX)) /* v ego not too big (big offset change
                                     because tyres increase in volume
                                     (Aufquellen der Reifen)? */

    {
        uint8 CaliState =
            WHS_GET_ME->Io.in->Frame.CaliMode; /* calibration mode states */
        /* driving straight ahead detected  */

        /* add samples only if wheel speed calibration mode is off */
        if (!VED__CTRL_GET_STATE((uint8)VED_CAL_WHS_OFFS, CaliState)) {
            /* add value to interval */
            VED_StatIntervalAdd(&Offset->SampleInterval, (Ratio),
                                (float32)1.0F);

            /* mark speed range, offset calculation started before overrun could
             * occur */
            Offset->IntervalRangeVolume += SpeedRange;
        }

        if (Offset->SampleInterval.Volume >=
            (float32)WHS_OFFS_SINT_VOLUME_MAX) {
            /* calculate mean and standard deviation */
            VED_StatIntervalMeanDev(&Offset->SampleInterval);

            /* check mean and standard deviation */
            if ((fABS(Offset->SampleInterval.Mean - 1.0F) <=
                 ((float32)WHS_OFFS_MEAN_MAX)) &&
                (Offset->SampleInterval.Dev <= WHS_OFFS_DEV_MAX)) {
                sint32 IntSpeedRange = 0; /* speed range sample interval */
                /* determine speed range for complete interval */
                IntSpeedRange = Offset->IntervalRangeVolume /
                                (sint32)Offset->SampleInterval.Volume;
                IntSpeedRange = TUE_CML_MinMax(0, (WHS_SPEEED_RANGE_VOLUME - 1),
                                               IntSpeedRange);

                /* at startup? no, then calculate offset, otherwise fill
                 * intervals for all speed ranges (else-branch) */
                if (Offset->Interval[0].Volume >= WHS_OFFS_INIT_VOLUME) {
                    /* calculate tuning factor to rise sample interval weight */
                    Offset->Factor =
                        VED_WhsCalcMeanFactor(Offset, IntSpeedRange);

                    /* tune up the sample interval */
                    VED_StatIntervalReduce(&Offset->SampleInterval,
                                           Offset->Factor);

                    /* take sample interval */
                    VED_StatIntervalMerge(&Offset->Interval[IntSpeedRange],
                                          &Offset->SampleInterval);

                    /* calculate mean of offset */
                    VED_StatIntervalMeanDev(&Offset->Interval[IntSpeedRange]);

                    /* check if offset interval should be reduced */
                    if (Offset->Interval[IntSpeedRange].Volume >=
                        WHS_OFFS_V_INT_VOL_MAX) {
                        VED_StatIntervalReduce(&Offset->Interval[IntSpeedRange],
                                               WHS_OFFS_V_INT_RED_FACT);
                    }
                } else {
                    sint32 ii = 0; /* loop counter */
                    /* fill intervals for all speed ranges at startup */
                    for (ii = 0; ii < WHS_SPEEED_RANGE_VOLUME; ii++) {
                        /* take sample interval */
                        VED_StatIntervalMerge(&Offset->Interval[ii],
                                              &Offset->SampleInterval);

                        /* calculate mean of offset */
                        VED_StatIntervalMeanDev(&Offset->Interval[ii]);

                    } /* END FOR */
                }     /* END ELSE */
            }         /* END IF */

            /* reset intervall */
            VED_StatIntervalInit(&Offset->SampleInterval);
            Offset->IntervalRangeVolume = 0;
        }
    }
}

/* ***********************************************************************
  @fn                     VED_WhsTrackStraight */
static void VED_WhsTrackStraight(VED_CrvStatus_t *Status,
                                 const VED_OutCurve_t *TrackExWsp) {
    float32 CurveLimit; /* curvature limit for straight ahead driving
                           (Grenzkruemmung fuer geradeaus) */

    /* Determine curvature limit for straight ahead driving */
    if (TrackExWsp->Variance >= WHS_STRAIGHT_QLT_HIGH) {
        /* curvature limit if quality is high (Grenzkruemmung bei hoher Guete)
         */
        CurveLimit = WHS_STRAIGHT_CMAX_QLT_HIGH;
    } else {
        /* curvature limit if quality is low (Grenzkruemmung bei niedriger
         * Guete) */
        CurveLimit = WHS_STRAIGHT_CMAX_QLT_LOW;
    }

    /* Check for straight ahead driving */
    *Status = VED_CheckCurve(TrackExWsp, CurveLimit, WHS_STRAIGHT_QLT_LOW);

    return;
}

/* **********************************************************************
  @fn                     VED_WhsAxleCalcRatioLeftRight */ /*!
  @brief                  Calculates ratio left / right

  @description            see brief description

  @param[in,out]          Axle data
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_WhsAxleCalcRatioLeftRight(VED_WhsAxisData_t *Axle) {
    if ((Axle->WspLeft > C_F32_DELTA) && (Axle->WspRight > C_F32_DELTA)) {
        /* wheel speeds > 0  -> calculate ratio */
        Axle->Ratio = (float32)(Axle->WspLeft / Axle->WspRight);
    } else {
        Axle->Ratio = (float32)1.0F;
    }
}

/* **********************************************************************
  @fn                     VED_WhsIsValid */ /*!
  @brief                  returns validity of wheel data

  @description            see short description

  @param[in]              - 
  @param[out]             -
  @return                 YwValid 

  @pre                    -
  @post                   -

**************************************************************************** */
boolean VED_WhsIsValid(void) {
    VED_WhsSenData_t *WspData = WHS_GET_DATA;

    return (WspData->WspFornt.WspValid);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
