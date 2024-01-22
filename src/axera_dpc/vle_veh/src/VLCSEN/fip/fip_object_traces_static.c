/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "emp_ext.h"
#include "si_ext.h"
#include "frame_sen_custom_types.h"
#include "vlc_sen.h"
#include "TM_Math_Cal.h"
#include "fip_object_traces.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
LOCAL SYMOBLIC CONSTANTS
*****************************************************************************/
/* CRITERIA FOR TRACE CREATION */
/*! minimum lifetime cycles of obj. */
#define FIP_MOST_LIFETIME_MIN CONV_CYCLES_TO_LIFETIME(7U)
/*! minimum probability of existence */
#define FIP_MOST_PROB_EXIST_MIN (99)
/*! minimum obstacle probability */
#define FIP_MOST_OBST_PROB_MIN ((percentage_t)85)
/*! maximum Y intersection gradient */
#define FIP_MOST_VALID_MAXYISGRAD (0.03f)
/*! maximum Y intersection delta in mm */
#define FIP_MOST_VALID_YISDELTA (0.75f)
/*! Angle between Trace segment and X-axis in degree*/
#define FIP_MOST_VALID_ANGLEVAR (25.0f)
/*!Max Angle between Trace segment and X-axis in degree*/
#define FIP_MOST_VALID_MAXANGLE (8.5f)
/*!Max variance of trace segment to curve*/
#define FIP_MOST_VALID_MAXVAR2CURVE (0.5f)

/*! length [m] for trace deletion when trace conditions are not fulfilled */
#define FIP_MOST_DEL_LENGTH (25.0f)
/*! smart tracing disable flag: optimizations for straight lines using fewer
 * points */
#define FIP_MOST_DISABLE_SMART_SAMPLING (SWITCH_ON)

/*Minimum value of absolute Y intersection gradient filter*/
#define FIP_MOST_MIN_ABS_YINTERSEC_GRAD_FILTR (0.003f)
/*maximum value of Y intersection delta value*/
#define FIP_MOST_MAX_YINTERSEC_DELTA (0.5f)
/*! length [m] for trace deletion when object is deleted */
#define FIP_MOST_DEAD_DEL_LENGTH (40.0f)
/*! time   [s] for trace deletion when object is deleted */
#define FIP_MOST_DEAD_DEL_TIME (5.0f)
/*Min points in trace for updating length */
#define VLC_STATIC_TRACE_TWO_POINTS (2)

/*! cycles for trace deletion when trace conditions are not fulfilled */
#define FIP_MOST_DEL_QUALITY_HYST_CYCLES ((sint8)15)

/*****************************************************************************
LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
GLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Moving object trace structure variable*/
SET_MEMSEC_VAR(FIP_MovObjStaticTraces)
static FIP_t_MovingObjectStaticTraces FIP_MovObjStaticTraces;

/*Moving object trace  pointer*/
FIP_t_MovingObjectStaticTraces *FIP_p_MovingObjectStaticTraces;

/*****************************************************************************
LOCAL VARIABLES
*****************************************************************************/
/*! list of Moving Object Traces */
SET_MEMSEC_VAR(FIP_a_MOSTrace)
static FIP_t_MovingObjStaticTrace FIP_a_MOSTrace[FIP_STATIC_TRACE_NO_OF_TRACES];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
FUNCTION PROTOTYPES
*****************************************************************************/
/* Checks for dead traces*/
static void FIP_v_CheckDeadStaticTrace(void);
/* Checking general conditions for traces creation*/
static void FIP_v_CheckGeneralConditionsStatic(void);
/* Checking conditions for traces creation*/
static void FIP_v_CheckStaticTraceConditions(void);
/* trace sampling once trace created*/
static void FIP_v_MOSTSampling(void);
/* Compensating ego vehicle motion */
static void FIP_v_MOSTEgoMotionCompensation(void);
/* Function to analyze the quality for the created traces, those who fulfilled
 * the trace conditions*/
static void FIP_v_MOSTQualityAnalyse(void);
/*start trace deletion when object is deleted, trace is NOT deleted !! */
static void FIP_v_StartObjectStaticTraceDeletion(ObjNumber_t iObj);
/* Create the traces for the object those who fulfilled the trace conditions*/
static void FIP_v_StaticTraceCreation(ObjNumber_t iObj);
/* if the trace conditions for the object not fulfilled, delete the trace*/
static void FIP_v_CheckStaticTraceDeletion(ObjNumber_t iObj);
/* if the trace does not have any object in it(called dead trace), delete that
 * trace */
static void FIP_v_CheckDeadStaticTraceDeletion(TraceID_t iTrace);
/* to compensate the trace points */
static void FIP_v_MOSTCompensatePoints(TraceID_t iTrc);
/* if the trace is growing more than 25 points, crop the trace */
static void FIP_v_CropStaticTrace(TraceID_t iTrace);
/* to get Y axis intersection for the trace */
boolean FIP_b_GetStaticTraceYAxisIntersection(TraceID_t iTrace,
                                              float32 *pfY,
                                              float32 *pfYVar);

/* Initialize the trace parameters,those who fulfilled the trace conditions*/
static void FIP_v_InitMovingObjectStaticTraces(TraceID_t iTrace);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
/*************************************************************************************************************************
  Functionname:    FIP_v_Init_Static_Traces */
void FIP_v_Init_Static_Traces(void) {
    TraceID_t iTrace;
    ObjNumber_t iObj = 0;

    /* storing the address of the Moving object structure variable in pointer
     * variable*/
    FIP_GET_MOV_OBJ_STATIC_TRACE_DATA_PTR = &FIP_MovObjStaticTraces;
    /* Initialize the trace parameters for FIP_STATIC_TRACE_NO_OF_TRACES
     * traces*/

    for (iTrace = 0u; iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;
         iTrace++) {
        /* Initialize trace parameters*/
        FIP_v_InitMovingObjectStaticTraces(iTrace);
    }
    /*Check all Fct objects and assign INVALID ID i.e 255 before starting actual
     * trace computation*/
    for (iObj = 0; iObj < Envm_N_OBJECTS; iObj++) {
        OBJ_GET_STATIC_TRACE_ID(iObj) = (TraceID_t)FIP_u_TRACE_INVALID_ID;
    }
}
/*************************************************************************************************************************
  Functionname:    FIP_v_InitMovingObjectStaticTraces */
static void FIP_v_InitMovingObjectStaticTraces(TraceID_t iTrace) {
    uint8 uIndex;
    if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) < FIP_u_TRACE_VALID_NO_OBJ_ID) {
        /* reset object data */
        OBJ_GET_STATIC_TRACE_ID(FIP_STATIC_TRACE_GET_VLC_ID(iTrace)) =
            (uint8)FIP_u_TRACE_INVALID_ID;
    }
    /* reset trace data */
    /* Rest variables used in FIP trace computation */
    /*!set initial values for the trace structure members*/
    FIP_a_MOSTrace[iTrace].fLength = 0.0f;
    FIP_a_MOSTrace[iTrace].fTraceWithoutObjTime = -1.0f;
    FIP_a_MOSTrace[iTrace].iBadCycles = (sint8)0;
    FIP_a_MOSTrace[iTrace].YIntersecDelta = 0.0f;
    FIP_a_MOSTrace[iTrace].YIntersecVar = 0.0f;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).ReachedEgoVeh = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).PointNotOnStraightLine = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_NoShadowObj = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_YIntersecGrad = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_MaxAngleGrad = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_AngleGradVar = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_MeanVarMOT2Curve = 0u;
    FIP_STATIC_TRACE_GET_QUALITY(iTrace).valid_YIntersecDelta = 0u;
    for (uIndex = 0; uIndex < FIP_STATIC_TRACE_NO_OF_POINTS; uIndex++) {
        FIP_a_MOSTrace[iTrace].fYVar[uIndex] = 0.f;
    }

    /* Rest variables used in EM/FIP trace computation */
    FIP_STATIC_TRACE_GET_VLC_ID(iTrace) = (uint8)FIP_u_TRACE_INVALID_ID;
    FIP_STATIC_TRACE_GET_EM_ID(iTrace) = (uint8)FIP_u_TRACE_INVALID_ID;

    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->iNumberOfPoints = (sint8)0;

    FIP_STATIC_TRACE_GET_Y_INTERSEC(iTrace) = FIPMOT_INVALID_VALUE;
    FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(iTrace) = 0.0f;
    FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION(iTrace) = FIPMOT_INVALID_VALUE;
    FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION_VAR(iTrace) =
        FIPMOT_INVALID_VALUE;
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->Legacy.TraceReachEgoVeh = 1u;

    for (uIndex = 0; uIndex < FIP_STATIC_TRACE_NO_OF_POINTS; uIndex++) {
        FIP_STATIC_TRACE_GET_X(iTrace)[uIndex] = 0.f;
        FIP_STATIC_TRACE_GET_Y(iTrace)[uIndex] = 0.f;
        FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrace)[uIndex] = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIPMOTCalculateMovingObjectTraces */
void FIP_v_CalculateMovingObjectStaticTraces(void) {
    uint8 i = 0;
    bool_t b_ret = 0;

    /*Check for dead traces. It should be called before Trace creation so it
     * frees VLC object ID to be selected as another trace*/
    FIP_v_CheckDeadStaticTrace();

    /*! check general conditions (vehicle dynamic) for calculating traces */
    /*currently no functionality is implemented, could be implemented in
     * future*/
    FIP_v_CheckGeneralConditionsStatic();

    /*! Check conditions for creation, deletion and holding of traces */
    FIP_v_CheckStaticTraceConditions();

    /*! Sampling of traces: adding new sampling points */
    FIP_v_MOSTSampling();

    /*! EGO motion compensation and shifting of sample points simultaneously */
    FIP_v_MOSTEgoMotionCompensation();

    /*! Quality analysis of traces */
    FIP_v_MOSTQualityAnalyse();

    /*Calculate Y axis intersection of all Traces */
    for (i = 0u; i < FIP_STATIC_TRACE_NO_OF_TRACES; i++) {
        b_ret = FIP_b_GetStaticTraceYAxisIntersection(
            i, &FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION(i),
            &FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION_VAR(i));
    }

    _PARAM_UNUSED(b_ret);
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CheckDeadStaticTrace */
static void FIP_v_CheckDeadStaticTrace(void) {
    TraceID_t iTrace;

    for (iTrace = 0U; iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;
         iTrace++) {
        if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) < TRACE_VALID_NO_OBJ_ID) {
            if ((OBJ_IS_NEW(FIP_STATIC_TRACE_GET_VLC_ID(iTrace))) ||
                (FIP_STATIC_TRACE_GET_EM_ID(iTrace) !=
                 OBJ_GENERAL(FIP_STATIC_TRACE_GET_VLC_ID(iTrace))
                     .uiID))  // OBJ_IS_DELETED(FIP_STATIC_TRACE_GET_VLC_ID(iTrace))
            {
                /*if the object in the trace is deleted, but the trace is not
                 * deleted, then delete the trace*/
                FIP_v_StartObjectStaticTraceDeletion(
                    (ObjNumber_t)FIP_STATIC_TRACE_GET_VLC_ID(iTrace));
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_MOSTSampling */
static void FIP_v_MOSTSampling(void) {
    TraceID_t iTrace;
    float32 fDistanceSqr;

    for (iTrace = 0U; iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;
         iTrace++) {
        if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) < TRACE_VALID_NO_OBJ_ID) {
            /* NORMAL tracing with an active object -> sampling */
            fDistanceSqr =
                SQR(FIP_STATIC_TRACE_GET_X(iTrace)[0] -
                    OBJ_LONG_DISPLACEMENT(
                        FIP_STATIC_TRACE_GET_VLC_ID(iTrace))) +
                SQR(FIP_STATIC_TRACE_GET_Y(iTrace)[0] -
                    OBJ_LAT_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTrace)));

            if (fDistanceSqr > SQR(FIP_MOST_SAMPLING_LENGTH)) {
                /* SAMPLING distance reached -> add/replace point */

                /*!@todo only allow traces as function from X (y=f(x)), if
                 * extension is needed remove deletion here */
                if ((FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >= 1) &&
                    (OBJ_LONG_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(
                         iTrace)) <= FIP_STATIC_TRACE_GET_X(iTrace)[0])) {
                    /*! new point is nearer than last one -> delete trace */
                    FIP_v_InitMovingObjectStaticTraces(iTrace);
                } else {
                    /* ADD new supporting point */

                    /*! check if new point fit to an straight line */
                    /* points NOT on a STRAIGHT line */
                    /* insert mode : for ego motion comp. WITH insertion */
                    FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace)++;
                    FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) *= ((sint8)-1);
                    FIP_STATIC_TRACE_GET_QUALITY(iTrace)
                        .PointNotOnStraightLine = 1U;

                    /* FIP_a_MOSTrace[iTrace].fLength     += SQRT( fDistanceSqr
                     * ); */
                }
            } else {
                /* do nothing ( just compensate ) */
            }
        } else {
            /* do nothing (no trace) */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_MOSTEgoMotionCompensation */
static void FIP_v_MOSTEgoMotionCompensation(void) {
    TraceID_t iTrace;
    float32 fDistance;
    float32 fDeltaX, fDeltaY;
    sint8 iPt;

    for (iTrace = 0U; iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;
         iTrace++) {
        if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) < FIP_u_TRACE_VALID_OBJ_ID) {
            /* valid trace */
            /*! @todo check this position for deletion check */
            /* ego motion compensation */
            if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >= 0) {
                FIP_v_MOSTCompensatePoints(iTrace);
            } else {
                /* SHIFT mode : save ego motion comp. result in next array
                 * element , add new entry */

                /* cNoPts is already increased */
                /* reset mode */
                FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) *= -1;

                /* check for points that will be shifted out */
                if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >
                    FIP_STATIC_TRACE_NO_OF_POINTS) {
                    /* list is full and will overflow */
                    /* calc distance between last 2 points */
                    fDeltaX = FIP_STATIC_TRACE_GET_X(
                                  iTrace)[FIP_STATIC_TRACE_NO_OF_POINTS - 1] -
                              FIP_STATIC_TRACE_GET_X(
                                  iTrace)[FIP_STATIC_TRACE_NO_OF_POINTS - 2];
                    fDeltaY = FIP_STATIC_TRACE_GET_Y(
                                  iTrace)[FIP_STATIC_TRACE_NO_OF_POINTS - 1] -
                              FIP_STATIC_TRACE_GET_Y(
                                  iTrace)[FIP_STATIC_TRACE_NO_OF_POINTS - 2];
                    fDistance = SQRT_(SQR(fDeltaX) + SQR(fDeltaY));
                    /* crop length */
                    FIP_a_MOSTrace[iTrace].fLength = MAX_FLOAT(
                        (FIP_a_MOSTrace[iTrace].fLength - fDistance), 0.0f);
                    /* limit NoPts to max */
                    FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) =
                        (sint8)FIP_STATIC_TRACE_NO_OF_POINTS;
                }
                /* shift points */
                for (iPt = (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1);
                     iPt >= 1; iPt--) {
                    /* shift */
                    FIP_STATIC_TRACE_GET_X(iTrace)
                    [iPt] = FIP_STATIC_TRACE_GET_X(iTrace)[iPt - 1];
                    FIP_STATIC_TRACE_GET_Y(iTrace)
                    [iPt] = FIP_STATIC_TRACE_GET_Y(iTrace)[iPt - 1];
                    FIP_a_MOSTrace[iTrace].fYVar[iPt] =
                        FIP_a_MOSTrace[iTrace].fYVar[iPt - 1];
                    FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrace)
                    [iPt] = FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrace)[iPt - 1];
                }
                /*compensate points*/
                FIP_v_MOSTCompensatePoints(iTrace);
                /* add new point */
                FIP_STATIC_TRACE_GET_X(iTrace)
                [0] =
                    OBJ_LONG_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTrace));
                FIP_STATIC_TRACE_GET_Y(iTrace)
                [0] = OBJ_LAT_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTrace));
                FIP_a_MOSTrace[iTrace].fYVar[0] =
                    FIP_f_GetObjObservationVariance(
                        (ObjNumber_t)FIP_STATIC_TRACE_GET_VLC_ID(iTrace));
                FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrace)
                [0] = SQRT_(FIP_a_MOSTrace[iTrace].fYVar[0]);
                /* update length */
                if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >=
                    VLC_STATIC_TRACE_TWO_POINTS) /*VLC_STATIC_TRACE_TWO_POINTS*/
                {
                    fDeltaX = FIP_STATIC_TRACE_GET_X(iTrace)[0] -
                              FIP_STATIC_TRACE_GET_X(iTrace)[1];
                    fDeltaY = FIP_STATIC_TRACE_GET_Y(iTrace)[0] -
                              FIP_STATIC_TRACE_GET_Y(iTrace)[1];
                    fDistance = SQRT_(SQR(fDeltaX) + SQR(fDeltaY));

                    FIP_a_MOSTrace[iTrace].fLength += fDistance;
                }
            }

            /*! delete next to last point of trace, if it's behind sensor
             * vehicle */
            FIP_v_CropStaticTrace(iTrace);
        } else {
            /* no trace -> do nothing */
        }
    } /* for */
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CropStaticTrace */
static void FIP_v_CropStaticTrace(TraceID_t iTrace) {
    float32 fDeltaX, fDeltaY;
    float32 fDistance;
    sint8 cNoPts;

    cNoPts = FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace);

    if (cNoPts >= VLC_STATIC_TRACE_TWO_POINTS) {
        /* when next to last point is behind ego vehicle crop trace */
        if (FIP_STATIC_TRACE_GET_X(iTrace)[cNoPts - 2] < 0.0f) {
            /* calc distance between last 2 points */
            fDeltaX = FIP_STATIC_TRACE_GET_X(iTrace)[cNoPts - 2] -
                      FIP_STATIC_TRACE_GET_X(iTrace)[cNoPts - 1];
            fDeltaY = FIP_STATIC_TRACE_GET_Y(iTrace)[cNoPts - 2] -
                      FIP_STATIC_TRACE_GET_Y(iTrace)[cNoPts - 1];
            fDistance = SQRT_(SQR(fDeltaX) + SQR(fDeltaY));

            /* reset value */
            FIP_STATIC_TRACE_GET_X(iTrace)[cNoPts - 1] = FIPMOT_INVALID_VALUE;
            /* decrease nr. of points */
            FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) =
                (sint8)MAX((FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1), 0);

            FIP_a_MOSTrace[iTrace].fLength =
                MAX_FLOAT((FIP_a_MOSTrace[iTrace].fLength - fDistance), 0.0f);
        }
    } else {
        /* < 2 points -> nothing to crop */
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_MOSTQualityAnalyse */
static void FIP_v_MOSTQualityAnalyse(void) {
    float32 f_tempYIntersection = FIPMOT_INVALID_VALUE, f_tempYIntersectionVar,
            f_tempGradient;
    float32 a_Angle[FIP_STATIC_TRACE_NO_OF_POINTS];
    float32 a_AngleGradient[FIP_STATIC_TRACE_NO_OF_POINTS];
    float32 f_DrivenDist, f_dX, f_dY;
    float32 f_Slope, f_C0, f_Y0;
    float32 f_temp_AngleGradVar, f_max_AngleGrad;
    float32 f_maxY, f_maxX, f_minX, f_minY, f_maxXmaxX, f_minXminX;
    float32 f_YCurve, f_temp_MeanVarMOT2Curve = 0.0f;
    bool_t b_valid;
    TraceID_t i;
    sint8 j;

    float32
        f_AbsYIntersectGradFilt; /* fABS(VLC_TRACE_GET_Y_INTERSEC_GRAD_FILT(i))
                                    */
    float32 f_AbsYIntersectDelta; /* fABS(FIP_a_MOSTrace[i].YIntersecDelta) */

    f_DrivenDist = TASK_CYCLE_TIME * EGO_SPEED_X_OBJ_SYNC;

    for (i = 0u; i < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES; i++) {
        if (FIP_STATIC_TRACE_GET_VLC_ID(i) < (uint8)FIP_u_TRACE_INVALID_ID) {
            /* check, if trace tail has reached ego-vehicle */

            /* evaluate quality */
            FIP_STATIC_TRACE_GET_QUALITY(i)
                .ReachedEgoVeh =
                (FIP_STATIC_TRACE_GET_X(
                     i)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(i) - 1] < 0.0f)
                    ? 1u
                    : 0u;

            FIP_STATIC_TRACE_REACHED_EGO_VEH(i) =
                (boolean)FIP_STATIC_TRACE_GET_QUALITY(i).ReachedEgoVeh;

            /* analyse y-intersection of trace tail */

            if ((f_DrivenDist > C_F32_DELTA) &&
                (FIP_a_MOSTrace[i]
                     .Quality
                     .ReachedEgoVeh)) /* use direct access to the quality
                                         instead of
                                         FIP_STATIC_TRACE_GET_QUALITY(i).ReachedEgoVeh,
                                         remove misara 12.4 */

            {
                b_valid = FIP_b_GetStaticTraceYAxisIntersection(
                    i, &f_tempYIntersection, &f_tempYIntersectionVar);
                if ((b_valid == TRUE) && (FIP_STATIC_TRACE_GET_Y_INTERSEC(i) <
                                          FIPMOT_INVALID_VALUE)) {
                    f_tempGradient = (FIP_STATIC_TRACE_GET_Y_INTERSEC(i) -
                                      f_tempYIntersection) /
                                     f_DrivenDist;
                    FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(i) =
                        ((0.8f) *
                         FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(i)) +
                        ((0.2f) * f_tempGradient);
                    FIP_a_MOSTrace[i].YIntersecDelta +=
                        (FIP_STATIC_TRACE_GET_Y_INTERSEC(i) -
                         f_tempYIntersection);

                    f_AbsYIntersectGradFilt =
                        fABS(FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(i));
                    f_AbsYIntersectDelta =
                        fABS(FIP_a_MOSTrace[i].YIntersecDelta);

                    if ((f_AbsYIntersectGradFilt <
                         FIP_MOST_MIN_ABS_YINTERSEC_GRAD_FILTR) &&
                        (f_AbsYIntersectDelta > FIP_MOST_MAX_YINTERSEC_DELTA)) {
                        FIP_a_MOSTrace[i].YIntersecDelta = 0.0f;
                    }
                    /* evaluate quality */
                    FIP_STATIC_TRACE_GET_QUALITY(i)
                        .valid_YIntersecGrad =
                        (fABS(FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(i)) <
                         FIP_MOST_VALID_MAXYISGRAD)
                            ? 1u
                            : 0u;
                    FIP_STATIC_TRACE_GET_QUALITY(i)
                        .valid_YIntersecDelta =
                        (fABS(FIP_a_MOSTrace[i].YIntersecDelta) <
                         FIP_MOST_VALID_YISDELTA)
                            ? 1u
                            : 0u;
                } else {
                    /* evaluate quality */
                    FIP_STATIC_TRACE_GET_QUALITY(i).valid_YIntersecGrad = 1u;
                    FIP_STATIC_TRACE_GET_QUALITY(i).valid_YIntersecDelta = 1u;
                }
                FIP_STATIC_TRACE_GET_Y_INTERSEC(i) = f_tempYIntersection;
                FIP_a_MOSTrace[i].YIntersecVar = f_tempYIntersectionVar;
            } else {
                /* evaluate quality */
                FIP_STATIC_TRACE_GET_QUALITY(i).valid_YIntersecGrad = 1u;
                FIP_STATIC_TRACE_GET_QUALITY(i).valid_YIntersecDelta = 1u;
            }

            /* analyse angle between two segments */

            f_temp_AngleGradVar = 0.0f;
            f_max_AngleGrad = 0.0f;
            for (j = 1; j < FIP_STATIC_TRACE_GET_NO_OF_POINTS(i); j++) {
                /* calculate angle between segment and x-axis */
                f_dX = (FIP_STATIC_TRACE_GET_X(i)[j] -
                        FIP_STATIC_TRACE_GET_X(i)[j - 1]);
                f_dY = (FIP_STATIC_TRACE_GET_Y(i)[j] -
                        FIP_STATIC_TRACE_GET_Y(i)[j - 1]);
                if (fABS(f_dX) > C_F32_DELTA) {
                    f_Slope = f_dY / f_dX;

                    a_Angle[j] = RAD2DEG(ATAN_(f_Slope));
                } else {
                    a_Angle[j] = 90.0f;
                }
                /* compare angles of two following segments */
                if (j > 1) {
                    a_AngleGradient[j] = a_Angle[j] - a_Angle[j - 1];
                    f_temp_AngleGradVar += SQR(a_AngleGradient[j]);
                    f_max_AngleGrad =
                        MAX_FLOAT(f_max_AngleGrad, fABS(a_AngleGradient[j]));
                }
            }
            if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(i) >
                VLC_STATIC_TRACE_TWO_POINTS) {
                f_temp_AngleGradVar =
                    f_temp_AngleGradVar /
                    ((float32)(FIP_STATIC_TRACE_GET_NO_OF_POINTS(i)) -
                     (float32)(VLC_STATIC_TRACE_TWO_POINTS));
            } else {
                f_temp_AngleGradVar = 0.0f;
            }
            /* evaluate quality */
            FIP_STATIC_TRACE_GET_QUALITY(i)
                .valid_AngleGradVar =
                (fABS(f_temp_AngleGradVar) < FIP_MOST_VALID_ANGLEVAR) ? 1u : 0u;
            FIP_STATIC_TRACE_GET_QUALITY(i)
                .valid_MaxAngleGrad =
                (fABS(f_max_AngleGrad) < FIP_MOST_VALID_MAXANGLE) ? 1u : 0u;

            /* analyse shape of trace */

            if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(i) >
                VLC_STATIC_TRACE_TWO_POINTS) {
                /* fit a curve (y(x) = 0.5*f_C0 *x^2 + YIntersection) through
                 * minimal and maximal point on the trace */
                f_maxY = FIP_STATIC_TRACE_GET_Y(i)[0];
                f_minY = FIP_STATIC_TRACE_GET_Y(
                    i)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(i) - 1];
                f_maxX = FIP_STATIC_TRACE_GET_X(i)[0];
                f_minX = FIP_STATIC_TRACE_GET_X(
                    i)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(i) - 1];
                f_maxXmaxX = f_maxX * f_maxX;
                f_minXminX = f_minX * f_minX;
                if (fABS(f_maxXmaxX - f_minXminX) > C_F32_DELTA) {
                    f_C0 = (2.0f * (f_maxY - f_minY)) /
                           ((f_maxXmaxX) - (f_minXminX));

                    f_Y0 = f_maxY - (0.5f * f_C0 * f_maxXmaxX);

                    /* calc mean variance of trace to curve */
                    f_temp_MeanVarMOT2Curve = 0.0f;
                    for (j = 1; j < FIP_STATIC_TRACE_GET_NO_OF_POINTS(i); j++) {
                        f_YCurve =
                            (0.5f * f_C0 * SQR(FIP_STATIC_TRACE_GET_X(i)[j])) +
                            f_Y0;

                        f_dY = (FIP_STATIC_TRACE_GET_Y(i)[j] - f_YCurve);
                        f_temp_MeanVarMOT2Curve += SQR(f_dY);
                    }
                    f_temp_MeanVarMOT2Curve /=
                        ((float32)FIP_STATIC_TRACE_GET_NO_OF_POINTS(i));
                    FIP_STATIC_TRACE_GET_QUALITY(i)
                        .valid_MeanVarMOT2Curve =
                        (fABS(f_temp_MeanVarMOT2Curve) <
                         FIP_MOST_VALID_MAXVAR2CURVE)
                            ? 1u
                            : 0u;
                } else {
                    FIP_STATIC_TRACE_GET_QUALITY(i).valid_MeanVarMOT2Curve = 1u;
                }
            } else {
                FIP_STATIC_TRACE_GET_QUALITY(i).valid_MeanVarMOT2Curve = 1u;
            }

            /* check, if corresponding object is classified as truck cabin */
            // we do not have shadow value
            if (FALSE /*(FIP_STATIC_TRACE_GET_VLC_ID(i) < TRACE_VALID_NO_OBJ_ID)
        &&(OBJ_IS_SHADOW( FIP_STATIC_TRACE_GET_VLC_ID(i) ) == TRUE )*/) {
                FIP_STATIC_TRACE_GET_QUALITY(i).valid_NoShadowObj = 0u;
            } else {
                FIP_STATIC_TRACE_GET_QUALITY(i).valid_NoShadowObj = 1u;
            }

        } else {
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_b_GetStaticTraceYAxisIntersection */
boolean FIP_b_GetStaticTraceYAxisIntersection(TraceID_t iTrace,
                                              float32 *pfY,
                                              float32 *pfYVar) {
    sint8 s_Tr;
    FIP_t_MovingObjStaticTrace *p_Trace;
    float32 f_Slope, f_SlopeVar, f_TraceDistXhead, f_TraceDistYhead,
        f_TraceDistYVarhead, f_TraceDistXtail, f_TraceDistYtail,
        f_TraceDistYVartail;
    bool_t b_Valid;
    ObjNumber_t u_iTracedObj;

    p_Trace = &(FIP_a_MOSTrace[iTrace]);
    b_Valid = TRUE;

    /*! @todo add dead traces , her only with object */
    /*if the trace has only one point in it.. no 'Y' intersection is possible*/
    if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) < TRACE_VALID_NO_OBJ_ID) {
        u_iTracedObj = (ObjNumber_t)FIP_STATIC_TRACE_GET_VLC_ID(iTrace);
        if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) == 1) {
            if ((FIP_STATIC_TRACE_GET_X(iTrace)[0] -
                 OBJ_LONG_DISPLACEMENT(u_iTracedObj)) < C_F32_DELTA) {
                b_Valid = FALSE;
                *pfY = FIPMOT_INVALID_VALUE;
                *pfYVar = FIPMOT_INVALID_VALUE;
            }
        }

        /* trace valid */
        if (b_Valid == TRUE) {
            /* determine Xmax point */
            /* trace has an object */
            f_TraceDistXhead = OBJ_LONG_DISPLACEMENT(u_iTracedObj);
            f_TraceDistYhead = OBJ_LAT_DISPLACEMENT(u_iTracedObj);
            /* To get the object variance over the distance traveled */
            f_TraceDistYVarhead = FIP_f_GetObjObservationVariance(u_iTracedObj);

            /* determine Xmin point */
            f_TraceDistXtail = FIP_STATIC_TRACE_GET_X(
                iTrace)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1];
            f_TraceDistYtail = FIP_STATIC_TRACE_GET_Y(
                iTrace)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1];
            f_TraceDistYVartail =
                p_Trace->fYVar[FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1];

            if (f_TraceDistXhead >= 0.0f) {
                /*****************************************/
                /* trace HEAD is IN FRONT OF ego vehicle */
                if (f_TraceDistXtail > 0.0f) {
                    /*****************************************/
                    /* trace HEAD is IN FRONT OF ego vehicle */
                    /* trace TAIL is IN FRONT OF ego vehicle */
                    /*   -> predict/interpolate to Y-axis    */
                    s_Tr = (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1);

                    if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >=
                        VLC_STATIC_TRACE_TWO_POINTS) {
                        if (fABS(FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail) > C_F32_DELTA) {
                            /*if the trace has maximum 2 points in it , find the
                            slope
                            SLOPE = Y/X;*/
                            f_Slope =
                                (FIP_STATIC_TRACE_GET_Y(iTrace)[s_Tr - 1] -
                                 f_TraceDistYtail) /
                                (FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail);
                            f_SlopeVar =
                                (p_Trace->fYVar[s_Tr - 1] -
                                 f_TraceDistYVartail) /
                                (FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail);
                            /* y(x=0)       =           dx         *  slope + y0
                             */
                            *pfY = (-(f_TraceDistXtail)*f_Slope) +
                                   f_TraceDistYtail;
                            *pfYVar = (-(f_TraceDistXtail)*f_SlopeVar) +
                                      f_TraceDistYVartail;
                            b_Valid = TRUE;
                        } else {
                            /* last segment is nearly parallel to y-axis */
                            /* y-intersection does not exist */
                            b_Valid = FALSE;
                            *pfY = FIPMOT_INVALID_VALUE;
                            *pfYVar = FIPMOT_INVALID_VALUE;
                        }
                    } else {
                        /* trace has only one point */
                        /* extrapolating an y-intersection is not reasonable */
                        b_Valid = FALSE;
                        *pfY = FIPMOT_INVALID_VALUE;
                        *pfYVar = FIPMOT_INVALID_VALUE;
                    }
                } else {
                    /*****************************************/
                    /* trace HEAD is IN FRONT OF ego vehicle */
                    /* trace TAIL is BEHIND      ego vehicle */
                    /*   -> sample it                        */
                    if (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) == 1) {
                        if (fABS(f_TraceDistXhead - f_TraceDistXtail) >
                            C_F32_DELTA) {
                            /* interpolate between obj and the one and only
                             * trace point */
                            f_Slope = (f_TraceDistYhead - f_TraceDistYtail) /
                                      (f_TraceDistXhead - f_TraceDistXtail);
                            f_SlopeVar =
                                (f_TraceDistYVarhead - f_TraceDistYVartail) /
                                (f_TraceDistXhead - f_TraceDistXtail);
                            /* y(x=0)       =           f_dX       *  slope +
                             * f_Y0              */
                            *pfY = (-(f_TraceDistXtail)*f_Slope) +
                                   f_TraceDistYtail;
                            *pfYVar = (-(f_TraceDistXtail)*f_SlopeVar) +
                                      f_TraceDistYVartail;
                            b_Valid = TRUE;
                        } else {
                            /* obj and one-and-only trace point are nearly
                             * parallel to y-axis */
                            /* y-intersection does not exist */
                            b_Valid = FALSE;
                            *pfY = FIPMOT_INVALID_VALUE;
                            *pfYVar = FIPMOT_INVALID_VALUE;
                        }
                    } else {
                        /* interpolate between last 2 trace points */
                        s_Tr = (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1);
                        if (fABS(FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail) > C_F32_DELTA) {
                            f_Slope =
                                (FIP_STATIC_TRACE_GET_Y(iTrace)[s_Tr - 1] -
                                 f_TraceDistYtail) /
                                (FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail);
                            f_SlopeVar =
                                (p_Trace->fYVar[s_Tr - 1] -
                                 f_TraceDistYVartail) /
                                (FIP_STATIC_TRACE_GET_X(iTrace)[s_Tr - 1] -
                                 f_TraceDistXtail);
                            /* y(x=0)       =           f_dX       *  slope +
                             * f_Y0              */
                            *pfY = (-(f_TraceDistXtail)*f_Slope) +
                                   f_TraceDistYtail;
                            *pfYVar = (-(f_TraceDistXtail)*f_SlopeVar) +
                                      f_TraceDistYVartail;
                            b_Valid = TRUE;
                        } else {
                            /* last segment is nearly parallel to y-axis */
                            /* y-intersection does not exist */
                            b_Valid = FALSE;
                            *pfY = FIPMOT_INVALID_VALUE;
                            *pfYVar = FIPMOT_INVALID_VALUE;
                        }
                    }
                }
            } else {
                /************************************/
                /* trace HEAD is BEHIND ego vehicle */
                *pfY = FIPMOT_INVALID_VALUE;
                *pfYVar = FIPMOT_INVALID_VALUE;
                b_Valid = FALSE;
            }
        }
    } else {
        /* no valid trace */
        b_Valid = FALSE;
        *pfY = FIPMOT_INVALID_VALUE;
        *pfYVar = FIPMOT_INVALID_VALUE;
    }
    return b_Valid;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_MOSTCompensatePoints */
static void FIP_v_MOSTCompensatePoints(TraceID_t iTrc) {
    float32 f_YJitter;
    const GDBTrafoMatrix2D_t *p_Mat = FIPGetTrafoMatrix2DCOFFwdTgtSync();
    const GDBTrafoMatrix2D_t *p_MatJitter =
        FIPGetTrafoMatrix2DCOFForJitTgtSync();
    sint8 s_Pt;

    /*update the ego motion matrices*/
    for (s_Pt = (sint8)0; s_Pt < MIN(FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrc),
                                     (sint8)FIP_STATIC_TRACE_NO_OF_POINTS);
         s_Pt++) {
        /* compensate */
        f_YJitter =
            GDBmathTrafoYPos2D(p_MatJitter, FIP_STATIC_TRACE_GET_X(iTrc)[s_Pt],
                               FIP_STATIC_TRACE_GET_Y(iTrc)[s_Pt]);
        GDBmathTrafoPos2D(p_Mat, &(FIP_STATIC_TRACE_GET_X(iTrc)[s_Pt]),
                          &(FIP_STATIC_TRACE_GET_Y(iTrc)[s_Pt]));

        FIP_a_MOSTrace[iTrc].fYVar[s_Pt] +=
            SQR(FIP_STATIC_TRACE_GET_Y(iTrc)[s_Pt] - f_YJitter);

        FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrc)
        [s_Pt] = SQRT_(FIP_a_MOSTrace[iTrc].fYVar[s_Pt]);
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CheckGeneralConditionsStatic */
static void FIP_v_CheckGeneralConditionsStatic(void) {
    /* GENERAL CONDITIONS */
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CheckStaticTraceConditions */
static void FIP_v_CheckStaticTraceConditions(void) {
    ObjNumber_t iObj;
    TraceID_t iTrace;
    boolean bTraceConditions;

    /* OBJECT CONDITIONS */
    for (iObj = 0; iObj < Envm_N_OBJECTS; iObj++) {
        bTraceConditions = FALSE;
        /*! The conditions for creating traces for the object as follows
         1.The Object should be in moving or in stationary state
         2.The object was moving and now stopped and if stopped confidence is
         above 80
         3.probability of existence of the object is greater than or equal to
         99.5% */
        if ((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_MOVING) ||
            ((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
             (OBJ_IS_MOVING_TO_STATIONARY(iObj)))) {
            if (!(OBJ_MAINTENANCE_STATE(iObj) ==
                  Envm_GEN_OBJECT_MT_STATE_DELETED)) {
                if ((OBJ_LIFECYCLES(iObj) >= FIP_MOST_LIFETIME_MIN)) {
                    if ((OBJ_PROBABILITY_OF_EXIST(iObj) >=
                         FIP_MOST_PROB_EXIST_MIN)) {
                        bTraceConditions =
                            TRUE; /* if ALL conditions are not fulfilled,
                                     bTraceConditions stays FALSE */
                    }             /* ( OBJ_PROBABILITY_OF_EXIST(iObj)    >=
                                     FIP_MOST_PROB_EXIST_MIN   ) */
                } /* IF ( OBJ_LIFECYCLES(iObj)   >=  FIP_MOST_LIFETIME_MIN   )
                     */
            }     /* IF !(OBJ_MAINTENANCE_STATE(iObj) ==
                     Envm_GEN_OBJECT_MT_STATE_DELETED) */
        }

        /* Trace creation/deletion */
        if (bTraceConditions == TRUE) {
            /* For current object, all trace conditions are fulfilled */
            if (OBJ_GET_STATIC_TRACE_ID(iObj) >= FIP_u_TRACE_VALID_OBJ_ID) {
                /* object not in trace list and trace conditions fulfilled
                 * (FIP_a_MOSTrace)                        */
                /* trace creation : */
                /*  - check for free element in trace list - if none ->
                 * do nothing  !!!!             */
                /*                                                   -> @todo:
                 * priorisation (not impl. yet) */
                /*! create a new trace */
                FIP_v_StaticTraceCreation(iObj);
            } else {
                /* object already in trace list and trace conditions fulfilled
                 * (FIP_a_MOSTrace) */
                (FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles)--;
                FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles = MAX(
                    FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles,
                    (sint8)0);
            } /* END IF bTraceConditions == TRUE */
        } else {
            if (OBJ_GET_STATIC_TRACE_ID(iObj) < FIP_u_TRACE_VALID_OBJ_ID) {
                /* object is in trace list and trace conditions NOT fulfilled
                 * (FIP_a_MOSTrace)                    */
                /*! check for trace deletion */
                FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles++;

                FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles = MIN(
                    FIP_a_MOSTrace[OBJ_GET_STATIC_TRACE_ID(iObj)].iBadCycles,
                    FIP_MOST_DEL_QUALITY_HYST_CYCLES);
                /*Delete trace for the object, that is not fulfilled the trace
                 * condition */
                FIP_v_CheckStaticTraceDeletion(iObj);
            }
        }
    } /* obj loop */

    /* check for dead traces (traces without objects) */
    for (iTrace = 0U; iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;
         iTrace++) {
        if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) == TRACE_VALID_NO_OBJ_ID) {
            /*! trace without an object */
            FIP_v_CheckDeadStaticTraceDeletion(iTrace);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_StaticTraceCreation */
static void FIP_v_StaticTraceCreation(ObjNumber_t iObj) {
    TraceID_t iTrace;

    iTrace = 0U;

    while (iTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES) {
        /*! @todo: prioritization needed */
        if (FIP_STATIC_TRACE_GET_VLC_ID(iTrace) > FIP_u_TRACE_VALID_OBJ_ID) {
            /* insert trace with current object data */
            FIP_STATIC_TRACE_GET_VLC_ID(iTrace) = (uint8)iObj;
            FIP_STATIC_TRACE_GET_EM_ID(iTrace) = OBJ_GENERAL(iObj).uiID;
            FIP_STATIC_TRACE_GET_X(iTrace)[0] = OBJ_LONG_DISPLACEMENT(iObj);
            FIP_STATIC_TRACE_GET_Y(iTrace)[0] = OBJ_LAT_DISPLACEMENT(iObj);
            FIP_a_MOSTrace[iTrace].fYVar[0] = FIP_f_GetObjObservationVariance(
                (ObjNumber_t)FIP_STATIC_TRACE_GET_VLC_ID(iTrace));
            FIP_STATIC_TRACE_GET_Y_STD_DEV(iTrace)
            [0] = SQRT_(FIP_a_MOSTrace[iTrace].fYVar[0]);
            FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) = ((sint8)1);
            FIP_a_MOSTrace[iTrace].iBadCycles = ((sint8)0);

            OBJ_GET_STATIC_TRACE_ID(iObj) = (uint8)iTrace;

            /* to leave loop */
            iTrace = (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES;

            /*! @todo: variance should be implemented */
        }
        iTrace++;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CheckStaticTraceDeletion */
static void FIP_v_CheckStaticTraceDeletion(ObjNumber_t iObj) {
    sint8 s_NbofPoints = FIP_STATIC_TRACE_GET_NO_OF_POINTS(
        (sint32)(OBJ_GET_STATIC_TRACE_ID(iObj)));
    TraceID_t u_DelTrace = OBJ_GET_STATIC_TRACE_ID(iObj);
    if (s_NbofPoints > 0) {
        if (((FIP_a_MOSTrace[(sint32)(OBJ_GET_STATIC_TRACE_ID(iObj))].fLength <
              FIP_MOST_DEL_LENGTH) &&
             (FIP_STATIC_TRACE_GET_X((sint32)(
                  OBJ_GET_STATIC_TRACE_ID(iObj)))[s_NbofPoints - 1] > 0.0f)) ||
            (FIP_a_MOSTrace[(sint32)(OBJ_GET_STATIC_TRACE_ID(iObj))]
                 .iBadCycles == FIP_MOST_DEL_QUALITY_HYST_CYCLES)) {
            /*! @todo -> EOMOT_DEL_TIME */

            FIP_v_InitMovingObjectStaticTraces(u_DelTrace);
        }
    } else {
        /* this should never be the case, but anyways... */
        FIP_v_InitMovingObjectStaticTraces(u_DelTrace);
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_StartObjectStaticTraceDeletion */
static void FIP_v_StartObjectStaticTraceDeletion(ObjNumber_t iObj) {
    uint8 u_MOTID;
    /* object is deleted -> start trace holding and deletion */
    /*!@todo : TRACE_VALID_NO_OBJ_ID can be removed here ... just for safety */
    if ((OBJ_GET_STATIC_TRACE_ID(iObj) < FIP_u_TRACE_VALID_OBJ_ID) &&
        (OBJ_GET_STATIC_TRACE_ID(iObj) < TRACE_VALID_NO_OBJ_ID)) {
        u_MOTID = OBJ_GET_STATIC_TRACE_ID(iObj);
        /* start time without obj. measurement */
        FIP_STATIC_TRACE_GET_VLC_ID(u_MOTID) = FIP_u_TRACE_VALID_NO_OBJ_ID;
        FIP_a_MOSTrace[u_MOTID].fTraceWithoutObjTime = 0.0f;
        OBJ_GET_STATIC_TRACE_ID(iObj) = (TraceID_t)FIP_u_TRACE_INVALID_ID;
    } else {
        /* no trace of this obj -> do nothing */
    }
}

/* ****************************************************************************

Functionname:    FIP_v_CheckDeadStaticTraceDeletion               */
static void FIP_v_CheckDeadStaticTraceDeletion(TraceID_t iTrace) {
    /* delete the trace in one of the following cases:  */
    /* trace is short                                   */
    /* object corresponding to trace was deleted long time ago */
    /* trace tail hasn't reached ego vehicle yet (except if trace has maximal
     * length) */
    if ((FIP_a_MOSTrace[iTrace].fLength < FIP_MOST_DEAD_DEL_LENGTH) ||
        (FIP_a_MOSTrace[iTrace].fTraceWithoutObjTime >
         FIP_MOST_DEAD_DEL_TIME) ||
        ((FIP_STATIC_TRACE_GET_X(
              iTrace)[FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) - 1] > 0.0f) &&
         (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) <
          FIP_STATIC_TRACE_NO_OF_POINTS))) {
        /*! delete trace */
        FIP_v_InitMovingObjectStaticTraces(iTrace);
    } else {
        /*! keep trace without object */
        FIP_a_MOSTrace[iTrace].fTraceWithoutObjTime += (float32)TASK_CYCLE_TIME;
    }
}

/*************************************************************************************************************************
  Functionname:    *FIPMOTGetMovingObjectTraces */
FIP_t_MovingObjStaticTrace *FIP_p_MOSTGetMovingObjectTraces(void) {
    return &(FIP_a_MOSTrace[0]);
}
/* END IF (VLC_CFG_USE_VLC_STATIC_TRACES) */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */