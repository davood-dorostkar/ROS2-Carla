/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp.h"
#include "cp_par.h"
#include "cp_si.h"
#include "cp_cd.h"
#include "stddef.h"
#include "TM_Global_Types.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @cond Doxygen_Suppress */
SET_MEMSEC_VAR(CPState)
CPState_t CPState;
/*! @endcond Doxygen_Suppress */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
/*! @brief CP_TRACE_APPROX_FAR_DISTANCE */
#define CP_TRACE_APPROX_FAR_DISTANCE 200.0f

/*! @brief Default invalid value used by several functions */
#define CP_DEFAULT_INVALID_VALUE 999.0f

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Additional data stored for traces (one entry per trace) */
SET_MEMSEC_VAR(CPTraceAdd)
/*! @vaddr:CP_MEAS_ID_TRACE_ADD_DATA @cycleid:VLC_ENV @vname:CPTraceAdd */
CPTraceAddData_t CPTraceAdd[CP_NUM_TRACES];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/
static void CPTracePreProc(void);
static void CPInitTraceAddData(CPTraceAddData_t* const pTraceAdd);
static void CPCalcMovObjTraceAddData(
    CPTraceAddData_t* const pCurAddData,
    const FIP_t_ObjectStaticTrace* const pTrace);

/*************************************************************************************************************************
  Functionname:    CPInitTraceAddData */
static void CPInitTraceAddData(CPTraceAddData_t* const pTraceAdd) {
    pTraceAdd->iObjNr = OBJ_INDEX_NO_OBJECT;
    pTraceAdd->uNumberOfPoints = 0u;
    pTraceAdd->uApproxUseLen = 0u;
    /*pTraceAdd->uTraceLaneIdx        = CP_LANE_INDEX_NONE;*/
    pTraceAdd->fApproxCloseDist = 0.f;
    pTraceAdd->fApproxFarDist = 0.f;
    pTraceAdd->fCurSqDist = 0.f;

    pTraceAdd->ApproxPoly.fC0 = 0.f;
    pTraceAdd->ApproxPoly.fC1 = 0.f;
    pTraceAdd->ApproxPoly.fC2 = 0.f;
    pTraceAdd->ApproxPoly.isValid = FALSE;
    pTraceAdd->isValid = FALSE;
    pTraceAdd->bUseTraceForFusion = FALSE;
}

/*************************************************************************************************************************
  Functionname:    CPCalcMovObjTraceAddData */
static void CPCalcMovObjTraceAddData(
    CPTraceAddData_t* const pCurAddData,
    const FIP_t_ObjectStaticTrace* const pTrace) {
    /* First check if trace valid or deleted */
    if ((pTrace->uiReferenceToVLCObject <= TRACE_VALID_NO_OBJ_ID) &&
        ((uint8)pTrace->iNumberOfPoints >= CP_NUM_TRACE_MIN_NUMBER_OF_POINTS)) {
        uint8 uNumApproxPts; /*!< Number of trace points used for poly approx */

        /* Check if trace has a valid object assigned */
        if (pTrace->uiReferenceToVLCObject < (uint8)Envm_N_OBJECTS) {
            /* Get the object number of the trace */
            const ObjNumber_t iTraceObjIdx =
                (ObjNumber_t)(pTrace->uiReferenceToVLCObject);

            /* Sanity check: if object associated with trace changed, then reset
            the trace, and assign new object ID */
            if (pCurAddData->iObjNr != iTraceObjIdx) {
                CPInitTraceAddData(pCurAddData);
                pCurAddData->iObjNr = iTraceObjIdx;
            }
        } else {
            /* Trace no longer has a valid object, set it's object number to no
             * object */
            pCurAddData->iObjNr = OBJ_INDEX_NO_OBJECT;
        }

        /* If number of points increased, then increase used approximation
         * length */
        if ((uint8)pTrace->iNumberOfPoints > pCurAddData->uNumberOfPoints) {
            const uint8 uLengthIncrease = ((uint8)(pTrace->iNumberOfPoints) -
                                           pCurAddData->uNumberOfPoints);
            pCurAddData->uApproxUseLen += uLengthIncrease;
        }

        /* Find the number of samples with positive X coordinate */
        uNumApproxPts = (uint8)pTrace->iNumberOfPoints;
        while ((uNumApproxPts > 1u) &&
               (pTrace->fTracePointX[uNumApproxPts - 2u] <= 0.f)) {
            uNumApproxPts--;
        }

        /* Take smaller of the two possible values between used approximation
        length
        and number of samples with positive X coordinate */
        uNumApproxPts = MIN(pCurAddData->uApproxUseLen, uNumApproxPts);

        /* Store the number of points used for poly approximation */
        pCurAddData->uApproxUseLen = uNumApproxPts;

        /* Verify that sufficient number of trace points for polynome
        extrapolation and we
        have a real trace weight */
        if ((uNumApproxPts >= CP_NUM_TRACE_MIN_NUMBER_OF_POINTS) &&
            (pCurAddData->iObjNr != OBJ_INDEX_NO_OBJECT)) {
            /* Calculate the trace approximation polynomial */
            CPCalcPointApproxPolyL2(&pCurAddData->ApproxPoly,
                                    pTrace->fTracePointX, pTrace->fTracePointY,
                                    uNumApproxPts);
            /* Alternatively one could use :
             * SICalcPointApproxPolyL2Ext(&pCurAddData->ApproxPoly,
             * pTrace->fTracePointX, pTrace->fTracePointY,
             * pTrace->fTracePointYStdDev, uNumApproxPts); */

            /* Calculate the summed squared distance of the sample points to the
            approximation polynomial (the residual) */
            pCurAddData->fCurSqDist = CPCalcSumSqDistance(
                &pCurAddData->ApproxPoly, pTrace->fTracePointX,
                pTrace->fTracePointY, uNumApproxPts);
            /* Alternatively one could use: fCurSqDist =
             * SICalcSumSqDistanceExt(&pCurAddData->ApproxPoly,
             * pTrace->fTracePointX, pTrace->fTracePointY,
             * pTrace->fTracePointYStdDev, uNumApproxPts); */

            /* Store the estimated distance of the starting point (currently
            only take X offset
            into account) */
            pCurAddData->fApproxCloseDist =
                pTrace->fTracePointX[uNumApproxPts - 1u];
            pCurAddData->fApproxFarDist = OBJ_LONG_DISPLACEMENT(
                pTrace->uiReferenceToVLCObject); /*Object Xdist*/
        } else {
            /* Trace too short, set it to invalid */
            pCurAddData->fApproxCloseDist = CP_DEFAULT_INVALID_VALUE;
            pCurAddData->fApproxFarDist = CP_DEFAULT_INVALID_VALUE;
        }
        /* Update number of points */
        pCurAddData->uNumberOfPoints = (uint8)pTrace->iNumberOfPoints;
    } else {
        /* Trace seems to have a reference to object greater
        TRACE_VALID_NO_OBJ_ID,
        which means the trace is deleted - reset data on it */
        CPInitTraceAddData(pCurAddData);
        /* Trace no longer has a valid object, set it's object number to no
         * object */
        pCurAddData->iObjNr = OBJ_INDEX_NO_OBJECT;
    }
}

/*************************************************************************************************************************
  Functionname:    CPTracePreProc */
static void CPTracePreProc(void) {
    TraceID_t uTrIdx;                 /*!< Current trace ID (index) */
    /* TraceID_t uNumValidTr = 0u; */ /*!< Counter of valid traces */

    if (CPState == CP_INIT) {
        /* Reset trace information for each trace */
        for (uTrIdx = 0u; uTrIdx < (TraceID_t)CP_NUM_TRACES; uTrIdx++) {
            CPInitTraceAddData(&CPTraceAdd[uTrIdx]);
        }
    } else {
        /* First pass: go through all traces, calculating the trace polynomials
         * as needed */
        for (uTrIdx = 0u; uTrIdx < CP_NUM_TRACES; uTrIdx++) {
            /* Get utility pointer to current trace's additional data */
            CPTraceAddData_t* const pCurTraceAddData = &CPTraceAdd[uTrIdx];
            CPCalcMovObjTraceAddData(
                pCurTraceAddData, FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(uTrIdx));
            /* If the trace polynomial approximation uses at least the minimum
              required
              number of points and it has positive weight, then it is considered
              'valid' */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    VLCCPProcess */
void VLCCPProcess(void) {
    /* Call trace preprocessing */
    CPTracePreProc();

    CPSIProcess();

    CPCDProcess();
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */