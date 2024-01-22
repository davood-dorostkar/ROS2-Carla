/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "tunnel_recognize.h"

static TunnelInternalData_t sTunnelInternalData;
/* ****************************************************************************

  Functionname:     TUETunnelRecognize */ /*!

@brief            special secene recognize like tunnel

@description      mainly about tunnel recgonition, we can not trust radar object
in tunnel situation.
             we analysis radar object feature and find potential tunnel
situation.

@param            void

@return           void

@pre              None
@post             No changes


**************************************************************************** */
uint8 TUETunnelRecognize(const ExtObjectList_t* pRadarObjList,
                         float fEgoCurve,
                         float fEgoSpeed) {
    uint8 uiNumOfObj = 0u;
    float fTotalDistY = 0.f;
    static uint8 uiTunnelProb = 0;
    (void)memset(&sTunnelInternalData, 0u, sizeof(TunnelInternalData_t));
    for (uint32 ui_Index = 0u; ui_Index < (uint32)TUE_RADAR_RAW_OBJECT_NUM;
         ui_Index++) {
        if (pRadarObjList->Objects[ui_Index].General.eObjMaintenanceState !=
                MT_STATE_DELETED &&
            pRadarObjList->Objects[ui_Index].General.eObjMaintenanceState !=
                MT_STATE_MERGE_DELETED)

        {
            BML_t_TrajRefPoint DistVEYPoint2Circle =
                Tue_CML_CalculateDistancePoint2Circle(
                    pRadarObjList->Objects[ui_Index].Kinematic.fDistX,
                    pRadarObjList->Objects[ui_Index].Kinematic.fDistY,
                    fEgoCurve);
            float32 f_Dist2TrajAbs = DistVEYPoint2Circle.f_DistToTraj;
            sTunnelInternalData.pObjectsDistYArray[uiNumOfObj++] =
                f_Dist2TrajAbs;
            fTotalDistY += f_Dist2TrajAbs;
        }
    }

    if (uiNumOfObj >= TU_TUNNEL_MIN_OBJ_SIZE &&
        fEgoSpeed >= TU_TUNNEL_MIN_VELOCITY_THREASHOLD) {
        float fVariance = TUE_CML_MinMax(
            0, TU_TUNNEL_MAX_VARIANCE,
            TUE_CML_EMPCalcVariance(sTunnelInternalData.pObjectsDistYArray,
                                    uiNumOfObj));
        float fStandarVariance = SQRT(fVariance);

        float fStandardGaussCDFLow = TUE_CML_CalcStdGaussianCDF(
            -TU_TUNNEL_PROB_DENSITY_CAL_DISTY_THREASHOLD,
            fTotalDistY / (float)uiNumOfObj, fStandarVariance);
        float fStandardGaussCDFHigh = TUE_CML_CalcStdGaussianCDF(
            TU_TUNNEL_PROB_DENSITY_CAL_DISTY_THREASHOLD,
            fTotalDistY / (float)uiNumOfObj, fStandarVariance);
        float fStandardGaussCDFBetween15 =
            fStandardGaussCDFHigh - fStandardGaussCDFLow;

        uiTunnelProb = TUE_CML_MinMax(
            0, TU_TUNNEL_MAX_PROBABILITY_VAL,
            uiTunnelProb +
                TUE_CML_MinMax(TU_TUNNEL_MAX_PROB_DECREASE_THREASHOLD,
                               TU_TUNNEL_MAX_PROB_INCREASE_THREASHOLD,
                               (int)((fStandardGaussCDFBetween15 -
                                      TU_TUNNEL_DISTY_PROB_DENSITY_THREASHOLD) *
                                     TU_TUNNEL_MAX_PROBABILITY_VAL)));
    } else {
        uiTunnelProb = 0;
    }
    // printf("tunnel, %d,%d,%f\n", uiTunnelProb, uiNumOfObj,
    // fStandardGaussCDFBetween15);
    return uiTunnelProb;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */