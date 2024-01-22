/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_sen.h"
//#include "vlc_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

SET_MEMSEC_VAR(VLCTrMat2DForward)
static GDBTrafoMatrix2D_t
    VLCTrMat2DForward; /*!< forward motion transformation Matrix */
SET_MEMSEC_VAR(VLCTrMat2DBackward)
static GDBTrafoMatrix2D_t
    VLCTrMat2DBackward; /*!< backward motion transformation Matrix */

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void VLCCalcMotionMatrices(fTime_t fCycleTime,
                                  const VED_VehDyn_t* pEgoDyn);

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCSenProcessInput */
void VLCSenProcessInput(fTime_t fCycleTime, const VED_VehDyn_t* pEgoDyn) {
    /*--- VARIABLES ---*/
    /* Calculate the Sensor Position Relative to CoG */
    // VLC_fBumperToCoG = (GET_EGO_STATIC_DATA_PTR->VehParMain.WheelBase
    //  * GET_EGO_STATIC_DATA_PTR->VehParMain.AxisLoadDistr)
    //  + GET_EGO_STATIC_DATA_PTR->VehParAdd.OverhangFront;
    // change input from parameter
    VLC_fBumperToCoG = (GET_VLCSEN_PARAMETERS->VLCSEN_Kf_WheelBase *
                        GET_VLCSEN_PARAMETERS->VLCSEN_Kf_AxisLoadDistr) +
                       GET_VLCSEN_PARAMETERS->VLCSEN_Kf_OverhangFront;

    VLCCalcMotionMatrices(fCycleTime, pEgoDyn);
}

/*************************************************************************************************************************
  Functionname:    VLCCalcMotionMatrices */
static void VLCCalcMotionMatrices(fTime_t fCycleTime,
                                  const VED_VehDyn_t* pEgoDyn) {
    fAccel_t fAcceleration = pEgoDyn->Longitudinal.MotVar.Accel;
    fYawRate_t fYawRate = pEgoDyn->Lateral.YawRate.YawRate;
    fVelocity_t fCorrVelo = pEgoDyn->Longitudinal.MotVar.Velocity;

    /* Since corrected speed is unsigned, correct for sign if moving reverse */
    if ((pEgoDyn->MotionState.MotState == VED_LONG_MOT_STATE_MOVE_RWD) &&
        (fCorrVelo > 0)) {
        fCorrVelo = -fCorrVelo;
    }

    /*! calculate matrices for ego motion compensation */
    GDBmathCalculateCOFEgomotionMatrices(&VLCTrMat2DForward,
                                         &VLCTrMat2DBackward, fCorrVelo,
                                         fAcceleration, fYawRate, fCycleTime);
}

/*************************************************************************************************************************
  Functionname:    VLCGetTrafoMatrix2DCOFForward */
const GDBTrafoMatrix2D_t* VLCGetTrafoMatrix2DCOFForward(void) {
    return &VLCTrMat2DForward;
}

/*************************************************************************************************************************
  Functionname:    VLCGetTrafoMatrix2DCOFBackward */
const GDBTrafoMatrix2D_t* VLCGetTrafoMatrix2DCOFBackward(void) {
    return &VLCTrMat2DBackward;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */