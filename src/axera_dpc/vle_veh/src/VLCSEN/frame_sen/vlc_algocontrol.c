/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_sen.h"
#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "Clarify if VLCSetOpModeOld function call still needed! If not remove include of vlc_inhibit_ext.h! Switch to new operation modes!")
#endif
#include "vlc_inhibit_ext.h"
#include "cd_ext.h"

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
/*! @cond Doxygen_Suppress */

static void VLCSenSetStates(CDState_t StCD,
                            CPState_t StCP,
                            SIState_t StSI,
                            FIPState_t StFIP,
                            VLCSenState_t StVLC);

#define VLC_SEN_STATE_PARAM(_CDSTATE_, _CPSTATE_, _SISTATE_, _SPMSTATE_,    \
                            _FIPSTATE_, _VLCSENSTATE_)                      \
    (_CDSTATE_),                                                            \
        VLC_SEN_STATE_PARAM_POST_CD((_CPSTATE_), (_SISTATE_), (_SPMSTATE_), \
                                    (_FIPSTATE_), (_VLCSENSTATE_))

#define VLC_SEN_STATE_PARAM_POST_CD(_CPSTATE_, _SISTATE_, _SPMSTATE_,   \
                                    _FIPSTATE_, _VLCSENSTATE_)          \
    (_CPSTATE_), VLC_SEN_STATE_PARAM_POST_CP((_SISTATE_), (_SPMSTATE_), \
                                             (_FIPSTATE_), (_VLCSENSTATE_))

#define VLC_SEN_STATE_PARAM_POST_CP(_SISTATE_, _SPMSTATE_, _FIPSTATE_,   \
                                    _VLCSENSTATE_)                       \
    (_SISTATE_), VLC_SEN_STATE_PARAM_POST_SI((_SPMSTATE_), (_FIPSTATE_), \
                                             (_VLCSENSTATE_))

#define VLC_SEN_STATE_PARAM_POST_SI(_SPMSTATE_, _FIPSTATE_, _VLCSENSTATE_) \
    VLC_SEN_STATE_PARAM_POST_SPM((_FIPSTATE_), (_VLCSENSTATE_))

#define VLC_SEN_STATE_PARAM_POST_SPM(_FIPSTATE_, _VLCSENSTATE_) \
    (_FIPSTATE_), (_VLCSENSTATE_)

/*! @endcond Doxygen_Suppress */

/*! @cond Doxygen_Suppress */
/*************************************************************************************************************************
  Functionname:    VLCSenSetStates */
static void VLCSenSetStates(CDState_t StCD,
                            CPState_t StCP,
                            SIState_t StSI,
                            FIPState_t StFIP,
                            VLCSenState_t StVLC) {
    CDState = StCD;
    CPState = StCP;
    SIState = StSI;
    FIPState = StFIP;
    VLCSenFrame.eVLCState = StVLC;
}
/*! @endcond Doxygen_Suppress */
/*************************************************************************************************************************
  Functionname:    VLCSenProcessStates */
void VLCSenProcessStates(VLC_OP_MODE_t eOpMode) {
    /*! ---  VLC State Matrix ---  */
    /*! set operation modes of all components depending on VLC OpMode  */
    VLCSetOpModeOld(eOpMode);

    if (VLCSenIsInitialized != TRUE) {
        eOpMode = VLC_MOD_INIT;
    }

    switch (eOpMode) {
        /*                                    CD              CP        SI SPM
         * FIP         VLC */
        case (VLC_OP_MODE_t)VLC_MOD_STARTUP:
        case (VLC_OP_MODE_t)VLC_MOD_INIT: {
            VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_INIT, CP_INIT, SI_INIT,
                                                SPM_INIT, FIP_INIT,
                                                VLC_SEN_INIT));
            break;
        }
        case (VLC_OP_MODE_t)VLC_MOD_RUNNING:
            VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_OK, CP_OK, SI_OK,
                                                SPM_OK, FIP_OK, VLC_SEN_RUN));
            break;
        case (VLC_OP_MODE_t)VLC_MOD_SHUTDOWN:
            VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_INIT, CP_INIT, SI_INIT,
                                                SPM_OK, FIP_INIT,
                                                VLC_SEN_SHUTDOWN));
            break;
        default:
            VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_INIT, CP_INIT, SI_INIT,
                                                SPM_INIT, FIP_INIT,
                                                VLC_SEN_INIT));
            break;
    }
}
/*************************************************************************************************************************
  Functionname:    VLCSenSignalErrorShutdown */
void VLCSenSignalErrorShutdown(const boolean isRecoveryPossible) {
    VLCSenIsInitialized = FALSE;

    if (isRecoveryPossible != TRUE) {
        VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_INIT, CP_INIT, SI_INIT,
                                            SPM_INIT, FIP_INIT, VLC_SEN_ERROR));
    } else {
        /**
         * this path is currently reached in case of an invalid 'ObjectList'
         * which can be caused by EM_MOD_INVALID_INPUTS. check if this is the
         * case by investigating status of 'PerformanceDegradation' data. this
         * structure should have the status of the 'ObjectList', namely
         * AL_SIG_STATE_INIT. except the situation where BlockageInformation
         * should be hold. this is signaled by setting SigHeaderStatus of
         * 'PerfDegr_t' struct to 'AL_SIG_STATE_OK. Freezing BlockageInformation
         * is only allowed in EmFctCycleMode Envm_VLC_CYCLE_INIT to prevent
         * false freezing in 'RunningMode'.
         *
         * this is only valid in case of having EM and VLC running synchronously
         * which seems currently not to be the case -> skip second condition
         * temporarily since 'EmFctCycleMode' data is currently used from
         * recording instead of the one from simulation!
         */

        VLCSenSetStates(VLC_SEN_STATE_PARAM(CD_STATE_INIT, CP_INIT, SI_INIT,
                                            SPM_INIT, FIP_INIT,
                                            VLC_SEN_SHUTDOWN));
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */