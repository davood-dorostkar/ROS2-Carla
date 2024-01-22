
/*!
   @defgroup acc_long_veh  AVLC_LONG_VEH
   @ingroup vlc_veh


@{ */
#ifndef VLC_LONG_VEH_EXT_H
#define VLC_LONG_VEH_EXT_H

#include "TM_Global_Types.h"

#include "stddef.h"
#include "assert.h"

#include "vlc_long_veh.h"
#include "vlc_config.h"
#include "switch_ext.h"
#include "display_ext.h"
#include "acc_obj_ext.h"
#include "isa.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void VLC_LC_EXEC(const times_t cycle_time,
                        const reqVLCVehDebugList_t* pVLCVehDebugPorts,
                        const VED_VehDyn_t* pVehDyn,
                        const PowerTrain_t* pPowerTrain,
                        const VLC_AccLeverInput_t* pAccLever,
                        const VLC_LongCtrlInput_t* pLongCtrlResp,
                        const VLC_acc_object_t* pAccDisplayObj,
                        const VLC_acc_output_data_t* pAccOutput,
                        const VLCSenAccOOI_t* pVLCAccOOIData,
                        const t_CamLaneInputData* pCamLaneData,
                        VLC_DFV2SenInfo_t* pDFVLongOut,
                        VLC_LongCtrlOutput_t* pLongCtrlCmd,
                        ISAInfo* p_isa_info,
                        PACCInfo* p_pacc_info);

extern void VLC_LONG_VEH_INIT(void);

#ifdef __cplusplus
};
#endif

#endif

/** @} end defgroup */
