
/*!
   @defgroup acc_long_sen  AVLC_LONG_SEN
   @ingroup vlc_sen


@{ */
#ifndef VLC_LONG_SEN_EXT_H
#define VLC_LONG_SEN_EXT_H
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "vlc_config.h"
#include "vlc_long_sen.h"
#include "overtake_assist.h"
#include "sen_sim.h"
//#include "switch_ext.h"
//#include "display_ext.h"
//#include "acc_obj_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void VLC_LONG_EXEC(const times_t cycle_time,
                          const boolean bSensorBlocked,
                          const Envm_t_GenObjectList *pEmGenObjList,
                          const AssessedObjList_t *pFctObjList,
                          const VED_VehDyn_t *pVehDyn,
                          const VLC_DFV2SenInfo_t *pDFVLongOut,
                          const VLCCustomInput_t *pVLCCustomInput,
                          VLC_acc_object_t *pAccDisplayObj,
                          VLCSenAccOOI_t *pVLCAccOOIData,
                          VLC_acc_output_data_t *pAccOutput,
                          OvertakeAssistInfo *p_overtake_assist_info);

#ifdef __cplusplus
};
#endif

#endif

/** @} end defgroup */
