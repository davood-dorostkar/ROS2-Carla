/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDE
*****************************************************************************/

#include "envm_ext.h"
#include "envm_consts.h"
#include "TM_Global_Types.h"
#include "stddef.h"

#include "assert.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

#define EM_SYNC_REF_INTFVER (1U)
/* payload size of the sequential freeze [bytes] */
#define EM_SEQ_FREEZE_PAYLOAD_SIZE (100U)

static const uint8 EM_MEAS_FUNC_CHANEL_ID = (60u);

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef struct {
    uint32 u_SizeOfData;
    uint32 u_SequenceCnt;
    uint8 a_Payload[EM_SEQ_FREEZE_PAYLOAD_SIZE];
} EM_t_SequentialFreeze;

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static uint32 u_SeqFreezePPARNearCnt;
static uint32 u_SeqFreezePPARFarCnt;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

// Amend by LiuYang 2019-05-05 for MTS //#ifdef Envm_MEASUREnvmENT

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */