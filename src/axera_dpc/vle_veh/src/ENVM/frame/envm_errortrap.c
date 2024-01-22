/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "envm_consts.h"
#include "stddef.h"
#include "assert.h"
#include "tue_common_libs.h"
/* Start EM default section for code and data used for MPU and cache definition.
   These two lines need to be at the start of every EM C file, but last include,
   to identify that code in this file is EM code and uses EM memory. */

/*****************************************************************************
VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static bool_t EMErrorTrap_b_listFull;
static ui32_t EMErrorTrap_u_nextSlot;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
FUNCTIONS
*****************************************************************************/

/*****************************************************************************
Functionname   EMErrorTrapInitGlobalData                                */
void EMErrorTrapInitGlobalData(void) {
    EMErrorTrap_b_listFull = FALSE;

    EMErrorTrap_u_nextSlot = 0u;
}

/*****************************************************************************
Functionname   EMErrorTrap                                              */
void EMErrorTrap(const char a_file[], i32_t s_line, ui32_t u_errorType) {
    /* determine length of file name */
    ui32_t u_length = 0u;
    while ((u_length < 300u) && (a_file[u_length])) {
        u_length++;
    }
    if (u_length >= 1u) {
        ui32_t u_idx = u_length - 1u;
        while ((u_idx > 0u) && (a_file[u_idx] != '\\')) {
            u_idx--;
        }
        if (u_idx > 0u) {
            u_idx++;
        }
        if (u_length >= u_idx) {
            u_length -= u_idx;
        }
        u_length = MIN(u_length, Envm_ERRORTRAP_MAX_FILELENGTH - 1u);

        /* first in first out strategy: save the first errors in this cycle */

        if (!EMErrorTrap_b_listFull) {
            /* increase nr of errors in this cycle -> increase nr of next free
             * slot */
            EMErrorTrap_u_nextSlot++;

            /* do not store more than Envm_ERRORTRAP_MAXERRORS errors */
            if (EMErrorTrap_u_nextSlot >= Envm_ERRORTRAP_MAXERRORS) {
                EMErrorTrap_b_listFull = TRUE;
            }
        }
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */