/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_inhibit_ext.h"
/*****************************************************************************
  MODULE GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE GLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*just to keep old interface to frame software*/
SET_MEMSEC_VAR(EXTERNAL_OP_MODE)
static VLC_OP_MODE_t EXTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
SET_MEMSEC_VAR(INTERNAL_OP_MODE)
static VLC_OP_MODE_t INTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL FUNCTIONS
*****************************************************************************/

/************************************************************************
    @fn               VLC_INHIBIT_INIT */ /*!
                                    @brief            initialize the global
                                inhibit memory
                                    @description      clears all information
                                from inhibit memory
                                    @param[out]       memory
                                    @return           void
                                    @pre              none
                                    @post             none
                                ****************************************************************************
                                */
void VLC_INHIBIT_INIT(vlc_inhibit_storage_t *memory) {
    uint8 i;
    for (i = 0u; i < (uint8)Fct_inhibit_storage_size; i++) {
        memory->INHIBIT[i].INHIBITION = 0u;
        memory->INHIBIT[i].INHiBIT_NR = 0u;
    }
    memory->INTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
    memory->EXTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
    EXTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
    INTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
    memory->INIT_DONE = TRUE;
}

/************************************************************************
    @fn               VLC_INHIBIT_START_CYCLE    */ /*!
                          @brief            initialize inhibit memory of a
                      single cycle in every cycle
                          @description      clears all information from a single
                      cycle inhibit memory
                          @param[out]       local_inhibit_buffer
                          @return           void
                          @pre              none
                          @post             none
                      ****************************************************************************
                      */
void VLC_INHIBIT_START_CYCLE(vlc_inhibit_t *local_inhibit_buffer) {
    local_inhibit_buffer->INHIBITION = (vlc_inhibition_t)0;
    local_inhibit_buffer->INHiBIT_NR = 0u;
    local_inhibit_buffer->INHIBITION_COUNT = 0u;
}

/************************************************************************
    @fn               VLC_INHIBIT_ADD_INHIBITION    */ /*!
                       @brief            adds an inhibition to local buffer in
                   case of a condition
                       @description      in case of "condition" is true, the
                   inhibition buffer (local_inhibit_buffer) is extended with the
                   current inhibition list
                       @param[in,out]    local_inhibit_buffer
                       @param[in]        condition
                       @param[in]        inhibit_list
                       @return           void
                       @pre              none
                       @post             none
                   ****************************************************************************
                   */
void VLC_INHIBIT_ADD_INHIBITION(vlc_inhibit_t *local_inhibit_buffer,
                                const uint8 condition,
                                const vlc_inhibition_t inhibit_list) {
    local_inhibit_buffer->INHIBITION_COUNT++;
    if (condition) {
        local_inhibit_buffer->INHIBITION |= inhibit_list;
        local_inhibit_buffer->INHiBIT_NR =
            local_inhibit_buffer->INHIBITION_COUNT;
    }
}

/************************************************************************
    @fn               VLC_INHIBIT_FINISH_CYCLE    */ /*!
                         @brief            stores the local value into
                     inhibition storage and puts out the merged inhibition list
                         @description      the current local inhibition status
                     is stored into the inhibition storage and the merged
                     inhibition output is written into the local inhibition
                     value
                         @param[in,out]    local_inhibit_buffer current cycle
                     inhibition buffer and merged output inhibition buffer
                         @param[in,out]    memory pointer to inhibition storage
                         @param[in]        task_id task identifier
                         @return           void
                         @pre              none
                         @post             none
                     ****************************************************************************
                     */
void VLC_INHIBIT_FINISH_CYCLE(vlc_inhibit_t *local_inhibit_buffer,
                              vlc_inhibit_storage_t *memory,
                              const vlc_inhibit_storage_size_t task_id) {
    uint8 i;

    /*store local value into memory*/
    memory->INHIBIT[task_id] = *local_inhibit_buffer;

    /*merge all external buffers into new local buffer*/
    local_inhibit_buffer->INHIBITION = 0u;
    for (i = 0u; i < (uint8)Fct_inhibit_storage_size; i++) {
        local_inhibit_buffer->INHIBITION |= memory->INHIBIT[i].INHIBITION;
    }
}

/************************************************************************
    @fn               VLC_INHIBIT_GET_INHIBITION    */ /*!
                       @brief            returns the information if a specific
                   function needs to be inhibited
                       @description      returns true if inhibit_function is set
                   in local_inhibit_buffer
                       @param[in]        local_inhibit_buffer current cycle
                   inhibition buffer and merged output inhibition buffer
                       @param[in]        inhibit_function function that is
                   checked for inhibition
                       @return           true if inhibition is set
                       @pre              none
                       @post             none
                   ****************************************************************************
                   */
uint8 VLC_INHIBIT_GET_INHIBITION(const vlc_inhibit_t local_inhibit_buffer,
                                 const vlc_inhibition_t inhibit_function) {
    uint8 ret_value;

    if ((local_inhibit_buffer.INHIBITION & inhibit_function) != 0u) {
        ret_value = TRUE;
    } else {
        ret_value = FALSE;
    }
    return ret_value;
}

/************************************************************************
    @fn               VLC_INHIBIT_GET_MODE    */ /*!
                             @brief            returns the VLC internal
                         operational mode
                             @description      returns the VLC internal
                         operational mode, that could be modified using
                         VLC_INHIBIT_SET_EXTERNAL_MODE
                             @param[in]        memory
                             @return           current internal op mode
                             @pre              none
                             @post             none
                         ****************************************************************************
                         */
VLC_OP_MODE_t VLC_INHIBIT_GET_MODE(const vlc_inhibit_storage_t *memory) {
    return memory->INTERNAL_OP_MODE;
}

/************************************************************************
    @fn               VLC_INHIBIT_SET_EXTERNAL_MODE   */ /*!
                     @brief            sets internal operational mode using
                 external mode
                     @description      sets internal operational mode using
                 external mode
                     @param[in,out]    memory
                     @return           none
                     @pre              --
                     @post             --
                 ****************************************************************************
                 */
void VLC_INHIBIT_SET_EXTERNAL_MODE(vlc_inhibit_storage_t *memory) {
    memory->EXTERNAL_OP_MODE = EXTERNAL_OP_MODE;
    switch (memory->EXTERNAL_OP_MODE) {
        case VLC_MOD_SHUTDOWN:
            /*go directly to stop*/
            memory->INTERNAL_OP_MODE = memory->EXTERNAL_OP_MODE;
            break;
        case VLC_MOD_RUNNING:
            if (memory->INTERNAL_OP_MODE == VLC_MOD_SHUTDOWN) {
                memory->INTERNAL_OP_MODE = VLC_MOD_RUNNING;
            }
            break;

        default:
            memory->INTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
            break;
    }

    /*behavior in states*/
    switch (memory->INTERNAL_OP_MODE) {
        case VLC_MOD_SHUTDOWN:
            memory->INTERNAL_OP_MODE = VLC_MOD_SHUTDOWN;
            break;

        case VLC_MOD_RUNNING:
            break;

        default:
            break;
            /*off*/
    }
    INTERNAL_OP_MODE = memory->INTERNAL_OP_MODE;
}

/*---------- these functions are used to keep the old interface to the frame
 * software -----------*/

/*****************************************************************************
  @fn             VLCSetOpMode */ /*!

                                               @description    sets the new
                                             opmode got from ACTL

                                               @param[in]      vlcOpMode new
                                             operational mode from ACTL

                                               @return         void

                                             *****************************************************************************/
void VLCSetOpModeOld(const VLC_OP_MODE_t vlcOpMode) {
    EXTERNAL_OP_MODE = vlcOpMode;
}

/*****************************************************************************
  @fn             VLCGetOpMode */ /*!

                                               @description    gets the internal
                                             opmode from VLC

                                               @param[in,out]  -

                                               @return         internlal opmode
                                             from VLC

                                             *****************************************************************************/
VLC_OP_MODE_t VLCGetOpMode(void) { return INTERNAL_OP_MODE; }

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */