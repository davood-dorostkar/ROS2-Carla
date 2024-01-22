#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup vlc_inhibit VLC_INHIBIT
   @ingroup acc_long_veh

@{ */
#ifndef VLC_INHIBIT_HE
#define VLC_INHIBIT_HE

/* INCLUDES */
#include "vlc_glob_ext.h"

/*usage:*/
/*
1. add an external buffer in the wrapper file:
-> vlc_inhibit_storage_t BUFFER;

2. add the VLC_INHIBIT_INIT function to the main INIT routine
-> VLC_INHIBIT_INIT(&BUFFER);

3. add a local inhibition buffer to each VLC task (e.g. to VLC_LC_EXEC and
VLC_LONG_EXEC)
-> vlc_inhibit_t LocalInhibitionBuffer;

3. at the beginning of every task call add VLC_INHIBIT_START_CYCLE
-> VLC_INHIBIT_START_CYCLE(&LocalInhibitionBuffer);

4. when gathering signals, check if they are valid and if their quality is good
enough, and add the inhibition handler and the failure indication if this is not
the case
-> VDATA_GET_SOMETHING(&data, &signal_status);
-> VLC_INHIBIT_ADD_INHIBITION(&LocalInhibitionBuffer,
signal_status==VDATA_SIG_STATUS_INVALID,
Fct_inhibit_FctACC|Fct_inhibit_FctFCA|Fct_inhibit_FctDM|Fct_inhibit_DspTmpNotAv);

... do step 4 as often as signals are read out in this task

5. call VLC_INHIBIT_FINISH_CYCLE to finish the inhibition check for this task.
This function will copy the information from LocalInhibitionBuffer to the global
BUFFER and writes back (to LocalInhibitionBuffer) the merged inhibition
information. Additionally a unique task_id is needed for every task. this
function call needs to be "thread save", that means it should be called with
"locked ressoureces"
-> OS_LOCK(VLC_RESSOURCE);
-> VLC_INHIBIT_FINISH_CYCLE(&LocalInhibitionBuffer, &BUFFER,
Fct_inhibit_task_LC_EXEC);
-> OS_UNLOCK(VLC_RESSOURCE);

6. map inhibition states to the different inhibit flags for every function and
the information for failure indication using VLC_INHIBIT_GET_INHIBITION
-> AVLC_INPUT_DATA.INPUT_STATUS.INHIBIT =
VLC_INHIBIT_GET_INHIBITION(LocalInhibitionBuffer, Fct_inhibit_FctACC);

*/

/*task ids for inhibition manager*/
typedef enum {
    Fct_inhibit_task_LC_EXEC,
    Fct_inhibit_task_LONG_EXEC,
    Fct_inhibit_task_LAT_EXEC,

    /*-------------------------------------------------------------*/
    Fct_inhibit_storage_size /*must be the last entry in this enum!*/
} vlc_inhibit_storage_size_t;

typedef uint32 vlc_inhibition_t;

typedef struct vlc_inhibit_t {
    vlc_inhibition_t INHIBITION; /*stores the current inhibition information*/
    uint16 INHiBIT_NR;           /*stores the last inhibition nr*/
    uint16 INHIBITION_COUNT;     /*stores the current inhibition nr*/
} vlc_inhibit_t;

typedef struct vlc_inhibit_storage_t {
    vlc_inhibit_t INHIBIT[Fct_inhibit_storage_size];
    uint8 INIT_DONE;
    VLC_OP_MODE_t INTERNAL_OP_MODE, EXTERNAL_OP_MODE;
} vlc_inhibit_storage_t;

/*function inhibition information*/
#define Fct_inhibit_FctCC (vlc_inhibition_t)0x00000001u
#define Fct_inhibit_FctACC (vlc_inhibition_t)0x00000002u
#define Fct_inhibit_FctFCA (vlc_inhibition_t)0x00000004u
#define Fct_inhibit_FctDM (vlc_inhibition_t)0x00000008u
#define Fct_inhibit_FctLDW (vlc_inhibition_t)0x00000010u
#define Fct_inhibit_FctDIE (vlc_inhibition_t)0x00000020u
#define Fct_inhibit_FctLODM (vlc_inhibition_t)0x00000040u
#define Fct_inhibit_FctLADM (vlc_inhibition_t)0x00000080u
/*add new functions ...*/

/*failure indication information*/
#define Fct_inhibit_DspTmpNotAv (vlc_inhibition_t)0x80000000u
#define Fct_inhibit_DspPerfDeg (vlc_inhibition_t)0x40000000u
#define Fct_inhibit_DspWorkshop (vlc_inhibition_t)0x20000000u

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */

extern void VLCSetOpModeOld(const VLC_OP_MODE_t vlcOpMode);
extern VLC_OP_MODE_t VLCGetOpMode(void);

extern void VLC_INHIBIT_INIT(vlc_inhibit_storage_t *memory);
extern void VLC_INHIBIT_START_CYCLE(vlc_inhibit_t *local_inhibit_buffer);
extern void VLC_INHIBIT_ADD_INHIBITION(vlc_inhibit_t *local_inhibit_buffer,
                                       const uint8 condition,
                                       const vlc_inhibition_t inhibit_list);
extern void VLC_INHIBIT_FINISH_CYCLE(vlc_inhibit_t *local_inhibit_buffer,
                                     vlc_inhibit_storage_t *memory,
                                     const vlc_inhibit_storage_size_t task_id);
extern uint8 VLC_INHIBIT_GET_INHIBITION(
    const vlc_inhibit_t local_inhibit_buffer,
    const vlc_inhibition_t inhibit_function);
extern void VLC_INHIBIT_SET_EXTERNAL_MODE(vlc_inhibit_storage_t *memory);
extern VLC_OP_MODE_t VLC_INHIBIT_GET_MODE(const vlc_inhibit_storage_t *memory);

// add define
#ifndef VLC_MOD_STARTUP
#define VLC_MOD_STARTUP 0U
#endif
#ifndef VLC_MOD_INIT
#define VLC_MOD_INIT 1U
#endif
#ifndef VLC_MOD_RUNNING
#define VLC_MOD_RUNNING 2U
#endif
#ifndef VLC_MOD_SHUTDOWN
#define VLC_MOD_SHUTDOWN 3U
#endif
#ifndef VLC_MOD_PAUSE
#define VLC_MOD_PAUSE 4U
#endif
#endif /* #ifndef VLC_INHIBIT_HE */
/** @} end defgroup */

#ifdef __cplusplus
}
#endif