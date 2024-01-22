/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "ops_par.h"

/*! @CompParamGroup EM_FPS_Param EM FPS parameter
     @ingroup em_appl_parameter
Parameters related to the function pre-selection (FPS)
 */
/*@StartParamGroup*/

/*! @brief       Enforce video confirmation for all radar objects
    @general
    @conseq      FALSE  Video confirmation from camera object is used
                 TRUE   Video confirmation is always on independet from camera
   object detection
    @attention   Only for development purpose
    @typical     FALSE   @unit  .    @min (0)   @max (1)   */
FPS_DEF_PARAM(boolean,
              EM_FPS_PAR_CAM_CONFIRM_ALW_ON,
              EM_FPS_PAR_CAM_CONFIRM_AWL_ON_DEFAULT)

/*! @brief       Enable reaction on point classified objects
    @general
    @conseq      FALSE  No reaction on point object
                 TRUE   Allow reaction on point object
    @attention   Only for development purpose
    @typical     FALSE   @unit  .    @min (0)   @max (1)   */
FPS_DEF_PARAM(boolean, EM_FPS_PAR_POINT_OBJ_ON, EM_FPS_PAR_POINT_OBJ_ON_DEFAULT)

/*! @brief       Enable full eba reaction on tracked objects depending on
   dynamic property
    @general
    @conseq      0      Object plausibility check is active. Only safe objects
   get a certain quality level
                 Bit1   Reaction on every stationary object is active
                 Bit2   Reaction on every moving object is active
    @attention   Only for development purpose. Reaction on ground-level and
   overhead objects will occur
    @typical     FALSE   @unit  .    @min (0)   @max (1)   */
FPS_DEF_PARAM(ui8_t,
              EM_FPS_PAR_FULL_QUAL_OBJ,
              EM_FPS_PAR_FULL_QUAL_OBJ_DEFAULT) /*!< Ennable full eba reaction
on tracked objects depending on dynamic property. 0 Normal operation, 1 Activate
full quality @values:
struct {
.offset(0) = ui8_t Value;
.offset(0).mask(1<<(OBJECT_PROPERTY_MOVING))     = ui8_t Moving;
.offset(0).mask(1<<(OBJECT_PROPERTY_STATIONARY)) = ui8_t Stationary;
.offset(0).mask(1<<(OBJECT_PROPERTY_ONCOMING))   = ui8_t Oncoming;
}
*/

const float32 RCSThreshTargetOffsetStat[SI_NOF_RG] = {
    SI_RCSAdd_STATIONARY_OFFSET_00, SI_RCSAdd_STATIONARY_OFFSET_01,
    SI_RCSAdd_STATIONARY_OFFSET_02, SI_RCSAdd_STATIONARY_OFFSET_03,
    SI_RCSAdd_STATIONARY_OFFSET_04, SI_RCSAdd_STATIONARY_OFFSET_05,
    SI_RCSAdd_STATIONARY_OFFSET_06, SI_RCSAdd_STATIONARY_OFFSET_07,
    SI_RCSAdd_STATIONARY_OFFSET_08, SI_RCSAdd_STATIONARY_OFFSET_09,
    SI_RCSAdd_STATIONARY_OFFSET_10, SI_RCSAdd_STATIONARY_OFFSET_11,
    SI_RCSAdd_STATIONARY_OFFSET_12, SI_RCSAdd_STATIONARY_OFFSET_13,
    SI_RCSAdd_STATIONARY_OFFSET_14, SI_RCSAdd_STATIONARY_OFFSET_15,
    SI_RCSAdd_STATIONARY_OFFSET_16, SI_RCSAdd_STATIONARY_OFFSET_17,
    SI_RCSAdd_STATIONARY_OFFSET_18, SI_RCSAdd_STATIONARY_OFFSET_19,
    SI_RCSAdd_STATIONARY_OFFSET_20, SI_RCSAdd_STATIONARY_OFFSET_21,
    SI_RCSAdd_STATIONARY_OFFSET_22, SI_RCSAdd_STATIONARY_OFFSET_23,
    SI_RCSAdd_STATIONARY_OFFSET_24, SI_RCSAdd_STATIONARY_OFFSET_25,
    SI_RCSAdd_STATIONARY_OFFSET_26, SI_RCSAdd_STATIONARY_OFFSET_27,
    SI_RCSAdd_STATIONARY_OFFSET_28, SI_RCSAdd_STATIONARY_OFFSET_29,
    SI_RCSAdd_STATIONARY_OFFSET_30, SI_RCSAdd_STATIONARY_OFFSET_31,
    SI_RCSAdd_STATIONARY_OFFSET_32, SI_RCSAdd_STATIONARY_OFFSET_33,
    SI_RCSAdd_STATIONARY_OFFSET_34, SI_RCSAdd_STATIONARY_OFFSET_35,
    SI_RCSAdd_STATIONARY_OFFSET_36, SI_RCSAdd_STATIONARY_OFFSET_37,
    SI_RCSAdd_STATIONARY_OFFSET_38, SI_RCSAdd_STATIONARY_OFFSET_39,
    SI_RCSAdd_STATIONARY_OFFSET_40, SI_RCSAdd_STATIONARY_OFFSET_41,
    SI_RCSAdd_STATIONARY_OFFSET_42, SI_RCSAdd_STATIONARY_OFFSET_43,
    SI_RCSAdd_STATIONARY_OFFSET_44, SI_RCSAdd_STATIONARY_OFFSET_45,
    SI_RCSAdd_STATIONARY_OFFSET_46, SI_RCSAdd_STATIONARY_OFFSET_47,
    SI_RCSAdd_STATIONARY_OFFSET_48, SI_RCSAdd_STATIONARY_OFFSET_49,
    SI_RCSAdd_STATIONARY_OFFSET_50, SI_RCSAdd_STATIONARY_OFFSET_51,
    SI_RCSAdd_STATIONARY_OFFSET_52, SI_RCSAdd_STATIONARY_OFFSET_53,
    SI_RCSAdd_STATIONARY_OFFSET_54, SI_RCSAdd_STATIONARY_OFFSET_55,
    SI_RCSAdd_STATIONARY_OFFSET_56, SI_RCSAdd_STATIONARY_OFFSET_57,
    SI_RCSAdd_STATIONARY_OFFSET_58, SI_RCSAdd_STATIONARY_OFFSET_59,
    SI_RCSAdd_STATIONARY_OFFSET_60};
/*@EndParamGroup*/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */