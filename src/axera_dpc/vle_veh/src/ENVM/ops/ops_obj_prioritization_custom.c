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
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops.h"
#include "ops_par.h"

/*! Object priorisation necessary only if generic object list is active */
#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE == 1)
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*!< Maximum number of object which can be selected based on customer specific
   requirements
     -> MUST be lower or equal to FPS_PRIO_QUOTA_CUSTOM */
#define FPS_PRIO_NO_CUST_OBJ_IN_OUTPUT (0u)

#if (FPS_CFG_QUOTA_PRIO_VERSION)
#if (FPS_PRIO_NO_CUST_OBJ_IN_OUTPUT > FPS_PRIO_QUOTA_CUSTOM)
#error \
    "FPS_PRIO_NO_CUST_OBJ_IN_OUTPUT MUST <=  FPS_PRIO_QUOTA_CUSTOM (fps_par.h, fps_obj_priorization.c)"
#endif
#endif

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/* ****************************************************************************

  @fn                FPS_b_ObjPrioByCustom                        */ /*!

             @brief             Determine if object must be selected based on
           customer specific requirements

             @description       Determine if object must be selected based on
           customer specific requirements

             @param[in]         ui_Obj: Object ID

             @param[out]        -

             @param[in,out]     -

             @return            b_PrioByCustom: TRUE -> object should be
           selected based on customer specific requirements
                                                FALSE -> otherwise

             @pre               -

             @post              -


           ****************************************************************************
           */
boolean FPS_b_ObjPrioByCustom(uint32 const ui_Obj) {
    boolean b_PrioByCustom;

    /*! Set default value */
    b_PrioByCustom = FALSE;

    /*! If custom specific requirements have to be fulfilled (custom specific
     * object selection) */

    UNREFERENCED_FORMAL_PARAMETER(ui_Obj);

    return b_PrioByCustom;
}

/* ****************************************************************************

  Functionname :     FPS_b_PrioObjRejectCustom                        */
boolean FPS_b_PrioObjRejectCustom(uint32 const ui_Obj,
                                  const float32 f_DistToTrajAbs) {
    /*! Set default value */
    boolean b_RejectObj = FALSE;

    UNREFERENCED_FORMAL_PARAMETER(f_DistToTrajAbs);

    UNREFERENCED_FORMAL_PARAMETER(ui_Obj);

    return b_RejectObj;
}

/* ****************************************************************************

  @fn                FPS_v_InitObjPrioStaticCustomData                        */ /*!

 @brief             Initialization of static variables

 @description       Initialization of static variables for object prioritization

 @param[in]         -

 @param[out]        -

 @param[in,out]     -

 @return            -

 @pre               -

 @post              -

**************************************************************************** */
void FPS_v_InitObjPrioStaticCustomData(void) {}

#endif /*!< CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */