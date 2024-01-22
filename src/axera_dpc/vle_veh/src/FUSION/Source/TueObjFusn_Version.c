/** \addtogroup version
 * \{
 *
 * \file        TueObjFusn_Version.c
 * \brief       Accessor for object fusion version
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      *
                      * <br>=====================================================<br>
                      * <b>Copyright 2014 by Tuerme.</b>
                      * <br>
                      * All rights reserved. Property of Tuerme.<br>
                      * Restricted rights to use, duplicate or disclose of this code<br>
                      * are granted through contract.
                      * <br>=====================================================<br>
                      */
/*==================[inclusions]============================================*/
#include "TueObjFusn_Version_int.h"
#include "TueObjFusn_ErrorCodes.h"
#include "tue_prv_common_types.h"

/**
 * @brief Get version and configuration information
 *
 * @param pVersion pointer to output structure
 *
 * @return TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_SLOW_CODE

boolean Fusion_fill_Version(
    CONSTP2VAR(TueObjFusn_VersionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pVersion) /* PRQA S 1503 */ /* external interface function */
{
    boolean bSuccess = FALSE;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pVersion) {
        bSuccess = FALSE;
    } else
#endif
    {
        pVersion->sw_branchID = TUEOBJFUSN_VERSION_SW_BRANCHID;
        pVersion->sw_major_version = TUEOBJFUSN_VERSION_SW_MAJOR_VERSION;
        pVersion->sw_minor_version = TUEOBJFUSN_VERSION_SW_MINOR_VERSION;
        pVersion->sw_patch_version = TUEOBJFUSN_VERSION_SW_PATCH_VERSION;
        pVersion->sw_configID = TUEOBJFUSN_VERSION_SW_CONFIGID;

        bSuccess = TRUE; /* no errror */
    }

    return bSuccess;
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/** \} */
