/** \addtogroup version
 *  @{
 *
 * \file        TueObjFusn_Version.h
 * \brief       Type definition and accessor for object fusion version
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_VERSION_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"

//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
/*==================[macros]================================================*/

/** @name branch ID enumeration */
/** mainline */
#define TUEOBJFUSN_VERSION_SW_BRANCHID_MA (0u)
/** Mid Platform Branch */
#define TUEOBJFUSN_VERSION_SW_BRANCHID_MIDPLATFORM (1u)
/** undefined */
#define TUEOBJFUSN_VERSION_SW_BRANCHID_UNDEFINED (255u)
/** @} */

/** @name config ID enumeration */
/** HPF */
#define TUEOBJFUSN_VERSION_SW_CONFIGID_HPF (0u)
/** MPF */
#define TUEOBJFUSN_VERSION_SW_CONFIGID_MPF (1u)
/** undefined */
#define TUEOBJFUSN_VERSION_SW_CONFIGID_UNDEFINED (255u)
/** @} */

/*==================[type definitions]======================================*/
/** Version and configuration information
 */
typedef struct TueObjFusn_VersionTypeTag {
    uint8 sw_branchID;      /** branch ID (0=mainline/...) */
    uint8 sw_major_version; /** major version number */
    uint8 sw_minor_version; /** minor version number */
    uint8 sw_patch_version; /** major version number */
    uint8 sw_configID;      /** configuration ID (HPF/MPF/...) */
} TueObjFusn_VersionType;

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

boolean Fusion_fill_Version(CONSTP2VAR(TueObjFusn_VersionType,
                                       AUTOMATIC,
                                       ObjFusn_VAR_NOINIT) pVersion);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_VERSION_H_
/*==================[end of file]===========================================*/

/** @} */
