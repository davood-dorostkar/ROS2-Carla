/** \addtogroup objectlist
 *  @{
 * \file    TueObjFusn_au16FsnID.h
 * \brief  This is a structure definition file for the debug output array
 *
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUEOBJFUSN_AU16FSNID_H_
#define TUEOBJFUSN_AU16FSNID_H_

// #include "TM_Global_Types.h"
#include "TM_PlatF_Types.h"
#include "Compiler.h"

#include "Compiler_Cfg.h"
// #include "TM_Global_Types.h"
#include "TM_PlatF_Types.h"
#include "TM_Soc_Ips.h"
#include "glob.h"
#include "Project_specific_Types.h"
#include "Fusion_Std_Types.h"

/*==================[macros]================================================*/

/** Variable-sized list of trackables.
 * the trackable list represents object and track lists
 */
typedef struct TueObjFusn_au16FsnIdTag {
    uint16 au16FsnId[32];

} TueObjFusn_au16FsnId;

#endif /**@} TUEOBJFUSN_AU16FSNID_H_ */
