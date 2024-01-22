

#ifndef _SI_CUSTOM_H_INCLUDED
#define _SI_CUSTOM_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_sen.h"
#include "si_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  MACROS (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS (KOMPONENTENINTERN)
*****************************************************************************/

/*! The structure describing the custom measurement output per OOI object */
typedef struct SICustMeasOOI {
    fVelocity_t fAbsSpeedX; /*!< Absolute object speed in X direction */
    fAccel_t fAbsAccelX;    /*!< Absolute object acceleration in X direction */
    float32 fTTC;           /*!< TTC time for given OOI object for evaluation of
                               performance @unit:s */
    float32 fTimeGap; /*!< Timegap time for given OOI object for evaluation of
                         performance @unit:s */
} SICustMeasOOI_t;

/*****************************************************************************
  KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  VARIABLEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENINTERN)
*****************************************************************************/

/*---si_customerfunctions----*/
extern void SIInitCustomerFunctions(void);
extern boolean SICustMergePreselection(ObjNumber_t ObjNr,
                                       ACCObjectQuality_t uiAccQual,
                                       boolean bFunPresel);
extern void SICustomProcess(void);
extern void SIMergeCustomObjects(ObjNumber_t uiObjectToKeep,
                                 ObjNumber_t uiObjectToDelete);
extern boolean SICheckCustomInlaneCriteria(ObjNumber_t iObjNr,
                                           const CPTrajOccupancy_t* pOccupancy);
extern boolean SICheckCustomOutlaneCriteria(
    ObjNumber_t iObjNr, const CPTrajOccupancy_t* pOccupancy);

extern void SICustomCorridorPreProcessing(
    ObjNumber_t iObj, SIBracketFuncEnable_t* pBracketFuncEnable);

extern void SICustFillMeasOOI(ObjNumber_t ObjId,
                              SICustMeasOOI_t* pDestMeasData);

extern const GDBVector2_t* SI_GetPickupDistSamplePoints(
    uint8* pui_NumPointsCurve);

#ifdef __cplusplus
};
#endif

/* Ende der bedingten Einbindung */
#else
#endif
