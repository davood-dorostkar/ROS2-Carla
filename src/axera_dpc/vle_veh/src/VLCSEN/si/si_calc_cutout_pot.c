/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static float32 SiCalculateCutOutPot(const fDistance_t p_dist,
                                    const fDistance_t FullPotLine,
                                    const fDistance_t ZeroPotLine);

/*************************************************************************************************************************
  Functionname:    SiCalculateCutOutPotFiltered */
void SiCalculateCutOutPotFiltered(const float32 p_dist,
                                  const float32 FullPotLine,
                                  const float32 ZeroPotLine,
                                  const ObjNumber_t ObjNr) {
    float32 PotFilter_Konst;
    const fVelocity_t fEgoSpeed = EGO_SPEED_X_OBJ_SYNC;
    float32 local_pot;

    local_pot = SiCalculateCutOutPot(p_dist, FullPotLine, ZeroPotLine);

    /* Constant for potential filtering */
    PotFilter_Konst = 0.5f;

    /* if distance of object is more than 2.1s no potential shall be calculated,
     * because results are not quite secure */
    /* or if road type is country road or city */
    if ((MIN_FLOAT(SiMaxDistCutOut, (fEgoSpeed * SiMaxDistPredictionTime)) <
         OBJ_LONG_DISPLACEMENT(ObjNr))) {
        local_pot = 0.0f; /*@todo: Save Cut Out potential in special variable */
    } else {
        /* filtering the potential */
        local_pot =
            GDB_FILTER(local_pot, (float32)OBJ_GET_CUT_OUT_POTENTIAL(ObjNr),
                       PotFilter_Konst);

        /* keep potential between 0% and 100% */
        if (local_pot >= 100.0f) {
            local_pot = 100.0f;
        } else if (local_pot <= 0.0f) {
            local_pot = 0.0f;
        } else {
            /* do nothing */
        }
    }

    OBJ_GET_CUT_OUT_POTENTIAL(ObjNr) = (uint8)local_pot;
}

/*************************************************************************************************************************
  Functionname:    SiCalculateCutOutPot */
static float32 SiCalculateCutOutPot(const fDistance_t p_dist,
                                    const fDistance_t FullPotLine,
                                    const fDistance_t ZeroPotLine) {
    float32 fLocReturnValue;
    float32 Potline = fABS(FullPotLine - ZeroPotLine);
    if (Potline < BML_f_AlmostZero) {
        fLocReturnValue = 0.0F;
    } else {
        fLocReturnValue =
            ((p_dist - ZeroPotLine) / (FullPotLine - ZeroPotLine)) * 100.0f;
    }
    return (fLocReturnValue);
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */