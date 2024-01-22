/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cd.h"

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  @fn            CDCalcCorridorWidth */
float32 CDCalcCorridorWidth(const float32 fDistX,
                            const float32 fLength,
                            const float32 fNarrow,
                            const float32 fWide,
                            const float32 fInflectionpoint) {
    float32 fWidth;

    if ((fDistX > fInflectionpoint) && (fDistX <= fLength) &&
        (fLength > fInflectionpoint)) {
        fWidth = (((fWide - fNarrow) * (fDistX - fInflectionpoint)) /
                  (fLength - fInflectionpoint)) +
                 fNarrow;
    } else if (fDistX > fLength) {
        fWidth = fWide;
    } else {
        fWidth = fNarrow;
    }

    return fWidth;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */