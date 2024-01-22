

#ifndef _EMP_CALCULATION_H_INCLUDED
#define _EMP_CALCULATION_H_INCLUDED
/*****************************************************************************
  INCLUDES
*****************************************************************************/
//#include "emp_main.h"
#include "emp_ext.h"
/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF STRUCT
   **************************************************************** */

/*! @brief Data type to store EMP Object Description
 */

typedef struct EMPObjExDesc {
    EMPPos2D_t sCenter;    /*!< Center */
    EMPVar2D_t sVar;       /*!< Variance in the location */
    EMPSize2D_t sGeometry; /*!< Geometry representated by Width and Length */
    float32 fOrientation;  /*!< Orientation of object */
} EMPObjExDesc_t;

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  PUBLIC FUNCTIONS
*****************************************************************************/
extern boolean EMPCalcObjExDesc(float32 fTime,
                                const EMPObjPred_t *pIn_Obj,
                                EMPObjExDesc_t *pOut_ExistenceDesc);
extern boolean EMPCalcCollProbExDesc(const EMPObjExDesc_t *pIn_ExDescObj1,
                                     const EMPObjExDesc_t *pIn_ExDescObj2,
                                     float32 *pOut_CollisionProb);
extern boolean EMPCalcMinDistTime(const EMPTrajPred_t *pIn_Obj1,
                                  const EMPTrajPred_t *pIn_Obj2,
                                  float32 *pOut_MinDistTime);
extern float32 EMPCalcObjObjDistAtTime(float32 fTime,
                                       const EMPTrajPred_t *pIn_Obj1,
                                       const EMPTrajPred_t *pIn_Obj2);
/*!  @cond Doxygen_Suppress */
extern float32 EMPCalcPredictionTime(float32 fYawRate);
/*! @endcond */
extern float32 EMPCalcVariance(const float32 afValues[], uint16 uiArraySize);

/*****************************************************************************
  LOCAL FUNCTIONS
*****************************************************************************/

#endif /* _EMP_CALCULATION_H_INCLUDED */
