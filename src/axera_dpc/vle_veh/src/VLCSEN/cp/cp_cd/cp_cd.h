/*---

**************************************************************************** */

#ifndef _CP_SPECIFIC_CD_INCLUDED
#define _CP_SPECIFIC_CD_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp.h"
#include "cp_cd_par.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF STRUCT CPCDObjToTrajRelation_t
    **************************************************************** */
/*! @brief Object to Trajectory relation structure

    @general this structure has displacement and velocity parameters of object
   w.r.t trajectory

    @conseq [None]

    @attention [None]

    */
typedef struct {
    float32 fDistToTraj;     /*!< lateral object displacement to a trajectory*/
    float32 fDistToTrajVar;  /*!< variance of lateral object displacement to a
                                trajectory*/
    float32 fVelocityToTraj; /*!< lateral object velocity to a trajectory*/
    float32 fVelocityToTrajVar; /*!< variance of lateral object velocity to a
                                   trajectory*/
} CPCDObjToTrajRelation_t;

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

extern void CPCDProcess(void);
extern void CPCDCriticalTimeAfterMerge(ObjNumber_t iObjId);

extern void CPCDProcessTrajectoriesMeas(CPTrajMeasInfo_t *pTrajectoriesMeas);

extern float32 CPCDGetCurvatureEgo(const float32 fXPosition);
extern float32 CPCDGetCurvatureRoad(const float32 fXPosition);

extern const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationEgo(
    ObjNumber_t iObjId);
extern const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationRoad(
    ObjNumber_t iObjId);

#endif /* end of #ifndef _CP_SPECIFIC_CD_INCLUDED */
