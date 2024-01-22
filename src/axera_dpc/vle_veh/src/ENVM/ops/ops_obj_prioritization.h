#ifndef _OPS_OBJ_PRIORITIZATION_INCLUDED
#define _OPS_OBJ_PRIORITIZATION_INCLUDED

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/

/************************************************************************/
/* FUNCTIONS                                                   */
/************************************************************************/
void OPSInitObjectPrioritization(void);
void FPS_v_InitObjPrioStaticData(void);

void TUE_OPS_PrioListOutputProcess(void);
static void OPS_v_GenerateRangeSortedObjPrioList(
    Envm_t_ObjectPrioIndexArray pa_ObjectPrioIndexRangeSorted);
extern void Envm_v_ObjOutSetPrioIndexList(
    const Envm_t_ObjectPrioIndexArray p_ObjectPrioIndex,
    const Envm_t_ObjectPrioIndexArray p_ObjectPrioIndexRangeSorted);
#endif