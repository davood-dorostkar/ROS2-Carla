

#ifndef _VLC_SEN_CUSTOM_TYPES_H_INCLUDED
#define _VLC_SEN_CUSTOM_TYPES_H_INCLUDED

#ifndef Envm_GEN_OBJECT_MT_STATE_INACTIVE
#define Envm_GEN_OBJECT_MT_STATE_INACTIVE (Envm_GEN_OBJECT_MT_STATE_DELETED)
#endif

#define VLC_SEN_BUMPER2COG_DIST_DEFAULT (2.5f)

/*generate Structure for general custom types*/
/*use ARS EM Geometry for CustDimension_t*/
typedef Envm_t_CR_Geometry VLCCustDimension_t;

/*declare VLCCustGeometry_t*/
typedef struct {
    float32 fDistX;
    float32 fDistY;
} VLCCustDistance_t;

typedef struct {
    VLCCustDimension_t Dimension; /*width, length and orientation*/
    VLCCustDistance_t Distance;   /*DistX, DistY*/
} VLCCustGeometry_t;              /* Object size related properties */

typedef struct TrajOccupancy {
    float32 fOverlap;                /*!<fOverlap */
    float32 fOverlapVar;             /*!<fOverlapVar */
    float32 fObjectOccupancy;        /*!<fObjectOccupancy */
    float32 fObjectOccupancyVar;     /*!<fObjectOccupancyVar */
    float32 fTrajectoryOccupancy;    /*!<fTrajectoryOccupancy */
    float32 fTrajectoryOccupancyVar; /*!<fTrajectoryOccupancyVar */
} ObjOccupancy_t;

typedef struct VLCCustomObjectProps {
    Envm_t_GenObjAttributes Attributes;
    Envm_t_CR_Geometry Geometry;
    ObjOccupancy_t Occupancy;
} VLCCustomObjectProperties_t;

#endif /* _VLC_SEN_CUSTOM_TYPES_H_INCLUDED */
