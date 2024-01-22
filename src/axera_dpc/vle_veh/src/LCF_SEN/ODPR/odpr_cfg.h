#ifndef ODPR_CFG_H
#define ODPR_CFG_H

/*****************************************************************************
  CONSTS
*****************************************************************************/
// #define UNIT_TEST

#define FOP_MAX_STEER_ANGLE_RAD \
    0.0524f  // Maximum steering angle //P_ODPFOP_MaxSteerAngle_rad
#define FOP_MAX_LAT_ACL_MPS2 \
    1.4f  // Maximum lateral acceleration //P_ODPFOP_MaxLatAcl_mps2
#define FOP_DIST_Y_MAX_CRV_OFFSET_MET \
    0.5f  // Maximum lateral curvature offset at target object position
          // //P_ODPFOP_DistYMaxCrvOffset_met
#define FOP_FAC_MAX_DIST_Y_ACTIVE_NU \
    1.25f  // Factor for max dist Y activation //P_ODPFOP_FacMaxDistYActive_nu
#define FOP_MAX_DIST_Y_HYST_MET \
    0.5f  // Maximum hysteresis for lateral object position validation
          // //P_ODPFOP_MaxDistYHyst_met
#define FOP_MAX_DIST_Y_MET \
    0.75f  // Maximum valid lateral target object distance
           // //P_ODPFOP_MaxDistY_met
#define FOP_OBJ_TYPE_DEBOUNCE_TIME_SEC \
    2.f  // Debounce time time for target object validation
         // //P_ODPFOP_ObjTypeDebounceTime_sec
#define FOP_FUSION_M_CAM_FRONT_NU \
    0x0010  // Note: Should be a constant -> Cam fusion state
            // //P_ODPFOP_FusionMCamFront_nu
#define FOP_FUSION_RADAR_FRONT_NU \
    0x0001  // Note: Should be a constant -> radar fusion
            // //P_ODPFOP_FusionRadarFront_nu
#define FOP_MIN_DUR_CAM_FUS_BEF_BRID_SEC \
    2.f  // Minimum duration for cam fusion before bridging
         // //P_ODPFOP_MinDurCamFusBefBrid_sec
#define FOP_OBJ_LOSS_DEBOUNCE_T_SEC \
    0.1f  // Object loss debounce time //P_ODPFOP_ObjLossDebounceT_sec
#define FOP_MIN_MEAS_STATE_FOR_PRED_SEC \
    1.f  // Minimum time span for the object measurement to be 'MEASURED'
         // state...                   //P_ODPFOP_MinMeasStateForPred
#define FOP_OBJ_YAW_SAMPLE_TIME_SEC \
    1.f  // Sample time used for update frequency of calculated object yaw angle
         // based on...         //P_ODPFOP_ObjYawSampleTime_sec
#define FOP_OBJ_YAW_SAMPLE_DIST_MET \
    4.f  // Sample distance used for update frequency of calculated object yaw
         // angle based on...     //P_ODPFOP_ObjYawSampleDist_met
#define FOP_OBJ_YAW_PT1_TIME_SEC \
    0.5f  // PT1 filter time constant for either own calculated or input yaw
          // angle signal             //P_ODPFOP_ObjYawPT1Time_sec
#define FOP_USE_OWN_OBJ_YAW_BOOL \
    FALSE  // If TRUE then own calculated object yaw angle is used, if FALSE
           // then input signal is used //P_ODPFOP_UseOwnObjYaw_bool
#define FOP_LONG_DIST_MAX_HYST_MET \
    5.f  // Hysteresis distance for Object x-position
         // //P_ODPFOP_LongDistMaxHyst_met
#define FOP_MIN_MEAS_STATE_FOR_PRED_SEC \
    1.f  // Minimum time span for the object measurement to be 'MEASURED' state
         // before 'PREDITED'... //P_ODPFOP_MinMeasStateForPred_sec
#define FOH_ACC_OBJ_DTCT_BTM \
    0x0001  // Bitmask for S_ODPFOP_AccObjInvBitfield_btf to check if a new ACC
            // obj has been detected    //P_ODPFOH_AccObjDtct_btm
#define FOH_MIN_TRAJ_VALID_TIME_SEC \
    2.f  // Minimum Trajectory validity time for switch to Object History Mode
         // //P_ODPFOH_MinTrajValidTime_sec
#define FOH_MAX_OBJ_DIST_HYST_LSM_MET \
    3.f  // Object hysteresis distance for Low Speed Mode
         // //P_ODPFOH_MaxObjDistHystLSM_met
#define FOH_NUM_OBJ_DIST_HIST_PART_NU \
    0x00  // Number of sections relatively to object x-distance
          // //P_ODPFOH_NumObjDistHistParts_nu
#define FOH_ENABLE_TRANS_AT_START_BOOL \
    TRUE  // TRUE if transition handling is enabled at Kalman initialization
          // //P_ODPFOH_EnableTransAtStart_bool
#define FOH_ACC_OBJ_CHNG_DURATION_SEC \
    3.f  // Maximum transition time duration during ACC object change
         // //P_ODPFOH_AccObjChngDuration_sec
#define FOH_USE_ESTIM_POS_Y_BOOL \
    FALSE  // Specify flag that use estimation position y
           // //P_ODPFOH_UseEstimPosY_bool
#define FOH_USE_POS_Y_CORR_BOOL \
    TRUE  // Specify flag that use position y correction
          // //P_ODPFOH_UsePosYcorr_bool
#define FOH_USE_POS_Y_REDUCTION_BOOL \
    FALSE  // TRUE: input object y-position is squared if it is below a certain
           // threshold...            //P_ODPFOH_UsePosYReduction_bool
#define FOH_PF_CRV_DECAY_NU \
    1.f  // Specify curve decay //P_ODPFOH_PF_CrvDecay_nu
#define FOH_PF_CRV_CHNG_DECAY_NU \
    1.f  // Specify curve change decay //P_ODPFOH_PF_CrvChngDecay_nu
#define FOH_USE_CRV_ONLY_LSM_BOOL \
    TRUE  // TRUE:only curvature is used for target trajectory(PosY0 and
          // heading are set to 0)...      //P_ODPFOH_UseCrvOnlyLSM_bool
#define FOH_MAX_DELTA_EGO_CRV_LSM_LPM \
    0.0002f  // Maximum allowed curvature in LowSpeedMode between calculated and
             // EgoCurvature             //P_ODPFOH_MaxDeltaEgoCrvLSM_1pm
#define FOH_PT1_POSY0_ENABLE_BOOL \
    FALSE  // RUE means low pass filter for lateral target position is enabled
           // //P_ODPFOH_PT1PosY0Enable_bool
#define FOH_PT1_HEAD_ENABLE_BOOL \
    TRUE  // TRUE if low pass filter for heading is enabled
          // //P_ODPFOH_PT1HeadEnable_bool
#define FOH_PT1_CRV_ENABLE_BOOL \
    TRUE  // TRUE if low pass filter for curvature is enabled
          // //P_ODPFOH_PT1CrvEnable_bool
#define FOH_PT1_CRV_CHNG_ENABLE_BOOL \
    FALSE  // Specify flag that PT1 curve change enable
           // //P_ODPFOH_PT1CrvChngEnable_bool
#define FOH_MIN_POSX_VELX_ON_BOOL \
    FALSE  // TRUE to switch on velocity dependent minimum long object position
           // logic                   //P_ODPFOH_MinPosXVelXOn_bool

#define ILE_INPUT_RADAR_OBJECT_NUM \
    30u  // Maximum number of side radar object lists
#define ILE_OBJ_REL_RO_RADAR_BOOL \
    TRUE  // Falg that object is expressed in radar coordinates
          // //P_ODPILE_ObjRelToRadar_bool
#define ILE_DEFAULT_LANE_WIDTH_MET \
    3.75f  // Default lane width for object in lane estimation with only von
           // lane boundary measured     //P_ODPILE_DefaultLaneWidth_met
#define ILE_RELEVANT_OBJ_LONG_DIST_MET \
    80.f  // Maximum relevant object distance for output selection
          // //P_ODPILE_RelevantObjLongDist_met

#define POLY_ORDER_3RD (3u)  // test without c1, only c0
#define POLY_ORDER_1ST (2u)
/* number of used data samples for polyfit */
#define SAMPLE_POINTS (32u)

#define FOH_ENABLE_KALMANFILTER TRUE

#ifdef UNIT_TEST
#define STATIc
#else
#define STATIc static
#endif

#endif