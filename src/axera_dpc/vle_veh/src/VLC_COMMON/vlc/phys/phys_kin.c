/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * guotao <guotao1@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "../mat/mat_std_ext.h"
#include "./phys_kin_ext.h"

static sint32 getFactor(const sint32 D);
static times_t getTTC(const distance_t ObjectDistance,
                      const velocity_t dv,
                      sint32 *help);
static times_t PHYS_CALCULATE_TTC_NO_ONCOMING(const velocity_t ObjectVelocity,
                                              const acceleration_t ObjectAccel,
                                              const velocity_t HostVelocity,
                                              const acceleration_t HostAccel,
                                              const distance_t ObjectDistance,
                                              velocity_t *ImpactVelocity);

/*************************************************************************************************************************
  Functionname:    PHYS_CALC_NEEDED_DECEL */
// acceleration_t PHYS_CALC_NEEDED_DECEL(const acceleration_t long_accel,
//                                       const velocity_t long_velocity,
//                                       times_t reaction_time,
//                                       const velocity_t obj_rel_long_velocity,
//                                       const acceleration_t obj_rel_long_accel,
//                                       const velocity_t obj_long_velocity,
//                                       const acceleration_t obj_long_accel,
//                                       const distance_t obj_long_diplacement) {
// #define Phys_vehicle_max_decel (-10000)
// #define Phys_min_brake_distance (10)   /*must be above zero!*/
// #define Phys_object_accel_thres (-100) /*must be below zero!*/

//     distance_t braking_distance;
//     velocity_t v_host_after_t_react;
//     velocity_t v_rel_after_t_react;
//     times_t t_host;
//     times_t t_obj;
//     acceleration_t needed_accel;
//     sint32 localhelper1, localhelper2;

//     if (obj_rel_long_velocity < (velocity_t)0) {
//         /*calculate own speed after reaction time*/
//         localhelper1 = (sint32)long_accel * (sint32)reaction_time;
//         localhelper1 /= (sint32)Time_s;
//         localhelper1 *= (sint32)Velocity_s;
//         localhelper1 /= (sint32)Acceleration_s;

//         localhelper1 = (sint32)long_velocity + localhelper1;
//         v_host_after_t_react =
//             (velocity_t)MAT_LIM(localhelper1, (sint32)0, (sint32)Speed_max);

//         /*calculate relative speed after reaction time*/
//         localhelper1 = (sint32)obj_rel_long_accel * (sint32)reaction_time;
//         localhelper1 /= (sint32)Time_s;
//         localhelper1 *= (sint32)Velocity_s;
//         localhelper1 /= (sint32)Acceleration_s;

//         localhelper1 = (sint32)obj_rel_long_velocity + localhelper1;
//         v_rel_after_t_react = (velocity_t)MAT_LIM(
//             localhelper1, (sint32)Speed_min, (sint32)Speed_max);

//         /*calculate remaining braking distance after reaction time*/
//         /* d1 = t*v_rel */
//         localhelper1 = (sint32)reaction_time * (sint32)obj_rel_long_velocity;
//         localhelper1 /= (sint32)Velocity_s;
//         localhelper1 *= (sint32)Distance_s;
//         /*scaled by Time_s*/

//         /* d2 = arel*(t^2)*/
//         localhelper2 = (sint32)reaction_time * (sint32)reaction_time;
//         localhelper2 /= (sint32)Time_s;
//         localhelper2 *= (sint32)obj_rel_long_accel;
//         localhelper2 /= (sint32)Acceleration_s;
//         localhelper2 *= (sint32)Distance_s;
//         /*scaled by Time_s*/

//         /* d_rel = d1 + 1/2 * d2*/
//         localhelper1 += localhelper2 / (sint32)2;
//         localhelper1 /= (sint32)Time_s;
//         localhelper1 += (sint32)obj_long_diplacement;
//         braking_distance =
//             (distance_t)MAT_LIM(localhelper1, (sint32)Phys_min_brake_distance,
//                                 (sint32)Distance_max);

//         /*calculate needed acceleration (deceleration)*/
//         localhelper1 =
//             (sint32)v_rel_after_t_react * (sint32)v_rel_after_t_react;
//         localhelper1 /= (sint32)2 * (sint32)Velocity_s;
//         localhelper1 *= (sint32)Distance_s;
//         localhelper1 /= (sint32)braking_distance;
//         localhelper1 *= (sint32)Acceleration_s;
//         localhelper1 /= (sint32)Velocity_s;

//         localhelper2 =
//             (sint32)long_accel + (sint32)obj_rel_long_accel - localhelper1;

//         needed_accel = (acceleration_t)MAT_LIM(localhelper2, (sint32)Accel_min,
//                                                (sint32)Accel_max);

//         if (obj_long_accel < Phys_object_accel_thres) {
//             if ((needed_accel - obj_long_accel) < Phys_object_accel_thres) {
//                 localhelper1 = (sint32)v_rel_after_t_react;
//                 localhelper1 *= ((sint32)Acceleration_s);
//                 /*comment: div/0 -> needed_accel-obj_long_accel will always be
//                  * below zero!*/
//                 localhelper1 /=
//                     (((sint32)needed_accel) - ((sint32)obj_long_accel));
//                 localhelper1 *= ((sint32)Time_s);
//                 localhelper1 /= ((sint32)Velocity_s);

//                 localhelper1 = (sint32)reaction_time + localhelper1;
//                 t_host = (times_t)MAT_LIM(localhelper1, (sint32)Time_min,
//                                           (sint32)Time_max);
//             } else {
//                 t_host = (times_t)0;
//             }

//             /*time, the object needs to come to a full stop*/
//             localhelper1 = (sint32)(-obj_long_velocity);
//             localhelper1 *= ((sint32)Acceleration_s);
//             /*comment: div/0 -> obj_long_accel will always be below zero!*/
//             localhelper1 /= ((sint32)obj_long_accel);
//             localhelper1 *= ((sint32)Time_s);
//             localhelper1 /= ((sint32)Velocity_s);

//             t_obj = (times_t)MAT_LIM(localhelper1, (sint32)Time_min,
//                                      (sint32)Time_max);

//             if (t_obj < t_host) {
//                 /* Object will come to a stop, while host vehicle is still
//                  * moving */
//                 localhelper1 =
//                     (sint32)obj_long_velocity * (sint32)obj_long_velocity;
//                 localhelper1 /= (sint32)Velocity_s;
//                 localhelper1 *= (sint32)Distance_s;
//                 /*comment: div/0 -> obj_long_accel will always be below zero!*/
//                 localhelper1 /= (((sint32)2) * ((sint32)obj_long_accel));
//                 localhelper1 *= (sint32)Acceleration_s;
//                 /*scaled by Velocity_s*/

//                 localhelper2 = (sint32)reaction_time * (sint32)long_velocity;
//                 localhelper2 /= (sint32)Time_s;
//                 localhelper2 *= (sint32)Distance_s;
//                 /*scaled by Velocity_s*/

//                 localhelper1 += localhelper2;
//                 localhelper1 /= (sint32)Velocity_s;

//                 localhelper2 =
//                     (sint32)reaction_time * (sint32)reaction_time / (sint32)2;
//                 localhelper2 /= (sint32)Time_s;
//                 localhelper2 *= (sint32)long_accel;
//                 localhelper2 /= (sint32)Time_s;
//                 localhelper2 *= (sint32)Distance_s;
//                 localhelper2 /= (sint32)Acceleration_s;

//                 /*distance to brake*/
//                 localhelper1 = ((sint32)obj_long_diplacement - localhelper2) -
//                                localhelper1;

//                 braking_distance = (distance_t)MAT_LIM(
//                     localhelper1, (sint32)Phys_min_brake_distance,
//                     (sint32)Distance_max);

//                 /*get needed acceleration*/
//                 localhelper1 = (sint32)-v_host_after_t_react *
//                                (sint32)v_host_after_t_react;
//                 localhelper1 /= (sint32)Velocity_s;
//                 localhelper1 *= (sint32)Acceleration_s;
//                 /*comment: div/0 -> braking_distance will always be above
//                  * zero!*/
//                 localhelper1 /= (((sint32)2) * ((sint32)braking_distance));
//                 localhelper1 *= (sint32)Distance_s;
//                 localhelper1 /= (sint32)Velocity_s;
//                 needed_accel = (acceleration_t)MAT_LIM(
//                     localhelper1, (sint32)Accel_min, (sint32)Accel_max);
//             }
//         }
//     } else {
//         needed_accel = (acceleration_t)0;
//     }

//     needed_accel = (acceleration_t)MAT_MAX((sint32)needed_accel,
//                                            (sint32)Phys_vehicle_max_decel);

//     return needed_accel;
// }

static void PHYS_CHECK_OBJ_FULL_STOP(const velocity_t ObjectVelocity,
                                     const acceleration_t ObjectAccel,
                                     const distance_t ObjectDistance,
                                     velocity_t vh,
                                     acceleration_t ah,
                                     times_t *p_ttc,
                                     sint32 *p_D,
                                     velocity_t *p_v_obj_at_ttc,
                                     sint32 *p_sqrt_factor) {
    sint32 help, help2;
    if ((MAT_SIGN(ObjectAccel) !=
         MAT_SIGN(ObjectVelocity)) /*if direction of acceleration is different
                                      to direction of speed*/
        && (ObjectAccel != 0)) {
        /*set object velocity at impact to 0*/
        *p_v_obj_at_ttc = 0;

        /*v_ttc = vo + ao*ttc*/
        help = MAT_MUL(ObjectAccel, *p_ttc, Acceleration_s, Time_s, Velocity_s);
        help += (sint32)ObjectVelocity;

        if (MAT_SIGN(help) != MAT_SIGN(ObjectVelocity)) {
            /*object has come to a full stop*/
            /*time to stop: t = -vo/ao */

            help = MAT_DIV(-ObjectVelocity, ObjectAccel, Velocity_s,
                           Acceleration_s, Time_s);
            *p_ttc = (times_t)MAT_LIM(help, 0, Time_max);

            /*d_remain = a/2*(t^2)+ v*t + s*/
            /*a/2*(t^2)*/
            help = (sint32)ObjectAccel;
            help *= (sint32)*p_ttc;
            help /= (sint32)2;
            help /= (sint32)Acceleration_s;
            help *= (sint32)*p_ttc;
            help /= (sint32)Time_s;
            help *= (sint32)Distance_s;
            help /= (sint32)Time_s;

            /*v*t*/
            help2 =
                MAT_MUL(ObjectVelocity, *p_ttc, Velocity_s, Time_s, Distance_s);
            help += help2;
            /*+s*/
            help += (sint32)ObjectDistance;
            /*distance to stopped object stored in help (d_remain)*/

            if (ah == 0) {
                if (vh == 0) {
                    *p_ttc = Time_max;
                } else {
                    /*ttc = d_remain/vh*/
                    help2 = MAT_DIV(help, vh, Distance_s, Velocity_s, Time_s);
                    *p_ttc = (times_t)MAT_LIM(help2, 0, Time_max);
                }
            } else {
                /*D = (v^2)2*a*d_remain*/
                *p_D = MAT_MUL(vh, vh, Velocity_s, Velocity_s, Factor_s);

                /*2*a*d_remain*/
                help2 = MAT_MUL((sint32)2 * (sint32)ah, help, Acceleration_s,
                                Distance_s, Factor_s);

                *p_D += help2;

                /*D is scaled by (local_factor^2)*/
                if (*p_D >= (sint32)0) {
                    /*calculate max factor for D to get highest accuracy of
                     * sqrt(D)*/
                    if (*p_D > 0) {
                        *p_sqrt_factor =
                            MAT_MIN(Signed_int32_max / *p_D, Factor_s);
                    } else {
                        *p_sqrt_factor = Factor_s;
                    }

                    *p_D *= *p_sqrt_factor;
                    *p_D /= (sint32)Factor_s;
                    *p_D *= *p_sqrt_factor;
                    /*D = sqrt(D)*/
                    *p_D = MAT_SQRT(*p_D);
                    *p_D *= (sint32)Acceleration_s;
                    *p_D /= *p_sqrt_factor;
                    *p_D *= (sint32)Velocity_s;

                    /*-(vh-D)/a*/
                    help = (sint32)vh * (sint32)Acceleration_s;
                    help -= *p_D;
                    help /= (sint32)Velocity_s;
                    help *= (sint32)Time_s;
                    help /= -(sint32)ah;
                    if (help < (sint32)0) {
                        help = Time_max;
                    }

                    /*-(vh+D)/a*/
                    help2 = (sint32)vh * (sint32)Acceleration_s;
                    help2 += *p_D;
                    help2 /= (sint32)Velocity_s;
                    help2 *= (sint32)Time_s;
                    help2 /= -(sint32)ah;
                    if (help2 < (sint32)0) {
                        help2 = Time_max;
                    }

                    /*take smaller positive ttc as real ttc*/
                    if (help >= help2) {
                        *p_ttc = (times_t)MAT_MIN(help2, (sint32)Time_max);
                    } else {
                        *p_ttc = (times_t)MAT_MIN(help, (sint32)Time_max);
                    }
                } else {
                    *p_ttc = Time_max;
                }
            }

            /*if host came also to stand still while new ttc - ttc =  inf*/
            if ((MAT_SIGN((sint32)ah) !=
                 MAT_SIGN((sint32)vh)) /*if direction of acceleration is
                                          different to direction of speed*/
                && (ah != 0)) {
                /*v_ttc = vh + ah*ttc*/
                help = MAT_MUL(ah, *p_ttc, Acceleration_s, Time_s, Velocity_s);
                help += (sint32)vh;

                if (MAT_SIGN(help) != MAT_SIGN(vh)) {
                    /*host came to full stop*/
                    *p_ttc = Time_max;
                    *p_v_obj_at_ttc = 0;
                }
            }
        } else {
            /*object is still moving after ttc*/
            /*v_ttc = vo + ao*ttc*/
            help = MAT_MUL(ObjectAccel, *p_ttc, Acceleration_s, Time_s,
                           Velocity_s);
            help += (sint32)ObjectVelocity;
            *p_v_obj_at_ttc = (velocity_t)MAT_LIM(
                help, (sint32)0,
                (sint32)MAT_SIGN(ObjectVelocity) * (sint32)Speed_max);
        }
    } else {
        /*object is still moving after ttc or ttc is higher*/
        /*v_ttc = vo + ao*ttc*/
        help = MAT_MUL(ObjectAccel, *p_ttc, Acceleration_s, Time_s, Velocity_s);
        help += (sint32)ObjectVelocity;
        *p_v_obj_at_ttc = (velocity_t)MAT_LIM(
            help, 0, (sint32)MAT_SIGN(ObjectVelocity) * (sint32)Speed_max);
    }
}

/*************************************************************************************************************************
  Functionname:    PHYS_CALCULATE_TTC_NO_ONCOMING */
static times_t PHYS_CALCULATE_TTC_NO_ONCOMING(const velocity_t ObjectVelocity,
                                              const acceleration_t ObjectAccel,
                                              const velocity_t HostVelocity,
                                              const acceleration_t HostAccel,
                                              const distance_t ObjectDistance,
                                              velocity_t *ImpactVelocity) {
#define Phys_max_relevant_ttc (times_t)20000

    velocity_t dv;
    acceleration_t da;
    times_t ttc;
    sint32 help, help2;
    sint32 D;
    velocity_t vh;
    acceleration_t ah;
    velocity_t v_obj_at_ttc;
    sint32 sqrt_factor;

    vh = HostVelocity;
    ah = HostAccel;
    ttc = Time_max;
    v_obj_at_ttc = ObjectVelocity;

    /*delta_a = a_host - a_object*/
    da = (acceleration_t)((sint32)ObjectAccel - (sint32)HostAccel);

    /*delta_v = v_host - v_object*/
    dv = (acceleration_t)((sint32)ObjectVelocity - (sint32)HostVelocity);

    if (da == 0) {
        ttc = getTTC(ObjectDistance, dv, &help);
    } else {
        /*difference in acceleration*/

        /*v(h^2)+ v(o^2)+ 2 * (d * da - vh * vo)*/
        /*v(h^2)*/
        D = MAT_MUL(vh, vh, Velocity_s, Velocity_s, Factor_s);

        /*v(o^2)*/
        help = MAT_MUL(ObjectVelocity, ObjectVelocity, Velocity_s, Velocity_s,
                       Factor_s);

        /*v(h^2)+ v(o^2)*/
        D += help;

        /*d * da*/
        help =
            MAT_MUL(-da, ObjectDistance, Distance_s, Acceleration_s, Factor_s);

        /*vh * vo*/
        help2 = MAT_MUL(vh, ObjectVelocity, Velocity_s, Velocity_s, Factor_s);

        /*d * da - vh * vo*/
        help -= help2;

        /*v(h^2)+ v(o^2)+ 2 * (d * da - vh * vo)*/
        D += (sint32)2 * help;

        if (D >= (sint32)0) {
            /*calculate max factor for D to get highest accuracy of sqrt(D)*/
            sqrt_factor = getFactor(D);

            D *= sqrt_factor;
            D /= (sint32)Factor_s;
            D *= sqrt_factor;

            D = MAT_SQRT(D);
            D *= (sint32)Acceleration_s;
            D /= sqrt_factor;
            D *= (sint32)Velocity_s;

            /*ttc1 = (-dv + sqrt(D)) / da*/
            help = -(sint32)dv * (sint32)Acceleration_s;
            help += D;
            help /= (sint32)Velocity_s;
            help *= (sint32)Time_s;
            help /= (sint32)da;
            if (help < (sint32)0) {
                help = Time_max;
            }

            /*ttc2 = (-dv - sqrt(D)) / da*/
            help2 = (sint32)-dv * (sint32)Acceleration_s;
            help2 -= D;
            help2 /= (sint32)Velocity_s;
            help2 *= (sint32)Time_s;
            help2 /= (sint32)da;
            if (help2 < (sint32)0) {
                help2 = Time_max;
            }
            ttc = (help >= help2 ? (times_t)MAT_MIN(help2, Time_max)
                                 : (times_t)MAT_MIN(help, Time_max));
        } else {
            ttc = Time_max;
        }
    }

    /*check if object has come to full stop while ttc*/
    PHYS_CHECK_OBJ_FULL_STOP(ObjectVelocity, ObjectAccel, ObjectDistance, vh,
                             ah, &ttc, &D, &v_obj_at_ttc, &sqrt_factor);

    /*calculate impact velocity*/
    /*if ttc is small enough to be effective*/
    if (ttc < Phys_max_relevant_ttc) {
        /*v_ttc = ah*ttc+vh*/
        help = MAT_MUL(ah, ttc, Acceleration_s, Time_s, Velocity_s);
        help += (sint32)vh;
        /*impactvelocity = v_ttc - v_obj_ttc*/
        help -= (sint32)v_obj_at_ttc;
    } else {
        help = 0;
    }

    *ImpactVelocity = (velocity_t)MAT_LIM(help, 0, Speed_max);

    return ttc;
}

/*************************************************************************************************************************
  Functionname:    PHYS_CALCULATE_TTC */
times_t PHYS_CALCULATE_TTC(const velocity_t ObjectVelocity,
                           const acceleration_t ObjectAccel,
                           const velocity_t HostVelocity,
                           const acceleration_t HostAccel,
                           const distance_t ObjectDistance,
                           velocity_t *ImpactVelocity) {
#define local_min_accel (acceleration_t)100

    velocity_t local_object_velocity;
    velocity_t local_host_velocity;
    velocity_t local_object_accel;
    velocity_t local_host_accel;
    velocity_t local_object_distance;

    velocity_t vh;
    acceleration_t ah;
    boolean switch_objects;
    sint32 tts_h, tts_o, help;

    switch_objects = FALSE;

    vh = HostVelocity;
    ah = HostAccel;

    /*copy object*/
    local_object_distance = ObjectDistance;
    local_object_velocity = ObjectVelocity;
    local_object_accel = ObjectAccel;
    local_host_velocity = HostVelocity;
    local_host_accel = HostAccel;

    if (ObjectVelocity < 0) {
        /*host vehicle may come to a full stop*/
        if (MAT_SIGN(ah) != MAT_SIGN(vh)) {
            /*object may not come to a full stop*/
            if ((MAT_SIGN(ObjectVelocity) == MAT_SIGN(ObjectAccel)) ||
                (ObjectAccel == 0)) {
                switch_objects = TRUE;
            } else {
                if (ObjectVelocity == 0) {
                    switch_objects = FALSE;
                } else {
                    /*target object may also come to a full stop*/
                    /*need to calculate tts for both objects*/
                    help = MAT_DIV(-ObjectVelocity, ObjectAccel, Velocity_s,
                                   Acceleration_s, Time_s);
                    tts_o = (times_t)MAT_LIM(help, 0, Time_max);

                    if (ah != 0) {
                        help = MAT_DIV(-vh, ah, Velocity_s, Acceleration_s,
                                       Time_s);
                        tts_h = (times_t)MAT_LIM(help, 0, Time_max);
                    } else {
                        if (vh == 0) {
                            switch_objects = TRUE;
                            tts_h = 0;
                        } else {
                            tts_h = Time_max;
                        }
                    }

                    /*if not already set, use calculations from tts*/
                    if (switch_objects == FALSE) {
                        if (tts_h < tts_o) {
                            switch_objects = TRUE;
                        }
                    }
                }
            }
        }
    }

    /*switch objects*/
    if (switch_objects == TRUE) {
        local_object_accel = (acceleration_t)-ah;
        local_object_velocity = (velocity_t)-vh;
        local_host_accel = (acceleration_t)-ObjectAccel;
        local_host_velocity = (acceleration_t)-ObjectVelocity;
    }

    /*avoid small values of a / arel due to fixed point limitations!*/
    if (MAT_ABS((sint32)local_object_accel) < (sint32)local_min_accel) {
        local_object_accel = 0;
    }

    if (MAT_ABS((sint32)local_object_accel - (sint32)local_host_accel) <
        (sint32)local_min_accel) {
        local_host_accel = local_object_accel;
    }

    return PHYS_CALCULATE_TTC_NO_ONCOMING(
        local_object_velocity, local_object_accel, local_host_velocity,
        local_host_accel, local_object_distance, ImpactVelocity);
}

static times_t getTTC(const distance_t ObjectDistance,
                      const velocity_t dv,
                      sint32 *help) {
    times_t ttc = 0;

    // same acceleration
    if (dv == 0) {
        // same speed
        ttc = Time_max;
    } else {
        // ttc = -d/dv
        *help = MAT_DIV(-ObjectDistance, dv, Distance_s, Velocity_s, Time_s);
        if (*help < (sint32)0) {
            // object faster
            ttc = Time_max;
        } else {
            ttc = (times_t)MAT_MIN(*help, Time_max);
        }
    }
    return ttc;
}

static sint32 getFactor(const sint32 D) {
    sint32 siSqrtFactor;
    if (D > 0) {
        siSqrtFactor = MAT_MIN((Signed_int32_max) / D, Factor_s);
    } else {
        siSqrtFactor = Factor_s;
    }

    return siSqrtFactor;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */