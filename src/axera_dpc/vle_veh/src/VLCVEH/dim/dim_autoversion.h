
#ifndef _DIM_AUTOVER_H_INCLUDED
#define _DIM_AUTOVER_H_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief       VLCDIM_SW_MAIN_VER_NO
    @general     -
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     0x02uL   @unit -     @min -   @max -   */
#define VLCDIM_SW_MAIN_VER_NO 0x02uL

/*! @brief       VLCDIM_SW_SUB_VER_NO
    @general     -
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     0x00uL   @unit -     @min -   @max -   */
#define VLCDIM_SW_SUB_VER_NO 0x00uL

/*! @brief       VLCDIM_SW_BUG_FIX_LEV
    @general     -
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     0x00uL   @unit -     @min -   @max -   */
#define VLCDIM_SW_BUG_FIX_LEV 0x00uL

/*! @brief       VLCDIM_SW_VERSION_NUMBER
  @general     -
  @conseq      @incp  -
               @decp  -
  @attention   -
  @typical     -   @unit -     @min -   @max -   */
#define VLCDIM_SW_VERSION_NUMBER                                     \
    ((VLCDIM_SW_MAIN_VER_NO << 16u) | (VLCDIM_SW_SUB_VER_NO << 8u) | \
     (VLCDIM_SW_BUG_FIX_LEV))

/*CHECKSUMCODE*/

/*! @brief       DIM_AUTOVERSION
@general     -
@conseq      @incp  -
             @decp  -
@attention   -
@typical     VLCDIM_SW_VERSION_NUMBER   @unit -     @min -   @max -   */
#define DIM_AUTOVERSION VLCDIM_SW_VERSION_NUMBER
/*/CHECKSUMCODE*/

#ifdef __cplusplus
};
#endif

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#endif /* _DIM_AUTOVER_H_INCLUDED */
