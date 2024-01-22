/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "vlc_glob_ext.h"
#include "switch_ext.h"
#include "TM_Math_Cal.h"

/*****************************************************************************
  @fn             SWITCH_INIT_SWITCH */ /*!

                                         @description    initialize switches

                                         @param[in]      pSwitch   Switch that
                                       shall be initialized

                                         @return         void

                                       *****************************************************************************/
void SWITCH_INIT_SWITCH(switch_t* const pSwitch) {
    pSwitch->AKT_STATUS = FALSE;
    pSwitch->LAST_STATUS = FALSE;
    pSwitch->CYCLE_TIMER = switch_cycletimer_init;
    pSwitch->DURATION_TIME_INACTIVE = switch_dtia_max_val;
    pSwitch->DURATION_TIME_ACTIVE = 0u;
    pSwitch->OK_WHILE_SWITCHED_ON = FALSE;
}

/*****************************************************************************
  @fn             SWITCH_SET_STATE */ /*!

                                           @description    sets a new switch
                                         state for a specific switch

                                           @param[in,out]  pSwitch  Switch that
                                         shall be set
                                           @param[in]      State    the new
                                         state for the switch (TRUE, FALSE)

                                           @return         void

                                         *****************************************************************************/
void SWITCH_SET_STATE(switch_t* const pSwitch, const boolean State) {
    const boolean lastState = (boolean)pSwitch->AKT_STATUS;

    /*save old value*/
    pSwitch->LAST_STATUS = pSwitch->AKT_STATUS;
    /*Set new value*/
    pSwitch->AKT_STATUS = State;

    /*count cycles of (in)activity*/
    if (State == TRUE) {
        if (pSwitch->DURATION_TIME_ACTIVE < switch_dta_max_val) {
            pSwitch->DURATION_TIME_ACTIVE++;
        }
    } else {
        if (pSwitch->DURATION_TIME_INACTIVE < switch_dtia_max_val) {
            pSwitch->DURATION_TIME_INACTIVE++;
        }
    }

    if (lastState == FALSE) {
        pSwitch->DURATION_TIME_ACTIVE = (uint16)0;
    } else {
        pSwitch->DURATION_TIME_INACTIVE = (uint16)0;
    }
}

/*****************************************************************************
  @fn           SWITCH_RISING_EDGE */ /*!

                                           @description  checks if the switch
                                         was switched on

                                           @param[in]    pSwitch    Switch to
                                         check
                                           @param[in]    Condition Condition
                                         that allows the switch to be switched
                                         on (0 = false, 1 = true)

                                           @return       SWITCH_STATE_OFF(0) =
                                         FALSE,
                                                         SWITCH_STATE_ON(1) =
                                         TRUE,
                                                         SWITCH_STATE_ACTION_OFF(2)
                                         = switched on, but the condition is not
                                         fulfilled

                                         *****************************************************************************/
switch_state_t SWITCH_RISING_EDGE(const switch_t* const pSwitch,
                                  const boolean Condition) {
    switch_state_t retValue = SWITCH_STATE_OFF;

    if ((pSwitch->AKT_STATUS == TRUE) && (pSwitch->LAST_STATUS == FALSE)) {
        if (Condition != FALSE) {
            retValue = SWITCH_STATE_ON;
        } else {
            retValue = SWITCH_STATE_ACTION_OFF;
        }
    }
    return retValue;
}

/*****************************************************************************
  @fn           SWITCH_FALLING_EDGE */ /*!

                                          @description  checks if the switch was
                                        switched off

                                          @param[in]    pSwitch   Switch to
                                        check
                                          @param[in]    Condition Condition that
                                        allows the switch to be switched off (0
                                        = false, 1 = true)

                                          @return       SWITCH_STATE_OFF(0) =
                                        FALSE,
                                                        SWITCH_STATE_ON(1) =
                                        TRUE,
                                                        SWITCH_STATE_ACTION_OFF(2)
                                        = switched on, but the condition is not
                                        fulfilled

                                        *****************************************************************************/
switch_state_t SWITCH_FALLING_EDGE(switch_t* const pSwitch,
                                   const boolean Condition) {
    switch_state_t retValue = SWITCH_STATE_OFF;

    // if (Condition == FALSE)
    // {
    //   pSwitch->OK_WHILE_SWITCHED_ON = FALSE;
    // }

    if ((pSwitch->AKT_STATUS == FALSE) && (pSwitch->LAST_STATUS == TRUE)) {
        if (Condition == TRUE)  // && (pSwitch->OK_WHILE_SWITCHED_ON == TRUE))
        {
            retValue = SWITCH_STATE_ON;
        } else {
            retValue = SWITCH_STATE_ACTION_OFF;
        }
    }

    // if((pSwitch->AKT_STATUS == FALSE) && (Condition == TRUE))
    // {
    //   /*switch was in off state while the condition is true*/
    //   pSwitch->OK_WHILE_SWITCHED_ON = TRUE;
    // }

    return retValue;
}

/*****************************************************************************
  @fn           SWITCH_HOLD_REPEAT */ /*!

                                           @description  returns
                                         SWTICH_STATE_ON, if the switch was hold
                                         long enough to get repeated signals
                                         from that switch

                                           @param[in]    pSwitch         Switch
                                         to check
                                           @param[in]    StartCondition
                                         Condition that allows the switch to be
                                         switched on (0 = false, 1 = true)
                                           @param[in]    HoldCondition
                                         Condition that allows the switch to
                                         send repeated signals after has been
                                         switched on (0 = false, 1 = true)
                                           @param[in]    StartTime       Time,
                                         the switch needs to be in on state
                                         before it returns true for the first
                                         time
                                           @param[in]    RepeatTime      Time,
                                         between the repeated signals after the
                                         switch has returned true the first time
                                         (if 0 -> function returns only one true
                                         after start time)

                                           @return       SWITCH_STATE_OFF(0) =
                                         FALSE,
                                                         SWITCH_STATE_ON(1) =
                                         TRUE,
                                                         SWITCH_STATE_ACTION_OFF(2)
                                         = currently not returned

                                         *****************************************************************************/
switch_state_t SWITCH_HOLD_REPEAT(switch_t* const pSwitch,
                                  const boolean StartCondition,
                                  const boolean HoldCondition,
                                  uint16 StartTime,
                                  uint16 RepeatTime) {
    switch_state_t retValue = SWITCH_STATE_OFF;

    if ((pSwitch->AKT_STATUS == FALSE) /*switch off*/
        || ((HoldCondition == FALSE) &&
            (pSwitch->CYCLE_TIMER != switch_cycletimer_init)) ||
        (StartCondition != TRUE) /*not all conditions true*/
        ) {
        /*init cycle timer*/
        pSwitch->CYCLE_TIMER = switch_cycletimer_init;
    } else /*button pressed with all conditions met*/
    {
        const switch_state_t StartOK =
            SWITCH_RISING_EDGE(pSwitch, StartCondition);

        if ((pSwitch->CYCLE_TIMER ==
             switch_cycletimer_init)        /*cycle timer not initialized*/
            && (StartOK == SWITCH_STATE_ON) /*start conditions true*/
            ) {                             /*initialize cycle with startTime*/
            pSwitch->CYCLE_TIMER =
                (uint16)MIN(switch_cycletimer_init - 1, (sint32)StartTime);
        } else {
            if ((pSwitch->CYCLE_TIMER > (uint16)0) /*cycle timer > 0*/
                && (pSwitch->CYCLE_TIMER <
                    switch_cycletimer_init) /*cycle timer is initialized*/
                && (HoldCondition == TRUE)  /*hold condition true*/
                ) {
                pSwitch->CYCLE_TIMER--; /*decrease cycle timer*/
            }

            if (pSwitch->CYCLE_TIMER == (uint16)0) /*cycle timer 0ed*/
            { /*initialize cycle timer with repeat time*/
                pSwitch->CYCLE_TIMER =
                    (uint16)MIN(switch_cycletimer_init - 1, (sint32)RepeatTime);
                if (pSwitch->CYCLE_TIMER == (uint16)0) {
                    pSwitch->CYCLE_TIMER =
                        switch_cycletimer_init; /*reinitialize cycle timer
                                                   (returns only one true after
                                                   start time if repeattime=0)*/
                }
                retValue = SWITCH_STATE_ON;
            }
        }
    }
    return retValue;
}

/*************************************************************************************************************************
  Functionname:    SWITCH_SET_COND_COUNTER                                                                          */ /*!

      @brief           SWITCH_SET_COND_COUNTER

      @description     decreases a counter until it is 0 in a specific
    condition, otherwise the counter will be reseted.
                       if the system is not engaged, the counter will be set to
    0.

      @return          void

      @param[in]       Condition : a condition that can be true or false
      @param[in]       InitValue : an initialization value, the Counter will be
    set to if the condition is false
      @param[in]       DecValue : the decrement value the counter will be
    decremented with if condition is true
      @param[in,out]   pCounter : a pointer to a counter
      @param[in]       SelectedFunctionActive :

    *************************************************************************************************************************/
void SWITCH_SET_COND_COUNTER(const boolean Condition,
                             const uint16 InitValue,
                             const uint16 DecValue,
                             uint16* const pCounter,
                             const boolean SelectedFunctionActive) {
    if (Condition == TRUE) {
        if (SelectedFunctionActive == FALSE) {
            (*pCounter) = (uint16)0;
        } else {
            if ((*pCounter) > DecValue) {
                (*pCounter) = (uint16)(*pCounter - DecValue);
            } else {
                (*pCounter) = (uint16)0;
            }
        }
    } else {
        (*pCounter) = InitValue;
    }
}

/*************************************************************************************************************************
  Functionname:    SWITCH_SET_COUNTER                                                                               */ /*!

      @brief           SWITCH_SET_COUNTER

      @description     decreases a counter until it is 0, the counter will not
    be reseted.

      @return          void

      @param[in]       DecValue :  the decrement value the counter will be
    decremented with
      @param[in,out]   pCounter : a pointer to a counter
    *************************************************************************************************************************/
void SWITCH_SET_COUNTER(const uint16 DecValue, uint16* const pCounter) {
    if ((*pCounter) >= DecValue) {
        (*pCounter) = (uint16)(*pCounter - DecValue);
    } else {
        (*pCounter) = (uint16)0;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */