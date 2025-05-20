#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*============================================================================*/
/* ⚙️  Application Configuration Constants                                    */
/*----------------------------------------------------------------------------*/
/*  Defines all system-wide constants related to traffic control logic,       */
/*  timing, thresholds, and event flags.                                      */
/*============================================================================*/

/*----------------------------------------------------------
 * 🚗 Vehicle Parameters
 * Controls random vehicle generation and traffic priority
 *---------------------------------------------------------*/
#define MAX_VEHICLES            20U     // Max number of vehicles per cycle
#define PRIORITY_THRESHOLD      15U     // Vehicles above this trigger longer green

/*----------------------------------------------------------
 * ⏱️ Standard Timing Durations (in milliseconds)
 * Used for normal traffic light and pedestrian control
 *---------------------------------------------------------*/
#define T_GREEN_MS              4000U   // Default green light duration
#define T_YELLOW_MS             1500U   // Yellow light transition time

/*----------------------------------------------------------
 * ⏱️ Adaptive and Pedestrian Timing
 * Includes green extension and pedestrian blink settings
 *---------------------------------------------------------*/
#define T_GREEN_EXTENSION_MS    (T_GREEN_MS * 3U / 2U) // Extended green when busy
#define T_PED_TOTAL_MS          5000U   // Total pedestrian crossing time
#define T_PED_BLINK_MS          1000U   // Duration of blinking before end
#define T_PED_BLINK_INTERVAL_MS 250U    // Blink interval for pedestrian light

/*----------------------------------------------------------
 * 🚩 Event Flags (bitmask)
 * Used by RTOS tasks to communicate events
 *---------------------------------------------------------*/
#define EMG_FLAG        (1U << 0)  // Emergency event active
#define PED_FLAG_NS     (1U << 1)  // Pedestrian request for North-South
#define PED_FLAG_EST    (1U << 2)  // Pedestrian request for East

#endif // APP_CONFIG_H
