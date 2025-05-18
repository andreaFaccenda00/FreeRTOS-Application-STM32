#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define MAX_VEHICLES            20U
#define PRIORITY_THRESHOLD      15U

#define T_GREEN_MS              4000U
#define T_YELLOW_MS             1500U

#define T_GREEN_EXTENSION_MS    (T_GREEN_MS * 3U/2U)
#define T_PED_TOTAL_MS          5000U
#define T_PED_BLINK_MS          1000U
#define T_PED_BLINK_INTERVAL_MS 250U

#define EMG_FLAG (1U << 0)
#define PED_FLAG (1U << 0)
#define IRQ_FLAG (1U << 2)

#endif // APP_CONFIG_H
