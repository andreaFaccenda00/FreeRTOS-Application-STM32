#ifndef IWDG_H
#define IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the Independent Watchdog (IWDG) in bare-metal mode.
 *         Prescaler = /256, Reload = 0xFFF, then start.
 */
void MX_IWDG_Init(void);

/**
 * @brief  Reload the IWDG counter to prevent a reset.
 */
void IWDG_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif // IWDG_H
