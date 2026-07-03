#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

/* ============================================================================
 * power_manager.h — HydraSense low-power / sleep manager  (STM32G070KBT6)
 *
 * Implements a real STOP-mode deep sleep with wake-on-motion via the BMA253
 * any-motion (slope) interrupt on MOTION_INT (PA0 / EXTI0), plus wake on the
 * physical button (PB3 / EXTI3) and on a BLE link-state edge (PA15).
 *
 * Why STOP (not the old "soft-off"): the previous s_soft_off only blanked the
 * LEDs while the CPU kept running the full loop at 64 MHz — that is not a
 * low-power state. STOP mode gates the core and most clocks; typical draw
 * drops from tens of mA to tens of µA, and the pack lasts "days" as the PRD
 * requires.
 *
 * Wake sources kept enabled during STOP:
 *    PA0  (EXTI0)  — BMA253 motion  → bottle picked up / moved
 *    PB3  (EXTI3)  — button press   → user wake
 *    PA15 (EXTI15) — BLE link edge  → phone connected (auto-sync on wake)
 * Wake source MASKED during STOP:
 *    PA11 (EXTI11) — PCF8563 1 Hz tick. Leaving it on would wake the MCU
 *                    every second and defeat the sleep. The PCF8563 keeps
 *                    real time in hardware; on wake we RTC_Read() to resync.
 *
 * NOTE (reminders while asleep): reminders only fire while the unit is awake.
 * If you want the hourly reminder to WAKE the unit, program a PCF8563 alarm
 * on RTC_INT and leave EXTI11 unmasked for that one event — see the block
 * comment in Power_EnterSleep().
 * ==========================================================================*/

#include "stm32g0xx_hal.h"
#include "bma253.h"
#include "hx711.h"
#include "rtc_manager.h"
#include "ble_jdy29.h"
#include <stdint.h>

/* Auto-sleep after this much continuous inactivity (no drink, motion, button
 * or BLE traffic). 5 min default; tune to taste. 0 disables auto-sleep. */
#ifndef POWER_AUTO_SLEEP_MS
#define POWER_AUTO_SLEEP_MS   300000UL   /* 5 minutes */
#endif

void    Power_Init(BMA253_Handle_t *imu, HX711_Handle_t *hx,
                   RTC_Handle_t *rtc, BLE_Handle_t *ble);

/* Enter STOP mode and block until a wake source fires, then restore clocks,
 * resync the RTC and re-arm peripherals. Caller should flush storage first. */
void    Power_EnterSleep(void);

/* Request sleep on the next App_Run() service (used by SLEEP cmd / button). */
void    Power_RequestSleep(void);
uint8_t Power_SleepRequested(void);

/* Mark "user is active now" — resets the auto-sleep idle timer. */
void    Power_NoteActivity(void);

/* True once POWER_AUTO_SLEEP_MS of inactivity has elapsed (and enabled). */
uint8_t Power_IdleTimedOut(void);

uint8_t Power_IsAsleep(void);

/* ISR hooks (called from the EXTI callbacks in main.c). */
void    Power_MotionISR(void);   /* PA0  BMA253 any-motion */
void    Power_WakeISR(void);     /* PB3 button / PA15 BLE  */

#endif /* POWER_MANAGER_H */
