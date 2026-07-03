/* ============================================================================
 * power_manager.c — HydraSense STOP-mode sleep + wake-on-motion
 *
 * See power_manager.h for the wake-source policy.
 * ==========================================================================*/

#pragma GCC optimize("Os")
#include "power_manager.h"
#include "main.h"
#include "ws2812b.h"
#include "buzzer.h"

/* SystemClock_Config lives in main.c — after STOP the core wakes on HSI16,
 * so the PLL (64 MHz) must be re-established before normal operation. */
extern void SystemClock_Config(void);

/* EXTI line for the PCF8563 1 Hz tick (PA11). Masked during STOP. */
#define EXTI_LINE_RTC_TICK   (1UL << 11)

static BMA253_Handle_t *s_imu   = NULL;
static HX711_Handle_t  *s_hx    = NULL;
static RTC_Handle_t    *s_rtc   = NULL;
static BLE_Handle_t    *s_ble   = NULL;

static volatile uint8_t s_wake_flag      = 0U;
static volatile uint8_t s_sleep_request  = 0U;
static uint8_t          s_asleep         = 0U;
static uint32_t         s_last_active_ms  = 0U;

void Power_Init(BMA253_Handle_t *imu, HX711_Handle_t *hx,
                RTC_Handle_t *rtc, BLE_Handle_t *ble)
{
    s_imu  = imu;
    s_hx   = hx;
    s_rtc  = rtc;
    s_ble  = ble;
    s_wake_flag     = 0U;
    s_sleep_request = 0U;
    s_asleep        = 0U;
    s_last_active_ms = HAL_GetTick();
}

void Power_NoteActivity(void)   { s_last_active_ms = HAL_GetTick(); }
void Power_RequestSleep(void)   { s_sleep_request = 1U; }
uint8_t Power_SleepRequested(void) { return s_sleep_request; }
uint8_t Power_IsAsleep(void)    { return s_asleep; }

uint8_t Power_IdleTimedOut(void)
{
#if (POWER_AUTO_SLEEP_MS == 0UL)
    return 0U;
#else
    return ((HAL_GetTick() - s_last_active_ms) >= POWER_AUTO_SLEEP_MS) ? 1U : 0U;
#endif
}

/* ISR context: keep tiny. Setting the flag makes the STOP loop below exit. */
void Power_MotionISR(void)
{
    s_wake_flag = 1U;
    if (s_imu) BMA253_MotionISR(s_imu);
    s_last_active_ms = HAL_GetTick();
}

void Power_WakeISR(void)
{
    s_wake_flag = 1U;
    s_last_active_ms = HAL_GetTick();
}

/* Read+discard BMA253 INT_STATUS_0 so a non-latched slope line settles low
 * before we arm the wake, avoiding an immediate false wake. Best-effort. */
static void Power_ClearMotionLatch(void)
{
    if (!s_imu) return;
    uint8_t reg = BMA253_REG_INT_STATUS_0, val = 0U;
    HAL_I2C_Master_Transmit(s_imu->hi2c, BMA253_I2C_ADDR, &reg, 1, 20);
    HAL_I2C_Master_Receive (s_imu->hi2c, BMA253_I2C_ADDR, &val, 1, 20);
    (void)val;
}

void Power_EnterSleep(void)
{
    s_sleep_request = 0U;

    /* ── 1. Quiesce loads ─────────────────────────────────────────────── */
    Buzzer_Stop();
    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
    WS2812B_SetAll(RGB_OFF);
    WS2812B_SendBlocking();          /* drive the ring dark, then DMA stops */
    if (s_hx) HX711_PowerDown();     /* HX711 into ~1 µA power-down         */

    /* ── 2. Arm wake-on-motion ────────────────────────────────────────── */
    Power_ClearMotionLatch();

    /* ── 3. Mask the 1 Hz RTC tick so it does not wake us every second.
     * (To use the reminder-alarm-wake option, program a PCF8563 alarm and
     *  DO NOT mask EXTI11 — the alarm INT will then wake the unit.) */
    EXTI->IMR1 &= ~EXTI_LINE_RTC_TICK;

    s_asleep    = 1U;
    s_wake_flag = 0U;

    /* ── 4. STOP loop: stay down until a permitted source sets s_wake_flag.
     * Any EXTI returns from WFI; after STOP the clock tree is on HSI16, so we
     * restore the 64 MHz PLL each time before re-testing the flag. */
    do {
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        SystemClock_Config();
        HAL_ResumeTick();
    } while (!s_wake_flag);

    /* ── 5. Wake: restore everything ──────────────────────────────────── */
    EXTI->IMR1 |= EXTI_LINE_RTC_TICK;      /* re-enable 1 Hz tick           */

    if (s_hx)  HX711_PowerUp();
    if (s_rtc && s_rtc->initialized) RTC_Read(s_rtc);   /* resync wall clock */

    /* Re-arm the UART RX interrupt — a byte may have arrived mid-wake and
     * the single-byte IT could be stale after STOP. */
    if (s_ble) {
        HAL_UART_AbortReceive(s_ble->huart);
        BLE_StartReceive(s_ble);
    }

    s_asleep = 0U;
    s_last_active_ms = HAL_GetTick();

    /* Gentle wake cue. */
    Buzzer_Play(BUZZER_SINGLE_BEEP);
    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
}
