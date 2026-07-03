#ifndef COMPONENT_TEST_H
#define COMPONENT_TEST_H
/**
  ******************************************************************************
  * @file    component_test.h
  * @brief   HydraSense per-component bring-up / test harness  (STM32G070KBT6)
  *
  *  HOW TO USE
  *  ----------
  *  1. In your STM32CubeIDE project symbols (or at the top of main.h) define:
  *           COMPONENT_TEST_MODE
  *     so main.c calls CTest_Init / CTest_RunOnce / CTest_RunLoop instead of
  *     the full application.
  *
  *  2. Pick which peripherals to exercise by COMMENTING / UNCOMMENTING the
  *     TEST_* switches in the block below. Anything left commented out is
  *     completely skipped (its driver is not even touched), so you can isolate
  *     a single part on a half-populated board.
  *
  *  3. Flash, run, and open  Window -> Show View -> Live Expressions  in
  *     CubeIDE. Add the single symbol:   g_ctest
  *     Every sensor value, pass/fail code and counter updates live in there —
  *     no UART printf needed. The WS2812B ring also shows a colour summary
  *     (see CTest_RunLoop): green = all enabled tests passing, red = at least
  *     one failed, blue breathing = still running first pass.
  *
  *  4. The BLE test additionally echoes a line over USART1 @ 9600 so you can
  *     watch it from the JDY-29 side in a serial terminal if you want.
  *
  *  Nothing here blocks forever; CTest_RunLoop() is safe to call every
  *  main-loop iteration.
  ******************************************************************************
  */

#include "stm32g0xx_hal.h"
#include <stdint.h>

/* ===========================================================================
 *  COMPONENT SELECT  —  comment a line out to SKIP that component entirely.
 * ===========================================================================
 *  (Leave only the ones populated on the board you are testing.)
 */
#define TEST_RGB          /* WS2812B 10-LED ring  (TIM1_CH1 + DMA1_Ch1, PA8)   */
#define TEST_BUZZER       /* Passive buzzer        (TIM3_CH2 PWM, PB5)          */
#define TEST_NTC          /* NTC thermistor        (ADC CH3, PA3)               */
#define TEST_TDS          /* TDS purity probe      (ADC CH2 + drive PA6)        */
#define TEST_BATTERY      /* Battery gauge + CHRG/STDBY (ADC CH7 PA7,PB0,PB1)   */
#define TEST_HX711        /* Load cell front-end   (DOUT PA4, SCK PA5)          */
#define TEST_BMA253       /* Motion IMU            (I2C1 0x18)                  */
#define TEST_RTC          /* PCF8563 RTC           (I2C1 0x51)                  */
#define TEST_EEPROM       /* M24512 log EEPROM     (I2C1 0x50)                  */
#define TEST_FLASH        /* Internal flash store  (data_storage, top 4 pages) */
#define TEST_BLE          /* JDY-29 BLE UART       (USART1 @ 9600, PA9/PA10)    */
#define TEST_BUTTON       /* User button           (PB3, active-low)           */

/* ===========================================================================
 *  Per-test result codes (kept identical to bringup_test.h for familiarity).
 * ===========================================================================
 */
typedef enum {
    CT_SKIP    = 0,   /* test not compiled in (TEST_* commented out) */
    CT_PENDING = 1,   /* enabled, not yet run / in progress          */
    CT_PASS    = 2,
    CT_FAIL    = 3,
} CTestResult_t;

/* ===========================================================================
 *  Live-Expressions snapshot.  Add `g_ctest` as a single watch in CubeIDE.
 * ===========================================================================
 */
typedef struct {
    /* ---- bookkeeping ---- */
    uint32_t uptime_ms;
    uint32_t loop_count;
    uint8_t  first_pass_done;   /* 1 after CTest_RunOnce() completes  */
    uint8_t  any_fail;          /* 1 if any enabled test == CT_FAIL   */
    uint8_t  enabled_count;     /* number of TEST_* compiled in       */
    uint8_t  pass_count;        /* enabled tests currently passing     */

    /* ---- WS2812B / RGB ---- */
    CTestResult_t rgb_status;
    uint32_t      rgb_send_count;
    uint8_t       rgb_busy;

    /* ---- Buzzer ---- */
    CTestResult_t buz_status;
    uint8_t       buz_busy;
    uint8_t       buz_pattern_idx;

    /* ---- NTC ---- */
    CTestResult_t ntc_status;
    int16_t       ntc_temp_x10;     /* 253 = 25.3 C */

    /* ---- TDS ---- */
    CTestResult_t tds_status;
    uint16_t      tds_ppm;

    /* ---- Battery ---- */
    CTestResult_t bat_status;
    uint16_t      bat_mv;
    uint8_t       bat_pct;
    uint8_t       bat_charging;
    uint8_t       bat_full;

    /* ---- HX711 ---- */
    CTestResult_t hx_status;
    uint8_t       hx_ready;
    int32_t       hx_raw;
    int32_t       hx_grams;

    /* ---- BMA253 IMU ---- */
    CTestResult_t bma_status;
    int16_t       bma_x_mg;
    int16_t       bma_y_mg;
    int16_t       bma_z_mg;
    uint8_t       bma_motion;

    /* ---- PCF8563 RTC ---- */
    CTestResult_t rtc_status;
    uint8_t       rtc_hh, rtc_mm, rtc_ss;
    uint8_t       rtc_tick;

    /* ---- M24512 EEPROM ---- */
    CTestResult_t ee_status;
    uint8_t       ee_present;
    uint16_t      ee_count;

    /* ---- Internal flash store ---- */
    CTestResult_t flash_status;
    uint32_t      flash_crc_expected;
    uint32_t      flash_crc_readback;

    /* ---- BLE ---- */
    CTestResult_t ble_status;
    uint8_t       ble_connected;
    uint16_t      ble_rx_bytes;

    /* ---- Button ---- */
    CTestResult_t btn_status;
    uint8_t       btn_level;        /* 1 = pressed (active-low decoded)   */
    uint32_t      btn_press_count;
} CompTest_t;

extern CompTest_t g_ctest;

/* ===========================================================================
 *  API  — called from main.c
 * ===========================================================================
 *  CTest_Init     : wires up every enabled driver. Call after all MX_*_Init().
 *  CTest_RunOnce  : one full pass (incl. a blocking RGB + buzzer walk).
 *  CTest_RunLoop  : call every main-loop iteration; polls on a 1 s cadence,
 *                   drives the RGB summary animation and buzzer/LED updates.
 *
 *  Argument order matches main.c:
 *      CTest_Init(&hadc, &hi2c1, &huart1, &htim1 [WS], &htim3 [buzzer]);
 */
void CTest_Init(ADC_HandleTypeDef *hadc,
                I2C_HandleTypeDef *hi2c,
                UART_HandleTypeDef *huart,
                TIM_HandleTypeDef *htim_ws,
                TIM_HandleTypeDef *htim_buz);

void CTest_RunOnce(void);
void CTest_RunLoop(void);

/* Optional ISR hooks — call these from stm32g0xx_it.c if you want the harness
 * to see live BLE bytes / RTC ticks / motion interrupts while testing.
 * They are no-ops unless the matching TEST_* is enabled. */
void CTest_BLE_RxISR(void);
void CTest_RTC_TickISR(void);
void CTest_Motion_ISR(void);

#endif /* COMPONENT_TEST_H */
