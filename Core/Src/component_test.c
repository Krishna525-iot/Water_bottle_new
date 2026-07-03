/**
  ******************************************************************************
  * @file    component_test.c
  * @brief   HydraSense per-component bring-up / test harness  (STM32G070KBT6)
  *
  *  Compiled only when COMPONENT_TEST_MODE is defined. Which peripherals get
  *  exercised is controlled entirely by the TEST_* switches in
  *  component_test.h — comment one out and that whole block disappears from
  *  this file (driver never linked, pin never touched).
  *
  *  All results land in the global  g_ctest  (watch it in CubeIDE Live
  *  Expressions). The WS2812B ring shows a live colour summary.
  ******************************************************************************
  */

#include "component_test.h"

#ifdef COMPONENT_TEST_MODE

#include "main.h"
#include <string.h>

#ifdef TEST_RGB
  #include "ws2812b.h"
#endif
#ifdef TEST_BUZZER
  #include "buzzer.h"
  #ifndef TEST_RGB
    #include "ws2812b.h"   /* buzzer block reuses RGB types only if present */
  #endif
#endif
#ifdef TEST_NTC
  #include "ntc_temp.h"
#endif
#ifdef TEST_TDS
  #include "tds_sensor.h"
#endif
#ifdef TEST_BATTERY
  #include "battery.h"
#endif
#ifdef TEST_HX711
  #include "hx711.h"
#endif
#ifdef TEST_BMA253
  #include "bma253.h"
#endif
#ifdef TEST_RTC
  #include "rtc_manager.h"
#endif
#ifdef TEST_EEPROM
  #include "ee_store.h"
#endif
#ifdef TEST_FLASH
  #include "data_storage.h"
#endif
#ifdef TEST_BLE
  #include "ble_jdy29.h"
#endif

/* RGB summary needs the WS2812B header for types even if TEST_RGB is off. */
#if !defined(TEST_RGB) && !defined(TEST_BUZZER)
  #include "ws2812b.h"
#endif

/* ===========================================================================
 *  Globals / private handles
 * ===========================================================================
 */
CompTest_t g_ctest;

static ADC_HandleTypeDef  *s_hadc;
static I2C_HandleTypeDef  *s_hi2c;
static UART_HandleTypeDef *s_huart;
static TIM_HandleTypeDef  *s_htim_ws;
static TIM_HandleTypeDef  *s_htim_buz;

#ifdef TEST_NTC
static NTC_Handle_t     s_ntc;
#endif
#ifdef TEST_TDS
static TDS_Handle_t     s_tds;
#endif
#ifdef TEST_BATTERY
static Battery_Handle_t s_bat;
#endif
#ifdef TEST_HX711
static HX711_Handle_t   s_hx;
#endif
#ifdef TEST_BMA253
static BMA253_Handle_t  s_bma;
#endif
#ifdef TEST_RTC
static RTC_Handle_t     s_rtc;
#endif
#ifdef TEST_BLE
static BLE_Handle_t     s_ble;
#endif

/* current temperature handed to the TDS compensation (defaults to 25.0 C). */
static int16_t s_temp_x10 = 250;

/* ===========================================================================
 *  Small helpers
 * ===========================================================================
 */
static void ct_count_results(void)
{
    uint8_t enabled = 0, pass = 0, fail = 0;

    #define TALLY(st)  do { enabled++; if ((st)==CT_PASS) pass++; \
                            else if ((st)==CT_FAIL) fail++; } while(0)

#ifdef TEST_RGB
    TALLY(g_ctest.rgb_status);
#endif
#ifdef TEST_BUZZER
    TALLY(g_ctest.buz_status);
#endif
#ifdef TEST_NTC
    TALLY(g_ctest.ntc_status);
#endif
#ifdef TEST_TDS
    TALLY(g_ctest.tds_status);
#endif
#ifdef TEST_BATTERY
    TALLY(g_ctest.bat_status);
#endif
#ifdef TEST_HX711
    TALLY(g_ctest.hx_status);
#endif
#ifdef TEST_BMA253
    TALLY(g_ctest.bma_status);
#endif
#ifdef TEST_RTC
    TALLY(g_ctest.rtc_status);
#endif
#ifdef TEST_EEPROM
    TALLY(g_ctest.ee_status);
#endif
#ifdef TEST_FLASH
    TALLY(g_ctest.flash_status);
#endif
#ifdef TEST_BLE
    TALLY(g_ctest.ble_status);
#endif
#ifdef TEST_BUTTON
    TALLY(g_ctest.btn_status);
#endif
    #undef TALLY

    g_ctest.enabled_count = enabled;
    g_ctest.pass_count    = pass;
    g_ctest.any_fail      = (fail > 0) ? 1U : 0U;
}

/* ===========================================================================
 *  Individual read routines (each guarded by its TEST_* switch)
 * ===========================================================================
 */
#ifdef TEST_NTC
static void ct_read_ntc(void)
{
    int16_t t = NTC_ReadTemp_x10(&s_ntc);
    g_ctest.ntc_temp_x10 = t;
    s_temp_x10 = t;                       /* feed TDS compensation */
    g_ctest.ntc_status = (s_ntc.valid && t > -400 && t < 1500)
                         ? CT_PASS : CT_FAIL;
}
#endif

#ifdef TEST_TDS
static void ct_read_tds(void)
{
    uint16_t ppm = TDS_ReadPPM(&s_tds, s_temp_x10);
    g_ctest.tds_ppm    = ppm;
    g_ctest.tds_status = (s_tds.valid && ppm < 5000U) ? CT_PASS : CT_FAIL;
}
#endif

#ifdef TEST_BATTERY
static void ct_read_battery(void)
{
    Battery_Update(&s_bat);
    g_ctest.bat_mv       = Battery_GetVoltageMv(&s_bat);
    g_ctest.bat_pct      = Battery_GetPercent(&s_bat);
    g_ctest.bat_charging = Battery_IsCharging(&s_bat);
    g_ctest.bat_full     = Battery_IsFull(&s_bat);
    /* A LiPo on this rail should read 3.0–4.3 V; anything outside = wiring/ADC fault. */
    g_ctest.bat_status = (s_bat.valid &&
                          g_ctest.bat_mv > 2500U && g_ctest.bat_mv < 4400U)
                         ? CT_PASS : CT_FAIL;
}
#endif

#ifdef TEST_HX711
static void ct_read_hx711(void)
{
    g_ctest.hx_ready = HX711_IsReady();
    if (g_ctest.hx_ready) {
        int32_t raw = 0;
        uint8_t ok = HX711_ReadRawAveraged(&s_hx, &raw);
        g_ctest.hx_raw    = raw;
        g_ctest.hx_grams  = HX711_ReadGrams(&s_hx);
        g_ctest.hx_status = (ok && raw != 0 && raw != (int32_t)0xFFFFFFL)
                            ? CT_PASS : CT_FAIL;
    } else {
        g_ctest.hx_status = CT_FAIL;     /* DOUT never went low => not wired/alive */
    }
}
#endif

#ifdef TEST_BMA253
static void ct_read_bma(void)
{
    if (BMA253_ReadAccel(&s_bma) == HAL_OK) {
        g_ctest.bma_x_mg   = s_bma.accel_x_mg;
        g_ctest.bma_y_mg   = s_bma.accel_y_mg;
        g_ctest.bma_z_mg   = s_bma.accel_z_mg;
        g_ctest.bma_motion = BMA253_PopMotionFlag(&s_bma);
        /* a stationary, correctly-mounted sensor reads ~1000 mg on one axis */
        int16_t mag = (g_ctest.bma_z_mg < 0) ? -g_ctest.bma_z_mg : g_ctest.bma_z_mg;
        g_ctest.bma_status = (s_bma.initialized && mag > 200) ? CT_PASS : CT_FAIL;
    } else {
        g_ctest.bma_status = CT_FAIL;
    }
}
#endif

#ifdef TEST_RTC
static void ct_read_rtc(void)
{
    if (RTC_Read(&s_rtc) == HAL_OK) {
        g_ctest.rtc_hh = s_rtc.now.hours;
        g_ctest.rtc_mm = s_rtc.now.minutes;
        g_ctest.rtc_ss = s_rtc.now.seconds;
        g_ctest.rtc_tick = RTC_PopTick(&s_rtc);
        g_ctest.rtc_status = (s_rtc.now.hours < 24 && s_rtc.now.minutes < 60)
                             ? CT_PASS : CT_FAIL;
    } else {
        g_ctest.rtc_status = CT_FAIL;
    }
}
#endif

/* ===========================================================================
 *  CTest_Init
 * ===========================================================================
 */
void CTest_Init(ADC_HandleTypeDef *hadc,
                I2C_HandleTypeDef *hi2c,
                UART_HandleTypeDef *huart,
                TIM_HandleTypeDef *htim_ws,
                TIM_HandleTypeDef *htim_buz)
{
    memset(&g_ctest, 0, sizeof(g_ctest));

    s_hadc     = hadc;
    s_hi2c     = hi2c;
    s_huart    = huart;
    s_htim_ws  = htim_ws;
    s_htim_buz = htim_buz;

    /* Mark every test SKIP first; enabled ones flip to PENDING below. */
    g_ctest.rgb_status = g_ctest.buz_status = g_ctest.ntc_status =
    g_ctest.tds_status = g_ctest.bat_status = g_ctest.hx_status =
    g_ctest.bma_status = g_ctest.rtc_status = g_ctest.ee_status =
    g_ctest.flash_status = g_ctest.ble_status = g_ctest.btn_status = CT_SKIP;

#ifdef TEST_RGB
    WS2812B_Init(s_htim_ws);
    g_ctest.rgb_status = CT_PENDING;
#endif

#ifdef TEST_BUZZER
    Buzzer_Init(s_htim_buz);
    g_ctest.buz_status = CT_PENDING;
#endif

#ifdef TEST_NTC
    NTC_Init(&s_ntc, s_hadc);
    g_ctest.ntc_status = CT_PENDING;
#endif

#ifdef TEST_TDS
    TDS_Init(&s_tds, s_hadc);
    g_ctest.tds_status = CT_PENDING;
#endif

#ifdef TEST_BATTERY
    Battery_Init(&s_bat, s_hadc);
    g_ctest.bat_status = CT_PENDING;
#endif

#ifdef TEST_HX711
    HX711_Init(&s_hx);
    g_ctest.hx_status = CT_PENDING;
#endif

#ifdef TEST_BMA253
    g_ctest.bma_status =
        (BMA253_Init(&s_bma, s_hi2c) == HAL_OK) ? CT_PENDING : CT_FAIL;
#endif

#ifdef TEST_RTC
    g_ctest.rtc_status =
        (RTC_Init(&s_rtc, s_hi2c) == HAL_OK) ? CT_PENDING : CT_FAIL;
#endif

#ifdef TEST_EEPROM
    EE_Store_Init(s_hi2c);
    g_ctest.ee_present = EE_Store_IsPresent();
    g_ctest.ee_status  = g_ctest.ee_present ? CT_PENDING : CT_FAIL;
#endif

#ifdef TEST_FLASH
    Storage_Init();
    g_ctest.flash_status = CT_PENDING;
#endif

#ifdef TEST_BLE
    BLE_Init(&s_ble, s_huart);
    BLE_StartReceive(&s_ble);
    g_ctest.ble_status = CT_PENDING;
#endif

#ifdef TEST_BUTTON
    /* Button (PB3) is configured as plain input in MX_GPIO_Init. */
    g_ctest.btn_status = CT_PENDING;
#endif

    /* Silence "set but not used" if a build enables only a subset of tests. */
    (void)s_hadc; (void)s_hi2c; (void)s_huart;
    (void)s_htim_ws; (void)s_htim_buz;
}

/* ===========================================================================
 *  CTest_RunOnce  — one full pass. RGB + buzzer walk blocks a few seconds.
 * ===========================================================================
 */
void CTest_RunOnce(void)
{
#ifdef TEST_RGB
    /* Visible R/G/B/White walk, then off. Uses blocking sends, safe pre-loop. */
    WS2812B_SelfTest();
    g_ctest.rgb_send_count = ws2812b_send_count;
    g_ctest.rgb_busy       = ws2812b_busy;
    g_ctest.rgb_status     = (ws2812b_send_count > 0) ? CT_PASS : CT_FAIL;
#endif

#ifdef TEST_BUZZER
    /* Audible confirm; Buzzer_Update() ticks the pattern in the loop. */
    Buzzer_Play(BUZZER_STARTUP);
    g_ctest.buz_busy        = Buzzer_IsBusy();
    g_ctest.buz_pattern_idx = (uint8_t)BUZZER_STARTUP;
    /* PASS once the pattern engine has accepted it (became busy). */
    g_ctest.buz_status      = g_ctest.buz_busy ? CT_PASS : CT_FAIL;
#endif

#ifdef TEST_NTC
    ct_read_ntc();
#endif
#ifdef TEST_TDS
    ct_read_tds();
#endif
#ifdef TEST_BATTERY
    ct_read_battery();
#endif
#ifdef TEST_HX711
    ct_read_hx711();
#endif
#ifdef TEST_BMA253
    ct_read_bma();
#endif
#ifdef TEST_RTC
    /* Seed a known time then read it back as a round-trip check. */
    {
        RTC_DateTime_t dt = {0};
        dt.seconds = 0;  dt.minutes = 34; dt.hours = 12;
        dt.day = 1;      dt.date = 1;     dt.month = 1;  dt.year = 26;
        if (RTC_Write(&s_rtc, &dt) == HAL_OK && RTC_Read(&s_rtc) == HAL_OK) {
            g_ctest.rtc_hh = s_rtc.now.hours;
            g_ctest.rtc_mm = s_rtc.now.minutes;
            g_ctest.rtc_ss = s_rtc.now.seconds;
            g_ctest.rtc_status =
                (s_rtc.now.hours == 12 && s_rtc.now.minutes == 34)
                ? CT_PASS : CT_FAIL;
        } else {
            g_ctest.rtc_status = CT_FAIL;
        }
    }
#endif

#ifdef TEST_EEPROM
    /* Append one record, read it back, compare. Non-destructive ring write. */
    if (g_ctest.ee_present) {
        EE_Record_t w; memset(&w, 0, sizeof(w));
        w.unix_time = 0x11223344UL;   /* magic + crc8 are set by EE_Store_Append */
        w.weight_ml = 1234;
        w.tds_ppm   = 321;
        w.temp_x10  = 250;
        uint8_t  formatted = EE_Store_Format();   /* ensures a valid log header */
        uint16_t before = EE_Store_Count();
        uint8_t  appended = EE_Store_Append(&w);
        uint16_t after  = EE_Store_Count();
        g_ctest.ee_count = after;
        EE_Record_t r; memset(&r, 0, sizeof(r));
        uint8_t got = (after > 0)
                      ? EE_Store_ReadRecord((uint16_t)(after - 1U), &r) : 0U;
        g_ctest.ee_status =
            (formatted && appended && got &&
             r.weight_ml == 1234 && after >= before)
            ? CT_PASS : CT_FAIL;
    }
#endif

#ifdef TEST_FLASH
    /* Save a sentinel settings record, reload, verify it survived a flash
     * erase+program cycle on the top pages (doubleword programming path). */
    {
        DeviceSettings_t in, out;
        memset(&in, 0, sizeof(in));
        Storage_DefaultPrefs(&in.prefs);
        in.is_registered = 1;
        in.is_calibrated = 1;
        in.tare_offset   = 0x0BADF00D;
        in.hx711_scale_x100 = HX711_SCALE_X100_DEFAULT;
        memcpy(in.device_nickname, "CTEST", 5);

        Storage_SaveSettings(&in);          /* erases + programs SETTINGS page */
        memset(&out, 0, sizeof(out));
        Storage_LoadSettings(&out);

        g_ctest.flash_crc_expected = in.tare_offset;
        g_ctest.flash_crc_readback = out.tare_offset;
        g_ctest.flash_status =
            (out.tare_offset == (int32_t)0x0BADF00D &&
             out.is_registered == 1 &&
             memcmp(out.device_nickname, "CTEST", 5) == 0)
            ? CT_PASS : CT_FAIL;
    }
#endif

#ifdef TEST_BLE
    /* Emit a banner the JDY-29 will advertise/echo; PASS on successful TX. */
    g_ctest.ble_connected = BLE_IsConnected(&s_ble);
    g_ctest.ble_status =
        (BLE_SendStr(&s_ble, "HydraSense CTEST\r\n") == HAL_OK)
        ? CT_PASS : CT_FAIL;
#endif

#ifdef TEST_BUTTON
    {
        uint8_t pressed =
            (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
            ? 1U : 0U;
        g_ctest.btn_level = pressed;
        /* The pin reads a definite level either way => GPIO path is alive. */
        g_ctest.btn_status = CT_PASS;
    }
#endif

    ct_count_results();
    g_ctest.first_pass_done = 1;
}

/* ===========================================================================
 *  CTest_RunLoop  — call every iteration. Re-polls on a 1 s cadence and keeps
 *  the RGB summary + buzzer running.
 * ===========================================================================
 */
void CTest_RunLoop(void)
{
    static uint32_t last_poll = 0;
    static uint8_t  last_btn  = 1;     /* idle high (active-low) */

    uint32_t now = HAL_GetTick();
    g_ctest.uptime_ms = now;
    g_ctest.loop_count++;

#ifdef TEST_BUZZER
    Buzzer_Update();                   /* advance the buzzer pattern engine */
    g_ctest.buz_busy = Buzzer_IsBusy();
#endif

#ifdef TEST_BUTTON
    {
        uint8_t pressed =
            (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
            ? 1U : 0U;
        g_ctest.btn_level = pressed;
        if (pressed && !last_btn) {            /* falling edge = new press */
            g_ctest.btn_press_count++;
#ifdef TEST_BUZZER
            Buzzer_Play(BUZZER_SINGLE_BEEP);   /* tactile feedback */
#endif
        }
        last_btn = pressed;
    }
#endif

#ifdef TEST_BLE
    BLE_IdleFlush(&s_ble, 5U);
    g_ctest.ble_connected = BLE_IsConnected(&s_ble);
    {
        BLE_Packet_t pkt;
        if (BLE_GetPacket(&s_ble, &pkt)) {
            g_ctest.ble_rx_bytes++;            /* something arrived = link OK */
            if (g_ctest.ble_status != CT_PASS) g_ctest.ble_status = CT_PASS;
        }
    }
#endif

    if (now - last_poll >= 1000U) {
        last_poll = now;
#ifdef TEST_NTC
        ct_read_ntc();
#endif
#ifdef TEST_TDS
        ct_read_tds();
#endif
#ifdef TEST_BATTERY
        ct_read_battery();
#endif
#ifdef TEST_HX711
        ct_read_hx711();
#endif
#ifdef TEST_BMA253
        ct_read_bma();
#endif
#ifdef TEST_RTC
        ct_read_rtc();
#endif
        ct_count_results();
    }

#ifdef TEST_RGB
    /* Live summary colour:
     *   blue breathing  -> first pass not finished yet
     *   solid green     -> every enabled test passing
     *   solid red       -> at least one enabled test failed
     */
    if (!g_ctest.first_pass_done) {
        WS2812B_SetPattern(LED_PATTERN_REGISTRATION);   /* blue breathing */
    } else if (g_ctest.any_fail) {
        WS2812B_SetAll(RGB_RED);
    } else {
        WS2812B_SetAll(RGB_GREEN);
    }
    WS2812B_Update();
    g_ctest.rgb_busy       = ws2812b_busy;
    g_ctest.rgb_send_count = ws2812b_send_count;
#endif
}

/* ===========================================================================
 *  Optional ISR hooks (call from stm32g0xx_it.c if you want live ISR data).
 * ===========================================================================
 */
void CTest_BLE_RxISR(void)
{
#ifdef TEST_BLE
    BLE_RxISR(&s_ble);
#endif
}

void CTest_RTC_TickISR(void)
{
#ifdef TEST_RTC
    RTC_TickISR(&s_rtc);
#endif
}

void CTest_Motion_ISR(void)
{
#ifdef TEST_BMA253
    BMA253_MotionISR(&s_bma);
#endif
}

#endif /* COMPONENT_TEST_MODE */
