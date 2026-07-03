/**
  ******************************************************************************
  * @file    system_stm32g0xx.c
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File
  *          for the HydraSense Smart Unit (STM32G070KBT6).
  *
  *  Standard CMSIS system file. SystemInit() runs from the reset handler before
  *  main(). The real clock tree (HSI16 + PLL -> 64 MHz) is configured later in
  *  SystemClock_Config() inside main.c, so SystemInit() only sets the vector
  *  table location. SystemCoreClock starts at the HSI default and is refreshed
  *  by SystemCoreClockUpdate() (call it after changing the clock if any HAL
  *  code relies on SystemCoreClock).
  ******************************************************************************
  */

#include "stm32g0xx.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE     8000000U
#endif
#if !defined  (HSI_VALUE)
  #define HSI_VALUE     16000000U
#endif
#if !defined  (LSI_VALUE)
  #define LSI_VALUE     32000U
#endif
#if !defined  (LSE_VALUE)
  #define LSE_VALUE     32768U
#endif

/* Vector table base offset. Keep at 0 (start of flash) for HydraSense. */
#define VECT_TAB_OFFSET  0x00000000U

/* SystemCoreClock holds the current core clock in Hz. Default = HSI16. */
uint32_t SystemCoreClock = 16000000U;

const uint32_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint32_t APBPrescTable[8]  = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
  * @brief  Setup the microcontroller system.
  */
void SystemInit(void)
{
  /* Configure the Vector Table location. */
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif
  /* Reset of the clock tree is handled by SystemClock_Config() in main.c. */
}

/**
  * @brief  Update SystemCoreClock from the current RCC register values.
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp;
  uint32_t pllvco;
  uint32_t pllr;
  uint32_t pllsource;
  uint32_t pllm;
  uint32_t hsidiv;

  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_0:                         /* HSE */
      SystemCoreClock = HSE_VALUE;
      break;

    case (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0):      /* LSI */
      SystemCoreClock = LSI_VALUE;
      break;

    case RCC_CFGR_SWS_1:                         /* LSE */
      SystemCoreClock = LSE_VALUE;
      break;

    case (RCC_CFGR_SWS_2):                       /* PLL */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

      if (pllsource == 0x03U)                    /* HSE source */
      {
        pllvco = (HSE_VALUE / pllm);
      }
      else                                       /* HSI16 source */
      {
        pllvco = (HSI_VALUE / pllm);
      }
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U);
      SystemCoreClock = pllvco / pllr;
      break;

    case 0x00000000U:                            /* HSI */
    default:
      hsidiv = (1UL << ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos));
      SystemCoreClock = (HSI_VALUE / hsidiv);
      break;
  }

  /* Apply AHB prescaler to obtain HCLK. */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
  SystemCoreClock >>= tmp;
}
