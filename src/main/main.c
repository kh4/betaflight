/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/fc_init.h"

#include "scheduler/scheduler.h"

void main_step(void)
{
    scheduler();
    processLoopback();
}

#ifndef NOMAIN
#if !defined(USE_CHIBIOS)
int main(void)
{
    init();
    while (true) {
        main_step();
    }
}
#endif

#if defined(USE_CHIBIOS)
#include "ch.h"
#include "hal.h"
#include "nvic.h"

volatile bool idleCounterClear = 0;
volatile uint32_t idleCounter = 0;

void appIdleHook(void)
{
    // Called when the scheduler has no tasks to run
    if (idleCounterClear) {
        idleCounter = 0;
        idleCounterClear = 0;
    } else {
        ++idleCounter;
    }
}

static THD_WORKING_AREA(waBetaFlightThread, 8 * 1024);
static THD_FUNCTION(BetaFlightThread, arg)
{
    (void)arg;
    chRegSetThreadName("Betaflight");
    while(1) {
        main_step();
    }
}

#if defined(USE_BRAINFPV_OSD)
#include "brainfpv_osd.h"
#include "drivers/display.h"
#include "io/displayport_max7456.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

void osdInit(displayPort_t *osdDisplayPortToUse);

extern binary_semaphore_t onScreenDisplaySemaphore;

static THD_WORKING_AREA(waOSDThread, 12 * 1024);
static THD_FUNCTION(OSDThread, arg)
{
    (void)arg;
    chRegSetThreadName("OSD");
	displayPort_t *osdDisplayPort = max7456DisplayPortInit(&masterConfig.vcdProfile, displayPortProfileMax7456());
    osdInit(osdDisplayPort);
    while (1) {
        // wait for VSYNC
        chBSemWaitTimeout(&onScreenDisplaySemaphore, MS2ST(100));
        osdMain();
    }
}
#endif

#if defined(USE_BRAINRE1_SPECTROGRAPH)
#include "spectrograph.h"
extern binary_semaphore_t spectrographDataReadySemaphore;

static THD_WORKING_AREA(waSpecThread, 1024);
static THD_FUNCTION(SpecThread, arg)
{
    (void)arg;
    chRegSetThreadName("Spectrograph");
    while (1) {
        // wait for data ready
        chBSemWait(&spectrographDataReadySemaphore);
        spectrographMain();
    }
}
#endif /* defined(USE_BRAINRE1_SPECTROGRAPH) */

uint8_t safe_boot = 0;

int main()
{
  // Check safe-boot request
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  RTC_ClearFlag(RTC_FLAG_TAMP1F);
  uint16_t rcc = RTC_ReadBackupRegister(RTC_BKP_DR3);
  if (rcc == 0xFFFF) {
    safe_boot = 1;
    RTC_WriteBackupRegister (RTC_BKP_DR3, 0);
  }

  halInit();
  chSysInit();

  init();

#if defined(USE_BRAINFPV_OSD)
  Video_Init();
#endif /* USE_BRAINFPV_OSD */

  /* re-init timer irq to make sure it works */
  extern void *isr_vector_table_base;
  NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
  st_lld_init();

  chThdCreateStatic(waBetaFlightThread, sizeof(waBetaFlightThread), HIGHPRIO, BetaFlightThread, NULL);

#if defined(USE_BRAINFPV_OSD)
  chThdCreateStatic(waOSDThread, sizeof(waOSDThread), NORMALPRIO, OSDThread, NULL);
#endif /* USE_BRAINFPV_OSD */

#if defined(USE_BRAINRE1_SPECTROGRAPH)
  if (masterConfig.bfOsdConfig.spec_enabled) {
    spectrographInit();
    chThdCreateStatic(waSpecThread, sizeof(waSpecThread), LOWPRIO, SpecThread, NULL);
  }
#endif /* USE_BRAINRE1_SPECTROGRAPH */

  // sleep forever
  chThdSleep(TIME_INFINITE);
}

#endif /* defined(USE_CHIBIOS) */
#endif /* NOMAIN */
