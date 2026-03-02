/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 by Alexander Polakov
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* Power management for Shanling M0 Pro using AXP2101 PMIC.
 * Based on Eros Q v4 code path in power-erosqnative.c. */

#include "power.h"
#include "adc.h"
#include "audiohw.h"
#include "system.h"
#include "kernel.h"
#ifdef HAVE_USB_CHARGING_ENABLE
# include "usb_core.h"
#endif
#include "axp-pmu.h"
#include "axp-2101.h"
#include "i2c-x1000.h"
#include "gpio-x1000.h"
#include "x1000/cpm.h"

/* Magic value written to CPM scratch pad to request bootloader recovery mode.
 * CPM scratch survives WDT reset; SPL saves it before zeroing. */
#define RECOVERY_MAGIC 0x524543 /* "REC" */

unsigned short battery_level_disksafe = 3470;

/* The OF shuts down at this voltage */
unsigned short battery_level_shutoff = 3400;

/* voltages (millivolt) of 0%, 10%, ... 100% when charging disabled.
 * Derived from a full battery_bench run (battery_bench.txt), reading the
 * voltage at the first sample where the AXP2101 fuel gauge reported each
 * 10% boundary. Shutoff at 8%/3400mV, so 0% extrapolated = 3400. */
unsigned short percent_to_volt_discharge[11] =
{
    3400, 3446, 3648, 3701, 3722, 3750, 3796, 3857, 3923, 4018, 4120
};

/* voltages (millivolt) of 0%, 10%, ... 100% when charging enabled.
 * No charge bench available; estimated ~60-100 mV above discharge at same SoC,
 * increasing toward full as charge current tapers. */
unsigned short percent_to_volt_charge[11] =
{
    3400, 3510, 3710, 3760, 3785, 3815, 3865, 3940, 4020, 4130, 4200
};

void power_init(void)
{
    i2c_x1000_set_freq(AXP_PMU_BUS, I2C_FREQ_400K);

    axp2101_init();

    /* Enable required ADCs */
    axp2101_adc_set_enabled(
        (1 << AXP2101_ADC_VBAT_VOLTAGE) |
        (1 << AXP2101_ADC_VBUS_VOLTAGE) |
        (1 << AXP2101_ADC_VSYS_VOLTAGE) |
        (1 << AXP2101_ADC_TS_VOLTAGE) |
        (1 << AXP2101_ADC_DIE_TEMPERATURE));

    /* Enable power outputs — DCDC1 and DCDC3 only (bits 0,2) */
    i2c_reg_modify1(AXP_PMU_BUS, AXP_PMU_ADDR,
                    AXP2101_REG_DCDC_ONOFF, 0, 0x05, NULL);
    /* LDO_ONOFF0 bit 3 = ALDO4 — powers jack detect circuit, keep enabled */
    i2c_reg_modify1(AXP_PMU_BUS, AXP_PMU_ADDR,
                    AXP2101_REG_LDO_ONOFF0, 0, 0x08, NULL);
    i2c_reg_modify1(AXP_PMU_BUS, AXP_PMU_ADDR,
                    AXP2101_REG_LDO_ONOFF1, 0, 0x01, NULL);

    /* Set power button delay to 1s */
    uint8_t regread = i2c_reg_read1(AXP_PMU_BUS, AXP_PMU_ADDR,
                    AXP2101_REG_LOGICTHRESH);
    if ((regread & 0x03) != 0x02) {
        i2c_reg_modify1(AXP_PMU_BUS, AXP_PMU_ADDR,
                        AXP2101_REG_LOGICTHRESH, 0x03, 0, NULL);
        i2c_reg_modify1(AXP_PMU_BUS, AXP_PMU_ADDR,
                        AXP2101_REG_LOGICTHRESH, 0, 0x02, NULL);
    }

    /* Set rail voltages from stock firmware configuration.
     * DCDC2/4/5 are left at defaults. */
    axp2101_supply_set_voltage(AXP2101_SUPPLY_DCDC1, 3300);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_DCDC3, 1200);

    axp2101_supply_set_voltage(AXP2101_SUPPLY_ALDO1, 3300);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_ALDO2, 2500);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_ALDO3, 500);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_ALDO4, 3300);

    axp2101_supply_set_voltage(AXP2101_SUPPLY_BLDO1, 500);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_BLDO2, 500);

    axp2101_supply_set_voltage(AXP2101_SUPPLY_DLDO1, 3300);
    axp2101_supply_set_voltage(AXP2101_SUPPLY_DLDO2, 500);

    axp2101_supply_set_voltage(AXP2101_SUPPLY_VCPUS, 500);
}

#ifdef HAVE_USB_CHARGING_ENABLE
void usb_charging_maxcurrent_change(int maxcurrent)
{
    axp2101_set_charge_current(maxcurrent);
}
#endif

void adc_init(void)
{
}

void power_off(void)
{
    axp2101_power_off();
    while(1);
}

void reboot_to_recovery(void)
{
    audiohw_close();

    /* Write magic to CPM scratch pad — SPL saves this before zeroing */
    REG_CPM_SCRATCH_PROT = 0x5a5a;
    REG_CPM_SCRATCH = RECOVERY_MAGIC;
    REG_CPM_SCRATCH_PROT = 0xa5a5;

    system_reboot();
    while(1);
}

bool charging_state(void)
{
    return axp2101_battery_status() == AXP2101_BATT_CHARGING;
}

int _battery_level(void)
{
    return axp2101_egauge_read();
}

int _battery_voltage(void)
{
    return axp2101_adc_read(AXP2101_ADC_VBAT_VOLTAGE);
}
