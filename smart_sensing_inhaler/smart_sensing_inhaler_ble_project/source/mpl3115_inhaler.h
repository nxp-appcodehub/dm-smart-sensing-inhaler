/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file fxls89xx_motion_wakeup.h
 * @brief The fxls89xx_motion_wakeup.h file details FXLS89xx accelerometer SDCD configuration
 *        to configure sensor for detecting motion tampering and raise an alert.
 */

#ifndef MPL3115_INHALER_H_
#define MPL3115_INHALER_H_

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "mpl3115_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define BOARD_LED_GPIO          GPIOA
#define BOARD_LED_GPIO_PIN      20U
#define MPL3115_DATA_SIZE (5) /* 3 byte Pressure/Altitude and 2 byte Temperature. */
/*! In MPL3115 the Auto Acquisition Time Step (ODR) can be set only in powers of 2 (i.e. 2^x, where x is the
 *  SAMPLING_EXPONENT).
 *  This gives a range of 1 second to 2^15 seconds (9 hours). */
#define MPL3115_SAMPLING_EXPONENT (2) /* 2 seconds */
/*! @brief Avg number of samples to compute baseline pressure value. */
#define NUM_AVG_SAMPLES 5
//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cMpl3115ConfigNormal[] = {
    /* Enable Data Ready and Event flags for Pressure, Temperature or either. */
    {MPL3115_PT_DATA_CFG,
     MPL3115_PT_DATA_CFG_TDEFE_ENABLED | MPL3115_PT_DATA_CFG_PDEFE_ENABLED | MPL3115_PT_DATA_CFG_DREM_ENABLED,
     MPL3115_PT_DATA_CFG_TDEFE_MASK | MPL3115_PT_DATA_CFG_PDEFE_MASK | MPL3115_PT_DATA_CFG_DREM_MASK},
    /* Set Over Sampling Ratio to 128. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_OS_OSR_128, MPL3115_CTRL_REG1_OS_MASK},
    /* Set Auto acquisition time step. */
    {MPL3115_CTRL_REG2, MPL3115_SAMPLING_EXPONENT, MPL3115_CTRL_REG2_ST_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of Status Register. */
const registerreadlist_t cMpl3115Status[] = {{.readFrom = MPL3115_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure+Temperature Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {{.readFrom = MPL3115_OUT_P_MSB, .numBytes = MPL3115_DATA_SIZE},
                                                   __END_READ_DATA__};


//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER;
    mpl3115_i2c_sensorhandle_t mpl3115Driver;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------






#endif /* MPL3115_INHALER_H_ */
