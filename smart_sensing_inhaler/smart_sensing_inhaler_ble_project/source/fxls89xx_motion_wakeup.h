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

#ifndef FXLS89XX_MOTION_WAKEUP_H_
#define FXLS89XX_MOTION_WAKEUP_H_

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "fxls8974_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"
#include "fxls89xx_motion_wakeup.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define FXLS8974_DATA_SIZE      6
#define FXLS8974_STANDBY_MODE   0
#define FXLS8974_ACTIVE_MODE    1
#define BOARD_LED_GPIO          GPIOA
#define BOARD_LED_GPIO_PIN      20U
//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for configuring SDCD-OT for tap-detection and Auto-Wake/Sleep in interrupt mode. */
const registerwritelist_t cFxls8974AwsConfig[] = {
    /* Set Full-scale range as 4G. */
    {FXLS8974_SENS_CONFIG1, FXLS8974_SENS_CONFIG1_FSR_4G, FXLS8974_SENS_CONFIG1_FSR_MASK},
    /* Set Wake ODR as 400Hz & Sleep Mode ODR as 6.25Hz. */
    {FXLS8974_SENS_CONFIG3, FXLS8974_SENS_CONFIG3_WAKE_ODR_400HZ | FXLS8974_SENS_CONFIG3_SLEEP_ODR_6_25HZ, FXLS8974_SENS_CONFIG3_WAKE_ODR_MASK | FXLS8974_SENS_CONFIG3_SLEEP_ODR_MASK},
    /* Enable SDCD OT for all 3 axes X, Y & Z and within-thresholds event latch disabled. */
    {FXLS8974_SDCD_CONFIG1, FXLS8974_SDCD_CONFIG1_X_OT_EN_EN | FXLS8974_SDCD_CONFIG1_Y_OT_EN_EN | FXLS8974_SDCD_CONFIG1_Z_OT_EN_EN | FXLS8974_SDCD_CONFIG1_OT_ELE_DIS,
    		FXLS8974_SDCD_CONFIG1_X_OT_EN_MASK | FXLS8974_SDCD_CONFIG1_Y_OT_EN_MASK | FXLS8974_SDCD_CONFIG1_Z_OT_EN_MASK | FXLS8974_SDCD_CONFIG1_OT_ELE_MASK},
    /* Enabling SDCD and Relative Data (N) � Data (N-1) mode for transient detection */
    {FXLS8974_SDCD_CONFIG2, FXLS8974_SDCD_CONFIG2_SDCD_EN_EN | FXLS8974_SDCD_CONFIG2_REF_UPDM_SDCD_REF, FXLS8974_SDCD_CONFIG2_SDCD_EN_MASK | FXLS8974_SDCD_CONFIG2_REF_UPDM_MASK},
    /* Set the SDCD_OT debounce count to 0 */
    {FXLS8974_SDCD_OT_DBCNT, 0, 0},
    /* Set the SDCD lower and upper thresholds to +/-100mg*/
    {FXLS8974_SDCD_LTHS_LSB, 0xCC, 0},
    {FXLS8974_SDCD_LTHS_MSB, 0xFF, 0},
    {FXLS8974_SDCD_UTHS_LSB, 0x34, 0},
    {FXLS8974_SDCD_UTHS_MSB, 0x00, 0},
    /* Enable SDCD outside of thresholds event Auto-WAKE/SLEEP transition source enable. */
    {FXLS8974_SENS_CONFIG4, FXLS8974_SENS_CONFIG4_WK_SDCD_OT_EN | FXLS8974_SENS_CONFIG4_INT_POL_ACT_HIGH, FXLS8974_SENS_CONFIG4_WK_SDCD_OT_MASK | FXLS8974_SENS_CONFIG4_INT_POL_MASK},
    /* Set the ASLP count to 5sec */
    {FXLS8974_ASLP_COUNT_LSB, 0xD0, 0},
    {FXLS8974_ASLP_COUNT_MSB, 0x07, 0},
    /* Enable Interrupts for WAKE mode. */
    {FXLS8974_INT_EN, FXLS8974_INT_EN_WAKE_OUT_EN_EN, FXLS8974_INT_EN_WAKE_OUT_EN_MASK},
    {FXLS8974_INT_PIN_SEL, FXLS8974_INT_PIN_SEL_WK_OUT_INT2_DIS, FXLS8974_INT_PIN_SEL_WK_OUT_INT2_MASK},
    __END_WRITE_DATA__};

const registerwritelist_t cFxls8974ConfigNormal[] = {
    /* Set Full-scale range as 2G. */
    {FXLS8974_SENS_CONFIG1, FXLS8974_SENS_CONFIG1_FSR_2G, FXLS8974_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 6.25Hz. */
    {FXLS8974_SENS_CONFIG3, FXLS8974_SENS_CONFIG3_WAKE_ODR_6_25HZ, FXLS8974_SENS_CONFIG3_WAKE_ODR_MASK},
    __END_WRITE_DATA__};

/*! @brief Read register list to read SysMode Register. */
const registerreadlist_t cFxls8974ReadSysMode[] = {{.readFrom = FXLS8974_SYS_MODE, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Read register list to read INT_STATUS Register. */
const registerreadlist_t cFxls8974ReadIntStatus[] = {{.readFrom = FXLS8974_INT_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief FXLS8974 Interrupt Status Register. */
const registerreadlist_t cFxls8974IntEn[] = {{.readFrom = FXLS8974_INT_STATUS, .numBytes = 1},
                                                    __END_READ_DATA__};

/*! @brief Address of DATA Ready Status Register. */
const registerreadlist_t cFxls8974DRDYEvent[] = {{.readFrom = FXLS8974_INT_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of Raw Accel Data in Normal Mode. */
const registerreadlist_t cFxls8974OutputNormal[] = {{.readFrom = FXLS8974_OUT_X_LSB, .numBytes = FXLS8974_DATA_SIZE},
                                                    __END_READ_DATA__};


//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

    fxls8974_i2c_sensorhandle_t fxls8974Driver;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------






#endif /* FXLS89XX_MOTION_WAKEUP_H_ */
