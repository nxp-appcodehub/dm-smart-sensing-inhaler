/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file fxls8974cf_poll.c
 * @brief The fxls8974cf_poll.c file implements the ISSDK FXLS8974 sensor driver
 *        example demonstration with polling mode.
 */

/*  SDK Includes */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "mpl3115_drv.h"
#include "systick_utils.h"
#include "fxls8974_drv.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define BOARD_LED_GPIO          GPIOA
#define BOARD_LED_GPIO_PIN      20U
#define MPL3115_DATA_SIZE (5) /* 3 byte Pressure/Altitude and 2 byte Temperature. */
#define FXLS8974_DATA_SIZE      6
/*! In MPL3115 the Auto Acquisition Time Step (ODR) can be set only in powers of 2 (i.e. 2^x, where x is the
 *  SAMPLING_EXPONENT).
 *  This gives a range of 1 second to 2^15 seconds (9 hours). */
#define MPL3115_SAMPLING_EXPONENT (2) /* 2 seconds */
/*! @brief Avg number of samples to compute baseline pressure value. */
#define NUM_AVG_SAMPLES 5

/*! @brief Total number of doses prescribed for patient. */
#define TOTAL_DOSES 255
/*! @brief Inhalation threshold in Pa*/
#define INHALE_THS 1000 /* Pa - Configurable*/

#define RESET_INHALER(x)                                                               \
        inhalation_count = 0;                                                          \
        start_inhalation = 1;                                                          \
        meds_inhaled = 0;                                                              \
        reset_inhaler = 0;                                                             \
        all_meds_inhaled = 0;                                                          \
        if (rawAccel.accel[0] > 300)                                                   \
        {                                                                              \
        	wrong_posture_movedown = 1;                                                \
        	wrong_posture_moveup = 0;                                                  \
        	correct_posture = 0;                                                       \
        	reset_inhaler = 1;                                                         \
        }                                                                              \
        else if (rawAccel.accel[0] < -300)                                             \
        {                                                                              \
        	wrong_posture_movedown = 0;                                                \
        	wrong_posture_moveup = 1;                                                  \
        	correct_posture = 0;                                                       \
        	reset_inhaler = 1;                                                         \
        }                                                                              \
		else                                                                           \
        {                                                                              \
        	wrong_posture_movedown = 0;                                                \
        	wrong_posture_moveup = 0;                                                  \
        	correct_posture = 1;                                                       \
        }                                                                              \

/*******************************************************************************
 * Constants
 ******************************************************************************/
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

const registerwritelist_t cFxls8974ConfigNormal[] = {
    /* Set Full-scale range as 2G. */
    {FXLS8974_SENS_CONFIG1, FXLS8974_SENS_CONFIG1_FSR_2G, FXLS8974_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 6.25Hz. */
    {FXLS8974_SENS_CONFIG3, FXLS8974_SENS_CONFIG3_WAKE_ODR_6_25HZ, FXLS8974_SENS_CONFIG3_WAKE_ODR_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of Status Register. */
const registerreadlist_t cMpl3115Status[] = {{.readFrom = MPL3115_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure+Temperature Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {{.readFrom = MPL3115_OUT_P_MSB, .numBytes = MPL3115_DATA_SIZE},
                                                   __END_READ_DATA__};

/*! @brief Address of DATA Ready Status Register. */
const registerreadlist_t cFxls8974DRDYEvent[] = {{.readFrom = FXLS8974_INT_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of Raw Accel Data in Normal Mode. */
const registerreadlist_t cFxls8974OutputNormal[] = {{.readFrom = FXLS8974_OUT_X_LSB, .numBytes = FXLS8974_DATA_SIZE},
                                                    __END_READ_DATA__};


uint8_t inhalation_count = 0;
uint8_t start_inhalation = 1;
uint8_t compute_baseline_pr = true;

uint32_t pressureInPascals;
uint8_t meds_inhaled = 0;
uint8_t total_doses = 0;
uint8_t max_inhaler_doses = TOTAL_DOSES;
uint8_t remaining_doses = TOTAL_DOSES;
uint8_t reset_inhaler = 0;
int32_t status;
uint8_t i = 0;
uint8_t dataReady;
uint8_t data[MPL3115_DATA_SIZE];
uint8_t accelData[FXLS8974_DATA_SIZE];
mpl3115_pressuredata_t rawData;
fxls8974_acceldata_t rawAccel;
uint32_t refPressure = 0;
uint32_t tempRefPressure = 0;
uint32_t pressureChange = 0;
uint32_t inhalerTHS = INHALE_THS;
uint8_t all_meds_inhaled = false;

uint8_t correct_posture = 1;
uint8_t wrong_posture_movedown = 0;
uint8_t wrong_posture_moveup = 0;

/* Create TSA table and add output variables. */
/*!
 * @brief Target Side Addressable (TSA) table created for this application.
 */
FMSTR_TSA_TABLE_BEGIN(main_table)

	FMSTR_TSA_RW_VAR(pressureInPascals, FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(pressureChange, FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(inhalerTHS, FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(refPressure, FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(total_doses, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(max_inhaler_doses, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(reset_inhaler, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(compute_baseline_pr, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(all_meds_inhaled, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(remaining_doses, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(correct_posture, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(wrong_posture_movedown, FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(wrong_posture_moveup, FMSTR_TSA_UINT8)

FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(main_table)
FMSTR_TSA_TABLE_LIST_END()

static void init_freemaster_lpuart(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t whoami;

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER;
    mpl3115_i2c_sensorhandle_t mpl3115Driver;
    fxls8974_i2c_sensorhandle_t fxls8974Driver;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /*! Initialize the MCU hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_SystickEnable();
    BOARD_InitDebugConsole();

    //PRINTF("\r\n ISSDK MPL3115/FXPQ3115 Inhaler Demo\r\n");

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_INITPINS_LED_GREEN_GPIO, BOARD_INITPINS_LED_GREEN_PIN, &led_config);
    GPIO_PinInit(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_PIN, &led_config);
	GPIO_PortSet(BOARD_INITPINS_LED_BLUE_GPIO, 1u << BOARD_INITPINS_LED_BLUE_PIN);

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        //PRINTF("\r\n I2C Initialization Failed\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        //PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        //PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return -1;
    }

    /*! FreeMASTER communication layer initialization */
    init_freemaster_lpuart();

    /*! Initialize FXLS8974 sensor driver. */
    status = FXLS8974_I2C_Initialize(&fxls8974Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXLS8974_I2C_ADDR,
                                     &whoami);
    if (SENSOR_ERROR_NONE != status)
    {

        return -1;
    }

    /*! Initialize MPL3115/FXPQ3115 sensor driver. */
    status = MPL3115_I2C_Initialize(&mpl3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MPL3115_I2C_ADDR,
                                    &whoami);
    if (SENSOR_ERROR_NONE != status)
    {
        return -1;
    }

    /*! Configure the FXLS8974 sensor. */
    status = FXLS8974_I2C_Configure(&fxls8974Driver, cFxls8974ConfigNormal);
    if (SENSOR_ERROR_NONE != status)
    {
        return -1;
    }

    /*! Configure the MPL3115/FXPQ3115 sensor. */
    status = MPL3115_I2C_Configure(&mpl3115Driver, cMpl3115ConfigNormal);
    if (SENSOR_ERROR_NONE != status)
    {
        return -1;
    }

    /*! FreeMASTER Driver Initialization */
    FMSTR_Init();

    for (;;) /* Forever loop */
    {
    	/*! FreeMASTER host communication polling mode */
    	FMSTR_Poll();

    	if (0 != remaining_doses)
    	{
			/* Get the baseline pressure reference value */
			if (true == compute_baseline_pr)
			{
				refPressure = 0;
				tempRefPressure = 0;
				while (i < NUM_AVG_SAMPLES)
				{

					GPIO_PortToggle(BOARD_INITPINS_LED_RED_GPIO, 1u << BOARD_INITPINS_LED_RED_PIN);
					GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO, 1u << BOARD_INITPINS_LED_GREEN_PIN);
	            	GPIO_PortSet(BOARD_INITPINS_LED_BLUE_GPIO, 1u << BOARD_INITPINS_LED_BLUE_PIN);

					/*! Wait for data ready from the MPL3115. */
					status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115Status, &dataReady);
					if (0 == (dataReady & MPL3115_DR_STATUS_PTDR_MASK))
					{ /* Loop, if new sample is not available. */
						continue;
					}

					/*! Read new raw sensor data from the MPL3115. */
					status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, data);
					if (ARM_DRIVER_OK != status)
					{
						return -1;
					}

					/*! Process the sample and convert the raw sensor data. */
					rawData.pressure = (uint32_t)((data[0]) << 16) | ((data[1]) << 8) | ((data[2]));
					pressureInPascals = rawData.pressure / MPL3115_PRESSURE_CONV_FACTOR;

					tempRefPressure += pressureInPascals;

					i++;
				}

				/*! Get the baselin/reference pressure value */
				refPressure = tempRefPressure/NUM_AVG_SAMPLES;
				compute_baseline_pr = false;
				inhalation_count = 0;
				i = 0;
			}
			else
			{

				/*! Wait for data ready from the FXLS8974. */
				status = FXLS8974_I2C_ReadData(&fxls8974Driver, cFxls8974DRDYEvent, &dataReady);
				if (0x80 == (dataReady & FXLS8974_INT_STATUS_SRC_DRDY_MASK))
				{
					/*! Read new raw sensor data from the FXLS8974. */
					status = FXLS8974_I2C_ReadData(&fxls8974Driver, cFxls8974OutputNormal, accelData);
					if (ARM_DRIVER_OK != status)
					{
						return -1;
					}

					/*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
					rawAccel.accel[0] = ((int16_t)accelData[1] << 8) | accelData[0];
					rawAccel.accel[1] = ((int16_t)accelData[3] << 8) | accelData[2];
					rawAccel.accel[2] = ((int16_t)accelData[5] << 8) | accelData[4];

					GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO, 1u << BOARD_INITPINS_LED_RED_PIN);
					GPIO_PortToggle(BOARD_INITPINS_LED_GREEN_GPIO, 1u << BOARD_INITPINS_LED_GREEN_PIN);
					GPIO_PortSet(BOARD_INITPINS_LED_BLUE_GPIO, 1u << BOARD_INITPINS_LED_BLUE_PIN);
				}

				if (rawAccel.accel[0] > 300)
				{
					GPIO_PortToggle(BOARD_INITPINS_LED_RED_GPIO, 1u << BOARD_INITPINS_LED_RED_PIN);
					GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO, 1u << BOARD_INITPINS_LED_GREEN_PIN);
					wrong_posture_movedown = 1;
					wrong_posture_moveup = 0;
					correct_posture = 0;
					reset_inhaler = 1;
				}
				else if (rawAccel.accel[0] < -300)
				{
					GPIO_PortToggle(BOARD_INITPINS_LED_RED_GPIO, 1u << BOARD_INITPINS_LED_RED_PIN);
					GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO, 1u << BOARD_INITPINS_LED_GREEN_PIN);
					wrong_posture_movedown = 0;
					wrong_posture_moveup = 1;
					correct_posture = 0;
					reset_inhaler = 1;
				}
				else
				{
					wrong_posture_movedown = 0;
					wrong_posture_moveup = 0;
					correct_posture = 1;
				}

				/*! Read instantaneous pressure values and compare with baseline/reference */
				/*! Wait for data ready from the MPL3115. */
				status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115Status, &dataReady);
				if (8 == (dataReady & MPL3115_DR_STATUS_PTDR_MASK))
				{ /* Loop, if new sample is not available. */

					/*! Read new raw sensor data from the MPL3115. */
					status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, data);
					if (ARM_DRIVER_OK != status)
					{
						//PRINTF("\r\n Read Failed. \r\n");
						return -1;
					}

					/*! Process the sample and convert the raw sensor data. */
					rawData.pressure = (uint32_t)((data[0]) << 16) | ((data[1]) << 8) | ((data[2]));
					pressureInPascals = rawData.pressure / MPL3115_PRESSURE_CONV_FACTOR;

					pressureChange = abs(refPressure - pressureInPascals);

					/*! Check instantaneous pressure value against reference pressure */
					if ((abs(refPressure - pressureInPascals)) > inhalerTHS)
					{

					    all_meds_inhaled = true;
						reset_inhaler = 1;
						compute_baseline_pr = true;
						remaining_doses--;
						total_doses++;

						if (remaining_doses == 0)
						{
							total_doses = total_doses;
						}
					}
					/* Reset Inhaler */
					if (reset_inhaler == 1)
					{
						RESET_INHALER();
					}

				}
			}
    	}
    }
}

/*!
 * @brief LPUART Module initialization (LPUART is a the standard block included e.g. in K66F)
 */
static void init_freemaster_lpuart(void)
{
    lpuart_config_t config;

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx     = false;
    config.enableRx     = false;

    LPUART_Init((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR, &config, BOARD_DEBUG_UART_CLK_FREQ);

    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR);

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    /* Enable UART interrupts. */
    EnableIRQ(LPUART1_IRQn);
    EnableGlobalIRQ(0);
#endif
}

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
/*
 *   Application interrupt handler of communication peripheral used in interrupt modes
 *   of FreeMASTER communication.
 *
 *   NXP MCUXpresso SDK framework defines interrupt vector table as a part of "startup_XXXXXX.x"
 *   assembler/C file. The table points to weakly defined symbols, which may be overwritten by the
 *   application specific implementation. FreeMASTER overrides the original weak definition and
 *   redirects the call to its own handler.
 *
 */

void LPUART1_IRQHandler(void)
{
    /* Call FreeMASTER Interrupt routine handler */
    FMSTR_SerialIsr();
}
#endif
