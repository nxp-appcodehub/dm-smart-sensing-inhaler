/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file frdmmcxw7x.h
 * @brief The frdmmcxw7x.h file defines GPIO pin mappings for frdmmcxw7x board
 */

#ifndef frdmmcxw7x_H_
#define frdmmcxw7x_H_

#include "pin_mux.h"
#include "RTE_Device.h"
#include "fsl_clock.h"
#include "gpio_driver.h"

extern gpioHandleKSDK_t D10;

// I2C_S2: Driver information for alternate I2C bus on shield

#define I2C_S2_DRIVER       Driver_I2C1
#define I2C_S2_DEVICE_INDEX I2C1_INDEX
#define I2C_S2_SIGNAL_EVENT I2C1_SignalEvent_t

// SPI: Driver information default SPI brought to shield
#define SPI_S_DRIVER       Driver_SPI1
#define SPI_S_BAUDRATE     500000U ///< Transfer baudrate - 500k
#define SPI_S_DEVICE_INDEX SPI1_INDEX
#define SPI_S_SIGNAL_EVENT SPI1_SignalEvent_t

// UART: Driver information for default UART to communicate with HOST PC.
#define HOST_S_DRIVER       Driver_USART1
#define HOST_S_SIGNAL_EVENT HOST_SignalEvent_t

/* @brief  Ask use input to resume after specified samples have been processed. */
#define ASK_USER_TO_RESUME(x)                                                          \
    static volatile bool askResume   = true;                                           \
    static uint16_t samplesToProcess = x - 1;                                          \
    if (askResume && !samplesToProcess--)                                              \
    {                                                                                  \
        PRINTF("\r\n Specified samples processed, press any key to continue... \r\n"); \
        GETCHAR();                                                                     \
        askResume = false;                                                             \
    }

/* Compatability definitions for frdmmcxw7x */
#define I2C1           LPI2C1
#define I2C_Type       LPI2C_Type
#define I2C_BASE_PTRS  LPI2C_BASE_PTRS
#define SPI1           LPSPI1
#define SPI_Type       LPSPI_Type
#define SPI_BASE_PTRS  LPSPI_BASE_PTRS
#define UART1          LPUART1
#define UART_Type      LPUART_Type
#define UART_BASE_PTRS LPUART_BASE_PTRS

/* @brief dummy arguement to Power Mode Wait Wrapper. */
#define SMC NULL

/* @brief Kinetis style Wrapper API for Power Mode Wait (Wait for Interrupt). */
status_t SMC_SetPowerModeWait(void *arg);
/* @brief Kinetis style Wrapper API for Power Mode VLPR (Wait for Interrupt). */
status_t APP_LowPower_EnterLowPower(void *arg);

#endif /* frdmmcxw7x_H_ */
