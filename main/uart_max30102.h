/**
 * @file uart.h
 * 
 * @author
 * Nicola Ferrante 
 * email: nicolaferrante20@gmail.com
 * 
 *
 * @brief 
 * UART asynchronous implementations, that uses separate RX and TX tasks
*/

#ifndef UART_H
#define UART_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/**
 * Default parameters for initialization.
 */
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)


/**
 * @brief Sets the number of pulses which the spo2 values are reset.
 * 
 * @details This is set to a default (recommended) value in the initializer.
 * The only way to change is by calling this function.
 * 
 * @param this is the address of the configuration structure.
 * @param reset_spo2_pulse_n is the number of pulses.
 * 
 * @returns status of execution.
 */
void init_uart(void);

/**
 * @brief Data to be sent via UART.
 * 
 * 
 * @param logName is the address of the configuration structure.
 * @param data is the number of pulses.
 * 
 */
int sendData(const char* logName, const char* data);

/**
 * @brief Task used for receive the data
 * 
 */
static void rx_task(void *arg);

/**
 * @brief Tasks used for send the data
 * .
 */
static void tx_task(void *arg);


#endif
