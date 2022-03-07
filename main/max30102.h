/**
 * @file max30102.h
 * 
 * @author
 * Nicola Ferrante
 * email: nicolaferrante20@gmail.com
 * 
 *
 * @brief 
 * This is a library for interfacing the MAX30102 Heart Rate/Pulse Oxymetry sensor 
 * with the ESP32 processor. This part contains 
 *  - Initialization parameters and functions,
 *  - Processing and update functions which then read the sensor.
*/
#ifndef MAX30102_H
#define MAX30102_H

#include <math.h>
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#define MAX30102_I2C_ADDR 0x57
#define MAX30102_I2C_TIMEOUT 1000

#define MAX30102_BYTES_PER_SAMPLE 6
#define MAX30102_SAMPLE_LEN_MAX 32

#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

#define MAX30102_FIFO_WR_PTR    0x04
#define MAX30102_OVF_COUNTER    0x05
#define MAX30102_FIFO_RD_PTR    0x06
#define FIFO_A_FULL_EN		    0x80
#define PRG_RDY_EN			    0x40
#define ALC_OVF_EN			    0x20
#define PROX_INT_EN			    0x10
#define MAX30102_FIFO_DATA      0x07
#define ACK_CHECK_EN            0x1              // I2C master will check ack from slave
#define ACK_CHECK_DIS           0x0              // I2C master will not check ack from slave
#define ACK_VAL                 0x0              // I2C ack value
#define NACK_VAL                0x1              // I2C nack value
#define SPO2_RES 				16				 //SPO2 ADC resolution 18,17,16,15 bits

#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_FIFO_CONFIG_SMP_AVE 5
#define MAX30102_FIFO_CONFIG_ROLL_OVER_EN 4
#define MAX30102_FIFO_CONFIG_FIFO_A_FULL 0

#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_MODE_SHDN 7
#define MAX30102_MODE_RESET 6
#define MAX30102_MODE_MODE 0

#define MAX30102_SPO2_CONFIG 0x0a
#define MAX30102_SPO2_ADC_RGE 5
#define MAX30102_SPO2_SR 2
#define MAX30102_SPO2_LEW_PW 0

#define MAX30102_LED_IR_PA1 0x0c
#define MAX30102_LED_RED_PA2 0x0d
#define REG_PROX_INT_THRESH 0x30
#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

#define MAX30102_DIE_TINT 0x1f
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1
#define FIFO_A_FULL 			30				 //Options: 17 - 32			default:17
/**
 * FIFO Configuration structure for 30102
 * Almost Full Value
 */

typedef enum AlmostFull {
    MAX30102_ALMOST_FULL_0             = 0x00,
    MAX30102_ALMOST_FULL_1             = 0x01,
    MAX30102_ALMOST_FULL_2             = 0x02,
    MAX30102_ALMOST_FULL_3             = 0x03,
    MAX30102_ALMOST_FULL_4             = 0x04,
    MAX30102_ALMOST_FULL_5             = 0x05,
    MAX30102_ALMOST_FULL_6             = 0x06,
    MAX30102_ALMOST_FULL_7             = 0x07,
    MAX30102_ALMOST_FULL_8             = 0x08,
    MAX30102_ALMOST_FULL_9             = 0x09,
    MAX30102_ALMOST_FULL_10            = 0x0A,
    MAX30102_ALMOST_FULL_11            = 0x0B,
    MAX30102_ALMOST_FULL_12            = 0x0C,
    MAX30102_ALMOST_FULL_13            = 0x0D,
    MAX30102_ALMOST_FULL_14            = 0x0E,
    MAX30102_ALMOST_FULL_15            = 0x0F
}max30102_almost_full_t;
typedef enum ADCRange{
    MAX30102_ADC_RANGE_2048               = 0x00,
    MAX30102_ADC_RANGE_4096               = 0x01,
    MAX30102_ADC_RANGE_8192               = 0x02,
    MAX30102_ADC_RANGE_16384              = 0x03  
}max30102_adc_range_t;
/**
 * Sampling rate enum.
 * Internal sampling rates from 50Hz up to 1KHz.
 */
typedef enum SamplingRate {
    MAX30102_SAMPLING_RATE_50HZ           = 0x00,
    MAX30102_SAMPLING_RATE_100HZ          = 0x01,
    MAX30102_SAMPLING_RATE_200HZ          = 0x02,
    MAX30102_SAMPLING_RATE_400HZ          = 0x03,
    MAX30102_SAMPLING_RATE_800HZ          = 0x04,
    MAX30102_SAMPLING_RATE_1000HZ          = 0x05,
    MAX30102_SAMPLING_RATE_1600HZ          = 0x06,
    MAX30102_SAMPLING_RATE_3200HZ          = 0x07
} max30102_sampling_rate_t;

typedef enum LEDCurrent {
    MAX30102_LED_CURRENT_0MA              = 0x00,
    MAX30102_LED_CURRENT_4_4MA            = 0x16,
    MAX30102_LED_CURRENT_7_6MA            = 0x26,
    MAX30102_LED_CURRENT_11MA             = 0x37,
    MAX30102_LED_CURRENT_14_2MA           = 0x47,
    MAX30102_LED_CURRENT_17_4MA           = 0x57,
    MAX30102_LED_CURRENT_20_8MA           = 0x68,
    MAX30102_LED_CURRENT_24MA             = 0x78,
    MAX30102_LED_CURRENT_27_1MA           = 0x88,
    MAX30102_LED_CURRENT_30_6MA           = 0x99,
    MAX30102_LED_CURRENT_33_8MA           = 0xA9,
    MAX30102_LED_CURRENT_37MA             = 0xB9,
    MAX30102_LED_CURRENT_40_2MA           = 0xC9,
    MAX30102_LED_CURRENT_43_6MA           = 0xDA,
    MAX30102_LED_CURRENT_46_8MA           = 0xEA,
    MAX30102_LED_CURRENT_50MA             = 0xFA,
    MAX30102_LED_CURRENT_51MA             = 0xFF
} max30102_current_t;
typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32,
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

typedef enum max30102_multi_led_ctrl_t
{
    max30102_led_off,
    max30102_led_red,
    max30102_led_ir
} max30102_multi_led_ctrl_t;

typedef struct _max30102_fifo_t {
    uint16_t raw_ir;
    uint16_t raw_red;
} max30102_fifo_t;

typedef struct max30102_t
{
    i2c_port_t i2c_num;
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];
    uint8_t _interrupt_flag;
} max30102_t;

static QueueHandle_t msg_queue;
static const int msg_queue_len = 5;     // Size of msg_queue
typedef struct Message {
  int  bpm;
  int SpO2;
} Message;
Message msg;
void sensor_task_manager(void* arg);
void i2c_task_0(void* arg);
void check_ret(esp_err_t ret,uint8_t sensor_data_h);
void idle_task_0(void* arg);
void i2c_task_1(void* arg);


/**
 * @brief MAX30102 initiation function.
 *
 * @param this Pointer to max30102_t object instance.
 * @param i2cnum Pointer to I2C object handle
 */
esp_err_t max30102_init(max30102_t *this,i2c_port_t i2c_num);
esp_err_t max30102_write_register(max30102_t* this, uint8_t address, uint8_t val);
esp_err_t max30102_read_register(max30102_t* this, uint8_t address, uint8_t* reg);
void print_array(uint8_t *array,uint16_t size);

/**
 * @brief Read FIFO content and store to buffer in max30102_t object instance.
 *
 * @param sensorDataRED 
 * @param sensorDataIR 
 */
esp_err_t max30102_read_fifo(i2c_port_t i2c_num, uint16_t sensorDataRED[],uint16_t sensorDataIR[]);


/**
 * @brief Sets the sample averaging.
 * 
 * @details This is automatically called by the initializer function.
 * 
 * @param this is the address of the configuration structure.
 * @param sample_averaging is the sample averaging desired.
 * 
 * @returns status of execution.
 * */
esp_err_t max30102_set_sample_averaging(max30102_t* this,  max30102_smp_ave_t sample_averaging);
/**
 * @brief Sets the rollever .
 * 
 * @details This is automatically called by the initializer function.
 * 
 * @param this is the address of the configuration structure.
 * @param enabled is for enable the rollover.
 * 
 * @returns status of execution.
 * */
esp_err_t max30102_set_roll_over(max30102_t* this, bool enabled);
/**
 * @brief Sets the number of samples left to trigger almost full
 * 
 * @details This is automatically called by the initializer function.
 * 
 * @param this is the address of the configuration structure.
 * @param almost_full is the desired sample averaging.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_set_almost_full(max30102_t* this, max30102_almost_full_t almost_full);
/**
 * @brief Sets the led currents.
 * 
 * @details This is automatically called by the initializer function
 * or automatically when needed by the library.
 * 
 * @param this is the address of the configuration structure.
 * @param red_current is the current value of the Red Led.
 * @param ir_current is the current value of the IR Led.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_set_led_current( max30102_t* this,  max30102_current_t red_current,  max30102_current_t ir_current );
/**
 * @brief Sets the resolution. High or standard.
 * 
 * @details This is automatically called by the initializer function.
 * 
 * @param this is the address of the configuration structure.
 * @param enable is if high resolution will be enabled.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_set_high_res(max30102_t* this, max30102_adc_range_t adc_range);

/**
 * @brief Set the internal sampling rate.
 * 
 * @details This is automatically called by the initializer function.
 * 
 * @param this is the address of the configuration structure.
 * @param rate is the sampling rate desired.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_set_sampling_rate( max30102_t* this,  max30102_sampling_rate_t rate );

esp_err_t max30102_print_registers(max30102_t* this);
#endif


/**
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/