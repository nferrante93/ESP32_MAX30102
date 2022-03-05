/**
 * @file max30102.c
 * 
 * @author
 * Nicola Ferrante 
 * email: nicolaferrante20@gmail.com
 * 
 *
 * @brief 
 * The implemetation of the processing that
 * will read the raw sensor data from the max30102 sensor.
 * 
*/

#include <stdio.h>
#include "max30102.h"
#include <string.h>
#include "uart_max30102.h"
#define I2C_PORT I2C_NUM_0
/**
 * @brief MAX30102 initiation function.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param hi2c Pointer to I2C object handle
 */
void max30102_init(max30102_t *this, i2c_port_t i2c_num)
{
    this->i2c_num = i2c_num;
    this->_interrupt_flag = 0;
    memset(this->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(this->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
}

/**
 * @brief Write buffer of buflen bytes to a register of the MAX30102.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to write to.
 * @param buf Val containing the bytes to write.
 */
esp_err_t max30102_write_register( max30102_t* this,
                                   uint8_t address,
                                   uint8_t val      )
{
    // start transmission to device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, address, true); // send register address
    i2c_master_write_byte(cmd, val, true); // send value to write

    // end transmission
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Read buffer of buflen bytes to a register of the MAX30102.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param address Register address to read from.
 * @param reg reg containing the reg to read.
 */
esp_err_t max30102_read_register( max30102_t* this,
                                  uint8_t address,
                                  uint8_t* reg     )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);

    //i2c_master_stop(cmd);
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, reg, 1); //1 is NACK

    // end transmission
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );

    i2c_cmd_link_delete(cmd);
    return ret;
}

// Reads num bytes starting from address register on device in to _buff array
esp_err_t max30102_read_from( max30102_t* this,
                              uint8_t address,
                              uint8_t* reg,
                              uint8_t size )
{
    if(!size)
        return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, address, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if(size > 1)
        i2c_master_read(cmd, reg, size-1, 0); //0 is ACK

    i2c_master_read_byte(cmd, reg+size-1, 1); //1 is NACK

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Reset the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_reset(max30102_t *this)
{
    uint8_t val = 0x40;
    max30102_write_register(this, MAX30102_MODE_CONFIG, &val);
}

/**
 * @brief Enable A_FULL interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_a_full(max30102_t *this, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
    reg &= ~(0x01 << MAX30102_INTERRUPT_A_FULL);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_A_FULL);
    max30102_write_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
}

/**
 * @brief Enable PPG_RDY interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_ppg_rdy(max30102_t *this, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
    reg &= ~(0x01 << MAX30102_INTERRUPT_PPG_RDY);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_PPG_RDY);
    max30102_write_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
}

/**
 * @brief Enable ALC_OVF interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_alc_ovf(max30102_t *this, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
    reg &= ~(0x01 << MAX30102_INTERRUPT_ALC_OVF);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_ALC_OVF);
    max30102_write_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);
}


/**
 * @brief Set interrupt flag on interrupt. To be called in the corresponding external interrupt handler.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_on_interrupt(max30102_t *obj)
{
    obj->_interrupt_flag = 1;
}

/**
 * @brief Check whether the interrupt flag is active.
 *
 * @param obj Pointer to max30102_t object instance.
 * @return uint8_t Active (1) or inactive (0).
 */
uint8_t max30102_has_interrupt(max30102_t *obj)
{
    return obj->_interrupt_flag;
}

/**
 * @brief Read interrupt status registers (0x00 and 0x01) and perform corresponding tasks.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_interrupt_handler(max30102_t *this)
{
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30102_read_register(this, MAX30102_INTERRUPT_ENABLE_1, &reg);

    if ((reg[0] >> MAX30102_INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
        max30102_read_fifo(this);
    }

    if ((reg[0] >> MAX30102_INTERRUPT_PPG_RDY) & 0x01)
    {
        // New FIFO data ready
    }

    if ((reg[0] >> MAX30102_INTERRUPT_ALC_OVF) & 0x01)
    {
        // Ambient light overflow
    }


    // Reset interrupt flag
   // this->_interrupt_flag = 0;
}

/**
 * @brief Shutdown the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param shdn Shutdown bit.
 */
void max30102_shutdown(max30102_t *this, uint8_t shdn)
{
    uint8_t config;
    max30102_read_register(this, MAX30102_MODE_CONFIG, &config);
    config = (config & 0x7f) | (shdn << MAX30102_MODE_SHDN);
    max30102_write_register(this, MAX30102_MODE_CONFIG, &config);
}

/**
 * @brief Set measurement mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param mode Measurement mode enum (max30102_mode_t).
 */
void max30102_set_mode(max30102_t *obj, max30102_mode_t mode)
{
    uint8_t config;
    max30102_read_register(obj, MAX30102_MODE_CONFIG, &config);
    config = (config & 0xf8) | mode;
    max30102_write_register(obj, MAX30102_MODE_CONFIG, &config);
    //max30102_clear_fifo(obj);
}

/**
 * @brief Set sampling rate.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param sr Sampling rate enum (max30102_spo2_st_t).
 */
void max30102_set_sampling_rate(max30102_t *obj, max30102_sr_t sr)
{
    uint8_t current_spO2_reg;
    max30102_read_register(obj, MAX30102_SPO2_CONFIG, &current_spO2_reg);

    max30102_write_register(obj, MAX30102_SPO2_CONFIG, (current_spO2_reg & 0xE3) | (sr<<2));
}

/**
 * @brief Set led pulse width.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param pw Pulse width enum (max30102_led_pw_t).
 */
void max30102_set_led_pulse_width(max30102_t *obj, max30102_led_pw_t pw)
{
    uint8_t current_spO2_reg;
    max30102_read_register(obj, MAX30102_SPO2_CONFIG, &current_spO2_reg);

    max30102_write_register(obj, MAX30102_SPO2_CONFIG, (current_spO2_reg & 0xFC) | pw );
}

/**
 * @brief Set ADC resolution.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param adc ADC resolution enum (max30102_adc_t).
 */
void max30102_set_adc_resolution(max30102_t *obj, max30102_adc_t adc)
{
    uint8_t config;
    max30102_read_register(obj, MAX30102_SPO2_CONFIG, &config);
    config = (config & 0x1f) | (adc << MAX30102_SPO2_ADC_RGE);
    max30102_write_register(obj, MAX30102_SPO2_CONFIG, &config);
}

/**
 * @brief Set LED current.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_1(max30102_t *obj, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write_register(obj, MAX30102_LED_IR_PA1, &pa);
}

/**
 * @brief Set LED current.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_2(max30102_t *obj, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write_register(obj, MAX30102_LED_RED_PA2, &pa);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_1_2(max30102_t *obj, max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2)
{
    uint8_t val = 0;
    val |= ((slot1 << MAX30102_MULTI_LED_CTRL_SLOT1) | (slot2 << MAX30102_MULTI_LED_CTRL_SLOT2));
    max30102_write_register(obj, MAX30102_MULTI_LED_CTRL_1, &val);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_3_4(max30102_t *obj, max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4)
{
    uint8_t val = 0;
    val |= ((slot3 << MAX30102_MULTI_LED_CTRL_SLOT3) | (slot4 << MAX30102_MULTI_LED_CTRL_SLOT4));
    max30102_write_register(obj, MAX30102_MULTI_LED_CTRL_2, &val);
}

/**
 * @brief
 *
 * @param obj Pointer to max30102_t object instance.
 * @param smp_ave
 * @param roll_over_en Roll over enabled(1) or disabled(0).
 * @param fifo_a_full Number of empty samples when A_FULL interrupt issued (0 < fifo_a_full < 15).
 */
void max30102_set_fifo_config(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full)
{
    uint8_t config = 0x00;
    config |= smp_ave << MAX30102_FIFO_CONFIG_SMP_AVE;
    config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIG_ROLL_OVER_EN);
    config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIG_FIFO_A_FULL);
    max30102_write_register(obj, MAX30102_FIFO_CONFIG, &config);
}

/**
 * @brief Clear all FIFO pointers in the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_clear_fifo(max30102_t *obj)
{
    uint8_t val = 0x00;
    max30102_write_register(obj, MAX30102_FIFO_WR_PTR, &val);
    max30102_write_register(obj, MAX30102_FIFO_RD_PTR, &val);
    max30102_write_register(obj, MAX30102_OVF_COUNTER, &val);
}

/**
 * @brief Read FIFO content and store to buffer in max30102_t object instance.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_read_fifo(max30102_t *obj)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wr_ptr = 0, rd_ptr = 0;
    max30102_read_register(obj, MAX30102_FIFO_WR_PTR, &wr_ptr);
    max30102_read_register(obj, MAX30102_FIFO_RD_PTR, &rd_ptr);

    int8_t num_samples;
    char buff[2];
    num_samples = (int8_t)wr_ptr - (int8_t)rd_ptr;
    if (num_samples < 1)
    {
        num_samples += 32;
    }

    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
    for (int8_t i = 0; i < num_samples; i++)
    {
        uint8_t sample[6];
        max30102_read_from(obj, MAX30102_FIFO_DATA, sample, 6);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[3])) & 0x3ffff;
        uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;
        obj->_ir_samples[i] = ir_sample;
        obj->_red_samples[i] = red_sample;
        buff[0] = (char) ir_sample;
        buff[1] = (char) red_sample;
        static const char *TX_BPM_TAG = "TX_BPM";
        esp_log_level_set(TX_BPM_TAG, ESP_LOG_INFO);
        sendData(TX_BPM_TAG, buff[0]);
        static const char *TX_SpO2_TAG = "TX_SpO2";
        esp_log_level_set(TX_SpO2_TAG, ESP_LOG_INFO);
        sendData(TX_SpO2_TAG, buff[1]);
    }
}



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