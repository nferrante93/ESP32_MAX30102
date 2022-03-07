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
 * @param this Pointer to max30102_t object instance.
 * @param i2cnum Pointer to I2C object handle
 */
esp_err_t max30102_init(max30102_t *this, i2c_port_t i2c_num)
{
    this->i2c_num = i2c_num;
    this->_interrupt_flag = 0;
    memset(this->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(this->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));

    esp_err_t ret = max30102_write_register(this,MAX30102_INTERRUPT_ENABLE_1,PROX_INT_EN);
    if(ret != ESP_OK) return ret;
    ret = max30102_write_register(this,MAX30102_FIFO_WR_PTR,0x00);
    if(ret != ESP_OK) return ret;
    ret = max30102_write_register(this,MAX30102_OVF_COUNTER,0x00);
    if(ret != ESP_OK) return ret;
    ret = max30102_write_register(this,MAX30102_FIFO_RD_PTR,0x00);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_sample_averaging(this, max30102_smp_ave_4);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_roll_over(this, 1);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_almost_full(this,MAX30102_ALMOST_FULL_15);
    if(ret != ESP_OK) return ret;
    ret = max30102_write_register(this,MAX30102_MODE_CONFIG, 0x03);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_sampling_rate(this, MAX30102_SAMPLING_RATE_100HZ);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_high_res(this, MAX30102_ADC_RANGE_4096);
    if(ret != ESP_OK) return ret;
    ret = max30102_set_led_current(this, MAX30102_LED_CURRENT_50MA, MAX30102_LED_CURRENT_50MA);
    if(ret != ESP_OK) return ret;

    return ret;
}
esp_err_t max30102_set_led_current( max30102_t* this,max30102_current_t red_current,max30102_current_t ir_current )
{
   esp_err_t ret = max30102_write_register(this, MAX30102_LED_IR_PA1, red_current);
   if(ret != ESP_OK) return ret;
   return max30102_write_register(this, MAX30102_LED_RED_PA2, ir_current);
}
esp_err_t max30102_set_sampling_rate( max30102_t* this, max30102_sampling_rate_t rate )
{
    uint8_t current_spO2_reg;

    esp_err_t ret = max30102_read_register( this,MAX30102_SPO2_CONFIG,&current_spO2_reg );
    if(ret != ESP_OK) return ret;
    printf("Setting the sampling rate...");
    printf("%x\n", (current_spO2_reg & 0xE3) | (rate<<2) );
    return max30102_write_register( this,MAX30102_SPO2_CONFIG, (current_spO2_reg & 0xE3) | (rate<<2) );
}

esp_err_t max30102_set_high_res(max30102_t* this, max30102_adc_range_t adc_range) {
    uint8_t current_spO2_reg;

    esp_err_t ret = max30102_read_register(this, MAX30102_SPO2_CONFIG, &current_spO2_reg);
    if(ret != ESP_OK) return ret;
    printf("Setting the ADC range...");
    printf("%x\n", (current_spO2_reg & 0x9F) | (adc_range<<5) );
    return max30102_write_register( this,MAX30102_SPO2_CONFIG,(current_spO2_reg & 0x9F) | (adc_range<<5) );
    
}
esp_err_t max30102_set_sample_averaging(max30102_t* this,  max30102_smp_ave_t sample_averaging)
{
    uint8_t current_fifo_conf_reg;

    //Tratar erros
    esp_err_t ret = max30102_read_register( this, MAX30102_FIFO_CONFIG, &current_fifo_conf_reg );
    if(ret != ESP_OK) return ret;
    printf("Setting the sample averaging...");
    printf("%x\n", (current_fifo_conf_reg & 0x1F) | (sample_averaging<<5) );
    return max30102_write_register( this, MAX30102_FIFO_CONFIG,(current_fifo_conf_reg & 0x1F) | (sample_averaging<<5) );

}
esp_err_t max30102_set_roll_over(max30102_t* this, bool enabled) {
    uint8_t previous;

    //Tratar erros
    esp_err_t ret = max30102_read_register(this, MAX30102_FIFO_CONFIG, &previous);
    if(ret != ESP_OK) return ret;
    if(enabled) {
        return max30102_write_register( this, MAX30102_FIFO_CONFIG, previous | MAX30102_FIFO_CONFIG_ROLL_OVER_EN );
    } else {
        return max30102_write_register( this, MAX30102_FIFO_CONFIG,previous & ~MAX30102_FIFO_CONFIG_ROLL_OVER_EN );
    }
}
esp_err_t max30102_set_almost_full(max30102_t* this, max30102_almost_full_t almost_full)
{
        uint8_t current_fifo_conf_reg;

    //Tratar erros
    esp_err_t ret = max30102_read_register( this,MAX30102_FIFO_CONFIG,&current_fifo_conf_reg );
    if(ret != ESP_OK) return ret;
    printf("Setting the almost full value...");
    printf("%x\n", (current_fifo_conf_reg & 0xF0) | almost_full );
    return max30102_write_register( this, MAX30102_FIFO_CONFIG, (current_fifo_conf_reg & 0xF0) | almost_full );
}
/**
 * @brief Write buffer of buflen bytes to a register of the MAX30102.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to write to.
 * @param buf Val containing the bytes to write.
 */
esp_err_t max30102_write_register( max30102_t* this, uint8_t address, uint8_t val)
{
    // start transmission to device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, address, true); // send register address
    i2c_master_write_byte(cmd, val, true); // send value to write

    // end transmission
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num, cmd, 1000 / portTICK_RATE_MS );
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
esp_err_t max30102_read_register( max30102_t* this, uint8_t address,uint8_t* reg)
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
esp_err_t max30102_read_from( max30102_t* this, uint8_t address, uint8_t* reg, uint8_t size )
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



void check_ret(esp_err_t ret,uint8_t sensor_data_h){
	if(ret == ESP_ERR_TIMEOUT) {
		printf("I2C timeout\n");
	} else if(ret == ESP_OK) {
		printf("******************* \n");
		printf("TASK[%d]  MASTER READ SENSOR( EN_MAX30102_READING_TASK )\n", 0);
		printf("*******************\n");
		printf("data: %02x\n", sensor_data_h);
	} else {
		printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
	}
}
esp_err_t max30102_read_fifo(i2c_port_t i2c_num, uint16_t sensorDataRED[],uint16_t sensorDataIR[])
{

	uint8_t LED_1[FIFO_A_FULL/2][3],LED_2[FIFO_A_FULL/2][3];

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_I2C_ADDR << 1 | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, MAX30102_FIFO_DATA, 1);
	i2c_master_stop(cmd);	//send the stop bit
	int ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		printf("ESP NOT OK!!\n");
		return ret;
	}
	vTaskDelay(25 / portTICK_RATE_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_I2C_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

	for(int i=0; i < FIFO_A_FULL/2; i++){
		i2c_master_read_byte(cmd, &LED_1[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][2], ACK_VAL);

		i2c_master_read_byte(cmd, &LED_2[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][2], i==FIFO_A_FULL/2-1? NACK_VAL: ACK_VAL);	//NACK_VAL on the last iteration
	}

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	for(int i=0; i < FIFO_A_FULL/2; i++){
		/*sensorDataRED[i] = (((LED_1[i][0] &  0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);
		sensorDataIR[i]= (((LED_2[i][0] &  0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);	//*/
		//printf("\t%x %x %x\n",LED_1[i][0]&0x03,LED_1[i][1],LED_1[i][2]);

		sensorDataRED[i] = 	((((LED_1[i][0] &  0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES))>>2;
		sensorDataIR[i]= 	((((LED_2[i][0] &  0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES))>>2;	//
		/*#if SPO2_RES <= 16
		sensorDataRED[i] =sensorDataRED[i] >>2;
		sensorDataIR[i]  =sensorDataIR[i]  >>2;
#endif*/
		//fprintf(stdout,"%x, %x, %x\t%x, %x, %x\n",LED_1[i][0],LED_1[i][1],LED_1[i][2],LED_2[i][0],LED_2[i][1],LED_2[i][2]);
#ifdef PRINT_ALL_SENSOR_DATA
		fprintf(stdout,"0x%x\t0x%x\n",sensorDataRED[i],sensorDataIR[i]);
        for (int  i= 0;  i< FIFO_A_FULL/2; i++) 
        {
            static const char *TX_BPM_TAG = "TX_BPM";
            esp_log_level_set(TX_BPM_TAG, ESP_LOG_INFO);
            sendData(TX_BPM_TAG, RAWsensorDataRED[i]);
            static const char *TX_SpO2_TAG = "TX_SpO2";
            esp_log_level_set(TX_SpO2_TAG, ESP_LOG_INFO);
            sendData(TX_SpO2_TAG, RAWsensorDataIR[i+(FIFO_A_FULL/2)]);
	    }
#endif
	}


	/*int data1 = (((LED_1[i][0] && 0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES);
		/int data2 = (((LED_2[i][0] && 0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES);	*/

	//vTaskDelay(6000 / portTICK_RATE_MS);//magic delay

	return ret;
}


esp_err_t max30102_print_registers(max30102_t* this)
{
    uint8_t int_status, int_enable, fifo_write, fifo_ovf_cnt, fifo_read;
    uint8_t fifo_data, mode_conf, sp02_conf, led_conf, temp_int, temp_frac;
    uint8_t rev_id, part_id;
    esp_err_t ret;

    ret = max30102_read_register(this, MAX30102_INTERRUPT_STATUS_1, &int_status);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_INTERRUPT_ENABLE_1, &int_enable);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_FIFO_WR_PTR, &fifo_write);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register( this, MAX30102_OVF_COUNTER, &fifo_ovf_cnt );
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_FIFO_RD_PTR, &fifo_read);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_FIFO_DATA, &fifo_data);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_MODE_CONFIG, &mode_conf);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_SPO2_CONFIG, &sp02_conf);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_LED_IR_PA1, &led_conf);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_LED_RED_PA2, &temp_int);
    if(ret != ESP_OK) return ret;
    ret = max30102_read_register(this, MAX30102_I2C_ADDR, &rev_id);
    if(ret != ESP_OK) return ret;

    printf("%x\t", int_status);
    printf("%x\t", int_enable);
    printf("%x\t", fifo_write);
    printf("%x\t", fifo_ovf_cnt);
    printf("%x\t", fifo_read);
    printf("%x\t", fifo_data);
    printf("%x\t", mode_conf);
    printf("%x\t", sp02_conf);
    printf("%x\t", led_conf);
    printf("%x\t", temp_int);
    printf("%x\n", rev_id);

    return ESP_OK;
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