/**
 *  ESP32-MAX30102 
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

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "max30102.h"
#include "uart_max30102.h"
#include "freertos/semphr.h"
#include "esp_freertos_hooks.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_pm.h"

#define I2C_SDA                 (GPIO_NUM_21)
#define I2C_SCL                 (GPIO_NUM_22)
#define I2C_FRQ                     400000
#define I2C_PORT                   I2C_NUM_0
#define CONFIG_LED_PIN          (GPIO_NUM_27)
#define CONFIG_BUTTON_PIN       (GPIO_NUM_0)
#define CONFIG_INT_PIN          (GPIO_NUM_19)
#define GPIO_INPUT_PIN_SEL  (1ULL << CONFIG_INT_PIN)
#define ESP_INTR_FLAG_DEFAULT          0
static bool sensor_have_finger[2];  //flag for finger presence on sensor


int queueSize = 50;

SemaphoreHandle_t xSemaphore = NULL;
bool led_status = false;

max30102_t max30102 = {};



esp_err_t i2c_master_init(i2c_port_t i2c_port){
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FRQ;
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}


// interrupt service routine, called when the INT is low
void IRAM_ATTR INT_isr_handler(void* arg) {
	
    // notify the INT task
    xTaskCreate(sensor_task_manager, "isr_task_manager", 1024 * 4, (void* ) arg, 10, NULL);

}
void sensor_task_manager(void* arg)
{
	ESP_LOGI("Sensor_TASK_MANAGER","on core %d\tpin: %d",xPortGetCoreID(),(int)arg);
	uint8_t data=0x00;
	i2c_port_t port = (i2c_port_t)arg == I2C_NUM_0;

    esp_err_t ret = max30102_read_register(&max30102, MAX30102_INTERRUPT_STATUS_1, &data);
    if(ret != ESP_OK) return ret;
    printf("data: %d\n",data);
	bool fifo_a_full_int = data>>7 & 0x01;
	bool prox_int = data>>4 & 0x01;
	bool alc_ovf = data>>5 & 0x01;
#ifndef PLOT
	printf("\tINT Reason: 0x%02x\t",data);
	fifo_a_full_int ? printf("\tFIFO A FULL\n") : NULL;
	prox_int 		? printf("\tPROX INT\n") : NULL;
#endif
	if (prox_int){
		sensor_have_finger[0] = true ;
		//clear buffer
		max30102_write_register(&max30102,MAX30102_INTERRUPT_ENABLE_1, FIFO_A_FULL_EN);	//0b1000 0000 //disable prox interrupt and enable fifo_a_full
	}

	//xTaskCreate(i2c_task_0, "i2c_test_task_0", 1024 * 4, (void* ) 0, 10, NULL);
	xTaskCreate(i2c_task_1, "i2c_task_1", 1024 * 4, (void* ) port, 10, NULL);	//4kB stack size
	vTaskDelete(NULL);
}
void i2c_task_0(void* arg)
{
	printf("Start task 0!\n");
	int i = 0;
	int ret;

	uint8_t sensor_data_h;
	while(1){
		ret = max30102_read_register(&max30102, MAX30102_LED_RED_PA2, &sensor_data_h);
		check_ret(ret,sensor_data_h);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);

}
void i2c_task_1(void* arg)
{	double mean1, mean2;
    #ifndef PLOT
    printf("Start i2c_task_1!\n");
    #endif
    int *buffer_pos;
    uint8_t *raw_pptr_IR,*raw_pptr_RED;
    raw_pptr_IR =  malloc(FIFO_A_FULL);
    raw_pptr_RED = malloc(FIFO_A_FULL);
    i2c_port_t port = I2C_NUM_0;	
    uint16_t RAWsensorDataRED[FIFO_A_FULL/2], RAWsensorDataIR[FIFO_A_FULL/2];
    max30102_read_fifo(port, RAWsensorDataRED,RAWsensorDataIR);
    vTaskDelete(NULL);
}
void print_array(uint8_t *array,uint16_t size){
	for (int  i= 0;  i< size; i++) {
		printf("array[%d]=0x%x\n",i,array[i]);
	}
}

int isr_io_config()
{
	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // enable interrupt on falling (1->0) edge for button pin
	gpio_set_intr_type(CONFIG_INT_PIN, GPIO_INTR_NEGEDGE);
    
    rtc_gpio_pullup_en(CONFIG_INT_PIN);
	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin

    gpio_isr_handler_add(CONFIG_INT_PIN, INT_isr_handler, (void*) CONFIG_INT_PIN);
	return 0;
}


void app_main(void)
{

    init_uart();
    //Init I2C_NUM_0
    ESP_ERROR_CHECK(i2c_master_init(I2C_PORT));
    //Init sensor at I2C_NUM_0
    ESP_ERROR_CHECK(max30102_init( &max30102,I2C_PORT));
    ESP_ERROR_CHECK(max30102_print_registers(&max30102));
    isr_io_config();
    //Start test task

}
