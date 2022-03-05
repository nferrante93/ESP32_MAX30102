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

#define I2C_SDA                 (GPIO_NUM_21)
#define I2C_SCL                 (GPIO_NUM_22)
#define I2C_FRQ                     100000
#define I2C_PORT                   I2C_NUM_0
#define CONFIG_LED_PIN          (GPIO_NUM_27)
#define CONFIG_BUTTON_PIN       (GPIO_NUM_0)
#define CONFIG_INT_PIN          (GPIO_NUM_19)
#define GPIO_INPUT_PIN_SEL  (1ULL << CONFIG_INT_PIN)
#define ESP_INTR_FLAG_DEFAULT          0


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
	xSemaphoreGiveFromISR(xSemaphore, NULL);
    //max30102_on_interrupt(&max30102);

}
void get_bpm(void* param) {
    printf("MAX30102 Test\n");

    //Update sensor data
    for(;;)
    {
        // wait for the notification from the ISR
		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
        /*if (max30102_has_interrupt(&max30102))
        {
            max30102_interrupt_handler(&max30102);
        }*/
            max30102_interrupt_handler(&max30102);
            led_status = !led_status;
		    gpio_set_level(CONFIG_LED_PIN, led_status);
        }
    }
    //Update rate: 100Hz
    vTaskDelay(100/portTICK_PERIOD_MS);
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

	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin

    gpio_isr_handler_add(CONFIG_INT_PIN, INT_isr_handler, (void*) CONFIG_INT_PIN);
	return 0;
}


void app_main(void)
{
    init_uart();
    // Create queues
    msg_queue = xQueueCreate(msg_queue_len, sizeof(msg));
    //Init I2C_NUM_0
    ESP_ERROR_CHECK(i2c_master_init(I2C_PORT));
    //Init sensor at I2C_NUM_0
    max30102_init( &max30102,I2C_PORT);

    max30102_reset(&max30102);
    max30102_clear_fifo(&max30102);
    max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

    // Sensor settings
    max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
    max30102_set_adc_resolution(&max30102, max30102_adc_2048);
    max30102_set_sampling_rate(&max30102, max30102_sr_800);
    max30102_set_led_current_1(&max30102, 50);
    max30102_set_led_current_2(&max30102, 40);

    // Enter SpO2 mode
    max30102_set_mode(&max30102, max30102_spo2);
    max30102_set_a_full(&max30102, 1);
    uint8_t en_reg[2] = {0};
    max30102_read_register(&max30102, 0x00, en_reg);

    isr_io_config();
    //Start test task
    xTaskCreate(get_bpm, "Get BPM", 8192, NULL, 1, NULL);
    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();
	
	// configure led pin as GPIO pin
	gpio_pad_select_gpio(CONFIG_BUTTON_PIN);
	gpio_pad_select_gpio(CONFIG_LED_PIN);

	// set the correct direction
    gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT);

}



