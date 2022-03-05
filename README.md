ESP-IDF Heart Rate and Oximeter Sensor
====================

This is a template application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).
This driver developed for the MAX30102 Heart Rate and Pulse Oximetry sensor uses an I2C communcations with an interrupt for the buffer reading as suggested in the [MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf).


*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*
## Summary 
* Heartbeat recognition and heart rate display
* SPO2 Measurement
## Quick Start
### Minimimal Setup
![alt text](https://github.com/nferrante93/esp32_Oximeter/blob/master/minimal_connections.bmp)
The Connection with the ESP32-C3 are as follow:

 ESP32        | MAX30102 
------------- | -------------
G21           | SDA
G22           | SCL
+3.3 V        | Vin
GND           | GND
G18           | INT
## Registers
The MAX30102 is controlled/adjusted using its eight bit registers. A register map detailed in the [MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf). 

![alt text](https://github.com/nferrante93/ESP32_Oximeter/blob/master/images/registers1.bmp)
![alt text](https://github.com/nferrante93/ESP32_Oximeter/blob/master/images/registers2.bmp)

We can see the register types, what each bit in the register does, the register address, its state when the device is powered on or reset (POR), and whether it is read only or writable.

### Interrupts Registers
![alt text](https://github.com/nferrante93/ESP32_Oximeter/blob/master/images/interrupts_registers.bmp)

The interrupts triggered by Interrupt status 1 and Interrupt status 2 are as follows: FIFO almost full flag
(A_FULL), new FIFO data ready (PPG_RDY), ambient light cancellation overflow (ALC_OVF), proximity
threshold triggered (PROX_INT), power ready flag (PWR_RDY).
### Settings Interrupts Registers
This interrupt table is implemented as an enum.
```
/**
 * Interrupt Mode Enum.
 * A_FULL: FIFO Almost Full Flag.
 * PPG_RDY: New FIFO Data Ready.
 * ALC_OVF: Ambient Light Cancellation Overflow.
 * PWR_RDY: Power Flag Ready
 * DIE_TEMP_RDY: Internal Temperature Ready Flag
 */
typedef enum _max30102_interrupt_t {
  MAX30102_INTERRUPT_A_FULL   				  = 0x07,
  MAX30102_INTERRUPT_PPG_RDY   				  = 0x06,
  MAX30102_INTERRUPT_ALC_OVF    			  = 0x05,
  MAX30102_INTERRUPT_DIE_TEMP_RDY   		          = 0x01,
} max30102_interrupt_t;

```
* Enable the required interrupts:
```
// Enable FIFO_A_FULL interrupt
max30102_set_a_full(&max30102, 1);
```
* Run interrupt handler once interrupt flag is active:
```
// If interrupt flag is active
if (max30102_has_interrupt(&max30102))
// Run interrupt handler to read FIFO
max30102_interrupt_handler(&max30102);

```
## SpO2
This is implemented in the c max30102_update function. The basic equations are discussed in detail in the [MAX30102 Application Node](https://pdfserv.maximintegrated.com/en/an/AN6409.pdf).
![alt text](https://github.com/nferrante93/ESP32_Oximeter/blob/master/images/max30102SpO2.bmp)
```
float ratio_rms = log( sqrt( this->red_ac_sq_sum /
                                     (float)this->samples_recorded ) ) /
                          log( sqrt( this->ir_ac_sq_sum /
                                     (float)this->samples_recorded ) );

        if(this->debug)
            printf("RMS Ratio: %f\n", ratio_rms);

        this->current_spO2 = 104.0 - 17.0 * ratio_rms;
        data->spO2 = this->current_spO2;

```
## References
[MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf)

[MAX30102 Application Node](https://pdfserv.maximintegrated.com/en/an/AN6409.pdf)
