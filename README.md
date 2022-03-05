ESP-IDF Heart Rate and Oximeter Sensor
====================

This driver developed for the MAX30102 Heart Rate and Pulse Oximetry sensor uses an I2C communcations with an interrupt for the buffer reading as suggested in the [MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf).
<p align="center">
<img src="https://github.com/nferrante93/ESP32_MAX30102/blob/main/images/Breadboard_connections.bmp" width="500" height="600">
</p>
*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*

## Summary 
* Heartbeat recognition and heart rate display
* SPO2 Measurement
## Quick Start
### Minimimal Setup

<p align="center">
<img src="https://github.com/nferrante93/esp32-max30102/blob/main/images/minimal_connections.bmp">
</p>
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
<p align="center">
<img src="https://github.com/nferrante93/esp32-max30102/blob/main/images/registers1.bmp">
<img src="https://github.com/nferrante93/esp32-max30102/blob/main/images/registers2.bmp">
</p>
We can see the register types, what each bit in the register does, the register address, its state when the device is powered on or reset (POR), and whether it is read only or writable.

### Interrupts Registers
<p align="center">
<img src="https://github.com/nferrante93/esp32-max30102/blob/main/images/interrupts_registers.bmp">
</p>

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
#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

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
The second possibility is to use the Semaphore for notify the Getbpm task when the interrupts happens and use also a led that wil blink when the interrupts is triggered.

## SpO2
This is implemented in the c max30102_update function. The basic equations are discussed in detail in the [MAX30102 Application Node](https://pdfserv.maximintegrated.com/en/an/AN6409.pdf).
<p align="center">
<img src="https://github.com/nferrante93/esp32-max30102/blob/main/images/max30102SpO2.bmp">
</p>
A possible implementations of the conversion from the raw data to SpO2 values is shown below.

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

## Future Improvements
* DC Removal from the raw data
* Digital Signal Processing of the raw data with butterworth filter and Mean Median Filter 
* SpO2 and Heart Rate Measuring

## References
[MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf)

[MAX30102 Application Node](https://pdfserv.maximintegrated.com/en/an/AN6409.pdf)
