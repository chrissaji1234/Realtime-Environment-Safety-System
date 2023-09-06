# Realtime-Environment-Safety-System

Created By: Christopher Saji

## Project Description

This project uses a SparkFun Audio Sensor and a UVM-30A UV Sensor in conjunction with a STM32 NUCLEO-L4R5ZI to measure the analog voltages outputted from the sensors. These voltages are then converted by the microcontroller into dB and UV Index readings. Using FreeRTOS, the L4R5ZI is able to alert the user of dangerous readings through the use of an LCD, LEDs, and a vibration motor.

## Project Functionality

The safety system constantly takes measurements of the current dB level and the UV Index using the two sensors. If the dB level exceeds 84 dB (sounds greater than 84 dB are detrimental to hearing), the red LED will turn on. Otherwise, the green LED will be illuminated. If the UV index exceeds a value of 7, the vibration motor will begin to vibrate. The current levels from both sensors are displayed to the user via the LCD.

## Sensors


### SparkFun Audio Sensor

This audio sensor has three outputs - audio, envelope, and gate. The audio terminal outputs the raw audio signal from the microphone. The envelope terminal outputs the amplitude of the sound captured from the microphone. The gate terminal outputs a one or a zero based on if the amplitude of sound has past a certain threshold. For this project, I used the envelope terminal of the sensor, the analog representation of the amplitude is needed in order to convert the captured sound into a dB reading. 

In order to increase the sensitivity of the sound sensor, I needed to replace the built resistor of 100K Ohm. I used a hot air gun to solder off the built in resistor and a solder iron to attach a 1Meg Ohm resistor to the board. This effectively increased the arithmetic gain by 10 and the gain by 20dB. I also soldered headers to the output terminals of the board in order for the sensor to be placed on a breadboard.

To convert the readings from the audio sensor into a dB measurement, I used a SPL meter and took note of the ADC readings from the microcontroller at different dB levels. I then plotted the measurements into Microsoft Excel and calculated a linear regression formula based on the readings.  This produces readings that are only a few dB off of the SPL meter. 

### UVM-30A UV Sensor

This UV sensor is able to measure the intensity of ultraviolet radiation from the spectral range 200nm-370nm. From its analog output terminal, the sensor is able to output different voltages based on the intensity of the UV being measured by the sensor within 1 UV Index. 

Shown below are the different UV Indices based on the voltage being outputted by sensor:

![UV Index Chart](https://github.com/chrissaji1234/Realtime-Environment-Safety-System/blob/master/Photos/UV%20Index%20Chart.jpg)

## Design

To program this project, I used STM's Cube IDE and CubeMX code generation software.

### Pin Initialization
Using CubeMX, I initialized pin PA3 and pin PC5 as single-ended devices on channels 8 and 14 respectively on ADC1. I then initialized pins PC0, PC1, and PC3 as GPIO outputs for the green LED, red LED, and vibration respectively. Pins PB9 and PB8 were initialized as SDA and SCL lines respectively for I2C usage with the LCD.  Lastly, I initialized PG8 and PG7 for STLINK_RX and STLINK_TX for UART. This is used for debugging purposes with a terminal.

### FreeRTOS Initialization

Since the STM32 has only one ADC, I had to figure out how to read from multiple channels at the same time. I decided to use FreeRTOS to have multiple threads running concurrently. I know I could of used only DMA to use multiple channels, but I also wanted to incorporate a RTOS because many safety systems in use have time constraints. Using CubeMX, I turned on the FreeRTOS  middleware and configured the system to use CMSIS_V1. In order to enable thread preemption, I had to enable USE_PREEMPTION. I then created four different tasks with the two threads relating to reading the values from the ADC of being of highest priority and outputting the information to the LCD and feedback devices being of lower priority. I also initialized a binary semaphore for both threads reading each individual sensor because one channel can only use the ADC at a given time. Finally, I changed the Timebase Source to TIM1, because the RTOS will use the SysTick. 

### Threads

#### readAudioSensor

This thread reads the analog voltage from the audio sensor. It first starts by acquiring if the semaphore is available. If it isn't, the thread waits until it is available. If it is, it calls the custom ADC_Select_CH8 function to configure the ADC to read channel 8. The thread uses polling to acquire the raw analog value. Using a linear regression formula calculated beforehand using an SPL meter, the thread calculates the dB value. It finally stores the value into a string and releases the semaphore.

#### readUVSensor

This thread reads the analog voltage from the UV sensor. The thread begins by checking if the semaphore is available or not. If the semaphore is not available, the thread will wait for it. If the semaphore is available, the thread will call the custom ADC_Select_CH14 function to configure the ADC to read channel 14. The thread uses polling to acquire the raw analog value. The thread then converts the raw analog reading to the real millivolt reading. Next, the thread then checks what the UV index is based on the voltage calculated. Finally, the current UV index is converted to a string and the semaphore is released.

#### feedbackDevices

This thread controls the LCD and vibration motor that allow the user to know if the current dB and UV reading is dangerous respectively. If the current dB reading is higher than or equal to 85 dB, the red LED will turn on. Otherwise, the green LED will be illuminated. In addition to this, if the UV index is greater than or equal to 8, the vibration motor will begin to vibrate.

#### lcdOutput

This thread controls the output of the LCD display. The thread begins by clearing the current information on the LCD and bringing the cursor back to home. It then outputs the current dB measurement to the screen. It then moves the cursor to the start of the next line and then displays the current UV index reading to the screen.

## Photos

### System Working Inside (Low Noise / Low UV)

![System working Inside](https://github.com/chrissaji1234/Realtime-Environment-Safety-System/blob/master/Photos/Inside.jpg)

### System Working Outside (High Noise/ High UV)

![System working Outside](https://github.com/chrissaji1234/Realtime-Environment-Safety-System/blob/master/Photos/Outside_NEW.jpg)

## References

### [STM 32 L4R5ZI Pin Outs](https://www.st.com/resource/en/user_manual/um2179-stm32-nucleo144-boards-mb1312-stmicroelectronics.pdf#page=35)

### [STM32 L4 MCU Documentation](https://www.st.com/resource/en/reference_manual/dm00310109-stm32l4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

### [STM32 L4 ADC Documentation](https://www.st.com/resource/en/product_training/stm32l4_analog_adc.pdf)

### [STM32 L4 HAL Documentation](https://www.st.com/resource/en/user_manual/um1884-description-of-stm32l4l4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)

### [SparkFun Audio Sensor](https://www.sparkfun.com/products/12642)

### [UVM-30A UV Sensor](https://curtocircuito.com.br/datasheet/sensor/raios_ultravioleta.pdf)

### [1602 LCD Display Documentation](https://controllerstech.com/i2c-lcd-in-stm32/)


