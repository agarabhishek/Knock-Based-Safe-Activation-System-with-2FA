# Knock-Based-Safe-Activation-System-with-2FA

This project was built as a part of the Embedded Systems Hardware course. It is a secret safe system having multiple authentication levels. a fixed number of knocks followed by OTP verification. 

## Technologies

- Bluetooth
- SPI, I2C

## Components

- STM32F767Z Microcontroller
- MPU-6050, Piezo sesnor, HC-05, SIM 900A
- Stepper and Servo motors
- 4x4 Keypad, 16x2 LCD

## Knock Detection

For detecting knocks, piezoelectric sensors were used. The analog output was fed into ADC.
To detect successive knocks, interrupt had to be used. I used ADC Watchdog. 
Setting a high-threshold on the ADC value, an interrupt was sent whenever a knock was detected and thereby the value crossed the threshold. At the same time, a timer would start. 
If two more interrupts (knocks) were detected within the timer period, then the successive knocks were considered successful.
