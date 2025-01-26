# 6 DoF Robot Arm Control via Logitech Joystick & STM32 
![STM32](https://img.shields.io/badge/Microcontroller-STM32-white)
![Project Status](https://img.shields.io/badge/Project-In%20Progress-yellow)
![](https://geps.dev/progress/90)

## Current Status
* Python script is functional
* LCD 1602 driver is functional [Repo](https://github.com/simarubhi/LCD1602_Driver_STM32)
* MPU6050 accelerometer driver is functional [Repo](https://github.com/simarubhi/MPU6050_Driver_STM32)
* PCA9685 motor driver is functional [Repo](https://github.com/simarubhi/PCA9685_Driver_STM32)
* Currently, testing, formatting, polishing, and presenting finished project
* Free RTOS needs to be restructured to work properly
* Video below of latest progress update

<div align="center">
      <a href="https://www.youtube.com/watch?v=ikd64a26juY">
         <img src="https://img.youtube.com/vi/ikd64a26juY/0.jpg" style="width: 600px;">
      </a>
</div>
* All axis calibration is now done, FreeRTOS is now being restructured to work properly


## Current Progress
1. Python HID interpreting script for the Logitech Joystick, sending data to the STM32 via serial communication _DONE_
2. LCD 1602 driver _DONE_
3. MPU6050 driver _DONE_
4. PCA8695 driver _DONE_
5. FreeRTOS _IN PROGRESS_
6. Presentation, documentation, formatting, etc. _IN PROGRESS_
