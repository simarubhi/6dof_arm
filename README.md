# 6DoF Robotic Arm

![STM32](https://img.shields.io/badge/Microcontroller-STM32-white)
![Project Status](https://img.shields.io/badge/Project-In%20Progress-yellow)

## Current Status

Check out updates in the [update log](https://github.com/simarubhi/6dof_arm/update_log)!

## Project Goals

The goal of this project is to create a 6 degree of freedom robotic arm from scratch. The arm will be able be capable of a range of tasks including video recording, soldering and general building assistance, automated testing. and more.

The priorities are the following:

1. The arm's movement will be smooth, accurate (&plusmn;1cm), with high repeatability (&plusmn;5mm).
2. The arm should be able to hold 2.5kg while maintaining regular operation.
3. The cost for a single unit should be at most $600 and all materials should be available online or easily manufactured.

## Technologies Used & Stages

### Stage 1 - Finished

This stage will consist of a test run of the concept with a pre-built 6DoF controlled using a STM32 Nucleo Board and Logitech Joystick to control the arm via a Python script and UART. This will help identify mechanical requirements for the custom built chassis later on, improving upon the flaws of the pre-built kit.

### Stage 2 - In Progress

This stage will consist of designing the arm in Fusion 360 as well as creating a custom PCB with a STM32 chip, motor drivers, and additional sensors/ICs to control the arm. The current goal is to be able to control the arm via a custom made joystick/controller system, as well as with software.

The subtasks are as follows:

1. Create a harmonic drive gearbox, used for compact high gear reduction with minimal backlash. **_Current Task_**
2. Prototype the base of the arm.
3. Prototype the first joint of the arm.
4. Prototype the second joint of the arm.
5. Prototype the arm gripper. The overall design and features desired are undecided at this time.
6. Test prototype with load bearing and general movement.
7. Add limit switches and advanced software/hardware features such as cooling, PID control, etc.
8. Design final iteration of chassis and finish development of software and joystick/controller control.
