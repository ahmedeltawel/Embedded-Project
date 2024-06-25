# Arduino-Based Car Project

## Project Overview
This project was developed at the German University in Cairo in December 2021. The aim was to build a car that can maintain its lane, light up in the dark, and play music. The project utilized various components and libraries to achieve these functionalities.

### Team Members
- Anas ElNemr
- Ahmed Eltawel
- Ahmed Farouk
- Abdelrahman Hafez

## Project Description
The project involved creating a car with the following capabilities:
- Lane keeping
- Lighting in darkness
- Playing music

### Components Used
1. 4 LEDs
2. 1 SD Card
3. 2 Wheel Car Kit
4. 2 Line Followers (TCRT5000)
5. Light Ambient Sensor
6. MP3 Driver
7. 7 Segment Display
8. Joystick
9. Speaker
10. Dual H-Bridge Motor Driver (L298N)
11. 2.4" Colour TFT LCD Display

### Libraries Utilized
- `Arduino_FreeRTOS.h` (for scheduling)
- `semphr.h` (Semaphore library)
- `AFMotor.h` (Adafruit Motor Shield firmware)
- `Adafruit_TFTLCD.h` (for 8-bit TFT LCDs)
- `Arduino.h`
- `SoftwareSerial.h` (for serial communication)
- `DFRobotDFPlayerMini.h` (for MP3 playback)
- `Adafruit_GFX.h` (fonts library)
- `TouchScreen.h` (for 4-wire resistive touch screen)
- `SPI.h` (for SPI devices)
- `MCUFRIEND_kbv.h` (inherits from Adafruit_GFX)

## Circuit Design
The project circuit was designed using Fritzing.

## Input Handling
There are four main inputs to the Arduino:
1. **Light Sensor:** Activates LEDs if the value is above 500.
2. **Joystick:** Changes the car's gear.
3. **Line Follower:** Directs the car to move left or right.
4. **Touch Screen:** Executes tasks based on touch coordinates.

## Output Handling
There are four main output devices:
1. **Headlights:** Respond to the light sensor.
2. **7-Segment Display:** Shows the gear position.
3. **Motors:** Control car movement based on line follower input.
4. **Speaker & MP3 Module:** Play music selected via the touch screen.

## Task Prioritization and Scheduling
Tasks are divided across two Arduinos:

- **Bottom Arduino:**
  - **Task #1:** Motor control, line followers, and blinker warning (highest priority, 20ms delay).
  - **Task #2:** Light sensor and headlights (lower priority, 100ms delay).

- **Top Arduino:**
  - **Task #1:** LCD and MP3 control (lower priority, 100ms delay).
  - **Task #2:** 7-segment display and joystick (highest priority, 20ms delay).

## Challenges Faced
1. **Line Follower Sensors:** Difficulty in detecting the black line, resolved by adjusting sensor positions.
2. **Touch Screen Issues:** Screen displayed white due to library issues, fixed by using an older library version.

## Work Division
- **Top Level (Entertainment/Sound System):** Anas ElNemr & Ahmed Farouk
  - Handled touch screen design, MP3 module, joystick, and 7-segment display.
- **Bottom Level (Car Movement):** Ahmed Eltawel & Abdelrahman Hafez
  - Managed motor wiring, line follower sensors, light sensor, headlights, and lane departure detection.
- **Common Tasks:** Building the car, connecting both Arduinos, setting up I2C communication, and task scheduling.

## Conclusion
This project successfully combined multiple components and programming libraries to create a functional Arduino-based car with lane-keeping, lighting, and music playback features.
