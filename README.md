# Smartwatch Prototype

## Project Description

This project details the development of a smartwatch prototype using embedded software to monitor physical activities and environmental conditions. The device integrates several sensors to provide users with real-time data on their performance and surroundings.

## Features

The smartwatch prototype is designed to measure:

* Distance traveled
* Steps taken
* Speed (average and instantaneous)
* Calories burned
* Altitude
* Temperature
* Humidity 

## Sensors Used

* **GPS Module:** Provides geographic location data to calculate distance, speed, altitude, and calories. 
* **ADXL345 Accelerometer:** Measures acceleration in three axes to detect steps, motion, and tilt. 
* **OLED Display:** Used to display real-time information to the user.
* **DHT11 Sensor:** Measures air temperature and relative humidity. 

## Code Implementation Details

###   GPS Module

* **Data Processing:** NMEA sentences are parsed to extract latitude, longitude, and time. 
* **Distance Calculation:** The Haversine formula is used to calculate the distance between two points. 
* **Speed Calculation:** Average speed is calculated by dividing total distance by total time, and instantaneous speed is calculated from the distance and time between recent GPS readings. 
* **Calorie Estimation:** Calories burned are estimated using the user's weight, average speed, and MET (Metabolic Equivalent of Task) values. 
* **Altitude Delta:** Altitude variations are tracked to measure the user's physical exertion. 
* **Error Handling:** Thresholds are implemented to filter out minor GPS inaccuracies in stationary conditions.

###   ADXL345 Accelerometer

* **Step Counting:** The accelerometer detects characteristic oscillations of walking and running to count steps. 
* **Data Processing:** Raw sensor data is processed using a moving average filter to reduce noise. 
* **Magnitude Calculation:** Acceleration magnitude is calculated as  `Magnitude = sqrt(x^2 + y^2 + z^2)`.

###   OLED Display

* **Data Display:** Sensor data (steps, temperature, humidity, etc.) is displayed on the OLED screen.
* **User Interface:** The code includes basic screen switching using button input.

###   DHT11 Sensor

* **Data Acquisition:** The DHT11 sensor measures temperature and humidity, and this data is displayed on the OLED display.

###   I2C Communication

* The I2C protocol is used for communication with the ADXL345 accelerometer, and the OLED display.

###   FreeRTOS

* FreeRTOS is used to manage concurrent tasks, including sensor data acquisition, processing, and display.

## Hardware Requirements

* ESP32 Microcontroller
* GPS Module
* ADXL345 Accelerometer
* OLED Display
* DHT11 Sensor

## Software Requirements

* ESP-IDF (Espressif IoT Development Framework)

## Installation and Setup

1.  Clone the repository.
2.  Set up the ESP-IDF environment.
3.  Connect the hardware components as per the schematics.
4.  Configure the project in ESP-IDF (e.g., serial port, Wi-Fi credentials if applicable).
5.  Build and flash the project to the ESP32.

## Repository Contents

* `main.c`:  Main source code file containing the logic for sensor interfacing, data processing, and display.
* `Smartwatch_model_eng.pdf`: English report detailing the project.
* `Modelo_Smartwatch_pt.pdf`: Portuguese report detailing the project.
* `projeto_final_integrado/`: Contains all project files.

## Future Improvements

* Optimize energy consumption to increase battery life.
* Implement more precise sensor calibration. 
* Add heart rate monitoring functionality. 
* Expand the user interface with more features.

