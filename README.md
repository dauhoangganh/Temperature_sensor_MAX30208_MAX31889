This repo contains micropython drivers for MAX30208 and MAX31889 temperature sensors from Analog Devices. These drivers can be used with Raspberry Pi Pico to control temperature sensors. Only the minimum necesssary registers that nare needed to run the sensors are shown in the driver files. Users can freely add more regsiter settings to the driver files if needed.

temperature_sensor_main.py is the main file that is used to set up the sensor, read sensor data and convert read data to degree celcius temperature.
