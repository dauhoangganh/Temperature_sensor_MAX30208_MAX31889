#This file is used to read data from temperature sensor
#combine this file with the main.py file if you connect temperature sensor to the sensorhub dev board and
#want to read temperature from the sensor

import machine
import utime
from ustruct import unpack, unpack_from
import sys

#############################################################################
#I2C address
MAX31889_ADDR = 0x50
#DEVICE_ID
DEVICE_ID = 0x30

#REGISTER
TEMP_SENSOR_SETUP = 0x14
PART_IDENTIFIER = 0xFF
STATUS = 0x0
FIFO_DATA = 0x08
FIFO_OF_CTR_REG = 0x06 #FIFO overflow counter
FIFO_CONF2_REG = 0x0A #FIFO configuration 2

#BIT POSITON FOR STATUS REGISTER
MAX_TEMP_RDY_BIT = 0
A_FULL_BIT = 7

##############################################################################

###############################################################################
# Settings

# Initialize I2C with pins
i2c = machine.I2C(0,
                  scl=machine.Pin(17),
                  sda=machine.Pin(16),
                  freq=100000)

###############################################################################
# Check if I2C connection is established
devices = i2c.scan()
if devices:
    for d in devices:
        print("I2C address: " + hex(d))
utime.sleep_ms(100)

#Function to get a specific bit value from a register and check if that bit is 1 or not
def get_bit(value, bit):
    """This function takes two arguments: value (the byte to check) and bit (the bit position to check)."""
    return (value >> bit) & 0x01
 ##############################################################################
id = i2c.readfrom_mem(MAX31889_ADDR, PART_IDENTIFIER, 1)
if (id != bytearray((DEVICE_ID,))):
    print("ERROR: Could not communicate with MAX31889")
    sys.exit()
else:
    print("Successfully connected to MAX31889")
utime.sleep_ms(100)

while True:
    #set convert_T bit of temp_sensor_setup register to 1 to start temp measurement
    temp_sensor_field = i2c.readfrom_mem(MAX31889_ADDR, TEMP_SENSOR_SETUP, 1)
    temp_sensor_field = unpack("<b", temp_sensor_field)[0] | 0x1
    i2c.writeto_mem(MAX31889_ADDR, TEMP_SENSOR_SETUP, bytearray([temp_sensor_field]))

    utime.sleep_ms(1000)

    status = i2c.readfrom_mem(MAX31889_ADDR, STATUS, 1)
    status = unpack("<b", status)[0]

    # Check FIFO STATUS
    if get_bit(status, A_FULL_BIT):
        overflow_counter = i2c.readfrom_mem(MAX31889_ADDR, FIFO_OF_CTR_REG, 1)
        overflow_counter &=  0x1F #Clear unused bits
        print("\nFIFO overflowed by %i words", overflow_counter)

        #Flush FIFO
        FIFO_Config2 = i2c.readfrom_mem(MAX31889_ADDR, FIFO_CONF2_REG, 1)
        FIFO_Config2 = unpack("<b", FIFO_Config2)[0]
        FIFO_Config2 |= (1<<4)
        print("\nFIFO flushed")
    utime.sleep_ms(100)
    #if a temperature sensor measurement has completed and new data is available to be read by the master
    if get_bit(status, MAX_TEMP_RDY_BIT): 
        temperature_data = i2c.readfrom_mem(MAX31889_ADDR, FIFO_DATA, 2)
        temperature_data = unpack(">h", temperature_data)[0]
        print("Temperature in deg C: ", 0.005*temperature_data)
    utime.sleep(1)

