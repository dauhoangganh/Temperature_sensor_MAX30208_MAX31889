import machine
import utime
from ustruct import unpack, unpack_from
import sys

#############################################################################
#I2C ADDRESS
MAX31889_ADDR = 0x50
#DEVICE_ID
DEVICE_ID = 0x30

#REGISTER
TEMP_SENSOR_SETUP = 0x14
PART_IDENTIFIER = 0xFF
STATUS = 0x0
FIFO_DATA = 0x08
FIFO_OF_CTR_REG = 0x06 #FIFO overflow counter
FIFO_DATA_COUNTER_REG = 0x07 #FIFO Datacounter
FIFO_CONF2_REG = 0x0A #FIFO configuration 2
GPIO_SETUP_REG = 0x20 #GPIO setup register

#BIT POSITON FOR STATUS REGISTER
MAX_TEMP_RDY_BIT = 0
A_FULL_BIT = 7

#GPIO MODE
GPIO_MODE_HIGHZ = 0 #HiZ Input
GPIO_MODE_OPEN_DRAIN = 1 # Open-drain output
GPIO_MODE_INTERNAL_PULLDOWN = 2 #1MΩ Internal Pulldown Input 
GPIO_MODE_INT_CONV = 3 #output an interrupt (GPIO0) or temperature conversion input (GPIO1)

#BIT POSITION FOR GPIO REGISTER
GPIO0_MODE0_BIT = 0
GPIO0_MODE1_BIT = 1
GPIO1_MODE0_BIT = 6
GPIO1_MODE1_BIT = 7
##############################################################################

##################Frequently used function to shorten the program###########################
#Function to get a specific bit value from a register and check if that bit is 1 or not
def get_bit(reg_value, bit):
    """This function takes two arguments: reg_value (the register 8-bit value) and bit (the bit position to check)."""
    return (reg_value >> bit) & 0x01

#Function to set a specific bit of a register to 1
def set_bit(reg_value, bit):
    """This function takes two arguments: reg_value (the register 8-bit value) and bit (the bit position to set)."""
    reg_value |= (1<<bit)
    return reg_value

#Function to clear a specific bit (make it 0)
def clear_bit(reg_value, bit):
    """This function takes two arguments: reg_value (the register 8-bit value) and bit (the bit position to clear)."""
    reg_value &= ~(1<<bit)
    return reg_value
##############################################################################################

#MAX31889 class
class MAX31889:
    def __init__(
            self,
            address = MAX31889_ADDR,
            i2c=None,
            gpio0=2, #default GPIO0 mode is binary 10
            gpio1=2, #default GPIO1 mode is binary 10
            **kwargs):
        self.address = address
        if i2c is None:
            raise ValueError("A I2C object is required. Please checkself I2C connection to MCU")
        self.i2c=i2c
        id = self.i2c.readfrom_mem(MAX31889_ADDR, PART_IDENTIFIER, 1)
        if (id != bytearray((DEVICE_ID,))):
            print("\nERROR: Could not communicate with MAX31889")
            sys.exit()
        else:
            print("\nSuccessfully connected to MAX31889")
        
        #set default GPIO mode
        self.set_GPIO_mode(gpio0, gpio1)
        utime.sleep_ms(100)
    

    def measure_temperature(self):
                
        #set convert_T bit of temp_sensor_setup register to 1 to start temperature measurement
        temp_sensor_field = self.i2c.readfrom_mem(MAX31889_ADDR, TEMP_SENSOR_SETUP, 1)
        temp_sensor_field = unpack("<b", temp_sensor_field)[0]
        temp_sensor_field = set_bit(temp_sensor_field, 0)
        self.i2c.writeto_mem(MAX31889_ADDR, TEMP_SENSOR_SETUP, bytearray([temp_sensor_field]))

        ###### Wait for the measurement to finish ###########
        # Timeout in milliseconds
        time_out = 1000
        # Start time
        start_time = utime.ticks_ms()
        while True:
            temp_sensor_field = self.i2c.readfrom_mem(MAX31889_ADDR, TEMP_SENSOR_SETUP, 1)
            temp_sensor_field = unpack("<b", temp_sensor_field)[0]
            if (get_bit(temp_sensor_field, 0) == 0):
                break
            # Check for timeout
            if utime.ticks_diff(utime.ticks_ms(), start_time) > time_out:
                print("\nMeasurement Timeout reached")
                sys.exit()
            utime.sleep_ms(1)
        ######################################################

        ########### Check STATUS register for A_FULL condition #####################
        status = self.i2c.readfrom_mem(MAX31889_ADDR, STATUS, 1)
        status = unpack("<b", status)[0]
        
        # Check FIFO STATUS
        if get_bit(status, A_FULL_BIT):
            overflow_counter = self.i2c.readfrom_mem(MAX31889_ADDR, FIFO_OF_CTR_REG, 1)
            overflow_counter = unpack("<b", overflow_counter)[0] 
            overflow_counter &=  0x1F #Clear unused bits
            print(f"\nFIFO overflowed by {overflow_counter} words")

            #Flush FIFO
            FIFO_Config2 =self.i2c.readfrom_mem(MAX31889_ADDR, FIFO_CONF2_REG, 1)
            FIFO_Config2 = unpack("<b", FIFO_Config2)[0]
            set_bit(FIFO_Config2 ,4)
            print("\nFIFO flushed")
            utime.sleep_ms(100)
        ##############################################################################

        #if a temperature sensor measurement has completed and new data is available to be read by the master
        if get_bit(status, MAX_TEMP_RDY_BIT): 
            print("Num_available samples in FIFO: ", self.No_of_samples_FIFO())
            temperature_data =self.i2c.readfrom_mem(MAX31889_ADDR, FIFO_DATA, 2)
            temperature_data = unpack(">h", temperature_data)[0]
            print("\nTemperature in deg C: ", self.convert_to_deg_C(temperature_data))
        utime.sleep(1)

    ########## SEt GPIO mode function #################################################
    def set_GPIO_mode(self, GPIO0_mode, GPIO1_mode):
        GPIO_reg_value = self.i2c.readfrom_mem(MAX31889_ADDR, GPIO_SETUP_REG, 1)
        GPIO_reg_value = unpack("<b", GPIO_reg_value)[0]
        #If GPIO0 mode is 01 or 11, then set bit 0 of GPIO register value to 1, else clear bit 0 to 0
        if (GPIO0_mode & 0x01):
            set_bit(GPIO_reg_value, GPIO0_MODE0_BIT)
        else: clear_bit(GPIO_reg_value, GPIO0_MODE0_BIT) 
        #If GPIO0 mode is 10 or 11, then set bit 1 of GPIO register value to 1, else clear bit 1 to 0
        if (GPIO0_mode & 0x10):
            set_bit(GPIO_reg_value, GPIO0_MODE1_BIT)
        else: clear_bit(GPIO_reg_value, GPIO0_MODE1_BIT) 
        #If GPIO1 mode is 01 or 11, then set bit 6 of GPIO register value to 1, else clear bit 6 to 0
        if (GPIO1_mode & 0x01):
            set_bit(GPIO_reg_value, GPIO1_MODE0_BIT)
        else: clear_bit(GPIO_reg_value, GPIO1_MODE0_BIT) 
        #If GPIO1 mode is 10 or 11, then set bit 7 of GPIO register value to 1, else clear bit 7 to 0
        if (GPIO1_mode & 0x01):
            set_bit(GPIO_reg_value, GPIO1_MODE1_BIT)
        else: clear_bit(GPIO_reg_value, GPIO1_MODE1_BIT) 
    ####################################################################################

    ############### Convert binary temperature to degree celcius in decimal ############
    def convert_to_deg_C(self, temperature_data):
        #if MSB is 0, then convert temperature data to decimal
        if ~(temperature_data & (1<<15)):
            return temperature_data*0.005
        else: #is MSB = 1, convert the two's complement value to the decimal value
            temperature_data = (temperature_data - (1 << 16))*0.005
            return temperature_data
    ####################################################################################

    ######### Number of samples available in the FIFO after the last read ##############
    def No_of_samples_FIFO(self):
        ovf_counter = self.i2c.readfrom(MAX31889_ADDR, FIFO_OF_CTR_REG, 1)
        data_counter = self.i2c.readfrom(MAX31889_ADDR, FIFO_DATA_COUNTER_REG, 1)
        ovf_counter = unpack("<b", ovf_counter)[0]
        data_counter = unpack("<b", data_counter)[0]
        if ovf_counter == 0:
            num_available_samples = data_counter
        else:
            num_available_samples = 32
        return num_available_samples
    #####################################################################################
 ##############################################################################


