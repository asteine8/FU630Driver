# ADS1115 control and data conversion functions
# Phillia Steiner 4/12/18

# Connect:
# Vcc to +3v3, GND to ground, SDA to pin 3 (SDA1), SCL to pin 5 (SCL1), and ADDR to GND

# Uses the Adafruit ADS1x15 library Copyright (c) 2016 Adafruit Industries


import Adafruit_ADS1x15

import random

def ReadFromADCDifferential(device, channel, gain):
    # Gets differenetial data from the ADS1115 with variable channel and gain

    return device.read_adc_difference(channel, gain)

    # return random.randint(-2**15,2**15) # Placeholder code

def ReadFromADC(device, channel, gain):
    # Gets non differential data from the ADS1115

    return device.read_adc(channel, gain)

def ConvertToVoltage(data, gain):
    # Converts ADS1115 signed data to a voltage value based on the gain of the reading

    if gain == 1:
        return data * 0.000125
    elif gain == 2:
        return data * 0.0000625
    elif gain == 4:
        return data * 0.00003125
    elif gain == 8:
        return data * 0.000015625
    elif gain == 16:
	    return data * 0.0000078125