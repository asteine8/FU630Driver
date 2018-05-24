# MCP4922 control and data conversion functions
# Phillia Steiner 4/12/18

# Connect:
# Vcc to +3.3v, and 1 GND pin to GND
# SDI to pin 19 (MOSI), SCLK to pin 23 (SPI CLK), CS to pin 24 (CE0), LDAC to GND, SHDN to Vcc
# Note that Vref should be between Vcc and GND

import spidev

def WriteToDAC(device, channel, data, gain):
    # Writes to and turns on selected channel
    # Gain settings:
    #   >>1 = 1x gain
    #   >>0 = 2x gain

    bytesOut = [0,0] # Preallocate as local variable

    bytesOut[0] |= (channel << 7) | (gain << 5) # select channel and gain
    bytesOut[0] |= (1 << 4) # Don't shutdown the channel

    bytesOut[1] = data & 255 # Use 255 as 8 bit bitmask to get first 8 bits of data
    bytesOut[0] |= ((data >> 8) & 15) # Use 15 as 4 bit bitmask to get last 4 bits of data

    device.xfer2(bytesOut) # Write Bytes to DAC
    print(bin(bytesOut[1]))
    print(bin(bytesOut[0]))


def DeactivateDAC(device, channel):
    # Calling this function dectivates the selected channel on the provided device

    bytesOut = [0,0] # Preallocate as local variable

    bytesOut[0] |= (0 << 4) | (channel << 7) # Write a 0 to the SHDN bit and select channel
    
    device.xfer2(bytesOut) # Write Bytes to DAC
    print("DAC Off")

def ConvertVoltageToDACValue(voltage, vRef):
    # Converts a voltage value to data to write to the MCP4922 given a value of vRef

    return voltage / vRef * 4096