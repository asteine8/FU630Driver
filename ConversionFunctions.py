import math

def TTLVoltageToOpPower(voltage):
    # Function to convert measured ttl voltage (volts) to optical power (mW)
    if voltage > 3.0138:
        return 94.8 # Function fails to model higher voltages
    else:
        return 481*voltage - 630 - 79.8*(voltage**2) # Apply modeling function

def OpPowerToTTLVoltage(power):
    # Function to convert measured optical power (mW) to ttl voltage (volts)
    if power > 94.8:
        return 3 # Function does not model for optical power above 94.8 mW
    else:
        return 0.00626566*(481-0.447214*math.sqrt(151325-1596*power)) # Apply modeling function

def PowerOverVoltageSlopeAtPower(power):
    # Returns the slope of power over voltage at a point defined by optical power
    voltage = OpPowerToTTLVoltage(power)
    return 481 - 159.6 *  voltage

def PhotodiodeVoltageToCurrent(voltage, shuntResistance):
    # Function to convert a measured voltage (volts) to current (mA) given the shunt resitance
    return 1000 * voltage / shuntResistance

def PhotodiodeVoltageToOpPower(voltage, shuntResistance):
    # Function to convert measured photodiode voltage (volts) to optical power (mW)
    current = PhotodiodeVoltageToCurrent(abs(voltage), shuntResistance)
    # return 117 * current + 0.275 #Laser 1
    return 42.769 * current + 0.149 #Laser 2
