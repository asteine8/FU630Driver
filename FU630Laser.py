"""
Phillia Steiner, 2018
FU630Laser.py
This package allows one to interface with a FU-630SLD pump laser utilizing the ADS1115 ADC and the MCP4922 DAC modules. Four pump lasers can be attached at once due to the limited number of spi CE ports (two count).
"""

import ConversionFunctions as convert
import Devices.Peripherals

import spidev
import Adafruit_ADS1x15

import time
# import random

class FU630_Laser:

    # DAC Consts
    DAC_PORT = 0 # Port 0 for spi (default)
    DAC_CE = 0 # CE pin for DAC (Chip Enable)
    DAC_SPI_SPEED = 100000 # Clock speed for DAC spi comms (100kHz is good)
    DAC_GAIN = 1 # 1x gain
    MAX_DAC_VOLTAGE = 3.29 # Max voltage output for DAC
    VREF_VOLTAGE = 3.3 # Connect VREF to +3.3 volts on the DAC

    TTL_DAC_CHANNEL = 0 # 0 = channel A, 1 = channel B

    # ADC Consts
    ADS1115_ADDRESS = 0x48 # Current I2C Address of the ADS1115 with address pin to NC (find using "sudo i2cdetect -y 1")

    # ADC Gain Settings
    #  1 = +/- 4.096 V
    #  2 = +/- 2.048 V
    #  4 = +/- 1.024 V
    #  8 = +/- 0.512 V
    # 16 = +/- 0.256 V
    ADC_GAIN = 2 # FSR = +- 1.024 Volts
    NUM_ADC_SAMPLES = 10 # Number of samples to average (10 is good enough)

    # ADC Differential Channel Settings (Recommended to use settings 1 - 3 to allow for multiple devices per ADC)
    # 0 = A0 - A1
    # 1 = A0 - A3
    # 2 = A1 - A3
    # 3 = A2 - A3
    PHOTODIODE_ADC_CHANNEL = 3 # Standard channel (Differential)
    PHOTODIODE_SHUNT_RESISTANCE = 980 # Should be ~1k, value shoud be experimentally determined in ohms

    peripheral = Devices.Peripherals # Create an object to reference the Peripherals package from

    MCP4922 = spidev.SpiDev() # Create a spi object for the DAC
    ADS1115 = Adafruit_ADS1x15.ADS1115(address=ADS1115_ADDRESS) # Create an I2C object for the ADC

    NUM_DATA_POINTS = 20 # Number of data points to store (Only really need 2 but 3 are taken just in case)
    voltageData = list(range(NUM_DATA_POINTS)) # Store three data points (one extra for later use)
    opPowerData = list(range(NUM_DATA_POINTS))

    targetOpPower = 0 # The optical power output (in mW) that is continuously optimized to
    currentTTLVoltage = 0 # Value currently written to DAC

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Thresholds and Step Size for gradient range
    # All measurments are in volts

    MaxCushioningThreshold = 0.01 # Voltage from target where maximum step size is availible
    MinCushioningThreshold = 0.50 # Voltage from target where minimum step size is availible

    MaxStepSize = 0.05 # Maximum current limiter voltage step size
    MinStepSize = 0 # Minimum current limiter voltage step size (Set to 0 to not optimize towards target inside minimum threshold)

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # Calibration and precision constants
    TTL_VOLTAGE_SIG_FIGS = 3
    OPTICAL_POWER_SIG_FIGS = 3

    def __init__(self): # Do on class initialization
        self.MCP4922.open(self.DAC_PORT, self.DAC_CE) # Open spi port 0, device (CE) 0 (Connect to pin 24)
        self.MCP4922.max_speed_hz = self.DAC_SPI_SPEED # Set clk to max 100kHz (Can be higher...)
        self.optimizationState = 0
        self.functionList = [self.JumpToOpPower, self.ApplyGradientOptimization] # Function to call in order

    def GetDeltaOpticalPower(self, targetPower):
        # Gets current difference between current and target optical power and shuffles in new data to stack

        # Get current optical power
        self.currentPower = convert.PhotodiodeVoltageToOpPower(self.peripheral.GetPhotodiodeVoltage(self.ADS1115, self.PHOTODIODE_ADC_CHANNEL, self.ADC_GAIN, self.NUM_ADC_SAMPLES), self.PHOTODIODE_SHUNT_RESISTANCE)

        # Shuffle in data
        for i in range(1, self.NUM_DATA_POINTS-1, -1):
            self.voltageData[i] = self.voltageData[i-1]
            self.opPowerData[i] = self.opPowerData[i-1]

        # Record Current Optical Power and Voltage
        self.voltageData[0] = self.currentTTLVoltage
        self.opPowerData[0] = self.currentPower

        print("Current Optical Power: " + str(self.opPowerData[0]) + " mW")
        print("Current Voltage to Current Limiter: " + str(self.voltageData[0]) + " Volts")

        return targetPower - self.currentPower # Return difference (negative if under power)

    def JumpToOpPower(self, targetPower):

        # Convert target optical power to a voltage using a preset function
        voltage = convert.OpPowerToTTLVoltage(targetPower)

        # Change TTL voltage to calculated target
        self.peripheral.WriteToDAC(self.MCP4922, self.TTL_DAC_CHANNEL, voltage, self.DAC_GAIN, self.VREF_VOLTAGE)

        self.currentTTLVoltage = voltage # Update DAC voltage

    def ModulateVoltageByStepAndDirection(self, stepSize, dir):
        # Change Voltage from DAC via step and direction

        voltage = self.voltageData[0] + (stepSize * dir) # Calculate new voltage

        if (voltage > self.MAX_DAC_VOLTAGE):
            voltage = self.MAX_DAC_VOLTAGE
        elif (voltage < 0):
            voltage = 0

        self.peripheral.WriteToDAC(self.MCP4922, self.TTL_DAC_CHANNEL, voltage, self.DAC_GAIN, self.VREF_VOLTAGE) # Write voltage to DAC

        self.currentTTLVoltage = voltage #  Change current voltage to actual current voltage

        print("Step size: " + str(stepSize) + " | dir: " + str(dir))

    def ApplyGradientOptimization(self, targetPower):
        # Apply a gradient mediated optimization within set thresholds

        if targetPower == 0: # Check for laser off
            self.TurnOffLaser() # Turn off the laser
            return

        deltaVoltage = self.GetDeltaOpticalPower(targetPower) # Also records data into data records

        # Get direction to target voltage
        if deltaVoltage < 0:
            dir = -1
        else:
            dir = 1

        if abs(deltaVoltage) >= self.MaxCushioningThreshold: # Outside gradient so use max step size

            self.ModulateVoltageByStepAndDirection(self.MaxStepSize, dir)

        elif abs(deltaVoltage) > self.MinCushioningThreshold: # In gradient range

            gradientModulatedPercentage = (abs(deltaVoltage) - self.MinCushioningThreshold) / abs(self.MaxCushioningThreshold - self.MinCushioningThreshold)

            print("% Step Size: " + str(gradientModulatedPercentage) + "%")

            stepSize = self.MaxStepSize * gradientModulatedPercentage

            self.ModulateVoltageByStepAndDirection(stepSize, dir)

        else: # Inside Minimum step size threshold

            self.ModulateVoltageByStepAndDirection(self.MinStepSize, dir)


    def TurnOffLaser(self):
        # Sets TTL voltage to 0 to completely shutdown the current source and consequently the attached laser

        self.peripheral.WriteToDAC(self.MCP4922, self.TTL_DAC_CHANNEL, 0, self.DAC_GAIN, self.VREF_VOLTAGE) # Write 0 volts to the DAC

        self.currentTTLVoltage = 0 # Update DAC voltage

    def ModifyOptimizationState(self):
        if self.optimizationState < len(self.functionList) - 1:
            self.optimizationState += 1

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # User Functions (The only functions a user really needs to use)

    def SetTargetOpticalPower(self, targetPower):
        # Function to set target optical power to enhance readability of user code
        self.targetOpPower = targetPower
        self.optimizationState = 0 # Reset optimization state

    def OptimizeOpticalPower(self):
        # optimize to set optical power target

        if self.targetOpPower == 0: # If target optical power is 0
            self.TurnOffLaser() # Turn off Laser

        self.functionList[self.optimizationState](self.targetOpPower) # Call appropiate function with target optical power
        self.ModifyOptimizationState() # Change the optimization State
