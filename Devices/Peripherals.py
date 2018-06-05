
import ADS1115
import MCP4922

def Voltage(power):
    # A function to convert optical power to voltage
    voltage = power
    return voltage

def OpPower(voltage):
    # A function to convert voltage
    power = voltage 
    return power

def GetPhotodiodeVoltage(device, channel, gain, numSamples):

    runSum = 0

    for i in range(numSamples): # Average samples
        runSum += ADS1115.ReadFromADCDifferential(device, channel, gain)
    
    return ADS1115.ConvertToVoltage(runSum/numSamples, gain)

def WriteToDAC(spiDevice, channel, voltage, gain, vRefVoltage):

    DACData = MCP4922.ConvertVoltageToDACValue(voltage, vRefVoltage)

    MCP4922.WriteToDAC(spiDevice, channel, DACData, gain)


