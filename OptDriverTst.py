import FU630Driver.FU630Laser
import time

print(dir(FU630Driver))

testLaser = FU630Driver.FU630Laser.FU630_Laser() # Create a FU630_Laser object

powpow = input("Enter Target Optical Power (mW):") # Request target optical power
testLaser.SetTargetOpticalPower(powpow)

print("")
while True:
	
	testLaser.OptimizeOpticalPower() # Optimize Laser Power

	pdVoltage = testLaser.peripheral.GetPhotodiodeVoltage(testLaser.ADS1115,testLaser.PHOTODIODE_ADC_CHANNEL,testLaser.ADC_GAIN,testLaser.NUM_ADC_SAMPLES)
	print("Voltage: " + str(testLaser.voltageData[0]) + " | Optical Power: " + str(testLaser.opPowerData[0]) + " | PD Voltage: " + str(pdVoltage) )
	time.sleep(1)
