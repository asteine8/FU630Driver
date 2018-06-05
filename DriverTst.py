import FU630Driver

testLaser = FU630Driver.FU630Laser.FU630_Laser() # Create a FU630_Laser object

powpow = input("Enter Target Optical Power (mW):") # Request target optical power
testLaser.JumpToOpPower(powpow)

print("")

testLaser.RecordData() # Update data

print("Voltage: " + str(testLaser.voltageData[0]) + " | Optical Power: " + str(testLaser.opPowerData[0]))