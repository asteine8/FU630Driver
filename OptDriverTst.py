import FU630Driver.FU630Laser
import time

# print(dir(FU630Driver))

testLaser = FU630Driver.FU630Laser.FU630_Laser() # Create a FU630_Laser object

powpow = input("Enter Target Optical Power (mW) (Usually stable between 10 and 70 mW):") # Request target optical power
testLaser.SetTargetOpticalPower(powpow)

print("")
while True:
	
	testLaser.OptimizeOpticalPower() # Optimize Laser Power

	print("Optimization cycle over \n \n")
