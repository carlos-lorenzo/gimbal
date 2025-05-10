#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

void SetOffsets(MPU6050 mpu, int TheOffsets[6]) {
    const int iAx = 0;
    const int iAy = 1;
    const int iAz = 2;
    const int iGx = 3;  
    const int iGy = 4;
    const int iGz = 5;

	mpu.setXAccelOffset(TheOffsets[iAx]);
	mpu.setYAccelOffset(TheOffsets[iAy]);
	mpu.setZAccelOffset(TheOffsets[iAz]);
	mpu.setXGyroOffset(TheOffsets[iGx]);
	mpu.setYGyroOffset(TheOffsets[iGy]);
	mpu.setZGyroOffset(TheOffsets[iGz]);
}

/*Initializate function*/
void Initialize(MPU6050 mpu) {
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
	Serial.begin(9600);
	
    // Init the module
	Serial.println("Initializing MPU...");
	mpu.initialize();
	Serial.println("MPU initializated");
	// Check module connection
	Serial.println("Testing device connections...");
	if(mpu.testConnection() ==  false){
		Serial.println("MPU6050 connection failed");
	    while(true);
	}
	else{
		Serial.println("MPU6050 connection successful");
	}


	
}

float tapear(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

