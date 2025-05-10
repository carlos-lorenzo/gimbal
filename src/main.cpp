#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Arduino.h"
#include "utils.h"
#include "Servo.h"

void display_cumulative_angle();

MPU6050 mpu;
Servo z_servo;
Servo y_servo;
Servo x_servo;

int Calibrated[6] = {1321, 1105, 1527, 97, 21, 47};
const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

#define USE_DMP

uint8_t MPUIntStatus;
uint16_t packetSize;
uint8_t devStatus;
uint8_t FIFOBuffer[64];

bool blinkState;



/*---Orientation/Motion Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

uint16_t targetAngle = 90;

bool motorsTriggered = true;

float sensorTargetAngleY;
float sensorTargetAngleX;
float sensorTargetAngleZ;

float currentAngleX;
float prevAngleX;
float currentAngleY;
float prevAngleY;

float currentAngleZ;
float prevAngleZ;
float prevRawAngleZ = 0;


float cumulativeAngleZ = targetAngle;
float prevCumulativeAngleZ = targetAngle;
float prevZWrittenAngle = targetAngle;
float angleZToWrite = targetAngle;

float cumulativeAngleX = targetAngle;
float cumulativeAngleY = targetAngle;
int BUTTON_PIN = 6;
byte buttonState;

int x_index = 0;
int y_index = 1; //1
int z_index = 1; // 2

void setup() {

	Initialize(mpu);
	SetOffsets(mpu, Calibrated);



	Serial.println(F("Initializing DMP..."));
  	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(Calibrated[iGx]);
	mpu.setYGyroOffset(Calibrated[iGy]);
	mpu.setZGyroOffset(Calibrated[iGz]);

	Serial.println("These are the Active offsets: ");
	mpu.PrintActiveOffsets();

	Serial.println(F("Enabling DMP..."));   //Turning ON DMP
	mpu.setDMPEnabled(true);

	/*Enable Arduino interrupt detection*/
	Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
	Serial.println(F(")..."));
	MPUIntStatus = mpu.getIntStatus();

	/* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
	Serial.println(F("DMP ready! Waiting for first interrupt..."));
	packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  	Serial.println(F("Done :) "));

    z_servo.attach(9);
	// y_servo.attach(5);
	// x_servo.attach(3);
	pinMode(BUTTON_PIN, INPUT_PULLUP);


    z_servo.write(targetAngle);
	// y_servo.write(targetAngle);
	// x_servo.write(targetAngle);

}

void display_cumulative_angle() {

	// Serial.print("X: ");
	// Serial.print(cumulativeAngleX);
	// Serial.print("\t");
	// Serial.print("Y: ");
	// Serial.print(cumulativeAngleY);
	// Serial.print("\t");

	Serial.print("Raw angle: ");
	Serial.print(ypr[z_index] * 180/M_PI);
	Serial.print("\t");

	Serial.print("Prev angle: ");
	Serial.print(prevAngleZ);
	Serial.print("\t");

	
	Serial.print("Current angle: ");
	Serial.print(currentAngleZ);
	Serial.print("\t");

	Serial.print("Difference: ");
	Serial.print(currentAngleZ - prevAngleZ);
	Serial.print("\t");

	Serial.print("Cum Z: ");
	Serial.println(cumulativeAngleZ);
}


void loop() {




	#ifdef USE_DMP
		prevAngleX = currentAngleX;
		prevAngleY = currentAngleY;
		prevAngleZ = currentAngleZ;

		prevCumulativeAngleZ = cumulativeAngleZ;

		if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
			
			mpu.dmpGetQuaternion(&q, FIFOBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			#ifdef OUTPUT_READABLE_YAWPITCHROLL
				/* Display Euler angles in degrees */
				// Serial.print("ypr\t");
				// Serial.print(ypr[x_index] * 180/M_PI);
				// Serial.print("\t");
				// Serial.print(ypr[y_index] * 180/M_PI);
				// Serial.print("Raw angle: ");
				// Serial.print(ypr[z_index] * 180/M_PI);
				// Serial.print("\t");
			#endif

			currentAngleX = ypr[x_index] * 180/M_PI;
			currentAngleY = ypr[y_index] * 180/M_PI;
			currentAngleZ = ypr[z_index] * 180/M_PI;
		}

		if (currentAngleZ < -90 && prevRawAngleZ > 90) {
			currentAngleZ = 360 - abs(currentAngleZ);
		} else {
			cumulativeAngleZ += currentAngleZ - prevAngleZ;
		}
		
		display_cumulative_angle();
		//cumulativeAngleX += currentAngleX - prevAngleX;
		//cumulativeAngleY += currentAngleY - prevAngleY;
		
		prevRawAngleZ = ypr[z_index] * 180/M_PI;
		

		// Button logic
		buttonState = digitalRead(BUTTON_PIN);
		if (buttonState == LOW) {
			//motorsTriggered = !motorsTriggered;
		}

		if (motorsTriggered) {
			// angleXToWrite = tapear(ypr[x_index] * 180/M_PI , sensorTargetAngleX - 90, sensorTargetAngleX + 90, 0, 180);
			// x_servo.write(angleXToWrite);

			// angleYToWrite = tapear(ypr[y_index] * 180/M_PI , sensorTargetAngleY - 90, sensorTargetAngleY + 90, 0, 180);
			// y_servo.write(angleYToWrite);
			angleZToWrite = prevZWrittenAngle - cumulativeAngleZ + prevCumulativeAngleZ;
			z_servo.write(angleZToWrite);
			prevZWrittenAngle = angleZToWrite;
			
		} else {
			z_servo.write(targetAngle);
			y_servo.write(targetAngle);
			x_servo.write(targetAngle);
		}
		delay(15);
	#endif
}




