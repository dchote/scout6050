#define __PROG_TYPES_COMPAT__

#include <Pinoccio.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int angle[3];
int rangle[3]; 
int cangle[3]; 

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// MPU 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


void setup() {
	Wire.begin();

	Serial.begin(115200);

	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		mpu.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {/* ERROR */}

}



void loop() {
	if (!dmpReady) return; // Only Start the Programm if MPU are Ready ! 

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) { 
		mpu.resetFIFO(); 
		
		
	} else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) {
			fifoCount = mpu.getFIFOCount();
		}

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// Calculate Angle in 1/100° 
		angle[0] = (ypr[0] * 180/M_PI) * 100;
		angle[1] = (ypr[1] * 180/M_PI) * 100;
		angle[2] = (ypr[2] * 180/M_PI) * 100;

		// Calculate Angle in °
		rangle[0] = angle[0] / 100; 
		rangle[1] = angle[1] / 100;
		rangle[2] = angle[2] / 100;
		
		if (rangle[0] != cangle[0]) {
			cangle[0] = rangle[0];
			
			Serial.print("X: ");
			Serial.println(rangle[0]);
		}
		
		if (rangle[1] != cangle[1]) {
			cangle[1] = rangle[1];
			
			Serial.print("Y: ");
			Serial.println(rangle[1]);
		}
		
		if (rangle[2] != cangle[2]) {
			cangle[2] = rangle[2];
			
			Serial.print("Z: ");
			Serial.println(rangle[2]);
		}
		
	}



}

