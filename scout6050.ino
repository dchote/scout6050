#define __PROG_TYPES_COMPAT__

#include <Pinoccio.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <aJSON.h>

WIFI_PROFILE profile = {
                /* SSID */ "",
 /* WPA/WPA2 passphrase */ "",
          /* IP address */ "",
         /* subnet mask */ "",
          /* Gateway IP */ "" };

IPAddress server(172,20,0,250);

PinoccioWifiClient wifiClient;
mqttClient mqtt(server, 1883, callback, wifiClient);

static SYS_Timer_t appTimer;

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

bool streamEnabled = true;
bool positionChanged = false;


// MPU 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


void setup() {
	Serial.begin(115200);
	
	Serial.println("Initializing...");
	
	Pinoccio.init();
	Wire.begin();
	mpu.initialize();
	
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		mpu.setDMPEnabled(true);
		
		attachInterrupt(0, dmpDataReady, RISING);
		
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
		
		Serial.println("MPU OK! Starting Wifi...");
		
		Wifi.begin(&profile);
		
		appTimer.interval = 10000;
		appTimer.mode = SYS_TIMER_PERIODIC_MODE;
		appTimer.handler = appTimerHandler;
		SYS_TimerStart(&appTimer);
		
		Serial.println("Connecting MQTT...");
		
		if (mqtt.connect("pinoccio", "username", "password")) {
			Serial.println("MQTT Connected!");
		    mqtt.subscribe("dchote/scout6050-control");
			RgbLed.blinkCyan(200);
		}
	} else {
		Serial.println("MPU Failed!");
	}
	
	RgbLed.blinkRed(200);
}



void loop() {	
	Pinoccio.loop();
	mqtt.loop();

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
			positionChanged = true;
		}
		
		if (rangle[1] != cangle[1]) {
			cangle[1] = rangle[1];
			positionChanged = true;
		}
		
		if (rangle[2] != cangle[2]) {
			cangle[2] = rangle[2];
			positionChanged = true;
		}
		
	}
	
	
	
	if (positionChanged) {
		positionChanged = false;
		/*
		Serial.print("y: ");
		Serial.print(rangle[0]);
		Serial.print(" p: ");
		Serial.print(rangle[1]);
		Serial.print(" r: ");
		Serial.println(rangle[2]);
		*/
		// stream changed data
		if ((streamEnabled && mqtt.connected())) {
			char* string = jsonPosition();
			mqtt.publish("dchote/scout6050", string);
			free(string);		
		}
	}
	
	
}

static void appTimerHandler(SYS_Timer_t *timer) {	
	/*
	if (mqtt.connected()) {
		RgbLed.blinkCyan(200);
	} else {
		RgbLed.blinkGreen(200);
	}

	// Send health
	char* string = jsonHealth();
	mqtt.publish("dchote/scout6050", string);
	free(string);
	*/
	RgbLed.blinkGreen(200);
}

void callback(char* topic, byte* payload, unsigned int length) {
	return;
	
	/*
	
	callbacks are hosed... will try again later
	
	*/
	
		
	byte* payloadCopy = (byte*)malloc(length);
	memcpy(payloadCopy, payload, length);
	
	char buffer[length];
	int i;

	for (i = 0; i < length; i++) {
		buffer[i] = payloadCopy[i];
	}
	buffer[i] = '\0';
	
	
	mqtt.publish("dchote/scout6050", "shit");
	
	free(payloadCopy);
	
	return;
	aJsonObject* command = aJson.parse(buffer);
	
	if (command == NULL) {
		Serial.println("Failed to parse command packet");
		return;
	}
	
	aJsonObject* commandPayload = aJson.getObjectItem(command, "payload");
	
	if (commandPayload == NULL) {
		Serial.println("Failed to retrieve command packet payload type");
		return;
	}
	
	String payloadType = String(commandPayload->valuestring);
	aJson.deleteItem(commandPayload);
	aJson.deleteItem(command);
		
	Serial.print("Payload Type: ");
	Serial.println(payloadType);
	
	if (payloadType == "startStream") {
		Serial.println("Starting stream");
		streamEnabled = true;
	} else if (payloadType == "stopStream") {
		Serial.println("Stopping stream");
		streamEnabled = false;
	} else if (payloadType == "health") {
		Serial.println("Health request");
		mqtt.publish("dchote/scout6050", jsonHealth());
	} else if (payloadType == "ping") {
		Serial.println("Ping");
		
		/*
		aJsonObject* timestampObject = aJson.getObjectItem(root, "timestamp");
		
		if (timestampObject != NULL) {
			int timestamp = timestampObject->valueint;
			mqtt.publish("dchote/scout6050", jsonPong(timestamp));
		}
		*/
	}
	
	
	//free(payloadCopy);
	
}

char* jsonPosition() {
	aJsonObject* root = aJson.createObject();
	
	if (root == NULL) {
		Serial.println("Failed to create jsonPosition packet");
		return "error";
	}
	
	aJson.addNumberToObject(root, "y", rangle[0]);
	aJson.addNumberToObject(root, "p", rangle[1]);
	aJson.addNumberToObject(root, "r", rangle[2]);
	
	char* string = aJson.print(root);
	
	aJson.deleteItem(root);
	
	return string;
}

char* jsonHealth() {
	aJsonObject* root = aJson.createObject();
	
	if (root == NULL) {
		Serial.println("Failed to create jsonHealth packet");
		return "error";
	}
	
	aJson.addStringToObject(root, "payload", "health");
	
	char voltage[7];
	char temperature[7];
	
	dtostrf(Pinoccio.getBatteryVoltage(), 7, 2, voltage);
    dtostrf(Pinoccio.getTemperature(), 7, 2, temperature);
	
	aJson.addStringToObject(root, "batteryVoltage", voltage);
	aJson.addStringToObject(root, "temperature", temperature);
	
	char* string = aJson.print(root);
	
	aJson.deleteItem(root);
	
	return string;
}

char* jsonPong(int timestamp) {
	aJsonObject* root = aJson.createObject();
	
	if (root == NULL) {
		Serial.println("Failed to create jsonPong packet");
		return "error";
	}
	
	aJson.addStringToObject(root, "payload", "pong");
	aJson.addNumberToObject(root, "timestamp", timestamp);
	
	char* string = aJson.print(root);
	
	aJson.deleteItem(root);
	
	return string;
}
