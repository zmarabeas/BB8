#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

#define SERVO_PIN 5
#define SAMPLE_RATE 50
#define SERVO_SPEED 100

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo s;


double setpoint, input, output;
double Kp = 1, Ki = 0, Kd = .02;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
unsigned long currentTime;
unsigned long servoTime;

void setup() {
	Serial.begin(9600);
	pinMode(SERVO_PIN, OUTPUT);
	s.attach(SERVO_PIN);

	/*
	while(!bno.begin()){
		Serial.println("BNO055 not detected.");
		delay(1000);
	}
	bno.setExtCrystalUse(true);
*/
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(-255,255);
	myPID.SetSampleTime(20);

	s.write(90);

	setpoint = 0;
}

void loop() {
	sensors_event_t event;
	bno.getEvent(&event);
	//input = event.orientation.y;
	input = analogRead(2);
	input = map(input, 0, 1024, -255, 255);
	input = constrain(input, -45, 45);
	myPID.Compute();

	Serial.print("input: ");
	Serial.print(input);
	Serial.print(" output: ");
	output = map(output, -135, 135, 40, 140);
	if(output > 90){
		output += 10;
	}
	//output = constrain(output, 30, 150);
	Serial.println(output);
	//s.write(output);
}
