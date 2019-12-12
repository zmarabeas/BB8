#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <RF24.h>
#include <printf.h>
#include <PID_v1.h>

/*
TODO:
	1. check if printf lag
	2. servo speed
	3. servo mixing
	4. side motor limit
	5. servo limit
	6. imu
	7. side to side pot
*/
#define NECK_SERVO_LEFT 9
#define NECK_SERVO_RIGHT 10
#define HEAD_SERVO 3
#define DRIVE_MOTOR 6
#define SIDE_MOTOR 5
#define DEADBAND 5

const byte address[6] = "00001";

typedef struct inputs{
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	byte rsw;
	byte lsw;
}inputs;

RF24 radio(7, 8);
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo neck_left;
Servo neck_right;
Servo head_spin;
Servo drive;
Servo side;

void setup() {
	Serial.begin(9600);
	Serial.println("begin");
	printf_begin();
	radio.begin();
	radio.setRetries(15,15);
	radio.setPALevel(RF24_PA_MAX);
	radio.openReadingPipe(1, address);
	radio.startListening();
	radio.printDetails();

	/*
	while(!bno.begin()){
		Serial.println("BNO055 not detected.");
		delay(1000);
	}
	bno.setExtCrystalUse(true);
*/

	pinMode(HEAD_SERVO, OUTPUT);
	pinMode(DRIVE_MOTOR, OUTPUT);
	pinMode(SIDE_MOTOR, OUTPUT);
	pinMode(NECK_SERVO_LEFT, OUTPUT);
	pinMode(NECK_SERVO_RIGHT, OUTPUT);

	head_spin.attach(HEAD_SERVO);
	drive.attach(DRIVE_MOTOR);
	side.attach(SIDE_MOTOR);
	neck_left.attach(NECK_SERVO_LEFT);
	neck_right.attach(NECK_SERVO_RIGHT);

	head_spin.write(90);
	drive.write(90);
	side.write(90);
	neck_left.write(90);
	neck_right.write(90);
}

unsigned long lastReceived;
void loop() {
	if((millis() - lastReceived) > 200){
		head_spin.write(90);
		drive.write(90);
		side.write(90);
		neck_left.write(90);
		neck_right.write(90);
		Serial.println("no controller detected");	
	}
	//sensors_event_t event;
	//bno.getEvent(&event);
	if(radio.available()){
		int payload_size = radio.getDynamicPayloadSize();
		if(payload_size > 1){
			lastReceived = millis();
			/*
			Serial.println("packet");
			Serial.println("------------------");
*/
			inputs i;
			radio.read(&i, payload_size);
			//printInputs(i);

			//head spin
			uint8_t spin;
			if(i.lsw == 1 && i.rsw == 1)
				spin = 90;
			else if(i.lsw == 1)
				spin = 0;
			else if(i.rsw == 1)
				spin = 180; 
			else
				spin = 90;
			head_spin.write(spin);
			//Serial.println(spin);

			int driveSpeed = map(i.ly, 0,180,1000,2000);
			drive.writeMicroseconds(deadband(driveSpeed, 1500, DEADBAND));
		//	Serial.print(driveSpeed);
		//	Serial.print(" ");
			int sideSpeed = map(i.lx, 0,180,1000,2000);
			side.writeMicroseconds(deadband(sideSpeed, 1500, DEADBAND));
	//		Serial.println(sideSpeed);
			//neck servos

			Serial.print(i.ry);
			Serial.print(" ");
			Serial.print(i.rx);
			Serial.print(" ");
			uint8_t left = ((i.ry - 90) + (i.rx - 90))/2;
			left += 90;
			left = constrain(left, 0, 180);
			Serial.print(left);
			uint8_t right = ((i.ry - 90) - (i.rx - 90))/2;
			Serial.print(" ");
			right += 90;
			right = constrain(right, 0, 180);

			Serial.println(right);
			neck_left.write(deadband(left, 90, DEADBAND));
			neck_right.write(deadband(right, 90, DEADBAND));


		}
	}
}

int deadband(int data, int midpoint, int range){
  if(data > midpoint && data < midpoint + range)
    return midpoint;
  else if(data < midpoint && data > midpoint - range)
    return midpoint;
  else
    return data;
}

void printInputs(inputs i){
	printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
		\n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
		(i.rsw == 1) ? "pressed" : "released");
}
