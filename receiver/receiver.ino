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
#define DRIVE_MOTOR 5
#define SIDE_MOTOR 6 

#define POT_PIN 2

#define DEADBAND 100
#define FILTER 9
#define TIMEOUT 500

#define LEFT_START 1425
#define RIGHT_START 1492

const byte address[6] = "00001";

typedef struct inputs{
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	byte rsw;
	byte lsw;
}inputs;

double setpoint_d, input_d, output_d;
double setpoint_s1, input_s1, output_s1;
double setpoint_s2, input_s2, output_s2;

double Kpd = 1, Kid = 0, Kdd = .02;
double Kps1 = 1, Kis1 = 0, Kds1 = .02;
double Kps2 = 4, Kis2 = 0, Kds2 = 0.02;

RF24 radio(7, 8);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

PID drive_PID(&input_d, &output_d, &setpoint_d, Kpd, Kid, Kdd, DIRECT);
PID side_PID_1(&input_s1, &output_s1, &setpoint_s1, Kps1, Kis1, Kds1, DIRECT);
PID side_PID_2(&input_s2, &output_s2, &setpoint_s2, Kps2, Kis2, Kds2, DIRECT);

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
	side_PID_1.SetMode(AUTOMATIC);
	side_PID_1.SetOutputLimits(0,180);
	side_PID_1.SetSampleTime(20);
*/

	side_PID_2.SetMode(AUTOMATIC);
	side_PID_2.SetOutputLimits(-1000, 1000);
	side_PID_2.SetSampleTime(20);

	pinMode(SIDE_MOTOR, OUTPUT);
	side.attach(SIDE_MOTOR);
	side.write(90);

	
	while(!bno.begin()){
		Serial.println("BNO055 not detected.");
		delay(1000);
	}
	bno.setExtCrystalUse(true);
/*
	drive_PID.setMode(AUTOMATIC);
	drive_PID.setOutputLimits(0,180);
	drive_PID.SetSampleTime(20);
*/
	pinMode(HEAD_SERVO, OUTPUT);
	pinMode(DRIVE_MOTOR, OUTPUT);
	pinMode(NECK_SERVO_LEFT, OUTPUT);
	pinMode(NECK_SERVO_RIGHT, OUTPUT);

	head_spin.attach(HEAD_SERVO);
	drive.attach(DRIVE_MOTOR);
	neck_left.attach(NECK_SERVO_LEFT);
	neck_right.attach(NECK_SERVO_RIGHT);

	head_spin.write(90);
	drive.write(90);
	neck_left.writeMicroseconds(LEFT_START);
	neck_right.writeMicroseconds(RIGHT_START);

}

unsigned long lastReceived;
long leftFiltered, rightFiltered;
void loop() {
	if((millis() - lastReceived) > TIMEOUT){
		/*
		head_spin.write(90);
		drive.write(90);
		side.write(90);
		neck_left.write(90);
		neck_right.write(90);
*/
		Serial.println("no controller detected");	
	}
	if(radio.available()){
		int payload_size = radio.getDynamicPayloadSize();
		if(payload_size > 1){
			lastReceived = millis();
			inputs i;
			radio.read(&i, payload_size);
			//printInputs(i);
			int sidePot = analogRead(POT_PIN);
			input_s2 = map(sidePot, 0, 1023, -1000, 1000) - 100;
			//input_s2 = constrain((input_s2-5), -25, 25);
			setpoint_s2 = i.lx;
			setpoint_s2 = map(i.lx, 0, 180, -300, 300);
			//Serial.print("input: ");
			//setpoint_s2 = 0;
			//Serial.print(input_s2);
			//Serial.print(" setpoint : ");
			//Serial.print(setpoint_s2);
			side_PID_2.Compute();
			output_s2 = map(output_s2, -1000, 1000, 1000, 2000);
			if(output_s2 > 1520){
				output_s2 += 200;
			}
			output_s2 = constrain(output_s2, 1000, 2000);

			//Serial.print(" output : ");
			//Serial.println(output_s2);
			//side.writeMicroseconds(output_s2);



			
			//head spin
			int spin;
			if(i.lsw == 1 && i.rsw == 1)
				spin = 1500;
			else if(i.lsw == 1)
				spin = 1000;
			else if(i.rsw == 1)
				spin = 2000; 
			else
				spin = 1500;
      //Serial.println(spin);
			head_spin.writeMicroseconds(spin);


			sensors_event_t event;
			bno.getEvent(&event);
      //Serial.print("drive axis: ");
      //Serial.print(event.orientation.y);
      //Serial.print(" side axis: ");
      //Serial.print(event.orientation.z);
	//		not sure which axis this is going to be
			//input = event.orientation.x;
			//int8_t ly = map(i.ly, 0,180,-30,30);
			//setpoint = 90 + ly;
			//drive_PID.comptue();
			//Serial.print("output: ");
			//Serial.println(output);

			//drive.write(deadband(output, 90, DEADBAND));

			int driveSpeed = map(i.ly, 0,180,1000,2000);
			drive.writeMicroseconds(deadband(driveSpeed, 1500, DEADBAND));
			int sideSpeed = map(i.lx, 0,180,1000,2000);
			side.writeMicroseconds(deadband(sideSpeed, 1500, DEADBAND));

			//neck servos
			int left = ((i.ry - 90) + (i.rx - 90));
			left += 90;
      
      left = map(left, 0,180, 550,2400);
			int right = ((i.ry - 90) - (i.rx - 90));
			right += 90;
      right = map(right, 0,180, 550,2450);
      
      leftFiltered = constrain(filter(left, leftFiltered, FILTER), 900, 2000);
      rightFiltered = constrain(filter(right, rightFiltered, FILTER), 900, 2000);

			neck_left.writeMicroseconds(deadband(leftFiltered, 1500, DEADBAND));
			neck_right.writeMicroseconds(deadband(rightFiltered, 1500, DEADBAND));
      //Serial.print(" prev: ");
      //Serial.print(left);
      //Serial.print(" rightPrev: ");
      //Serial.print(right); 
      //Serial.print(" right: ");
      //Serial.print(rightFiltered);
      //Serial.print(" left: ");
      //Serial.println(leftFiltered);


		}
	}
}

int filter(int input, int current, int filter){
    return (input + (current * filter))/(filter+1);
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
