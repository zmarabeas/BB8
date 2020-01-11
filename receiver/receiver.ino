#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <Servo.h>
#include <Adafruit_TiCoServo.h>
#include <RF24.h>
#include <printf.h>
#include <Ramp.h>
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
#define NECK_SERVO_LEFT 11
#define NECK_SERVO_RIGHT 12
#define HEAD_SERVO 3
#define DRIVE_MOTOR 6
#define SIDE_MOTOR 5
#define POT_PIN 2

#define DRIVE_DIR 36
#define SIDE_DIR 40

#define DEADBAND 50
#define FILTER 9
#define TIMEOUT 500

#define LEFT_START 1425
#define RIGHT_START 1492

#define NECK_SPEED 1000

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

double Kpd = 3, Kid = 0, Kdd = .02;
double Kps1 = 3, Kis1 = 0, Kds1 = .02;
double Kps2 = 1, Kis2 = 0, Kds2 = 0.02;

bool newLeft, newRight;
int prevLeft, prevRight;
int outLeft, outRight;
int driveSpeed, sideSpeed;

rampInt rightRamp;
rampInt leftRamp;
RF24 radio(7, 8);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

PID drive_PID(&input_d, &output_d, &setpoint_d, Kpd, Kid, Kdd, DIRECT);
PID side_PID_1(&input_s1, &output_s1, &setpoint_s1, Kps1, Kis1, Kds1, DIRECT);
PID side_PID_2(&input_s2, &output_s2, &setpoint_s2, Kps2, Kis2, Kds2, DIRECT);

Adafruit_TiCoServo neck_left;
Adafruit_TiCoServo neck_right;
Adafruit_TiCoServo head_spin;
Adafruit_TiCoServo drive;
Adafruit_TiCoServo side;

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
	
	while(!bno.begin()){
		Serial.println("BNO055 not detected.");
		delay(1000);
	}
	bno.setExtCrystalUse(true);

	drive_PID.SetMode(AUTOMATIC);
	drive_PID.SetOutputLimits(-90,90);
	drive_PID.SetSampleTime(20);

  side_PID_1.SetMode(AUTOMATIC);
  side_PID_1.SetOutputLimits(-90,90);
  side_PID_1.SetSampleTime(5);

  side_PID_2.SetMode(AUTOMATIC);
  side_PID_2.SetOutputLimits(-90, 90);
  side_PID_2.SetSampleTime(5);

  pinMode(SIDE_MOTOR, OUTPUT);
  pinMode(SIDE_DIR, OUTPUT);	
	pinMode(DRIVE_MOTOR, OUTPUT);
  pinMode(DRIVE_DIR, OUTPUT);
	pinMode(NECK_SERVO_LEFT, OUTPUT);
	pinMode(NECK_SERVO_RIGHT, OUTPUT);
  pinMode(HEAD_SERVO, OUTPUT);

	head_spin.attach(HEAD_SERVO);
	drive.attach(DRIVE_MOTOR);
	neck_left.attach(NECK_SERVO_LEFT);
	neck_right.attach(NECK_SERVO_RIGHT);
  side.attach(SIDE_MOTOR);
  
  side.write(1500);
  head_spin.write(1500);
	drive.write(1500);
	neck_right.writeMicroseconds(LEFT_START);
  neck_left.writeMicroseconds(RIGHT_START);
	

}

int startTime = millis();
bool started = false;
unsigned long lastReceived;
long leftFiltered, rightFiltered;
void loop() {
	if((millis() - lastReceived) > TIMEOUT){
		
		head_spin.write(1500);
		drive.write(1500);
		side.write(1500);
		neck_left.write(1500);
		neck_right.write(1500);
		Serial.println("no controller detected");	
	}
	if(radio.available()){
		int payload_size = radio.getDynamicPayloadSize();
		if(payload_size > 1){
			lastReceived = millis();
			inputs i;
			radio.read(&i, payload_size);

      sensors_event_t event;
      bno.getEvent(&event);
      //drive axis
      int lx = map(i.lx, 0,180, -90, 90);
      input_d = event.orientation.y*-1 + lx;
      setpoint_d = 0;
      drive_PID.Compute();

      //imu
      input_s1 = (event.orientation.z+13);
      setpoint_s1 = map(i.ly, 0, 180, -90, 90)*-1;		
      side_PID_1.Compute();	

      //potentiometer
			//setpoint_s2 = i.ly;      
      setpoint_s2 = output_s1;
      setpoint_s2 = constrain(setpoint_s2, -90, 90);
      int sidePot = analogRead(POT_PIN);
      input_s2 = map(sidePot, 0, 1023, -255, 255)+21;
      input_s2 = constrain(input_s2, -90, 90);

			side_PID_2.Compute();
			output_s2 = map(output_s2, -90, 90, 1000, 2000);
			output_s2 = constrain(output_s2, 1000, 2000);
		
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

			driveSpeed = map(output_d, -90, 90,1000,2000);	
      //Serial.print(driveSpeed);
      if(driveSpeed < 1520){
          driveSpeed-=100;
      }else if(driveSpeed > 1550){
          driveSpeed+=150;
      }
      driveSpeed = constrain(driveSpeed, 1000, 2000);
			sideSpeed = constrain(output_s2, 1000, 2000);
      //Serial.print(" ");
      //Serial.println(driveSpeed);

      //printSensorData(event, input_s2, output_s1, output_s2);
			
			int left = ((i.ry - 90) + (i.rx - 90));
			left += 90;
      
      left = map(left, 0,180, 650,2300);
      if(prevLeft != left){
          newLeft = true;
        }
      prevLeft = left;

      if(newLeft){
          leftRamp.go(left, NECK_SPEED, LINEAR, ONCEFORWARD);
          newLeft = false;
        }
			int right = ((i.ry - 90) - (i.rx - 90));
			right += 90;
      right = map(right, 0,180, 650,2350);
      if(prevRight != right){
          newRight = true;
        }
      prevRight = right;

      if(newRight){
          rightRamp.go(right, NECK_SPEED, LINEAR, ONCEFORWARD);
          newRight = false;
        }
      
      outLeft = leftRamp.update();
      outRight = rightRamp.update();
      if(started){
			  neck_left.writeMicroseconds(deadband(outLeft, 1500, DEADBAND));
			  neck_right.writeMicroseconds(deadband(outRight, 1500, DEADBAND));
        side.writeMicroseconds(deadband(output_s2, 1500, DEADBAND));
        drive.writeMicroseconds(deadband(driveSpeed, 1500, DEADBAND));
      }

      if(millis()>startTime+5000){
          started = true;
        }

		}
	}
 delay(5);
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

void printSensorData(sensors_event_t event, int sidePot, int out1, int out2){
    Serial.print("drive axis: ");
    Serial.print(event.orientation.y);
    Serial.print(" side axis: ");
    Serial.print(event.orientation.z+13);
    Serial.print(" side pot: ");
    Serial.print(sidePot);
    Serial.print(" output1 : ");
    Serial.print(out1);
    Serial.print(" output2 : ");
    Serial.println(out2);
  }


void printInputs(inputs i){
	printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
		\n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
		(i.rsw == 1) ? "pressed" : "released");
}
