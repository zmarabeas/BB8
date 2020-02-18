#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_TiCoServo.h>
#include <RF24.h>
#include <printf.h>
#include <PID_v1.h>
#include "sounds.h"

#define NECK_SERVO_LEFT 11
#define NECK_SERVO_RIGHT 12
#define HEAD_SERVO 3
#define DRIVE_MOTOR 6
#define SIDE_MOTOR 5
#define POT_PIN 2

#define DEADBAND 50
#define FILTER 9
#define TIMEOUT 500
#define LOOP_TIME 20

#define LEFT_START 1425
#define RIGHT_START 1492
#define NECK_SPEED 1000

const byte address[6] = "00BB8";

typedef struct inputs{
  float lx;
  float ly;
  float rx;
  float ry;
  byte rsw;
  byte lsw;
  bool enable;
  byte sound;
}inputs;

int lx = 90;
int ly = 90;
int rx = 90;
int ry = 90;
bool rsw = 0;
bool lsw = 0;
bool sound;
bool enable = false;

double setpoint_d, input_d, output_d;
double setpoint_s1, input_s1, output_s1;
double setpoint_s2, input_s2, output_s2;

double Kpd = 2, Kid = 0, Kdd = .02;
double Kps1 = 3, Kis1 = 0, Kds1 = .02;
double Kps2 = 1, Kis2 = 0, Kds2 = 0.02;

PID drive_PID(&input_d, &output_d, &setpoint_d, Kpd, Kid, Kdd, DIRECT);
PID side_PID_1(&input_s1, &output_s1, &setpoint_s1, Kps1, Kis1, Kds1, DIRECT);
PID side_PID_2(&input_s2, &output_s2, &setpoint_s2, Kps2, Kis2, Kds2, DIRECT);

RF24 radio(7, 8);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Adafruit_TiCoServo neck_left;
Adafruit_TiCoServo neck_right;
Adafruit_TiCoServo head_spin;
Adafruit_TiCoServo drive;
Adafruit_TiCoServo side;

int left, right;
int driveSpeed, sideSpeed;


void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("begin");
  sendCommand(CMD_SET_VOLUME, 0x10);
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
  side_PID_1.SetSampleTime(20);

  side_PID_2.SetMode(AUTOMATIC);
  side_PID_2.SetOutputLimits(-90, 90);
  side_PID_2.SetSampleTime(20);

  pinMode(SIDE_MOTOR, OUTPUT);
  pinMode(DRIVE_MOTOR, OUTPUT);
  pinMode(NECK_SERVO_LEFT, OUTPUT);
  pinMode(NECK_SERVO_RIGHT, OUTPUT);
  pinMode(HEAD_SERVO, OUTPUT);

  side.attach(SIDE_MOTOR);
  drive.attach(DRIVE_MOTOR);
  neck_left.attach(NECK_SERVO_LEFT);
  neck_right.attach(NECK_SERVO_RIGHT);
  head_spin.attach(HEAD_SERVO);
  
  side.write(1500);
  drive.write(1500);
  neck_right.writeMicroseconds(LEFT_START);
  neck_left.writeMicroseconds(RIGHT_START);
  head_spin.write(1500);
}

unsigned long startTime = millis();
bool started = false;
unsigned long looptime = 0;
unsigned long lastReceived, currMillis, prevMillis = 0;
unsigned long lastSound=0;

void loop() {
  currMillis = millis();
  if((currMillis - lastReceived) > TIMEOUT || !enable){
    head_spin.write(1500);
    drive.write(1500);
    side.write(1500);
    neck_left.write(1500);
    neck_right.write(1500);
    if((currMillis - lastReceived) > TIMEOUT)
      Serial.println("no controller detected"); 
  }
  if(radio.available()){
    int payload_size = radio.getDynamicPayloadSize();
    if(payload_size > 1){
      lastReceived = currMillis; 
      inputs i;
      radio.read(&i, payload_size);
      lx = i.lx;
      ly = i.ly;
      rx = i.ry;
      ry = i.rx;
      ry = map(ry, 0, 180, 180, 0);
      rsw = i.rsw;
      lsw = i.lsw;
      enable = i.enable;
      sound = i.sound;
    }
  }
  
  
  if((currMillis - prevMillis) < LOOP_TIME){
//    Serial.println(currMillis - prevMillis);
  }else{
    prevMillis = currMillis;
    sensors_event_t event;
    bno.getEvent(&event);
  
    //drive axis
    driveSpeed = map(lx, 0,180, -90, 90);
    input_d = event.orientation.y*-1 + driveSpeed/3;
    setpoint_d = 0;
    drive_PID.Compute();
    driveSpeed = map(output_d, -90, 90,1000,2000);  
    if(driveSpeed < 1520){
        driveSpeed-=100;
    }else if(driveSpeed > 1550){
        driveSpeed+=150;
    }
    driveSpeed = constrain(driveSpeed, 900, 2000);
  
    //imu
    input_s1 = (event.orientation.z+13);
    setpoint_s1 = map(ly, 0, 180, -30, 30);
    side_PID_1.Compute(); 
//    Serial.print("imu: ");
//    Serial.print(input_s1);
//    Serial.print(" speed: ");
//  
    //potentiometer
    setpoint_s2 = output_s1;
    setpoint_s2 = constrain(setpoint_s2, -90, 90);
    int sidePot = analogRead(POT_PIN);
    input_s2 = map(sidePot, 0, 1023, -255, 255)+47;
    input_s2 = constrain(input_s2, -90, 90);
    side_PID_2.Compute();
    output_s2 = map(output_s2, -90, 90, 1000, 2000);
    sideSpeed = constrain(output_s2, 1000, 2000);
//    Serial.println(sideSpeed);
  
    //head spin
    int spin;
    if(lsw == 1 && rsw == 1)
      spin = 1500;
    else if(lsw == 1)
      spin = 1000;
    else if(rsw == 1)
      spin = 2000; 
    else
      spin = 1500;
    head_spin.writeMicroseconds(spin);
    
    //neck servos
    left = ((ry - 90) + (rx - 90));
    left += 90;
    left = map(left, 0,180, 650,2300);
    left = constrain(left, 650, 2300);
  
    right = ((ry - 90) - (rx - 90));
    right += 90;
    right = map(right, 0,180, 650,2300);
    right = constrain(right, 650, 2300);
  
    if(started && enable){
      neck_left.writeMicroseconds(left);
      neck_right.writeMicroseconds(right);
      side.writeMicroseconds(output_s2);
      drive.writeMicroseconds(deadband(driveSpeed, 1500, DEADBAND));
    }

    //sounds
    if(sound == 0){
      if(currMillis - lastSound > 2000){
        lastSound = currMillis;
        sendCommand(CMD_PLAY_WITHFOLDER, 0x0101);
      }
    }
    
    if(currMillis>startTime+5000){
        started = true;
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

double remap(double s, double a1, double a2, double b1, double b2){
    return b1 + (s-a1)*(b2-b1)/(a2-a1);
}


void printInputs(inputs i){
  printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
    \n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
    (i.rsw == 1) ? "pressed" : "released");
}
