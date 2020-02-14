#include <LiquidCrystal.h>
#include <RF24.h>
#include <printf.h>

//JOYSTICKS
#define BL_Y 1
#define BL_X 0
#define BL_B 10
#define TL_Y 2
#define TL_X 3
#define TL_B 9
#define BR_Y 4
#define BR_X 5
#define BR_B 23
#define TR_Y 6
#define TR_X 7
#define TR_B 25

//BUTTONS
#define GREEN_RIGHT 39
#define GREEN_RIGHT 41
#define RED_RIGHT 43
#define RED_LEFT 45
#define BLACK_RIGHT 47
#define BLACK_LEFT 49
#define BLUE_LEFT 27
#define BLUE_RIGHT 29
#define WHITE_LEFT 31
#define WHITE_RIGHT 33
#define YELLOW_LEFT 35
#define YELLOW_RIGHT 37

//SWITCHES
#define TOP_L 48
#define TOP_M 46
#define TOP_R 44
#define MID_L 42
#define MID_M 40
#define MID_R 38
#define BOT_L_UP 34
#define BOT_L_DOWN 32
#define BOT_M_UP 30
#define BOT_M_DOWN 26
#define BOT_R_UP 24
#define BOT_R_DOWN 22

//POTS
#define LEFT_POT 9
#define RIGHT_POT 8 //inverted for some reason

#define BUTTON_DELAY 200

const byte address[6] = "00BB8";

typedef struct inputs{
  float lx;
  float ly;
  float rx;
  float ry;
  byte rsw;
  byte lsw;
}inputs;

inputs i;
inputs filtered_i;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
RF24 radio(7,8);

double filter_left, filter_right = 0;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Address: ");
  lcd.print((char*)address);
  analogWrite(6, 100); //contrast for lcd

  pinMode(BLUE_LEFT, INPUT_PULLUP);
  pinMode(BLUE_RIGHT, INPUT_PULLUP);
  Serial.println(sizeof(inputs));
  radio.begin();
  radio.setRetries(15,15);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.stopListening();
  //radio.printDetails();

}

void loop() {
    lcd.setCursor(0, 1);
    filter_left = remap(analogRead(LEFT_POT), 0, 1023, 0.0, 1.0);
    filter_right = remap(analogRead(RIGHT_POT), 0, 1023, 1.0, 0.0);
    lcd.print("L: ");
    lcd.print(filter_left);
    lcd.print(" R: ");
    lcd.print(filter_right);    
    i.lx = map(analogRead(BL_X), 0, 1024, 0, 180);
    i.ly = map(analogRead(BL_Y), 0, 1024, 0, 180);
    i.rx = map(analogRead(BR_X), 0, 1024, 0, 180);
    i.ry = map(analogRead(BR_Y), 0, 1024, 0, 180);
    
    filtered_i.lsw = (digitalRead(BLUE_LEFT)) ? 0 : 1;
    filtered_i.rsw = (digitalRead(BLUE_RIGHT)) ? 0 : 1;
    filter(0, (double)filter_left, &i, &filtered_i);
    filter(1, (double)filter_right, &i, &filtered_i);
    radio.write(&filtered_i, sizeof(inputs));
    
//    Serial.print(i.ry);
//    Serial.print(" ");
//    Serial.print(i.lx);
//    Serial.print(" ");
//    Serial.print(i.ly);
//    Serial.print(" ");
//    Serial.print(i.rx);
//    Serial.print(" ");    
//    Serial.print(filtered_i.lx);
//    Serial.print(" ");
//    Serial.print(filtered_i.ly);
//    Serial.print(" ");
//    Serial.print(filtered_i.rx);
//    Serial.print(" ");
//    Serial.print(filtered_i.ry);
//    Serial.println(" ");
    
}

void printInputs(inputs i){
  printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
    \n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
    (i.rsw == 1) ? "pressed" : "released");
}

int filter(bool side, double tf, inputs *curr_inputs, inputs *prev_inputs){
  //(1-a)*filtered + a*current
  double ts = 1.0/100.0;
  double a = ts/(tf+ts);
  if(side == 0){
    prev_inputs->lx = ((1.0-a)*prev_inputs->lx + a*curr_inputs->lx);
    prev_inputs->ly = ((1.0-a)*prev_inputs->ly + a*curr_inputs->ly);
  }else{
    prev_inputs->rx = ((1.0-a)*prev_inputs->rx + a*curr_inputs->rx);
    prev_inputs->ry = ((1.0-a)*prev_inputs->ry + a*curr_inputs->ry);
  }
}

double remap(double s, double a1, double a2, double b1, double b2){
    return b1 + (s-a1)*(b2-b1)/(a2-a1);
}
  
