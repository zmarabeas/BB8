#include <RF24.h>
#include <printf.h>

#define LSW 2 //digital pin 2
#define LVX 1 //analog pin 1
#define LVY 0 //analog pin 0
#define RSW 3 //digital pin 3
#define RVX 3 //analog pin 3
#define RVY 2 //analog pin 2

#define BUTTON_DELAY 200

const byte address[6] = "00001";

typedef struct inputs{
  uint8_t lx;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  byte rsw;
  byte lsw;
}inputs;

inputs i;

RF24 radio(9,10);

void setup() {
  Serial.begin(9600);
  printf_begin();

  pinMode(LSW, INPUT_PULLUP);
  pinMode(RSW, INPUT_PULLUP);

  radio.begin();
  radio.setRetries(15,15);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.stopListening();
  radio.printDetails();
}
int prevX, prevY = 0;
void loop() {
  i.lsw = (digitalRead(LSW)) ? 0 : 1;
  i.rsw = (digitalRead(RSW)) ? 0 : 1;
  i.lx = map(analogRead(LVX), 0, 1024, 0, 180);
  i.ly = map(analogRead(LVY), 0, 1024, 0, 180);
  i.rx = map(analogRead(RVX), 0, 1024, 0, 180);
  i.ry = map(analogRead(RVY), 0, 1024, 0, 180);
  radio.write(&i, sizeof(inputs));
  //printInputs(i);
}

void printInputs(inputs i){
  printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
    \n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
    (i.rsw == 1) ? "pressed" : "released");
}

int filter(int input, int current, int filter){
  
  return (input + (current * filter))/(filter+1);
  }
