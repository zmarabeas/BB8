#include <RF24.h>
#include <printf.h>
#include <FastLED.h>

#define NUM_LEDS 8
#define LED_STRIP 3
#define LED_PIN 4

RF24 radio(9, 10);

CRGB leds[NUM_LEDS];

typedef struct inputs{
  float lx;
  float ly;
  float rx;
  float ry;
  bool rsw;
  bool lsw;
  bool enable;
  bool sound;
}inputs;

const byte address[6] = "00BB8";

void setup() {
  Serial.begin(9600);
  printf_begin();

  FastLED.addLeds<NEOPIXEL, LED_STRIP>(leds, NUM_LEDS);
  defaultColor();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
//  radio.begin();
//  radio.setRetries(15,15);
//  radio.setPALevel(RF24_PA_MAX);
//  radio.openReadingPipe(1, address);
//  radio.startListening();
//  radio.printDetails();
}

void loop() {
  leds[4] = CRGB(0,0,0);
  leds[5] = CRGB(0,0,0);
  FastLED.show();
  delay(5000);
  leds[4] = CRGB(255/2,0,0);
  leds[5] = CRGB(255/2,0,0);
  FastLED.show();
  delay(10000);
  

//  if(radio.available()){
//    int payload_size = radio.getDynamicPayloadSize();
//    if(payload_size > 1){
//      inputs i;
//      radio.read(&i, payload_size);
//      Serial.print("sound: ");
//      Serial.print(i.sound);
//      Serial.print(" lx: ");
//      Serial.println(i.lx);
//    }
//  }

}

void defaultColor(){
  leds[0] = CRGB(0,0,255/2);
  leds[1] = CRGB(0,0,255/2);
  leds[2] = CRGB(255/2,255/2,255/2);
  leds[3] = CRGB(255/2,255/2,255/2);
  leds[4] = CRGB(0,0,0);
  leds[5] = CRGB(0,0,0);
  leds[6] = CRGB(120/2,120/2,120/2);
  leds[7] = CRGB(0,0,255/2);
  FastLED.show();
  }

void printInputs(inputs i){
  printf("LX: %d LY: %d RX: %d RY: %d LSW: %s RSW: %s \
    \n", i.lx, i.ly, i.rx, i.ry, (i.lsw == 1) ? "pressed" : "released", \
    (i.rsw == 1) ? "pressed" : "released");
}
