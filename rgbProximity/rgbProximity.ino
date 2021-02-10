/*
 * rgbProximity Sketch
 * Controlling RGB colors through proximity sensing on the TouchBaord.
 * 
 * Requirements:
 * * BareConductive Touch Board with headers
 * * 3 x promity sensors (copper tape, painted, or etc.)
 * * NeoPixels
 *    * GND to GND
 *    * Data In to pin 11
 *    * Power to 5V
 */

// compiler error handling
#include "Compiler_Errors.h"

// proximity includes and defines
#include <MPR121.h>
#include <Wire.h>
#define MPR121_ADDR 0x5C
#define MPR121_INT 4

// NeoPixel includes
#include <Adafruit_NeoPixel.h>

// proximity defines
// mapping and filter definitions
#define LOW_DIFF 0
#define HIGH_DIFF 50
#define filterWeight 0.3f // 0.0f to 1.0f - higher value = more smoothing
float lastRedProx = 0;
float lastGreenProx = 0;
float lastBlueProx = 0;

// the electrodes to monitor
#define RED_ELECTRODE 7
#define GREEN_ELECTRODE 9
#define BLUE_ELECTRODE 11

// NeoPixel defines and init
//#define NP_NUM 7 // startlight skirt
#define NP_NUM 21 // startlight skirt + MakeItWearIt Top
//#define NP_NUM 24 // ring
//#define NP_NUM 27 // Mimic Amber Top
// Audio is enabled on pin 6, so we use pin 11
#define NP_PIN 11
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NP_NUM, NP_PIN, NEO_GRB + NEO_KHZ800);
// 0 (off) to 255 (max brightness)
#define BRIGHTNESS 255


void setup() {
  // NeoPixels
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
  pixels.show();

  Serial.begin(57600);
  //while(!Serial);
  Serial.println("RGB Proximity...");
  if(!MPR121.begin(0x5C)){ 
    Serial.println("error setting up MPR121");  
    switch(MPR121.getError()){
      case NO_ERROR:
        Serial.println("no error");
        pixels.setPixelColor(2, pixels.Color(0, 255, 0)); 
        break;  
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0)); // Red
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        break;      
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        break;
      default:
        Serial.println("unknown error");
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        break;      
    }
    while(1);
  }
  // slow down some of the MPR121 baseline filtering to avoid 
  // filtering out slow hand movements
  MPR121.setRegister(MPR121_NHDF, 0x01); //noise half delta (falling)
  MPR121.setRegister(MPR121_FDLF, 0x3F); //filter delay limit (falling)
}

// Helpers
unsigned int readingFromElectrode(int electrode) {
  // read the difference between the measured baseline and the measured continuous data
  int reading = MPR121.getBaselineData(electrode)-MPR121.getFilteredData(electrode);
  
  // print out the reading value for debug
  //Serial.println(reading);
  
  // constrain the reading between our low and high mapping values
  unsigned int prox = constrain(reading, LOW_DIFF, HIGH_DIFF);
  return prox;
}

// implement a simple (IIR lowpass) smoothing filter
float smoothFilter(float lastProx, unsigned int prox) {
  return (filterWeight*lastProx) + ((1-filterWeight)*(float)prox);  
}

// map the LOW_DIFF..HIGH_DIFF range to 0..255 (8-bit resolution)
uint8_t mapToColorRange(int prox) {
  return (uint8_t)map(prox,LOW_DIFF,HIGH_DIFF,0,255);
}

void loop() {
  Serial.println("...update all of the data from the MPR121...");
  MPR121.updateAll();

  unsigned int redProx = readingFromElectrode(RED_ELECTRODE);
  Serial.println(redProx);
  lastRedProx = smoothFilter(lastRedProx, redProx);
  Serial.println(lastRedProx);
  int redVal = mapToColorRange(lastRedProx);
  Serial.println(redVal);
 
  unsigned int greenProx = readingFromElectrode(GREEN_ELECTRODE);
  lastGreenProx = smoothFilter(lastGreenProx, greenProx);
  int greenVal = mapToColorRange(lastGreenProx);
  
  unsigned int blueProx = readingFromElectrode(BLUE_ELECTRODE);
  lastBlueProx = smoothFilter(lastBlueProx, blueProx);
  int blueVal = mapToColorRange(lastBlueProx);

  Serial.println("...shifting pixels by 1...");
  int idx = 0;
  uint32_t color;
  for(int i=2;i<=NP_NUM;i++){
    idx = NP_NUM - i;
    color = pixels.getPixelColor(idx);
    pixels.setPixelColor(idx+1, color);
  }

  Serial.println("...seting new color");
  //Serial.println(redVal);
  //Serial.println(greenVal);
  //Serial.println(blueVal);
  pixels.setPixelColor(0, pixels.Color(redVal, greenVal, blueVal));
  //Serial.println("...show...");
  pixels.show();
  //Serial.println("...done...");
  delay(100);
}
