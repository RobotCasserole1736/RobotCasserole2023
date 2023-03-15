#include <FastLED.h>

#define COLOR_SWITCH_PIN  25 // Subject to change
#define LED_STRIP_1_PIN   22 // Subject to change
#define LED_STRIP_2_PIN   23 // Subject to change
#define NUM_LEDS_STRIP_1  23 // Subject to change
#define NUM_LEDS_STRIP_2  46 // Subject to change
#define BRIGHTNESS        70
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB
#define UPDATE_RATE_HZ 25
CRGB ledStrip1[NUM_LEDS_STRIP_1];
CRGB ledStrip2[NUM_LEDS_STRIP_2];

void setup() {
  // Set up switch for switching colors
    pinMode(COLOR_SWITCH_PIN,INPUT);
  // Configure the fast LED Library for our display
    FastLED.addLeds<LED_TYPE, LED_STRIP_1_PIN, COLOR_ORDER>(ledStrip1, NUM_LEDS_STRIP_1).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_STRIP_2_PIN, COLOR_ORDER>(ledStrip2, NUM_LEDS_STRIP_2).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    //NOTE: Might use something like this to switch between red and blue
    // Configure the roboRIO communication pin to recieve data
    // pinMode(CMD_INPUT_PIN, INPUT);

    // Initialize the FastLED display buffer array  to blank out the display
    for (uint8_t i = 0; i<NUM_LEDS_STRIP_1; i++){
      ledStrip1[i] = CRGB(0,0,0);
    }
    for (uint8_t i = 0; i<NUM_LEDS_STRIP_2; i++){
      ledStrip2[i] = CRGB(0,0,0);
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(COLOR_SWITCH_PIN) == HIGH){
    red();
  } else {
    blue();
  }
  FastLED.delay(1000 / UPDATE_RATE_HZ);
}

void red() {
  for (uint8_t i = 0; i<NUM_LEDS_STRIP_1; i++){
    {   
      ledStrip1[i] = CRGB(0,255,0);
    }
  }
  for (uint8_t i = 0; i<NUM_LEDS_STRIP_2; i++){
    {   
      ledStrip2[i] = CRGB(0,255,0);
    }
  }
}

void blue() {
  for (uint8_t i = 0; i<NUM_LEDS_STRIP_1; i++){
    {   
      ledStrip1[i] = CRGB(0,0,255);
    }
  }
  for (uint8_t i = 0; i<NUM_LEDS_STRIP_2; i++){
    {   
      ledStrip2[i] = CRGB(0,0,255);
    }
  }
}