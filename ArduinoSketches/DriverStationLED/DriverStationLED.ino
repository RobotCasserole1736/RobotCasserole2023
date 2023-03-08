#include <FastLED.h>

#define LED_STRIP_1_PIN   5 // Subject to change
#define LED_STRIP_2_PIN   6 // Subject to change
#define NUM_LEDS          30 // Subject to change
#define BRIGHTNESS        40
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB
#define UPDATE_RATE_HZ 25
CRGB ledStrip1[NUM_LEDS];
CRGB ledStrip2[NUM_LEDS];

void setup() {
  // Configure the fast LED Library for our display
    FastLED.addLeds<LED_TYPE, LED_STRIP_1_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_STRIP_2_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    //NOTE: Might use something like this to switch between red and blue
    // Configure the roboRIO communication pin to recieve data
    // pinMode(CMD_INPUT_PIN, INPUT);

    // Initialize the FastLED display buffer array  to blank out the display
    for (uint8_t i = 0; i<NUM_LEDS; i++){
      ledStrip1[i] = CRGB(0,0,0);
      ledStrip2[i] = CRGB(0,0,0);
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  red();
  FastLED.delay(1000 / UPDATE_RATE_HZ);
}

void red() {
  for (uint8_t i = 0; i<NUM_LEDS; i++){
    {   
      ledStrip1[i] = CRGB(255,0,0);
      ledStrip2[i] = CRGB(255,0,0);
    }
  }
}