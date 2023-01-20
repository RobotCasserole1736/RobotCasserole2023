#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    256
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


void setup() {
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS ); 
    
    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
}

void loop() {
  for( int i = 0; i < NUM_LEDS; ++i)
    {
        leds[i] = ColorFromPalette( currentPalette, CRGB::Purple, BRIGHTNESS, currentBlending );
    }
    
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
}
