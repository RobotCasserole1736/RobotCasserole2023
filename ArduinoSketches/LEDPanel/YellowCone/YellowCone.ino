#include <FastLED.h>

#define LED_PIN     5
#define ROBORIO_DATA_PIN 2
#define NUM_LEDS    256
#define BRIGHTNESS  70
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
#define UPDATES_PER_SECOND 100
CRGB led[NUM_LEDS];


CRGBPalette16 currentPalette;
//TBlendType    currentBlending;

const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 16;
//unsigned int grid[row][col];

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


void setup() {
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS ); 
    
    currentPalette = RainbowColors_p;
//    currentBlending = LINEARBLEND;

    FastLED.setBrightness(BRIGHTNESS);
    FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  
    //Set up a debug serial port
    Serial.begin(9600);
  
    //Configure the roboRIO communication pin to recieve data
    pinMode(ROBORIO_DATA_PIN, INPUT);
  
    FastLED.show(); //Ensure we get one LED update in prior to periodic - should blank out all LED's
  
//    Fire_init();
}

void loop() {
  
    // led[XY(7,7)] = CRGB(100,255,0);
    cone();
    //FastLED.show();
    //FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void cone()
{

    static double r = 0;
    static boolean redfade;
    
    if(r<=0){
        redfade = true;
    }
    else if(100<=r){
        redfade = false;
    }
    
    if (redfade==true){
        r+=10.0;
    }
    else if(redfade==false){
        r-=10.0;
    }

    uint8_t upperBound = 7;
    uint8_t lowerBound = 8;

    
    for (uint8_t i = 0; i < kMatrixWidth; i++)
    {
         for (uint8_t j = 0; j < kMatrixWidth; j++)
         {
              if (j >= upperBound && j <= lowerBound)
              {
                  led[XY(i,j)] = CRGB(r*0.5,r*1.5,0);
              }
         }
          if (i % 2 == 1 && upperBound > 0 && lowerBound < 15)
          {
              upperBound--;
              lowerBound++;
          }
    }

    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
}

/*
void Fire_init_particle(int idx)
{
    particle_locations[idx] = 1;
    particle_velocities[idx] = randFloat(0.5,2.2);
    particle_temperatures[idx] = randFloat(0.8,1);
    particle_cooling_rates[idx] = randFloat(0.03,0.05);
}

void Fire_update()
{

    //update all particles
    for(int idx = 0; idx < NUM_PARTICLES; idx++){
      particle_locations[idx] += particle_velocities[idx];
      particle_temperatures[idx] -= particle_cooling_rates[idx];

      //terminal case, reset particle
      if(particle_locations[idx] > NUM_LEDS || particle_temperatures[idx] < 0){
        Fire_init_particle(idx);
      }
    }

    //clear all leds
    for(int idx = 0; idx < NUM_LEDS; idx++){
      led[idx] = CHSV(0, 0, 0);
    }

    // display all particles
    for(int idx = 0; idx < NUM_PARTICLES; idx++){
      int hue = round(Fire_temp_to_hue(particle_temperatures[idx]));
      int sat = round(Fire_temp_to_sat(particle_temperatures[idx]));
      int value = round(Fire_temp_to_intensity(particle_temperatures[idx]));
      
      int firstLEDIdx = floor(particle_locations[idx]);
      int secondLEDIdx = ceil(particle_locations[idx]);
      double firstLEDFrac = 1.0 - (particle_locations[idx] - (double)firstLEDIdx);
      double secondLEDFrac = 1.0 - ((double)secondLEDIdx - particle_locations[idx]);
      
      led[firstLEDIdx] += CHSV(hue, sat, value * firstLEDFrac);
      led[secondLEDIdx] += CHSV(hue, sat, value * secondLEDFrac);
    }

    //Bottom Blue source
    led[0] += CRGB(0, 0, 200);
    led[1] += CRGB(0, 0, 100);
    led[2] += CRGB(0, 0, 50);
}

*/
//**************************************************************
// Pattern:Green Alert
//**************************************************************
void Green_Alert(){
  static double g = 0;
  static int greenmode = 0;
  static boolean greenFade;

  static double r = 0;
  static int redmode = 0;
  static boolean redFade;
  
  if(g<=0){
    greenFade = true;
    greenmode++;
    if(greenmode == 2){
      greenmode = 0;
    }
    g = 0.1;
  }
  else if(254.0<=g){
    greenFade = false;
    g = 254.0;
  }
  
  if (greenFade==true){
    g+=20.0;
  }
  else if(greenFade==false){
    g-=20.0;
  }
    
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(g*0.5,g,0);
  }
}

uint16_t XY( uint8_t x, uint8_t y)
{
    uint16_t i;
    if( y & 0x01)
    {
        // Odd rows run backwards
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
    } 
    else
    {
        // Even rows run forwards
        i = (y * kMatrixWidth) + x;
    } 
    return i;
}
