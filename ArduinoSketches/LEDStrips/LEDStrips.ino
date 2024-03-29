#include <FastLED.h>

//Constants related to hardware setup
#define NUM_LEDS 30
#define LED_PIN          5
#define CMD_INPUT_PIN    7

// Overall brigness control
#define BRIGHTNESS 100
// Desired FPS for the strip
#define FRAMES_PER_SECOND 120


const long pulseLengthYellowCone = 1000; //pulse length in microseconds to command yellow cone display
const long pulseLengthYellowConeAttention = 750; //pulse length in microseconds to command yellow cone display
const long pulseLengthPurpleCube = 1500; //pulse length in microseconds to command purple cube display
const long pulseLengthPurpleCubeAttention = 1250; //pulse length in microseconds to command purple cube display
const long pulseLengthGreenBlink = 2000;  //pulse length in microseconds to command green blink
const long pulseLengthTolerance = 100; //command pulse length tolerance
		

//Buffer containing the desired color of each LED
CRGB led[NUM_LEDS];
int pulseLen_us;

/**
 * One-time Init, at startup
 */
void setup()
{
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);

  //Set up a debug serial port
  Serial.begin(115200);

  //Configure the roboRIO communication pin to recieve data
  pinMode(CMD_INPUT_PIN, INPUT);

  FastLED.show(); //Ensure we get one LED update in prior to periodic - should blank out all LED's

  Fire_init();
}
/**
 * Periodic call. Will be called again and again.
 */
void loop()
{

  // do some periodic updates
  EVERY_N_MILLISECONDS(200)
  {
    pulseLen_us = pulseIn(CMD_INPUT_PIN, HIGH, 50000);
  }

    //Decode command PWM and display commanded image - default is team number + logo
  if(pulseLen_us <= (pulseLengthYellowCone + pulseLengthTolerance) && pulseLen_us >= (pulseLengthYellowCone - pulseLengthTolerance)){
    ColorSparkle_update(255, 255, 0);
  } else if(pulseLen_us <= (pulseLengthPurpleCube + pulseLengthTolerance) && pulseLen_us >= (pulseLengthPurpleCube - pulseLengthTolerance)){
    ColorSparkle_update(255, 0, 200); 
  } else if(pulseLen_us <= (pulseLengthGreenBlink + pulseLengthTolerance) && pulseLen_us >= (pulseLengthGreenBlink - pulseLengthTolerance)){
    Green_Alert();
  } else {
    CasseroleColorStripeChase_update();
  }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);


}

//**************************************************************
// Pattern: Casserole Color Stripes
//**************************************************************
#define STRIPE_WIDTH_PIXELS 5.0
#define STRIPE_SPEED_PIXELS_PER_LOOP 0.005
void CasseroleColorStripeChase_update()
{
  static double zeroPos = 0;
  zeroPos += STRIPE_SPEED_PIXELS_PER_LOOP;
  for (int i = 0; i < NUM_LEDS; i++)
  {

    //Create a "bumpy" waveform that shifts down the strip over time
    //Output range shoudl be [0,1]
    double pctDownStrip = (double)i / NUM_LEDS;
    double numCyclesOnStrip = (double)NUM_LEDS / (double)STRIPE_WIDTH_PIXELS / 2.0;
    double colorBump = sin(2 * PI * numCyclesOnStrip * (pctDownStrip - zeroPos)) * 0.5 + 0.5;

    //Square the value so that the edge is sharper.
    colorBump *= colorBump;

    //Scale to LED units
    colorBump *= 255;

    //Set the pixel color
    setPixel(i, 255,          //Red
             (int)colorBump,  //Green
             (int)colorBump); //Blue
  }
}

//**************************************************************
// Pattern: Solid Color Sparkle
//**************************************************************
#define CYCLE_FREQ_LOOPS
void ColorSparkle_update(int red, int grn, int blu)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {

    //Set all LED's to the input color, but
    //Randomly set an LED to white.
    if (random(0, NUM_LEDS) <= 0.15)
    {
      //shiny!
      setPixel(i, 255,    //Red
               (int)255,  //Green
               (int)255); //Blue
    }
    else
    {
      //Normal Color
      setPixel(i, red,    //Red
               (int)grn,  //Green
               (int)blu); //Blue
    }
  }
}
//**************************************************************
// Pattern: Rainbow Fade Chase
//**************************************************************
void Rainbow_Fade_Chase()
{
  static uint8_t hue = 0;
  //establish a counter
  static int counter = 0;
  static boolean up;
  if (counter == 0)
  {
    up = true;
  }
  else if (counter == NUM_LEDS)
  {
    up = false;
  }
  if (up == true)
  {
    counter++;
  }
  else if (up == false)
  {
    counter--;
  }
  led[counter] = CHSV(hue++, 255, 255);
  fadeall();
  led[counter] = CHSV(hue++, 255, 255);
  fadeall();
}



//**************************************************************
// Pattern: Particle Fire
//**************************************************************
#define NUM_PARTICLES 30
double particle_locations[NUM_PARTICLES];
double particle_velocities[NUM_PARTICLES];
double particle_temperatures[NUM_PARTICLES];
double particle_cooling_rates[NUM_PARTICLES];

double randFloat(double minVal, double maxVal){
  return random(minVal * 1000.0, maxVal * 1000.0) / 1000.0;
}

void Fire_init_particle(int idx){
  particle_locations[idx] = 1;
  particle_velocities[idx] = randFloat(0.5,2.2);
  particle_temperatures[idx] = randFloat(0.8,1);
  particle_cooling_rates[idx] = randFloat(0.03,0.05);
}

void Fire_init()
{
  for(int idx = 0; idx < NUM_PARTICLES; idx++){
    Fire_init_particle(idx);
  }

}

//maps a 0-1 temperatiure to a hue
double Fire_temp_to_hue(double temp_in){
  return 0 + temp_in * 45.0;
}

//Maps a 0-1 temperature to an  intensity
double Fire_temp_to_intensity(double temp_in){
  return temp_in  * 255;
}

//Maps a 0-1 temperature to a saturation
double Fire_temp_to_sat(double temp_in){
  return (1.0 - 0.5*(temp_in * temp_in)) * 255;
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

//**************************************************************
// Pattern:Blue Fade
//**************************************************************
void Blue_Fade(){
  static double b = 0;
  static int bluemode = 0;
  static boolean bluefade;
  
  if(b<=0){
    bluefade = true;
    bluemode++;
    if(bluemode == 2){
      bluemode = 0;
    }
    b = 0.1;
  }
  else if(254<=b){
    bluefade = false;
    b = 254;
  }
  
  if (bluefade==true){
    b+=2.0;
  }
  else if(bluefade==false){
    b-=2.0;
  }
    
  for (int i = 0; i < NUM_LEDS; i++){
    if((bluemode%2)==1){
      led[i] = CRGB(0, 0, b);
    }
    else{
      led[i] = CRGB(b, b, b);
    }
  }
}

//**************************************************************
// Pattern:Green Alert
//**************************************************************
void Green_Alert(){
  static double g = 0;
  static int greenmode = 0;
  static boolean greenFade;
  
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
    g+=40.0;
  }
  else if(greenFade==false){
    g-=40.0;
  }
    
  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0,0,g);
  }
}

//**************************************************************
// Pattern:Red Fade
//**************************************************************
void Red_Fade(){
  static double r = 0;
  static int redmode = 0;
  static boolean redfade;
  
  if(r<=0){
    redfade = true;
    redmode++;
    if(redmode == 2){
      redmode = 0;
    }
    r = 0.1;
  }
  else if(254<=r){
    redfade = false;
    r = 254;
  }
  
  if (redfade==true){
    r+=2.0;
  }
  else if(redfade==false){
    r-=2.0;
  }
    
  for (int i = 0; i < NUM_LEDS; i++){
    if((redmode%2)==1){
      led[i] = CRGB(r, 0, 0);
    }
    else{
      led[i] = CRGB(r, r, r);
    }
  }
}

//**************************************************************
// Utilities
//**************************************************************
void setPixel(int Pixel, byte red, byte green, byte blue)
{
  // FastLED
  led[Pixel].r = red;
  led[Pixel].b = green;
  led[Pixel].g = blue; //our strips are rbg thbbbttt
}
void fadeall(){
  for (int i = 0; i < NUM_LEDS; i++){
    led[i].nscale8(250);
  }
}
