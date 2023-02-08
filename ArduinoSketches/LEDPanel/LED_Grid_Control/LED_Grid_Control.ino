#include <FastLED.h>

#define LED_PIN     5
#define ROBORIO_DATA_PIN 2
#define NUM_LEDS    256
#define BRIGHTNESS  70
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
#define UPDATES_PER_SECOND 100
CRGB led[NUM_LEDS];


using namespace std;

//CRGBPalette16 currentPalette;
//TBlendType    currentBlending;

const uint8_t kMatrixWidth = 16;
//const uint8_t kMatrixHeight = 16;
//unsigned int grid[row][col];

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

//**************************************************************
// Pattern: Array that prints One with printArray function.
//**************************************************************
                                // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
const uint8_t oneArray[16][16] PROGMEM = {{0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //0
                                            {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0},  //1
                                            {0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0},  //2
                                            {0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0},  //3
                                            {0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0},  //4
                                            {0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,0},  //5
                                            {0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0},  //6
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //7
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //8
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //9
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //10
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //11
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //12
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //13
                                            {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0},  //14
                                            {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0}}; //15
                            
                                  // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
const uint8_t sevenArray[16][16] PROGMEM = {{0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0},  //0
                                            {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0},  //1
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //2
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //3
                                            {0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},  //4
                                            {0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},  //5
                                            {0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0},  //6
                                            {0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0},  //7
                                            {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},  //8
                                            {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},  //9
                                            {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},  //10
                                            {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},  //11
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //12
                                            {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //13
                                            {0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0},  //14
                                            {0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0}}; //15

                                  // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5
const uint8_t threeArray[16][16] PROGMEM = {{0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0},  //0
                                            {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0},  //1
                                            {0,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0},  //2
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //3
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //4
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //5
                                            {0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0},  //6
                                            {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0},  //7
                                            {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0},  //8
                                            {0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0},  //9
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //10
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //11
                                            {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0},  //12
                                            {0,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0},  //13
                                            {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0},  //14
                                            {0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0}}; //15
                                    
                                // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
const uint8_t sixArray[16][16] PROGMEM = {{0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0},  //0
                                          {0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0},  //1
                                          {0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},  //2
                                          {0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},  //3
                                          {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},  //4
                                          {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},  //5
                                          {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},  //6
                                          {0,0,1,1,0,1,1,1,1,1,1,1,0,0,0,0},  //7
                                          {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0},  //8
                                          {0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},  //9
                                          {0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},  //10
                                          {0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},  //11
                                          {0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},  //12
                                          {0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},  //13
                                          {0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0},  //14
                                          {0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0}}; //15
                            
                                // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
const uint8_t hatArray[16][16] PROGMEM = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //0
                                          {0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0},  //1
                                          {0,0,0,0,1,1,1,1,0,0,0,1,1,0,0,0},  //2
                                          {0,0,0,1,0,1,0,1,0,0,0,0,1,0,0,0},  //3
                                          {0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0},  //4
                                          {0,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0},  //5
                                          {0,0,0,1,1,1,0,0,0,1,0,0,0,1,0,0},  //6
                                          {0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0},  //7
                                          {0,0,1,0,0,0,0,0,0,0,0,1,0,1,0,0},  //8
                                          {0,0,1,1,0,0,0,0,1,1,0,0,0,1,1,0},  //9
                                          {0,0,0,1,1,1,1,0,0,0,1,0,1,0,1,0},  //10
                                          {0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0},  //11
                                          {0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0},  //12
                                          {0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0},  //13
                                          {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},  //14
                                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}; //15

//const uint8_t teamNumberArray[kMatrixWidth][kMatrixWidth * 4] PROGMEM = {
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0},
//  {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0},
//  {0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},
//  {0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},
//  {0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
//  {0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
//  {0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,0,1,1,1,1,1,1,1,0,0,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0},
//  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0},
//  {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0},
//  {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0}
//};

//                                // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
//const uint8_t testArray[16][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //0
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //1
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //2
//                                  {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},  //3
//                                  {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0},  //4
//                                  {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},  //5
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //6
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //7
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //8
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //9
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //10
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  //11
//                                  {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},  //12
//                                  {1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0},  //13
//                                  {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},  //14
//                                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}; //15

                                // 0 1 2 3 4 5 6 7 8 9 1 1 2 3 4 5 
const uint8_t numberLogoArray[16][16] = {{0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //0
                                  {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0},  //1
  //Need to make it               {0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0},  //2
                                  {0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0},  //3
                                  {0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0},  //4
                                  {0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,0},  //5
                                  {0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0},  //6
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //7
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //8
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //9
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //10
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //11
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //12
                                  {0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0},  //13
                                  {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0},  //14
                                  {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0}}; //15


void setup() {
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS ); 
    
//    currentPalette = RainbowColors_p;
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
//    purpleCube();
//    yellowCone();
    printArray(hatArray, -15, 13);
//    printArray(oneArray, -15, 13);
//    printArray(sevenArray, -15, 13);
//    printArray(threeArray, -15, 13);
//    printArray(sixArray, -15, 13);
//    printArray(teamNumberArray,-60,60);
//    FastLED.show();
//    FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void yellowCone()
{

    static double y = 0;
    static boolean yellowfade;
    
    if(y<=0){
        yellowfade = true;
    }
    else if(100<=y){
        yellowfade = false;
        FastLED.delay(1000);
        
    }
    
    if (yellowfade==true){
        y+=10.0;
    }
    else if(yellowfade==false){
        y-=10.0;
    }

    uint8_t upperBound = 7;
    uint8_t lowerBound = 8;

    
    for (uint8_t i = 0; i < kMatrixWidth; i++)
    {
         for (uint8_t j = 0; j < kMatrixWidth; j++)
         {
              if (j >= upperBound && j <= lowerBound)
              {
                  led[XY(i,j)] = CRGB(y*0.5,y*1.5,0);
              }
         }
          if (i % 2 == 1 && upperBound > 0 && lowerBound < 15)
          {
              upperBound--;
              lowerBound++;
          }
    }
}

void purpleCube() {

    static double p = 0;
    static boolean purplefade;
    
    if(p<=0){
        purplefade = true;
    }
    else if(100<=p){
       purplefade = false;
        FastLED.delay(1000);
        
    }
    
    if (purplefade==true){
        p+=10.0;
    }
    else if(purplefade==false){
        p-=10.0;
    }

    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){
          
            led[XY(i,j)] = CRGB(0,p*1.5,p*1.5);
        }
    }
}

void printArray(const uint8_t *arr,int leftOffset, int rightOffset){
  for(int s = rightOffset; s > leftOffset; s--){
    for (uint8_t i = 0; i < kMatrixWidth; i++){
      for (uint8_t j = 0; j < kMatrixWidth; j++){ 
        if ((j - s >= 0) && (j - s < kMatrixWidth))
          led[XY(i,j)] = CRGB(0,pgm_read_byte_near(arr + i*kMatrixWidth + (j - s)) * 255,0);
        else 
          led[XY(i,j)] = CRGB(0,0,0);
      }
    }
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
    delay(100);
  }
}
//utlitiy function that was found online. It redifines the way our code views the LEDS. It maps the Led index number onto a grid like cordinate plane.
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
