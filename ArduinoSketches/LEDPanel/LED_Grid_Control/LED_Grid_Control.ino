#include <FastLED.h>

#define LED_PIN     5
#define ROBORIO_DATA_PIN 2
#define NUM_LEDS    256
#define BRIGHTNESS  40
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB
#define UPDATE_RATE_HZ 25
CRGB led[NUM_LEDS];

using namespace std;

const uint8_t kMatrixWidth = 16;
//const uint8_t kMatrixHeight = 16;
//**************************************************************
// Pattern: Array that prints One with printArray function.
//**************************************************************
                              // 0123456789ABCDEF
const uint16_t oneArray[16] = {0b0000000110000000, //0
                               0b0000001110000000, //1
                               0b0000011110000000, //2
                               0b0000110110000000, //3
                               0b0001100110000000, //4
                               0b0011000110000000, //5
                               0b0010000110000000, //6
                               0b0000000110000000, //7
                               0b0000000110000000, //8
                               0b0000000110000000, //9
                               0b0000000110000000, //10
                               0b0000000110000000, //11
                               0b0000000110000000, //12
                               0b0000000110000000, //13
                               0b0011111111111100, //14
                               0b0011111111111100};//15

const uint16_t sevenArray[16] = {0b0011111111111100, //0
                                 0b0011111111111100, //1
                                 0b0000000000001100, //2
                                 0b0000000000011000, //3
                                 0b0000000000110000, //4
                                 0b0000000001100000, //5
                                 0b0000000011000000, //6
                                 0b0000000110000000, //7
                                 0b0000001100000000, //8
                                 0b0000011000000000, //9
                                 0b0000110000000000, //10
                                 0b0001100000000000, //11
                                 0b0011000000000000, //12
                                 0b0110000000000000, //13
                                 0b1100000000000000, //14
                                 0b0000000000000000};//15
                             
const uint16_t threeArray[16] = {0b0000000110000000, //0
                                 0b0000001110000000, //1
                                 0b0000011110000000, //2
                                 0b0000110110000000, //3
                                 0b0001100110000000, //4
                                 0b0011000110000000, //5
                                 0b0010000110000000, //6
                                 0b0000000110000000, //7
                                 0b0000000110000000, //8
                                 0b0000000110000000, //9
                                 0b0000000110000000, //10
                                 0b0000000110000000, //11
                                 0b0000000110000000, //12
                                 0b0000000110000000, //13
                                 0b0011111111111100, //14
                                 0b0011111111111100};//15         
                                                 
const uint16_t sixArray[16] =   {0b0000000110000000, //0
                                 0b0000001110000000, //1
                                 0b0000011110000000, //2
                                 0b0000110110000000, //3
                                 0b0001100110000000, //4
                                 0b0011000110000000, //5
                                 0b0010000110000000, //6
                                 0b0000000110000000, //7
                                 0b0000000110000000, //8
                                 0b0000000110000000, //9
                                 0b0000000110000000, //10
                                 0b0000000110000000, //11
                                 0b0000000110000000, //12
                                 0b0000000110000000, //13
                                 0b0011111111111100, //14
                                 0b0011111111111100};//15  
                                 
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

void setup() {
    Serial.begin(9600);
    delay( 3000 ); // power-up safety delay

    //Configure the fast LED Library for our display
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS ); 
  
    //Configure the roboRIO communication pin to recieve data
    pinMode(ROBORIO_DATA_PIN, INPUT);

    //Initialize the FastLED display buffer array  to blank out the display
    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){
            led[XY(i,j)] = CRGB(0,0,0);
        }
    }
    FastLED.show(); //Ensure we get one LED update in prior to periodic - should blank out all LED's
}

void loop() {
  
//    purpleCube();
//    yellowCone();
//    printArray(hatArray, -15, 13);
      printArray(oneArray);
      printArray(sevenArray);
      printArray(threeArray);
      printArray(sixArray);
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

//PrintArray implements a complete scroll of a single image across the entire LED display. The image is represented by an array
//The function does not return until the entire image has completed scrolling acroos the display 
//Rightoffset is applied to the starting postion of the image to ensure it starts off the right side of the display
//Leftoffset used to stop the scroll when the image has scrolled fully off of the left side of the display
void printArray(const uint16_t arr[]){
  
  //TODO: Make this thing show a single image and move the scrolling into the main arduino Loop() function
  Serial.println("***********SCROLLING*************");

	uint16_t mask = 0b1000000000000000;

  //Set up an internal buffer to hold the "scrolled" image array
  uint16_t img[16] =    {0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000, //0
                        0b0000000000000000}; //0
        
 //Loop to handle scrolling the image. Each iteration of this loop moves the image one increment across the display
 //S represents the current left-right "position" of the image on the LED display 
	for(int s = kMatrixWidth; s > -kMatrixWidth; s--){
      
    //Shift the image to the left or right to create the scrolling effect
    //First shift the image off to the right of the display then unshift it once per loop until it reaches the default unshifted position
    //Then start shifting it left to move it off the left side of the display
    for(uint8_t i = 0; i < kMatrixWidth; i++){
      if(s >= 0) {img[i] = arr[i] >> s;}
      else {img[i] = arr[i] << -s;}
    }


  //These two loops scan through the display to set each LED on or off as defined image array
  //i represents the current ROW of the display
  //j represents the current COLUMN of the display
		for (uint8_t i = 0; i < kMatrixWidth; i++){
			for (uint8_t j = 0; j < kMatrixWidth; j++){
      
      //Check if the current row/column LED should be lit by comparing the current mask value to the array row
      //Compare current mask to bit-packed image array. If current bit is TRUE, set the LED ON, otherwise set it OFF
				if((mask&img[i]) > 0){
				  led[XY(i,j)] = CRGB(0,255,0); 
				} else {
          led[XY(i,j)] = CRGB(0,0,0);   
        }
				mask = mask >> 1; //rotate the mask for the next column. NOTE: in arduino API, << is a shift operator, NOT rotate
			}
      mask = 0b1000000000000000; //reset the mask for the next row
		}// end of outer loop

   //Show the image and wait for the next update
		FastLED.show();
		FastLED.delay(1000 / UPDATE_RATE_HZ);
	}
}

//utility function that was found online. It redifines the way our code views the LEDS. It maps the Led index number onto a grid like cordinate plane.
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
