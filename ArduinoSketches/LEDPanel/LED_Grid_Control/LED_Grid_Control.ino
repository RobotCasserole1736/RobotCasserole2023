#include <FastLED.h>

#define LED_PIN          5
#define CMD_INPUT_PIN 7
#define NUM_LEDS    256
#define BRIGHTNESS  40
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB
#define UPDATE_RATE_HZ 25
#define FADE_RATE 60
CRGB led[NUM_LEDS];

const uint8_t kMatrixWidth = 16;
//const uint8_t kMatrixHeight = 16;
uint8_t fader = BRIGHTNESS;
const long pulseLengthYellowCone = 1000; //pulse length in microseconds to command yellow cone display
const long pulseLengthPurpleCube = 1500; //pulse length in microseconds to command purple cube display
const long pulseLengthTolerance = 100; //command pulse length tolerance

//**************************************************************
// Pattern: Array that prints '1' with printArray function
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
                               0b0000000110000000, //A
                               0b0000000110000000, //B
                               0b0000000110000000, //C
                               0b0000000110000000, //D
                               0b0011111111111100, //E
                               0b0011111111111100};//F

//**************************************************************
// Pattern: Array that prints '7' with printArray function
//**************************************************************
                                // 0123456789ABCDEF
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
                             
const uint16_t threeArray[16] = {0b0011111111111000, //0
                                 0b0011111111111100, //1
                                 0b0000000000011100, //2
                                 0b0000000000001100, //3
                                 0b0000000000001100, //4
                                 0b0000000000011100, //5
                                 0b0011111111111100, //6
                                 0b0011111111111100, //7
                                 0b0000000000011100, //8
                                 0b0000000000001100, //9
                                 0b0000000000001100, //10
                                 0b0000000000001100, //11
                                 0b0000000000001100, //12
                                 0b0000000000011100, //13
                                 0b0011111111111100, //14
                                 0b0011111111111000};//15        
                                                 
const uint16_t sixArray[16] =   {0b0001111111111000, //0
                                 0b0011111111111100, //1
                                 0b0011000000000000, //2
                                 0b0011000000000000, //3
                                 0b0011000000000000, //4
                                 0b0011000000000000, //5
                                 0b0011000000000000, //6
                                 0b0011111111111000, //7
                                 0b0011111111111100, //8
                                 0b0011000000001100, //9
                                 0b0011000000001100, //10
                                 0b0011000000001100, //11
                                 0b0011000000001100, //12
                                 0b0011000000001100, //13
                                 0b0011111111111100, //14
                                 0b0001111111111000};//15

const uint16_t teamNumberArray[16] = {0b0000100001111111, //0
                                      0b0001100000000010, //1
                                      0b0010100000000100, //2
                                      0b0100100000001000, //3
                                      0b0000100000010000, //4
                                      0b0000100000100000, //5
                                      0b1111111001000000, //6
                                      0b0000000000000000, //7
                                      0b0000000000000000, //8
                                      0b1111111001111111, //9
                                      0b0000001001000000, //10
                                      0b0000001001000000, //11
                                      0b1111111001111111, //12
                                      0b0000001001000001, //13
                                      0b0000001001000001, //14
                                      0b1111111001111111};//15
									 
							                    			//  0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF
const uint64_t longTeamNumberArray[16*4]  = {0b0000000110000000001111111111110000111111111110000001111111111, //0
                    										     0b0000001110000000001111111111110000111111111111000011111111111, //1
                    										     0b0000011110000000000000000000110000000000000111000011000000000, //2
                    										     0b0000110110000000000000000001100000000000000011000011000000000, //3
                    										     0b0001100110000000000000000011000000000000000011000011000000000, //4
                    										     0b0011000110000000000000000110000000000000000011000011000000000, //5
                    										     0b0010000110000000000000001100000000000000000111000011000000000, //6
                    										     0b0000000110000000000000011000000000111111111111000011111111111, //7
                    										     0b0000000110000000000000110000000000111111111111000011111111111, //8
                    										     0b0000000110000000000001100000000000000000000111000011000000001, //9
                    										     0b0000000110000000000011000000000000000000000011000011000000001, //10
                    										     0b0000000110000000000110000000000000000000000011000011000000001, //11
                    										     0b0000000110000000001100000000000000000000000011000011000000001, //12
                    										     0b0000000110000000011000000000000000000000000111000011000000001, //13
                    										     0b0011111111111100110000000000000000111111111111000011111111111, //14
                    										     0b0011111111111100110000000000000000111111111110000001111111111};//15
  	
                                //  0123456789ABCDEF 
const uint16_t hatArray[16] =    {0b0000000000110000, //0
                                  0b0000000011001100, //1
                                  0b0000011100000010, //2
                                  0b0001100000000010, //3
                                  0b0010000100001010, //4
                                  0b0100100010010010, //5
                                  0b0100010010010100, //6
                                  0b0100001010010100, //7
                                  0b0011001001111000, //8
                                  0b0000101110000100, //9
                                  0b0000010000000100, //10
                                  0b0000010001110100, //11
                                  0b0000010110001000, //12
                                  0b0000001000110000, //13
                                  0b0000000111000000, //14
                                  0b0000000000000000};//15
void setup() {
    Serial.begin(115200); // begin serial communication with arduino

    // Configure the fast LED Library for our display
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(led, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    // Configure the roboRIO communication pin to recieve data
    pinMode(CMD_INPUT_PIN, INPUT);

    // Initialize the FastLED display buffer array  to blank out the display
    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){
            led[XY(i,j)] = CRGB(0,0,0);
        }
    }

}

//**************************************************************
// Function: Main loop that runs continually when the arduino is
// powered on
//**************************************************************
int cmdPWM =0;
void loop() {
  //Read in command PWM from RoboRIO - value is pulse length in microseconds

  EVERY_N_MILLISECONDS(250)
  {
      cmdPWM = pulseIn(CMD_INPUT_PIN, HIGH, 50000);
  }

  //Decode command PWM and display commanded image - default is team number + logo
  if(cmdPWM <= (pulseLengthYellowCone + pulseLengthTolerance) && cmdPWM >= (pulseLengthYellowCone - pulseLengthTolerance)){
    yellowCone();
  } else if(cmdPWM <= (pulseLengthPurpleCube + pulseLengthTolerance) && cmdPWM >= (pulseLengthPurpleCube - pulseLengthTolerance)){
    purpleCube();
  } else {
    printLongArray(longTeamNumberArray);
    //printArray(hatArray);
  }

    FastLED.delay(1000 / UPDATE_RATE_HZ);

}

void yellowCone()
{
    //Records if image is fading in or fading out; must be declared static to prevent it going out of scope when function exits
    static boolean yellowfade = true;
    
    if(fader<=0){
        yellowfade = true;
    }
    else if(100<=fader){
        yellowfade = false;
    }
    
    if (yellowfade==true){
        fader+=10.0;
    }
    else if(yellowfade==false){
        fader-=10.0;
    }

    uint8_t upperBound = 7;
    uint8_t lowerBound = 8;
    
    for (uint8_t i = 0; i < kMatrixWidth; i++){
         for (uint8_t j = 0; j < kMatrixWidth; j++){
              if (j >= upperBound && j <= lowerBound){
                  led[XY(i,j)] = CRGB(fader*0.5,fader*1.5,0);
              } else {
                  led[XY(i,j)] = CRGB(0,0,0);
              }
         }
        if (i % 2 == 1 && upperBound > 0 && lowerBound < 15)
        {
            upperBound--;
            lowerBound++;
        }
    }

}

//**************************************************************
// Function: Display a purple cube when in Cube Mode
//**************************************************************
void purpleCube() {
    static boolean purplefade;
    
    if(fader<=0){
        purplefade = true;
    }
    else if(100<=fader){
       purplefade = false;      
    }
    
    if (purplefade==true){ 
      fader+=10.0;
    }
    else if(purplefade==false){
        fader-=10.0;
    }

    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){   
            led[XY(i,j)] = CRGB(0,fader*1.5,fader*1.5);
        }
    }

}

int s = kMatrixWidth; // scroll idx
//**************************************************************
// Function: PrintArray implements a complete scroll of a single
// image across the entire LED display. The image is represented
// by an array.
// The function does not return until the entire image has
// completed scrolling across the display. Rightoffset is
// applied to the starting postion of the image to ensure it
// starts off the right side of the display Leftoffset used to 
// stop the scroll when the image has scrolled fully off of the
// left side of the display.
//**************************************************************
void printArray(const uint16_t arr[]){
    uint16_t mask = 0b1000000000000000;

  //Set up an internal buffer to hold the "scrolled" image array
//                        0123456789ABCDEF
  uint16_t img[16] =   {0b0000000000000000, //0
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

 
 //Update to handle scrolling the image. Each iteration of this loop moves the image one increment across the display
 //S represents the current left-right "position" of the image on the LED display 
 if(s <= -kMatrixWidth){
  s = kMatrixWidth;
 } 
  
    //Shift the image to the left or right to create the scrolling effect
    //First shift the image off to the right of the display then unshift it once per loop until it reaches the default unshifted position
    //Then start shifting it left to move it off the left side of the display
    for(uint8_t i = 0; i < kMatrixWidth; i++){
      if(s >= 0) {img[i] = arr[i] >> s;}
      else {img[i] = arr[i] << -s;}
    }

        // These two loops scan through the display to set each LED on or off as defined image array
        // i represents the current ROW of the display
        // j represents the current COLUMN of the display
        for (uint8_t i = 0; i < kMatrixWidth; i++){
            for (uint8_t j = 0; j < kMatrixWidth; j++){
                // Check if the current row/column LED should be lit by comparing the current 
                // mask value to the array row
                // Compare current mask to bit-packed image array. 
                // If current bit is TRUE, set the LED ON, otherwise set it OFF
                if((mask&img[i]) > 0){
                    led[XY(i,j)] = CRGB(0,255,0); 
                }
                else {
                    led[XY(i,j)] = CRGB(0,0,0);
                }
                // Rotate the mask for the next column. 
                // NOTE: in arduino API, << is a shift operator, NOT rotate
                mask = mask >> 1; 
            }
            mask = 0b1000000000000000; //reset the mask for the next row
        } // end of outer loop

  s--;

}

// The printLongArray should do the same thing as the printArray function but with a longer message
//PrintArray implements a complete scroll of a single image across the entire LED display. The image is represented by an array
//The function does not return until the entire image has completed scrolling acroos the display 
//Rightoffset is applied to the starting postion of the image to ensure it starts off the right side of the display
//Leftoffset used to stop the scroll when the image has scrolled fully off of the left side of the display
void printLongArray(const uint64_t arr[]){
//                  0123456789ABCDEF
  uint16_t mask = 0b1000000000000000;

  //Set up an internal buffer to hold the "scrolled" image array
//					              0123456789ABCDEF
  uint16_t img[16] =   {0b0000000000000000,  //0
                        0b0000000000000000,  //1
                        0b0000000000000000,  //2
                        0b0000000000000000,  //3
                        0b0000000000000000,  //4  
                        0b0000000000000000,  //5
                        0b0000000000000000,  //6
                        0b0000000000000000,  //7 
                        0b0000000000000000,  //8
                        0b0000000000000000,  //9
                        0b0000000000000000,  //A
                        0b0000000000000000,  //B
                        0b0000000000000000,  //C    
                        0b0000000000000000,  //D
                        0b0000000000000000,  //E
                        0b0000000000000000}; //F

 
 //Update to handle scrolling the image. Each iteration of this loop moves the image one increment across the display
 //S represents the current left-right "position" of the image on the LED display 
 if(s <= -kMatrixWidth){
  s = kMatrixWidth*4;
 } 
  
        
    //Shift the image to the left or right to create the scrolling effect
    //First shift the image off to the right of the display then unshift it once per loop until it reaches the default unshifted position
    //Then start shifting it left to move it off the left side of the display
    for(uint8_t i = 0; i < kMatrixWidth; i++){
      if(s >= 0) 
        {img[i] = arr[i] >> s;}
      else 
        {img[i] = arr[i] << -s;}
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
      
      mask = 0b1000000000000000; //reset the mask for the next ro
    }// end of outer loop

    s--;

}

//utility function that was found online. It redifines the way our code views the LEDS. It maps the Led index number onto a grid like cordinate plane.
uint16_t XY( uint8_t x, uint8_t y)
{

  //Perform rotation for display
  uint8_t tmp = x;
  x = 15-y;
  y = tmp;
  
    uint16_t i;
    if( y & 0x01) {
        //  electrically, Odd rows run backwards
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
    } 
    else {
        // Even rows run forwards
        i = (y * kMatrixWidth) + x;
    } 
    return i;
}
