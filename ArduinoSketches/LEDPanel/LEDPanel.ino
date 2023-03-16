#include <FastLED.h>

#define LED_PIN          5
#define CMD_INPUT_PIN    7
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
const long pulseAttentionGrab = 1250; //pulse length in microseconds to command yellow cone display
const long pulseLengthPurpleCube = 1500; //pulse length in microseconds to command purple cube display
const long pulseLengthPurpleCubeAttention = 1250; //pulse length in microseconds to command purple cube display
const long pulseLengthGreenBlink = 2000;  //pulse length in microseconds to command green blink
const long pulseLengthTolerance = 100; //command pulse length tolerance
									 
							                    			//   0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF
const uint64_t longTeamNumberArray[64]  = {0b0000110000011111111111101111111110000111111111100000000000011000, //0
                    										   0b0001110000011111111111101111111111001111111111110000000001100110, //1
                    										   0b0011110000000000000001100000000111001100000000000000001110000001, //2
                    										   0b0110110000000000000011000000000011001100000000000000110000000001, //3
                    										   0b1100110000000000000110000000000011001100000000000001000010000101, //4
                    										   0b1000110000000000001100000000000111001100000000000010010001001001, //5
                    										   0b0000110000000000011000001111111111001100000000000010001001001010, //6
                    										   0b0000110000000000110000001111111111001111111111100010000101001010, //7
                    										   0b0000110000000001100000000000000111001111111111110001100100111100, //8
                    										   0b0000110000000011000000000000000011001100000000110000010111000010, //9
                    										   0b0000110000000110000000000000000011001100000000110000001000000010, //10
                    										   0b0000110000001100000000000000000011001100000000110000001000111010, //11
                    										   0b0000110000011000000000000000000011001100000000110000001011000100, //12
                    										   0b0000110000110000000000000000000111001100000000110000000100011000, //13
                    										   0b1111111110100000000000001111111111001111111111110000000011100000, //14
                    										   0b1111111110100000000000001111111110000111111111100000000000000000};//15

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

  Serial.println(cmdPWM);

  EVERY_N_MILLISECONDS(250)
  {
      cmdPWM = pulseIn(CMD_INPUT_PIN, HIGH, 50000);
  }

  //Decode command PWM and display commanded image - default is team number + logo
  if(cmdPWM <= (pulseLengthYellowCone + pulseLengthTolerance) && cmdPWM >= (pulseLengthYellowCone - pulseLengthTolerance)){
    yellowCone(false);
  } else if(cmdPWM <= (pulseLengthPurpleCube + pulseLengthTolerance) && cmdPWM >= (pulseLengthPurpleCube - pulseLengthTolerance)){
    purpleCube(false);
  } else if(cmdPWM <= (pulseLengthGreenBlink + pulseLengthTolerance) && cmdPWM >= (pulseLengthGreenBlink - pulseLengthTolerance)){
    greenBlink();
  } else if(cmdPWM <= (pulseAttentionGrab + pulseLengthTolerance) && cmdPWM >= (pulseAttentionGrab - pulseLengthTolerance)){
    attentionGrab();    
  } else {
    printLongArray(longTeamNumberArray);
    //printArray(hatArray);
  }

    FastLED.delay(1000 / UPDATE_RATE_HZ);

}

void yellowCone(boolean attention)
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
        fader+=20.0;
    }
    else if(yellowfade==false){
        fader-=20.0;
    }

    if(!attention){
          fader = 100.0;
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
void purpleCube(boolean attention) {
    static boolean purplefade;
    
    if(fader<=0){
        purplefade = true;
    }
    else if(100<=fader){
       purplefade = false;      
    }
    
    if (purplefade==true){ 
      fader+=20.0;
    }
    else if(purplefade==false){
        fader-=20.0;
    }

    
    if(!attention){
          fader = 100.0;
    }


    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){   
            led[XY(i,j)] = CRGB(0,fader*1.5,fader*1.5);
        }
    }

}

//**************************************************************
// Function: Display a green blink
//**************************************************************
void greenBlink() {
  
    static boolean greenBlinkState;
    
    greenBlinkState = !greenBlinkState;

    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){   
            led[XY(i,j)] = CRGB(greenBlinkState?50:0,0,0);
        }
    }

    FastLED.delay(200);

}

//**************************************************************
// Function: Display an obnoxious blink
//**************************************************************
void attentionGrab() {
  
    static boolean greenBlinkState;
    
    greenBlinkState = !greenBlinkState;

    for (uint8_t i = 0; i<kMatrixWidth; i++){
        for (uint8_t j = 0; j<kMatrixWidth; j++){   
            led[XY(i,j)] = CRGB(greenBlinkState?250:0,greenBlinkState?250:0,greenBlinkState?250:0);
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
  x = y;
  y = tmp;
  y = 15-y;
  
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
