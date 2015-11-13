/* It seems to solved the Arduino UNO reset problem (when the serial is open) by 
 * adding a capacitor between the reset pin and the ground.  
 * The capacitor is use to keep the reset line up.
 * A 1uF seems to work.
 * (see refs.:  http://forum.arduino.cc/index.php/topic,28723.0.html
 *              http://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection
 * 
 * Note: Press the reset button (with careful timing) or remove the capasitor for uploading Arduino code. 
 */

#include <FastLED.h>
#include <ArduinoUtil.h>
#define NUM_LEDS 60
#define DATA_PIN 11 //green
#define CLOCK_PIN 13 //blue

//GND_PIN  yellow
//DC5V_PIN red

#define PI 3.142

CRGB leds_[NUM_LEDS];

void setLEDs(CRGB color)
{
  for (int num = 0; num < NUM_LEDS; num++)
    leds_[num] = color; //turn off all LEDs
}


void breath_sine(CHSV &color, float frequency, float on_percentage)
{
  CRGB ledColor;
  int colorIntencityLevel = 0;

  ledColor = CHSV(color.h, color.s, 255);
  setLEDs(ledColor);

  colorIntencityLevel = 128 - (int) (128.0*sin(frequency*(millis()/1000.0)*2.0*PI));
  FastLED.setBrightness(colorIntencityLevel);
  FastLED.show();

//maybe we can use Amplitude to determine how fast the light change its intencity
}

void blinking_sine(CHSV &color, float frequency, float on_percentage)
{
  CRGB ledColor;
  int colorIntencityLevel = 0;
  float sineWave;
  float vertical_shifting;

  float Amplitude = 1;
  float off_percentage  = 1 - on_percentage;
  
  //max of on% or off% is 1
  vertical_shifting = Amplitude * (on_percentage - off_percentage); 
  
  //(millis()/1000)*2*PI to scale to 1 period per sec.
  sineWave = sin(frequency*(millis()/1000.0)*2.0*PI) + vertical_shifting; 

  if ( sineWave <= 0.0) { 
    setLEDs(CRGB::Black);  
  }
  else  {   
    ledColor = CHSV(color.h, color.s, color.v);
    setLEDs(ledColor);
  }
  FastLED.show();
}

void moving_sine(CHSV &color, float frequency, float on_percentage)
{
  CRGB ledColor;
  int colorIntencityLevel = 0;
  float sineWave;
  float vertical_shifting;

  float Amplitude = 1;
  float off_percentage  = 1 - on_percentage;
  
  float freq_per_millis = frequency*((float)millis()/1000.0);
   
  //max of on% or off% is 1 
  vertical_shifting = Amplitude * (on_percentage - off_percentage);

  //converting frequency per msec to cycle per msec prior to calculation
  sineWave = sin(freq_per_millis*2.0*PI) + vertical_shifting; 

  //Determine the Target LED
  int ledPos = (int) freq_per_millis - ((int) (freq_per_millis/(float) NUM_LEDS)) * NUM_LEDS;

  //Target LED state
  if ( sineWave <= 0.0) { 
    leds_[ledPos] = CRGB::Black; 
  }
  else  {   
    if (ledPos == 0) //restart a cycle
      leds_[NUM_LEDS-1] = CRGB::Black;
    else
      leds_[ledPos-1] = CRGB::Black;
      
    ledColor = CHSV(color.h, color.s, color.v);  
    leds_[ledPos] = ledColor;
  }
  
  FastLED.show();
  
}

void modeSelection(requestdataseq *request)
{

  static unsigned long cross_time = 0;
  static unsigned long hold_time = 0;
  //static CRGB ledColor;

  static bool newMode_flag = false; 
  static int currentMode = 0;
  
 
  if (currentMode != request->mode)  {
    currentMode = request->mode;
    /*TODO:
     * Using the newMode_flag to allow behaviours that have a specific start sequence 
     *  to always start with that sequence 
     *  i.e. moving led always start from first LED.
     * By using start_time ( = millis()), we can shift the sine wave to start from
     *  start_time i.e. millis - start_time.
     */  
    newMode_flag = true;
    setLEDs(CRGB::Black);
  }

  //ledColor = request->ledColor;
   
  switch (currentMode) {
    case 2:
      blinking_sine(request->ledHSVColor, request->frequency, request->on_pct);
      newMode_flag = false;
      break;
      
    case 3:
      breath_sine(request->ledHSVColor, request->frequency, request->on_pct);
      newMode_flag = false;
      break;
      
    case 8:
      moving_sine(request->ledHSVColor, request->frequency, request->on_pct);
      newMode_flag = false;
      break;

  }
}

requestdataseq processData(unsigned char  *rawbuf)
{

  requestdataseq request;
  
  request.mode = (unsigned int) rawbuf[0]; 
  
  request.frequency = convertbytes2float(rawbuf[1], rawbuf[2], rawbuf[3], rawbuf[4]);
  
  request.ledRGBColor.r = rawbuf[5];
  request.ledRGBColor.g = rawbuf[6];
  request.ledRGBColor.b = rawbuf[7];

  request.ledHSVColor =  rgb2hsv_approximate (request.ledRGBColor); //convert to HSV for color space for easy manipulation
    
  request.on_pct = convertbytes2float(rawbuf[8], rawbuf[9], rawbuf[10], rawbuf[11]);

  return request;
}

bool verifyDataChecksum(unsigned char  *rawbuf, int dataSize)
{
  byte sum = 0;

  for (int i = 0; i < dataSize -1; i++) {
    sum += rawbuf[i];
    Serial.println(rawbuf[i]);
  }

  Serial.println(rawbuf[dataSize-1]);
  if ( (sum & 0xFF) == rawbuf[dataSize-1] )   //check with the last byte 
    return true;
  else 
    return false;
}

bool processingSerialInput(unsigned char *rawbuf)
{
  static unsigned char currentData = 0;
  static unsigned char previousData = 0;
  int expectedDataSize = 0; //payload + checksum
  static long dataSize = -1; 
    
  currentData = Serial.read();
    
  //look for 255 follow by 15 header before retriving the rest of the data  
  if ( (previousData == 255) && (currentData == 15) )
  {
    while(!Serial.available()) { } //wait for data         
    expectedDataSize = (int) Serial.read(); 

    for (int dataSize = 0; dataSize < expectedDataSize; dataSize++)
    {
      rawbuf[dataSize] = 0;       
      while(!Serial.available()) { } //wait for data         
      rawbuf[dataSize] = Serial.read();
    }
    currentData = 0;
    previousData = 0;
    //return checksum
    return verifyDataChecksum(rawbuf, expectedDataSize);    
  }
  else 
  {
    previousData = currentData;              
  }

  return false;
}


void setup() 
{
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN>(leds_, NUM_LEDS);
  Serial.begin(115200);
  delay(2000);
}

void loop() 
{

  static requestdataseq request;
  //static databuffer rawbuf;
	static unsigned char rawbuf[14]; //payload + checksum size

  
	while (Serial.available()) {   
    if ( processingSerialInput(rawbuf)) //if chksum is good process the data
      request = processData(rawbuf);
  }
  
  modeSelection(&request);
}
