/* 
Led matrix array Spectrum Analyser
22/11/2015 Nick Metcalfe

Takes a mono analog RCA audio input (nominal 1v pp) and presents a 64-band
real-time colour spectrum analyser display with peak metering.

Using two of:
16x32 DIY Kit Red Green Dual-Color Dot Matrix Control Display Module
http://www.ebay.com/itm/231503508955

Built with Teensy 3.2 controller
https://www.pjrc.com/store/teensy32.html
and the Teensy Audio Library
http://www.pjrc.com/teensy/td_libs_Audio.html

Portions of code based on:

myMatrix – Arduino library for LED matrix panels
Copyright (c) 2015 Silviu - www.openhardware.ro
http://openhardware.ro/mymatrix/

CShiftPWM.cpp - ShiftPWM.h - Library for Arduino to PWM many outputs using shift registers
Copyright (c) 2011-2012 Elco Jacobs, www.elcojacobs.com
https://www.pjrc.com/teensy/td_libs_ShiftPWM.html

Logarithmic band calculations
http://code.compartmental.net/2007/03/21/fft-averages/


Copyright (c) 2015 Nick Metcalfe  valves@alphalink.com.au
All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <time.h>       /* time_t, struct tm, time, mktime */
#include "font5x7.h"

//Note: -- Use 96Mhz CPU Speed --

//Hardware setup
//These can be on any unused pin
const int m_latchPin = 21;
const int m_redPin = 11;
const int m_greenPin = 9;
const int m_clockPin = 13;
const int m_enablePin = 14;
const int m_ledPin = 20;

//Row drivers need to stay on these pins as they are used with 
//direct port access on GPIOB
const int m_aPin = 16;
const int m_bPin = 17;
const int m_cPin = 19;
const int m_dPin = 18;


//This should be changed to suit the installation.
//Should go up to four panels.
const unsigned char m_amountOfPanels = 2;      //Number of panels in this installation

//Screen refresh parameters
//At 800 hz line refresh rate, given we must push 16 rows of
//up to 128 pixels per line, the frame rate should be 50hz.
//At this speed, we can manage about 8 PWM brightness steps
//before using large quantities of CPU time.
const int m_ledFrequency = 800;                //Row refresh speed 800/16 rows = 50hz frame rate
const unsigned char m_maxBrightness = 8;       //PWM steps from dim to full brightness

//Default brightness of red and green used for all current graphics.
//Note: increasing these will increase current consumption. Take care if powering from USB.
const int m_redIntensity = 1;
const int m_greenIntensity = 4;                //green is dimmer

//Changing the gain changes the overall height of the bars in the display 
//by a proportionate amount. Note that increasing the gain for low-level 
//signals also increases background noise to a point where it may start to 
//show along the bottom of the display. m_shift can hide this.
const float m_gain = 240.0;                       //Input gain

//Shifts the bars down to conceal the zero-point as well as any additional 
//background noise. If random data shows at the very bottom of the display 
//when no input is present, increase this value by steps of 0.1 to just past 
//the point where it disappears. This should be 0.1 in a good quiet design.
const float m_shift = 1.0;                       //Shift bars down to mask background noise

//Controls how fast each spectrum bar shrinks back down. Larger numbers are faster.
const float m_decay = 1.0;                      //Speed of band decay

//Enable showing the red peak trace. Turn it off for a pure spectrum display.
const bool m_showPeak = true;                    //Show peaks

//How many 20ms cycles the peak line holds on the display before resetting. As the
//peak timer is restarted by peaks being nudged up, this should remain short or 
//the peak display tends to get stuck.
const int m_peakCounter = 5;

//How many pixels of spectrum need to show on the display for the peak line to appear.
//This hides the peak when no input is present in order to blank and save the display.
const int m_peakDisplayThreshold = 12;           //Minimum number of pixels under peak for it to show

//The noise gate mutes the input when the peak is below m_peakDisplayThreshold. This
//can be used to conceal narrow band background noise.
const bool m_noiseGate = true;

//Spectrum bar colour controls. These are pixel heights starting with zero at the bottom.
const int m_orangeSection = 10;                  //Height at which band bar turns orange
const int m_redSection = 14;                     //Height at which band bar turns red

//Shape of spread of frequency steps within the passband. Shifts the middle part left
//or right for visual balance. 0.05 for a hard logarithmic response curve to 0.40 for 
//a much more linear curve on two displays.
const float logScale = 0.14;  //Scale the logarithm curve deeper or shallower

//Enable the GPS clock.
//Comment out this whole line if no GPS is connected.
//#define ENABLE_GPSCLOCK

#ifdef ENABLE_GPSCLOCK
//GPS baud rate.
const int m_gpsBaud = 115200;

//Delay in 20ms ticks after peak disappears before GPS clock appears.
const int m_gpsDelay = 100;                   //2 seconds

//Time zone offset.
const int m_timeZone = +11;

#endif

//Uncomment this line to use a serial terminal (57600 baud) to see debugging
//messages.
//#define SERIAL_DEBUG

//-----------------------------------------------------------------------------------------------
// Shouldn't need to change stuff below here

//Screen size constants
const int m_amountOfRows = 16;                 //Number of rows
const int m_bankRegisters = 4;                 //4 registers per panel
const int m_bankOutputs = m_bankRegisters * 8; //Ouputs per panel
const unsigned char m_amountOfRegisters = m_amountOfPanels * m_bankRegisters; //Number of shift registers per row
const int m_amountOfColumns = m_amountOfRegisters * 8;  //Number of outputs per row
const int m_bufferSize=m_amountOfColumns * m_amountOfRows;  //Total size of single colour buffer

//Colour buffers are arranged as two buffers, one for each colour. These are laid out
//as linear frame buffer arrays with each byte corresponding to the pixel's brightness.
unsigned char * m_redPWMValues;
unsigned char * m_greenPWMValues;

//ISR variables
volatile unsigned char m_counter;                //PWM counter
unsigned char m_currentRow;                      //Displayed row
unsigned int m_rowOffset;                        //Offset into buffer for current row

//FFT display
bool m_refreshScreen = false;                    //Flag to completely redraw screen
unsigned int logGen1024[m_amountOfColumns][2] = {0}; //Linear1024 -> Log64 mapping
unsigned char m_bands[m_amountOfColumns] = {0};  //band pixel height
float m_bandVal[m_amountOfColumns] = {0.0};      //band FP value
const unsigned int logAmpSteps = 64;             //How many steps in the log amp converter
int logAmpOffset[logAmpSteps] = {0};             //Linear64 -> Log16 mapping

//Peak display
unsigned char m_peaks[m_amountOfColumns] = {0};  //peak pixel height
float m_peakVal[m_amountOfColumns] = {0.0};      //peak FP value

//GPS clock
#ifdef ENABLE_GPSCLOCK
bool m_gpsLock = false;                  //If the GPS has a lock
bool m_clockShowing = false;             //If the click is currently on screen
int gpsCounter = 0;                      //GPS display timeout counter
time_t m_currentTime = 0;                //current time
#endif

//Scrolling controls
bool scrolling = false;                  //If we are currently scrolling
int scrollPos = 0;                       //Current scrolling horizontal position offset
char *scrollString;                      //Pointer to current text string
uint8_t scrollY = 0;                     //Vertical position offset
uint8_t scrollFor = 0;                   //Text foreground colour
uint8_t scrollBk = 0;                    //Text background colour
const int scrollSpeed = 2;               //Scrolling speed in 20ms increments


// GUItool: begin automatically generated code
AudioInputAnalog         adc1(A1);        //xy=144,119
AudioAnalyzeFFT1024      fft1024_1;       //xy=498,64
AudioConnection          patchCord1(adc1, fft1024_1);
// GUItool: end automatically generated code

//-------------------------------------------------------------------------------------------------

// the setup() method runs once, when the sketch starts

void setup() {
  initDisplay();
  pinMode(m_ledPin, OUTPUT);
  AudioMemory(8);
#ifdef ENABLE_GPSCLOCK
  Serial1.begin(m_gpsBaud);
#endif
#ifdef SERIAL_DEBUG
  delay(1000);
  Serial.begin(57600);
#endif
  calcBands();
  calcAmplitudes();
  //Intro string
  hScroll(3, 1, 0, "Teensy Audio Spectrum Analyzer");// - v1.0 : Nick Metcalfe : 21/11/2015");
}  

// the loop() method runs over and over again,
// as long as the board has power

void loop() {
  static int peakCount = 0;      //Peak delay counter
  int barValue, barPeak;         //current values for bar and peak
  float maxPeak = 0;             //Sum of all peak values in frame
  static bool drawPeak = true;   //Show peak on display
  if (fft1024_1.available()) {
    if (m_refreshScreen) SetAll(0);
    for (int band = 0; band < m_amountOfColumns; band++) {
      //Get FFT data
      float fval = fft1024_1.read(logGen1024[band][0], logGen1024[band][1]);
      fval = fval * m_gain - m_shift;
      if (fval > logAmpSteps) fval = logAmpSteps;            //don't saturate the band

      //process bands bar value
      if (m_bandVal[band] > 0) m_bandVal[band] -= m_decay;   //decay current band
      if ((drawPeak || !m_noiseGate) && fval > m_bandVal[band]) m_bandVal[band] = fval; //set to current value if it's greater
      barValue = (int)m_bandVal[band];                       //reduce to a pixel location
      if (barValue > logAmpSteps - 1) barValue = logAmpSteps - 1; //apply limits
      if (barValue < 0) barValue = 0;
      barValue = logAmpOffset[barValue];

      //process peak bar value
      if (drawPeak || !m_noiseGate) fval = m_bandVal[band] + 0.1; //examine band data transposed slightly higher
      else fval += 0.1;
      if (fval > m_peakVal[band]) {                //if value is greater than stored data
        m_peakVal[band] = fval;                    //update stored data
        peakCount = m_peakCounter;                 //Start the peak display counter
      }
      barPeak = (int)m_peakVal[band];              //extract the pixel location of peak
      if (barPeak > logAmpSteps - 1) barPeak = logAmpSteps - 1; //apply limits
      if (barPeak < 0.0) barPeak = 0.0;
      barPeak = logAmpOffset[barPeak];
      maxPeak += barPeak;                          //sum up all the peaks
      
      //Erase previous band bar
      if (m_bands[band] <= barValue) {             //if current band bar is lower than previous
        setGreenPixel(band, m_bands[band], 0);     //erase pixel above
        setRedPixel(band, m_bands[band], 0);
      }
      //Erase previous peak
      if (m_showPeak && m_peaks[band] >= 0) setRedPixel(band, m_peaks[band], 0);
      
      //Draw new band bar. If scrolling, draw the whole bar. If not, draw just the new portion
      if (scrolling || m_refreshScreen || (barValue >= 0 && barValue > m_bands[band])) {
        int barBase;
        if (scrolling || m_refreshScreen) barBase = 0; //if scrolling, draw the whole bar
        else barBase = m_bands[band];               //else draw just the portion at the top
        if (barBase < 0) barBase = 0;
        //Bar drawing loop
        for (int i = barBase; i < barValue; i++) {
          if (i < m_redSection)                     //Less than red section always draws green
            setGreenPixel(band, i, m_greenIntensity);
          if (i >= m_orangeSection)                 //Above orange section always draws red
            setRedPixel(band, i, m_redIntensity);
        }
      }
      
      //Draw peak point
      int peakIntensity = (peakCount > 0) * m_redIntensity;//peakCount / 8 + 1;  //Peak intensity modulation disabled
      if (m_showPeak && drawPeak && peakCount > 0) setRedPixel(band, barPeak, peakIntensity);
      
      //Update pixel value trackers
      m_bands[band] = barValue;
      m_peaks[band] = barPeak;
    }
    m_refreshScreen = false;
    //Detect peak threshold - more than so many pixels under the peaks for it to show, and lingers until counter expires
    drawPeak = (maxPeak > m_peakDisplayThreshold && peakCount > 0);
    
    //Peak counter timeout
    if (peakCount > 0) {                                      //if the peak conter is active
      if (--peakCount == 0) {                                 //and decrementing it deactivates it
        for (int band = 0; band < m_amountOfColumns; band++) {  //clear the peak values
          m_peakVal[band] = 0;
        }
      }
    }

#ifdef ENABLE_GPSCLOCK
    handleGps();                                //Parse any incoming GPS messages
    //GPS display timeout counter - GPS clock delay after peaks disappear
    if (drawPeak) {                             //No clock if peaks are drawing
      gpsCounter = 0;                           //Reset the GPS display timeout counter
      if (m_clockShowing) {                     //if the clock was onscreen
        m_refreshScreen = true;                 //Redraw the screen
        m_clockShowing = false;                 //clock is no longer showing
      }
    }    
    if (gpsCounter < m_gpsDelay) gpsCounter++;  //increment the GPS display timeout counter
    //Draw the GPS clock    
    else drawGpsClock();                        //Timed out - draw the clock
#endif

  }
  //Manage the text scrolling
  hScrollStep();
  //Loop delay for about 50cps screen refresh
  delay(20);
}

//-------------------------------------------------------------------------------------------------
// ISR display handler

static IntervalTimer itimer;
//Initialise and start ISR timer
void InitTimer(void){
	itimer.begin(ShiftPWM_handleInterrupt,
	  1000000.0 / (m_ledFrequency * (m_maxBrightness+1)));
}

//Set the row output demultiplexers (2 x 74xx138)
// Optimised for pins 16, 17, 19, 18 as bits 0,1,2,3 of row
// See https://forum.pjrc.com/threads/17532-Tutorial-on-digital-I-O-ATMega-PIN-PORT-DDR-D-B-registers-vs-ARM-GPIO_PDIR-_PDOR?p=29216&viewfull=1#post29216
static inline void setRow(unsigned char row){
  GPIOB_PDOR = row & 0x0F;
}

//Fast inline function to present two-line serial RG data and perform one clock toggle
static inline void pwm_output_one_pin(const unsigned char counter, unsigned char * redPtr, unsigned char * greenPtr){
    digitalWriteFast(m_clockPin, LOW);
    digitalWriteFast(m_redPin, *(redPtr)<=counter );
    digitalWriteFast(m_greenPin, *(greenPtr)<=counter );
    digitalWriteFast(m_clockPin, HIGH);
}

//Main timer ISR
//We shift out a single line during the interrupt. The display has each bank of 
//32 outputs reversed, hence the funny pointer arithmetic
void ShiftPWM_handleInterrupt(void){
  unsigned char *redPtr = 0, *greenPtr = 0;
  int bankOffset = m_amountOfColumns;                      //Start past the last bank
 
  // Write shift register latch clock low 
  digitalWriteFast(m_latchPin, LOW);
  
  //Use port manipulation to send out all bits
  for(unsigned char i = 0; i < m_amountOfRegisters; i++){  // do one shift register at a time. This unrolls the loop for extra speed
    //Check for bank border
    if (i % m_bankRegisters == 0) {                        //edge of a bank?
      bankOffset -= m_bankOutputs;                         //skip backwards through banks
      redPtr=&m_redPWMValues[m_rowOffset + bankOffset];    //reset pointers to previous bank
      greenPtr=&m_greenPWMValues[m_rowOffset + bankOffset];
    }
    //Shift out 8 values for one register
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);  // This takes 12 or 13 clockcycles
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
    pwm_output_one_pin(m_counter, redPtr++, greenPtr++);
  }

  // Write shift register latch clock high
  digitalWriteFast(m_latchPin, HIGH);

  //Cycle through the PWM counter on each line and switch to next row when counter expires
  if(m_counter < m_maxBrightness){
    m_counter++;                       // Increase the counter for this row
  } else {                             // Row finished..
    m_counter=0;                       // Reset counter if it maximum brightness has been reached
    m_rowOffset += m_amountOfColumns;  // Move the pointer offset to next row
    if (++m_currentRow >= m_amountOfRows) {  //Just did last row?
      m_currentRow = 0;                //start again from the beginning
      m_rowOffset = 0;
    }
    setRow(m_currentRow);              //Switch display to new row
  }
}

//----------------------------------------------------------------------------------------------
// Display routines

void initDisplay() {
  //Reset variables
  m_counter = 0;
  m_redPWMValues = 0;
  m_greenPWMValues = 0;
  m_currentRow = 0;
  m_rowOffset = 0;  
  // initialize the digital pins.
  pinMode(m_enablePin, OUTPUT);
  digitalWrite(m_enablePin, LOW);
  pinMode(m_redPin, OUTPUT);
  pinMode(m_greenPin, OUTPUT);
  pinMode(m_clockPin, OUTPUT);
  pinMode(m_latchPin, OUTPUT);
  pinMode(m_aPin, OUTPUT);
  pinMode(m_bPin, OUTPUT);
  pinMode(m_cPin, OUTPUT);
  pinMode(m_dPin, OUTPUT);
  setRow(0);
  digitalWrite(m_clockPin, LOW);
  digitalWrite(m_redPin, LOW);
  digitalWrite(m_greenPin, LOW);
  //Allocate buffer memory
  AllocateBuffers();
  //Start timer
  InitTimer();
}

//Allocate and clear frame buffer memory
void AllocateBuffers(){
  cli(); // Disable interrupt
  m_redPWMValues = (unsigned char *) malloc(m_bufferSize); //resize array for PWMValues
  m_greenPWMValues = (unsigned char *) malloc(m_bufferSize); //resize array for PWMValues
  if (m_redPWMValues > 0 && m_greenPWMValues > 0) {
    SetAll(0);
  } else {
    Serial.println(F("Out of memory in SetAmountOfRegisters()"));
    //while(1){}
  }
  sei(); //Re-enable interrupt
}

//Fill frame buffer with fixed values
void SetAll(unsigned char red, unsigned char green){
  for(int k=0 ; k<(m_bufferSize);k++){
    m_redPWMValues[k]=red;
    m_greenPWMValues[k]=green;
  }
}
void SetAll(unsigned char value){
  SetAll(value, value);
}

//Red-only pixel routines, bottom-left coordinate system for FFT graphics
void setRedPixel(uint16_t x ,uint8_t y, uint8_t red)
{
  unsigned int address = ((m_amountOfRows - 1) - y) * m_amountOfColumns + x;
  if (address >= 0 && address < m_bufferSize)
    m_redPWMValues[address]=red;
}

//Green-only pixel routines, bottom-left coordinate system for FFT graphics
void setGreenPixel(uint16_t x ,uint8_t y, uint8_t green)
{
  unsigned int address = ((m_amountOfRows - 1) - y) * m_amountOfColumns + x;
  if (address >= 0 && address < m_bufferSize)
    m_greenPWMValues[address]=green;
}

//Base graphic pixel routine, top-left coordinate system for text
// Full-colour version
void setPixel(uint16_t x ,uint8_t y, uint8_t red, uint8_t green)
{
  unsigned int address = y * m_amountOfColumns + x;
  if (address >= 0 && address < m_bufferSize)
  {
    m_greenPWMValues[address]=green;
    m_redPWMValues[address]=red;
  }
}

//Base graphic pixel routine, top-left coordinate system for text
// limited bit-colour version using default intensities
void setPixel(uint16_t x ,uint8_t y, uint8_t colour) //color 2 Bit, (R)ed (G)reen 0b000000RG
{
  setPixel(x ,y, ((colour&0x02)>0)*m_redIntensity, (colour&0x01)*m_greenIntensity);
}

//Print a character
void printChar(uint16_t x,uint8_t y, uint8_t For_color, uint8_t Bk_color, char ch)
{
  uint16_t xx,yy,offset;
  xx=0;
  yy=0;
  offset=0;
  ch = ch-32;
  for (yy=0; yy < 7; yy++)
  {
    for (xx=0; xx < 5; xx++)
    {
      if (bitRead(pgm_read_byte(&font5x7[ch][yy]),4-xx)) // 4 == Font witdh -1
      {
        setPixel(x+xx,y+yy,For_color);
      }
      else
      {
        setPixel(x+xx,y+yy,Bk_color);
      }
    }
  }
}

//Print a whole string
void printString(uint16_t x, uint8_t y, uint8_t For_color, uint8_t Bk_color,char *p)
{   
   while(*p!='\0')
   {
     printChar(x,y,For_color,Bk_color,*p);
     x+=6; // 6 = font width + 1 pixel space
     p++;
   }
}

//Scroll a string over the display in the background
void hScroll(uint8_t y, uint8_t For_color, uint8_t Bk_color,char *mystring) 
{
  scrollString = mystring;
  scrollPos = -(m_amountOfColumns) - 1;
  scrollY= y;
  scrollFor = For_color;
  scrollBk = Bk_color;
  scrolling = true;
}

//Manage any scrolling text
//called periodically from loop()
void hScrollStep()
{
  static int div = 0;
  if (!scrolling) return;
  if (div++ < scrollSpeed) {
    return;
  }
  div = 0;
  int offset = scrollPos;
  int strLen = lenString(scrollString)*6;
  for (int xx=0; xx<m_amountOfColumns; xx++)
  {
    for (int yy=0; yy<7; yy++)
    {
      int c;
      if (getPixelHString(xx+offset,yy,scrollString)) c = scrollFor; else c = scrollBk;
      if (xx+offset >= 0 && xx+offset <= strLen-1) setPixel(xx,yy+scrollY,c);
    }
  }
  if (scrollPos++ == strLen) {
    scrolling = false;
  }
}

//Find the character length of a C string
unsigned int lenString(char *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}

//Get a pixel from a particular character in the font map
byte getPixelChar(uint8_t x, uint8_t y, char ch)
{
   ch = ch-32;
   if (x > 4) return 0; // 4 = font Width -1
   return bitRead(pgm_read_byte(&font5x7[ch][y]),4-x); // 4 = Font witdh -1  
}

//Get a pixel from a coordinate in a font-mapped string
byte getPixelHString(uint16_t x, uint16_t y, char *p)
{
   if (x < 0) return 0;
   p=p+x/6;
   return getPixelChar(x%6,y,*p);
}

//------------------------------------------------------------------------------------------------
// Logarithmic band calculations
// http://code.compartmental.net/2007/03/21/fft-averages/

const int sampleRate = 16384;
const int timeSize = 1024;    //FFT points
const int bandShift = 2;      //First two bands contain 50hz and display switching noise. Hide them..

//Calculate a logarithmic set of band ranges
void calcBands(void) {
  int bandOffset;     //Bring us back toward zero for larger values of logScale
  for (int i = 0; i < m_amountOfColumns; i++)
  {
    int lowFreq = (int)((sampleRate/2) / (float)pow(logScale / m_amountOfPanels + 1, m_amountOfColumns - i)) - 1;
    int hiFreq = (int)((sampleRate/2) / (float)pow(logScale / m_amountOfPanels + 1, m_amountOfColumns - 1 - i)) - 1;
    int lowBound = freqToIndex(lowFreq);
    int hiBound = freqToIndex(hiFreq);
    if (i == 0) bandOffset = lowBound;
    lowBound -= bandOffset;
    hiBound -= bandOffset + 1;
    if (lowBound < i + bandShift) lowBound = i + bandShift;
    if (hiBound < i + bandShift) hiBound = i + bandShift;
    if (lowBound > hiBound) lowBound = hiBound;
    if (i == m_amountOfColumns - 1) hiBound = 511;
    logGen1024[i][0] = lowBound;
    logGen1024[i][1] = hiBound;
#ifdef SERIAL_DEBUG
    Serial.print(i);
    Serial.print(" - Bounds:");
    Serial.print(lowBound);
    Serial.print(", ");
    Serial.println(hiBound);
#endif
  }
}

//Determine the FFT sample bandwidth
float getBandWidth()
{
  return (2.0/(float)timeSize) * (sampleRate / 2.0);
}

//Convert a frequency to a FFT bin index 
int freqToIndex(int freq)
{
  // special case: freq is lower than the bandwidth of spectrum[0]
  if ( freq < getBandWidth()/2 ) return 0;
  // special case: freq is within the bandwidth of spectrum[512]
  if ( freq > sampleRate/2 - getBandWidth()/2 ) return (timeSize / 2) - 1;
  // all other cases
  float fraction = (float)freq/(float) sampleRate;
  int i = (int)(timeSize * fraction);
  return i;
}

//Calculate a logarithmic amplitude lookup table
void calcAmplitudes() {
  for (int i = 0; i < logAmpSteps; i++)
  {  
    float db = 1.0 - ((float) i / (float)logAmpSteps);
    db = (1.0 - (db * db)) * (m_amountOfRows + 1);
    if (db < 0) logAmpOffset[i] = -1;    
    else logAmpOffset[i] = (int)db;
#ifdef SERIAL_DEBUG
    Serial.print(i);
    Serial.print(" - Amp:");
    Serial.println(logAmpOffset[i]);
#endif
  }
}


//--------------------------------------------------------------------------------------------------
// GPS Clock

#ifdef ENABLE_GPSCLOCK

//GPS NMEA message string containing time and date
const char m_gpsCmd[8] = "$GPRMC,";

//Simple GPS string parser
void handleGps() {
  static unsigned char state = 0;
  static int acc = 0;
  //static struct tm m_timeInfo = {0};
  static unsigned char m_hour = 0;
  static unsigned char m_minute = 0;
  static unsigned char m_second = 0;
  static unsigned char m_day = 0;
  static unsigned char m_month = 0;
  static unsigned char m_year = 0;
  while (Serial1.available()) {
    char b = Serial1.read();
    //Look for message string
    if (state < 7 && b != m_gpsCmd[state]) {
      state = 0;
      continue;
    }
    //Start converting time string
    else if (state == 7) {
      acc = (b - '0') * 10;
      //Check it was a number? If not, no GPS lock.
      m_gpsLock = (acc >= 0 && acc <= 9);
      if (!m_gpsLock) {
        state = 0;
        continue;
      }
    }
    else if (state == 8) m_hour = acc + (b - '0');
    else if (state == 9) acc = (b - '0') * 10;
    else if (state == 10) m_minute = acc + (b - '0');
    else if (state == 11) acc = (b - '0') * 10;
    else if (state == 12) m_second = acc + (b - '0');
    //Skip other data to get to date string
    else if (state > 12 && state < 21) {
      if (b == ',') state++;
      continue;
    }
    //Start converting date string 
    else if (state == 21) acc = (b - '0') * 10;
    else if (state == 22) m_day = acc + (b - '0');
    else if (state == 23) acc = (b - '0') * 10;
    else if (state == 24) m_month = acc + (b - '0');
    else if (state == 25) acc = (b - '0') * 10;
    else if (state == 26) m_year = acc + (b - '0');
    //Captured complete time and date
    else if (state >= 27) {
      //Sanity check      
      if (m_hour >= 0 && m_hour < 24 && m_minute >= 0 &&     
          m_minute < 60 && m_second >= 0 && m_second < 60 &&
          m_day > 0 && m_day < 32 && m_month > 0 &&
          m_month < 13 && m_year > 0 && m_year < 100) {
        //Construct a time_t structure
        struct tm * timeInfo;
        timeInfo = localtime ( &m_currentTime );
        timeInfo->tm_hour = m_hour;
        timeInfo->tm_min = m_minute;
        timeInfo->tm_sec = m_second;
        timeInfo->tm_year = m_year + 100;
        timeInfo->tm_mon = m_month - 1;
        timeInfo->tm_mday = m_day;
        m_currentTime = mktime(timeInfo);
        m_currentTime += (m_timeZone * 3600);        //Adjust for time zone        
#ifdef SERIAL_DEBUG
        timeInfo = localtime (&m_currentTime);
        Serial.printf ("Current local time and date: %s", asctime(timeInfo));
#endif
      }
      state = 0;
      continue;
    }
    state++;    
  }
}

//Draw the GPS clock
void drawGpsClock(){
  static char clockBuffer[100] = {0};
  if(!scrolling && m_gpsLock) {
    struct tm * timeInfo;
    timeInfo = localtime (&m_currentTime);
    sprintf(clockBuffer, "%d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    printString(8, 1, 1, 0, clockBuffer);
    sprintf(clockBuffer, "%d/%d/%d", timeInfo->tm_mday, timeInfo->tm_mon+1, timeInfo->tm_year - 100);
    printString(8, 8, 2, 0, clockBuffer);
    m_clockShowing = true;
  }
}  

#endif


