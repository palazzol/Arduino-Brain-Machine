
/***************************************************
  Sketch: Sound & Light Machine for Arduino
  Author: Chris Sparnicht - http://low.li
  Creation Date: 2011.01.31
  Last Modification Date: 2011.02.12
  License: Creative Commons 2.5 Attrib. & Share Alike

  Derivation and Notes:
  Make sure you have audio stereo 10K Ohm potentiometer to reduce
  the volume of the audio with your headset. If you don't,
  you might damage your ear drums, your arduino or your headset.

  Included with this sketch is a png diagram.

  This arduino sketch is based on the original Sound & Light Machine
  by - Mitch Altman - 19-Mar-07 as featured in Make Magazine 10.
  http://makezine.com/10/brainwave/

  See notes in code below for how I adapted Mitch Altman's version for Arduino

  The sleep coding comes partially from here:
  http://www.arduino.cc/playground/Learning/ArduinoSleepCode
***************************************************/

/***************************************************
  SOME INFORMATION ABOUT PROGMEM:
  First, you have to use #include <avr/pgmspace.h> for table arrays - PROGMEM
  The Arduino compiler creates code that will transfer all constants into RAM when the microcontroller
  resets.  This hardward could probably hold all this data in memory, but since the original system
  used a table (chunkybrainwaveTab) that is was too large to transfer into RAM in the original microcontroller,
  we're taking the same approach.  This is accomplished by using the library for PROGMEM.
  (This is used below in the definition for the chunkybrainwaveTab).  Since the
  C compiler assumes that constants are in RAM, rather than in program memory, when accessing
  the chunkybrainwaveTab, we need to use the pgm_read_byte() and pgm_read_word() macros, and we need
  to use the brainwveTab as an address, i.e., precede it with "&".  For example, to access
  chunkybrainwaveTab[3].bwType, which is a byte, this is how to do it:
   pgm_read_byte( &chunkybrainwaveTab[3].bwType );
  And to access chunkybrainwaveTab[3].bwDuration, which is a word, this is how to do it:
   pgm_read_word( &chunkybrainwaveTab[3].bwDuration );
 ***************************************************/

/***************************************************
  LIBRARIES - Define necessary libraries here.
***************************************************/
#include <avr/pgmspace.h> // for arrays - PROGMEM 
#include <avr/sleep.h> // A library to control the sleep mode
#include <avr/power.h> // A library to control power

#include <EEPROM.h>

// =============== uncomment for serial debugging ===============
// #define DEBUG
// =========================================================================

// By default, LEDs are connected to Supply (Common Anode)
// Add an option to use LEDs connected to ground (Common Cathode)
//#define LEDS_TO_GROUND

// By default, use Tone library.  Use this define if you want the
// beat frequency to be exact.  This forces the use of certain pins 
// for the audio, and is not portable.
//#define USE_RAW_TIMERS

#ifndef USE_RAW_TIMERS

// This class abstracts the connection to the Tone library
// There are two versions.  This one is equivalent to the original code
// and uses the Tone library
// The second version uses raw timers, so it's not as portable but
// the code is very precise.  It particular, the beat frequency is exact.

#include <Tone.h> // Download from https://github.com/bhagman/Tone

class TonePair
{
  public:
    TonePair() {};
    int begin(int rightEarPin, int leftEarPin) {
      rightEarTone.begin(rightEarPin); // Tone rightEarTone begins at pin output rightEarPin
      leftEarTone.begin(leftEarPin); // Tone leftEarTone begins at pin output leftEarPin
    }
    void play(float centralTone, float binauralBeatFreq) {
      rightEarTone.play(centralTone - (binauralBeatFreq/2));
      leftEarTone.play(centralTone + (binauralBeatFreq/2));
    }
    void stop() {
      rightEarTone.stop();
      leftEarTone.stop();
    }
  public:
    Tone rightEarTone;
    Tone leftEarTone;
};

#else

#include <avr/io.h>

class TonePair
{
  public:
    TonePair() {};
    void begin(int rightEarPin, int leftEarPin) {
      // Must use these pins, they are tied to the right timers
      if ((rightEarPin != 11) and (leftEarPin != 9)) {
#ifdef DEBUG
        Serial.println("Error: Must use pins 11 and 9 to use Raw Timers!");
#endif
        return -1;
      }     
      DDRB = 0b00000000;   // set PB1,PB3 pins as outputs
      PORTB = 0x00;        // all PORTB output pins Off
      
      // Right ear tone will use Timer2, 8-bit resolution
      TIMSK2 = 0x00;       // no Timer interrupts enabled
      //   8-bit Timer2 OC2A (PB3, pin 11) is set up for Fast PWM mode, toggling output on each compare
      //   Fclk = Clock = 16MHz
      //   Prescale = 256
      //   F = Fclk / (2 * Prescale * (1 + OCR2A) ) = 200.321Hz
      TCCR2A = 0b01000011;  // COM0A1:0=01 to toggle OC0A on Compare Match
                            // COM0B1:0=00 to disconnect OC0B
                            // bits 3:2 are unused
                            // WGM01:00=11 for Fast PWM Mode (WGM02=1 in TCCR0B)
      TCCR2B = 0b00001110;  // FOC0A=0 (no force compare)
                            // F0C0B=0 (no force compare)
                            // bits 5:4 are unused
                            // WGM2=1 for fast PWM Mode (WGM01:00=11 in TCCR0A)
                            // CS02:00=110 for divide by 256 prescaler
      TIMSK1 = 0x00;        //  no Timer interrupts enabled
      // Left ear tone will use Timer1, 16-bit resolution
      // set up T1 to accept Offset Frequencies on Right ear speaker through OC1A (but don't actually start the Timer1 here)
      //   16-bit Timer1 OC1A (PB1, pin 9) is set up for CTC mode, toggling output on each compare
      //   Fclk = Clock = 8MHz
      //   Prescale = 1
      //   F = Fclk / (2 * Prescale * (1 + OCR1A) )
      TCCR1A = 0b01000011;  // COM1A1:0=01 to toggle OC1A on Compare Match
                            // COM1B1:0=00 to disconnect OC1B
                            // bits 3:2 are unused
                            // WGM11:10=11 for Fast PWM Mode (WGM13:12=11 in TCCR1B)
      TCCR1B = 0b00011001;  // ICNC1=0 (no Noise Canceller)
                            // ICES1=0 (don't care about Input Capture Edge)
                            // bit 5 is unused
                            // WGM13:12=11 for for Fast PWM Mode (WGM11:11=00 in TCCR1A)
                            // CS12:10=001 for divide by 1 prescaler
      TCCR1C = 0b00000000;  // FOC1A=0 (no Force Output Compare for Channel A)
                            // FOC1B=0 (no Force Output Compare for Channel B)
                            // bits 5:0 are unused

    }
    void play(float centralTone, float binauralBeat) {
      int lowFreqPeriod = int(31250.0/(centralTone - binauralBeat/2) + 0.5); // on OC2A (PB3, pin 11)
      float actualLowFreq = 31250.0/lowFreqPeriod;
      float actualHighFreq = actualLowFreq + binauralBeat;
      int highFreqPeriod = int(8000000.0/actualHighFreq + 0.5);
      
      OCR2A = lowFreqPeriod - 1;  // on OC2A (PB3, pin 11)
      OCR1A = highFreqPeriod - 1; // on OC0A (PB1, pin 9)
      
      DDRB = 0b00001010;   // set PB1,PB3 pins as outputs
    }
    void stop() {
      DDRB = 0b00000000;   // set PB1,PB3 pins as inputs
    }
};

#endif

/***************************************************
  GLOBALS
  We isolate calls to pins with these globals so we can change
  which pin we'll use i one please, rather than having to search and replace
  in many places.
***************************************************/

#ifdef USE_RAW_TIMERS
// You must use 11 and 9 like this with Raw Timers!!
#define rightEarPin 11 // Define pinout for right ear
#define leftEarPin 9 // Define pinout for left ear
#else
#define rightEarPin 9 // Define pinout for right ear
#define leftEarPin 10 // Define pinout for left ear
#endif
#define rightEyePin 5 // Define pinout for right eye
#define leftEyePin 6 // Define pinout for left eye
#define interruptPin 2 // the input pin where the pushbutton is connected.
#define potPin A0 // user input potentiometer (session selection)

int LEDIntensity = 127; // Default value, will be overridden by valid value in EEPROM

#ifndef LEDS_TO_GROUND
// Common anode. 255 is off
#define LED_ON (255-LEDIntensity)
#define LED_OFF 255
#else
#define LED_ON (LEDIntensity)
#define LED_OFF 0
#endif

// ====== Altman's "chunky" frequency-hopping method =======

/***************************************************
  BRAINWAVE TABLE
  See 'Some information about PROGMEM' above.
  Table of values for meditation start with
  lots of Beta (awake / conscious)
  add Alpha (dreamy / trancy to connect with
      subconscious Theta that'll be coming up)
  reduce Beta (less conscious)
  start adding Theta (more subconscious)
  pulse in some Delta (creativity)
  and then reverse the above to come up refreshed
***************************************************/
struct chunkyBrainwaveElement {
  char bwType;  // 'a' for Alpha, 'b' for Beta, 't' for Theta,'d' for Delta or 'g' for gamma ('0' signifies last entry in table
  // A, B, T, D and G offer alternating flash instead of concurrent flash.
  int bwDuration;  // Duration of this Brainwave Type (divide by 100 to get seconds)
};

const chunkyBrainwaveElement chunkybrainwaveTab[] PROGMEM = {
  { 'b', 6000 },
  { 'a', 1000 },
  { 'b', 2000 },
  { 'a', 1500 },
  { 'b', 1500 },
  { 'a', 2000 },
  { 'b', 1000 },
  { 'a', 3000 },
  { 'b',  500 },
  { 'a', 6000 },
  { 't', 1000 },
  { 'A', 3000 },
  { 't', 2000 },
  { 'a', 2000 },
  { 't', 3000 },
  { 'A', 1500 },
  { 't', 6000 },
  { 'a', 1000 },
  { 'b',  100 },
  { 'a',  500 },
  { 'T', 5500 },
  { 'd',  100 },
  { 't', 4500 },
  { 'd',  500 },
  { 'T', 3500 },
  { 'd', 1000 },
  { 't', 2500 },
  { 'd', 1500 },
  { 'g',  100 },
  { 'T',  500 },
  { 'g',  100 },
  { 'd', 3000 },
  { 'g',  500 },
  { 'd', 6000 },
  { 'g', 1000 },
  { 'D', 3000 },
  { 'g',  500 },
  { 'd', 1500 },
  { 'g',  100 },
  { 't', 1000 },
  { 'D', 1000 },
  { 't', 2000 },
  { 'a',  100 },
  { 'd', 1000 },
  { 't', 3000 },
  { 'a',  500 },
  { 'B',  100 },
  { 'a', 1000 },
  { 't', 2200 },
  { 'A', 1500 },
  { 'b',  100 },
  { 'a', 3000 },
  { 'b',  500 },
  { 'a', 2000 },
  { 'B', 1200 },
  { 'a', 1500 },
  { 'b', 2000 },
  { 'a', 1000 },
  { 'b', 2500 },
  { 'A',  500 },
  { 'b', 6000 },
  { '0', 0 }
};


/***************************************************
  VARIABLES for tone generator
  The difference in Hz between two close tones can
  cause a 'beat'. The brain recognizes a pulse
  between the right ear and the left ear due
  to the difference between the two tones.
  Instead of assuming that one ear will always
  have a specific tone, we assume a central tone
  and create tones on the fly half the beat up
  and down from the central tone.
  If we set a central tone of 200, we can expect
  the following tones to be generated:
  Hz:      R Ear    L Ear    Beat
  Beta:    192.80   207.20   14.4
  Alpha:   194.45   205.55   11.1
  Theta:   197.00   203.00    6.0
  Delta:   198.90   201.10    2.2

  You can use any central tone you like. I think a
  lower tone between 100 and 220 is easier on the ears
  than higher tones for a meditation or relaxation.
  Others prefer something around 440 (A above Middle C).
  Isolating the central tone makes it easy for
  the user to choose a preferred frequency base.

***************************************************/
float binauralBeat[] = { 14.4, 11.1, 6.0, 2.2, 40.4 }; // For beta, alpha, gamma and delta beat differences.
TonePair tonePair;
float centralTone = 440.0; //We're starting at this tone and spreading the binaural beat from there.

//Blink statuses for function 'blink_LEDs' and 'alt_blink_LEDS
unsigned long int duration = 0;
unsigned long int onTime = 0;
unsigned long int offTime = 0;


// ====== Mindplace.com "creamy" frequency-transition method =======

struct creamyBrainwaveElement {
  int duration;  // Seconds
  float frequency; // Hz
};

const creamyBrainwaveElement proteusGoodMorning04[] = {
  {0, 8}, {120, 16}, {60, 25}, {60, 25}, {60, 20}, {60, 28}, {60, 20}, {60, 28},
  {60, 20}, {60, 28}, {60, 20}, {60, 28}, {60, 20}, {60, 8}, {60, 20}, {0, 0}
};

const creamyBrainwaveElement proteusGoodNight43[] = {
  {0, 9}, {280, 6}, {300, 3}, {200, 5}, {100, 3}, {20, 2}, {0, 0}
};

const creamyBrainwaveElement proteusVisuals22[] = {
  {0, 8}, {60, 10}, {60, 16}, {60, 20}, {0, 16}, {60, 24}, {0, 20}, {120, 28}, {60, 24},
  {60, 30}, {300, 24}, {30, 12}, {30, 24}, {30, 16}, {120, 30}, {150, 8}, {60, 8}, {0, 0}
};

const creamyBrainwaveElement proteusVisuals33[] = {
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15}, {0, 0}
};


const creamyBrainwaveElement proteusMeditation10[] = {
  {0, 16}, {60, 16}, {240, 4}, {360, 4}, {120, 2}, {1080, 2}, {60, 8}, {60, 24},
  {120, 24}, {0, 0}
};



/***************************************************
  Session selection potentiometer
***************************************************/

#define NUM_SESSIONS 6
#define SESSION_GOOD_MORNING 0
#define SESSION_GOOD_NIGHT 1
#define SESSION_VISUALS 2
#define SESSION_MEDITATION 3
#define SESSION_MEDITATION_CHUNKY 4
#define SESSION_SETUP 5

int currentSession;

#define NUM_PROTEUS_SESSIONS 4

const creamyBrainwaveElement *proteusSessions[NUM_PROTEUS_SESSIONS] = {
  proteusGoodMorning04, proteusGoodNight43,
  proteusVisuals33, proteusMeditation10
};

int blink_patterns[NUM_SESSIONS] = {
  0B100000000000, // .     SESSION_GOOD_MORNING
  0B100100000000, // ..    SESSION_GOOD_NIGHT
  0B100100100000, // ...   SESSION_VISUALS
  0B111000000000, // -     SESSION_MEDITATION
  0B111001110000, // --    SESSION_MEDITATION_CHUNKY
  0B101010101010, // blink SESSION_SETUP
};

#define BLINK_STATE(n) (((blink_patterns[n]>>n)&1)?HIGH:LOW)

#define BLINK_SEGMENT_DURATION 100 // milliseconds
#define BLINK_NUM_SEGMENTS 12
#define CURRENT_BLINK_SEGMENT ((millis()/BLINK_SEGMENT_DURATION)%BLINK_NUM_SEGMENTS)

int mapPot(int mapMin, int mapMax) {
  // trim a bit of the edge values, because POTs are lousy and might not get there
  return constrain(
           map(analogRead(potPin), 23, 1000, mapMin, mapMax),
           mapMin, mapMax);
}

void setLEDs(int state) {
  analogWrite(rightEyePin, state);
  analogWrite(leftEyePin, state);
}

void blinkSessionSelection(int session) {
  setLEDs(blink_patterns[session] >> CURRENT_BLINK_SEGMENT & 1 ? LED_ON : LED_OFF);
}

/***************************************************
  Button interrupt
***************************************************/
#define STATE_READY 0
#define STATE_RUNNING 1
#define STATE_SLEEPING 2
volatile int machineState = STATE_READY;

void buttonInterrupt()
{
  switch (machineState) {
    case STATE_READY:
      machineState = STATE_RUNNING;
      break;
    case STATE_RUNNING:
    case STATE_SLEEPING:
      machineState = STATE_READY; // Back to the normal ready/running cycle
      break;
  }
}

void init_from_EEPROM()
{
  // Format of EEPROM
  // 0/1 0x1234 - Magic Number
  // 2 - byte version
  // 3 - LEDIntensity
  
  int header = EEPROM.read(0);
  header = (header << 8) + EEPROM.read(1);
  
  if (header != 0x1234) // Not initialized
  {
      EEPROM.write(0, 0x12);
      EEPROM.write(1, 0x34);
      EEPROM.write(2, 0x00);
      EEPROM.write(3, LEDIntensity);
  }
  byte version = EEPROM.read(2);
  if (version == 0)
  {
      // This is an int, but I think 255 is the max value
      LEDIntensity = EEPROM.read(3);
  }
}

/***************************************************
  SETUP defines pins and tones.
  Arduino pins we'll use:
  pin  2 - push button (interrupt)
  pin  5 - right ear
  pin  6 - left ear
  pin  9 - Left eye LED1
  pin 10 - Right eye LED2
  pin 11 - Button input
  pin A0 - session selector potentiometer
  pin 5V - for common anode on LED's
  pin GND - ground for tones
*/
void setup()  {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  tonePair.begin(rightEarPin, leftEarPin); // Set pins for right and left ears
  pinMode(rightEyePin, OUTPUT); // Pin output at rightEye
  pinMode(leftEyePin, OUTPUT); // Pin output at leftEye
  pinMode(interruptPin, INPUT_PULLUP); // User input (push button)
  pinMode(potPin, INPUT); // User input (potentiometer)
  init_from_EEPROM();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, FALLING);
}


/***************************************************
   MAIN LOOP - tells our program what to do.
***************************************************/

void loop() {
  int j;
#ifdef DEBUG
  Serial.println("Waiting for session selection...");
#endif
  while (machineState == STATE_READY) {
    currentSession = mapPot(0, NUM_SESSIONS - 1);
    blinkSessionSelection(currentSession);
    delay(50);
  }
  setLEDs(LED_OFF);


#ifdef DEBUG
  Serial.print("Chose session #");
  Serial.println(currentSession);
#endif
  unsigned long startedAt = millis();
  j = 0;
  if (currentSession < NUM_PROTEUS_SESSIONS) {
    creamyBrainwaveElement *session = proteusSessions[currentSession];
    float currentFrequency = 0.0;
    while (session[j].frequency > 0.0) { // 0.0 signifies end of table
#ifdef DEBUG
      Serial.print(j);
      Serial.print(' ');
      Serial.print(currentFrequency);
      Serial.print(" -> ");
      Serial.print(session[j].frequency);
      Serial.print(" in ");
      Serial.print(session[j].duration);
      Serial.print(" @");
      Serial.println(millis() - startedAt);
#endif
      if (session[j].duration) {
        float elapsedDms = 0; // decimilliseconds since element's start
        float totalDms = session[j].duration * 10000.0;
        while (elapsedDms < totalDms) {
          float freq = currentFrequency + (session[j].frequency - currentFrequency) * (elapsedDms / totalDms);
          tonePair.play(centralTone, freq);
          unsigned long halfWaveLength = round(5000.0 / freq);
          setLEDs(LED_ON);
          if (delay_decimiliseconds(halfWaveLength)) {
            break;
          }
          setLEDs(LED_OFF);
          if (delay_decimiliseconds(halfWaveLength)) {
            break;
          }
          elapsedDms += (2.0 * halfWaveLength);
        };
      };
      if (machineState != STATE_RUNNING) {
        break; // interrupt button was pressed
      };
      currentFrequency = session[j].frequency;
      j++;
    };
  } else if (currentSession == SESSION_MEDITATION_CHUNKY) {
    while (pgm_read_byte(&chunkybrainwaveTab[j].bwType) != '0') {  // '0' signifies end of table
#ifdef DEBUG
      Serial.print(j);
      Serial.print(' ');
      Serial.print((char)(pgm_read_byte(&chunkybrainwaveTab[j].bwType)));
      Serial.print(" @");
      Serial.println(millis() - startedAt);
#endif
      if (do_chunky_brainwave_element(j)) {
        // interrupt button got us out of STATE_RUNNING
        break;
      }
      j++;
    }
  } else if (currentSession == SESSION_SETUP) {
    while (machineState == STATE_RUNNING) {
      LEDIntensity = mapPot(31, 255);
      setLEDs(LED_ON);
      delay(50);
    }
    EEPROM.update(0, LEDIntensity);
#ifdef DEBUG
    Serial.print("Set intensity (0-255) to ");
    Serial.println(LEDIntensity);
#endif
  };

  setLEDs(LED_OFF);
  tonePair.stop();
#ifdef DEBUG
  Serial.print("Done #");
  Serial.print(currentSession);
  Serial.print(' ');
  Serial.println(millis());
#endif
  if (machineState != STATE_READY) {
    // Session finished (we're not here due to an interrupt button push)
    // Shut down everything and put the CPU to sleep
    machineState = STATE_SLEEPING;
#ifdef DEBUG
    Serial.println("Sleeping...");
    delay(1000); // let the dust settle...
#endif
    sleep_enable();          // enables the sleep bit in the mcucr register so sleep is possible. just a safety pin
    sleep_mode();            // here the device is actually put to sleep
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    sleep_disable();            // first thing after waking from sleep: disable sleep...
    delay(1000); // let the dust settle...
#ifdef DEBUG
    Serial.println("Woke up.");
#endif
  }
}

/***************************************************
  This function delays the specified number of 1/10 milliseconds
***************************************************/

bool delay_decimiliseconds(unsigned long int dms) {
  // returns true if interrupt button got us out of STATE_RUNNING
  // https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/ says
  // "Currently, the largest value that will produce an accurate delay is 16383"
  // so we do each decimilisecond as a separate call
  for (int i = 0; i < dms; i++) {
    delayMicroseconds(100); //
    if (machineState != STATE_RUNNING) {
      return true; // there was an interrupt
    }
  }
  return false;
}

/***************************************************
  This function blinks the LEDs
  (connected to Pin 6, Pin 5 -
  for Left eye, Right eye, respectively)
  and keeps them blinking for the Duration specified
  (Duration given in 1/10 millisecs).
  This function also acts as a delay for the Duration specified.
  In this particular instance, digitalWrites are set
  for common anode, so "on" = LOW and "off" = HIGH.
***************************************************/

bool blink_LEDs( int duration, int onTime, int offTime) {
  // returns true if interrupt button got us out of STATE_RUNNING
  unsigned long int longDuration = duration*100L;
  for (int i = 0; i < (longDuration / (onTime + offTime)); i++) {
    analogWrite(rightEyePin, LED_ON);
    analogWrite(leftEyePin, LED_ON);
    // turn on LEDs
    if (delay_decimiliseconds(onTime)) {  //   for onTime
      return true;
    }
    analogWrite(rightEyePin, LED_OFF);
    analogWrite(leftEyePin, LED_OFF);
    // turn off LEDs
    if (delay_decimiliseconds(offTime)) { //   for offTime
      return true;
    }
  }
  return false;
}

bool alt_blink_LEDs( int duration, int onTime, int offTime) {
  // returns true if interrupt button got us out of STATE_RUNNING
  unsigned long int longDuration = duration*100L;
  for (int i = 0; i < (longDuration / (onTime + offTime)); i++) {
    analogWrite(rightEyePin, LED_ON);
    analogWrite(leftEyePin, LED_OFF);
    if (delay_decimiliseconds(onTime)) {  //   for onTime
      return true;
    }
    analogWrite(rightEyePin, LED_OFF);
    analogWrite(leftEyePin, LED_ON);
    if (delay_decimiliseconds(offTime)) { //   for offTime
      return true;
    }
  }
  return false;
}
/***************************************************
  This function starts with a central audio frequency and
  splits the difference between two tones
  to create a binaural beat (between Left and Right ears)
  for a Brainwave Element.
  (See notes above for beat creation method.)
***************************************************/

bool do_chunky_brainwave_element(int index) {
  // returns whatever bilnk_LEDs or alt_blink_LEDs returned,
  // i.e. true if interrupt button got us out of STATE_RUNNING
  char brainChr = pgm_read_byte(&chunkybrainwaveTab[index].bwType);

  switch (brainChr) {
    case 'b':
      // Beta
      tonePair.play(centralTone, binauralBeat[0]);
      //  Generate binaural beat of 14.4Hz
      //  delay for the time specified in the table while blinking the LEDs at the correct rate
      return blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 347, 347 );

    case 'B':
      // Beta - with alternating blinks
      tonePair.play(centralTone, binauralBeat[0]);
      //  Generate binaural beat of 14.4Hz
      //  delay for the time specified in the table while blinking the LEDs at the correct rate
      return alt_blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 347, 347 );

    case 'a':
      // Alpha
      tonePair.play(centralTone, binauralBeat[1]);
      // Generates a binaural beat of 11.1Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 451, 450 );

    case 'A':
      // Alpha
      tonePair.play(centralTone, binauralBeat[1]);
      // Generates a binaural beat of 11.1Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return alt_blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 451, 450 );

    case 't':
      // Theta
      // start Timer 1 with the correct Offset Frequency for a binaural beat for the Brainwave Type
      //   to Right ear speaker through output OC1A (PB3, pin 15)
      tonePair.play(centralTone, binauralBeat[2]);
      // Generates a binaural beat of 6.0Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 835, 835 );

    case 'T':
      // Theta
      // start Timer 1 with the correct Offset Frequency for a binaural beat for the Brainwave Type
      //   to Right ear speaker through output OC1A (PB3, pin 15)
      tonePair.play(centralTone, binauralBeat[2]);
      // Generates a binaural beat of 6.0Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return alt_blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 835, 835 );

    case 'd':
      // Delta
      tonePair.play(centralTone, binauralBeat[3]);
      // Generates a binaural beat of 2.2Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 2253, 2253 );

    case 'D':
      // Delta
      tonePair.play(centralTone, binauralBeat[3]);
      // Generates a binaural beat of 2.2Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return alt_blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 2253, 2253 );

    case 'g':
      // Gamma
      tonePair.play(centralTone, binauralBeat[4]);
      // Generates a binaural beat of 40.4Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 124, 124 );

    case 'G':
      // Gamma
      tonePair.play(centralTone, binauralBeat[4]);
      // Generates a binaural beat of 40.4Hz
      // delay for the time specified in the table while blinking the LEDs at the correct rate
      return alt_blink_LEDs( pgm_read_word(&chunkybrainwaveTab[index].bwDuration), 124, 124 );

    // this should never be executed, since we catch the end of table in the main loop
    default:
      return true;      // end of table
  }
}
