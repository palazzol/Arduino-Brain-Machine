
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

  See notes in code belosew for how I adapted Mitch Altman's version for Arduino

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
#define USE_RAW_TIMERS

// Minimum PWM brightness for LEDs
#define BRIGHTNESS_MIN 10
// Default brightness for LEDs - overridden by EEPROM
#define BRIGHTNESS_DEFAULT 127
// Max brightness
#define BRIGHTNESS_MAX (255-31)

// time between battery checks (two minutes default)
#define TIMEOUT_DURATION_MS 120000L
// the button debounce time; increase if the output flickers
#define DEBOUNCE_DELAY_MS 300L

// central tone frequency - overridden by EEPROM
#define DEFAULT_CENTRAL_TONE 440.0

/////////////////////////////////////////////////////////////////////////////

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
      rightEarTone.play(centralTone - (binauralBeatFreq / 2));
      leftEarTone.play(centralTone + (binauralBeatFreq / 2));
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

      // Using Timer2 and Timer1, so as not to impact Timer0
      // which is used by Arduino for other timing functions

      DDRB = 0b00000000;   // set PB1,PB3 pins as outputs
      PORTB = 0x00;        // all PORTB output pins Off

      // Right ear tone will use Timer2, 8-bit resolution
      TIMSK2 = 0x00;       // no Timer interrupts enabled
      //   8-bit Timer2 OC2A (PB3, pin 11) is set up for Fast PWM mode, toggling output on each compare
      //   Fclk = Clock = 16MHz
      //   Prescale = 256
      //   F = Fclk / (2 * Prescale * (1 + OCR2A) )
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

      // Strategy:
      // Set the frequency for the lower tone based to (central-binaural/2)
      // using Timer2.  This is an 8-bit timer, so it may not be exact
      // Then, get the actual frequency of the lower tone, and use it compute the upper
      // frequency as upper = lower + binaural.  Timer1 is 16-bits, and so the beat
      // frequency will be very accurate even if the absolute frequency if the pair
      // is less accurate.

      // 31250 = 16Mhz Clk / (2 * 256)
      int lowFreqPeriod = int(F_CPU / 2.0 / 256 / (centralTone - binauralBeat / 2) + 0.5);
      float actualLowFreq = F_CPU / 2.0 / 256 / lowFreqPeriod; // retrieve the value used
      float actualHighFreq = actualLowFreq + binauralBeat; // calculate the upper frequency
      // 8000000 = 16Mhz / (2 * 1)
      int highFreqPeriod = int(F_CPU / 2.0 / actualHighFreq + 0.5);

      // set the frequencies
      OCR2A = lowFreqPeriod - 1;  // on OC2A (PB3, pin 11)
      OCR1A = highFreqPeriod - 1; // on OC1A (PB1, pin 9)

      // enable the pins
      DDRB = 0b00001010;   // set PB1,PB3 pins as outputs
    }
    void stop() {
      // disable the pins
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


#ifndef LEDS_TO_GROUND
// Common anode. 255 is off
#define LED_ON (255-LEDIntensity)
#define LED_OFF 255
int LEDIntensity = 255-BRIGHTNESS_DEFAULT; // Default value, will be overridden by valid value in EEPROM
#else
#define LED_ON (LEDIntensity)
#define LED_OFF 0
int LEDIntensity = BRIGHTNESS_DEFAULT; // Default value, will be overridden by valid value in EEPROM
#endif

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

#define BETA_HZ 14.4
#define ALPHA_HZ 11.1
#define THETA_HZ 6.0
#define DELTA_HZ 2.2
#define GAMMA_HZ 40.4

TonePair tonePair;
float centralTone = DEFAULT_CENTRAL_TONE; //We're starting at this tone and spreading the binaural beat from there.

struct brainwaveElement {
  int duration;  // Seconds
  float frequency; // Hz
};

unsigned long timeout_start; //used for count down timer

// Internal use
#define MODE_SPECIAL_FREQ -1.0
#define MODE_CHUNKY 1
#define MODE_CREAMY 2
#define MODE_ALT_LEDS_BLINK 3

///////////////////////////////////////////////////////////////////
// These tags can be used to control the interpretation of elements
///////////////////////////////////////////////////////////////////

// These two tags affect all subsequent elements
// "Chunky" means each element sets the frequency and duration directly, and immediately
// "Creamy" means each element is a linear ramp up to the frequency over the duration
#define TAG_FORMAT_CHUNKY   { MODE_CHUNKY, MODE_SPECIAL_FREQ }  // ====== Altman's "chunky" frequency-hopping method =======
#define TAG_FORMAT_CREAMY   { MODE_CREAMY, MODE_SPECIAL_FREQ }  // ====== Mindplace.com "creamy" frequency-transition method =======
// This tag only affects the next element in the table
#define TAG_ALT_BLINK       { MODE_ALT_LEDS_BLINK, MODE_SPECIAL_FREQ }

// Note: All of these tables and the index are in Program Memory, to save RAM
// This requires special indexing to read the table values
const brainwaveElement proteusGoodMorning04[] PROGMEM = {
  TAG_FORMAT_CREAMY,
  {0, 8}, {120, 16}, {60, 25}, {60, 25}, {60, 20}, {60, 28}, {60, 20}, {60, 28},
  {60, 20}, {60, 28}, {60, 20}, {60, 28}, {60, 20}, {60, 8}, {60, 20}, {0, 0}
};

const brainwaveElement proteusGoodNight43[] PROGMEM = {
  TAG_FORMAT_CREAMY,
  {0, 9}, {280, 6}, {300, 3}, {200, 5}, {100, 3}, {20, 2}, {0, 0}
};

const brainwaveElement proteusVisuals22[] PROGMEM = {
  TAG_FORMAT_CREAMY,
  {0, 8}, {60, 10}, {60, 16}, {60, 20}, {0, 16}, {60, 24}, {0, 20}, {120, 28}, {60, 24},
  {60, 30}, {300, 24}, {30, 12}, {30, 24}, {30, 16}, {120, 30}, {150, 8}, {60, 8}, {0, 0}
};

const brainwaveElement proteusVisuals33[] PROGMEM = {
  TAG_FORMAT_CREAMY,
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15},
  {0, 4}, {75, 10}, {75, 5}, {0, 20}, {75, 5}, {75, 15}, {0, 0}
};

const brainwaveElement proteusMeditation10[] PROGMEM = {
  TAG_FORMAT_CREAMY,
  {0, 16}, {60, 16}, {240, 4}, {360, 4}, {120, 2}, {1080, 2}, {60, 8}, {60, 24},
  {120, 24}, {0, 0}
};

/***************************************************
  BRAINWAVE TABLE
  Table of values for meditation start with
  lots of Beta (awake / conscious)
  add Alpha (dreamy / trancy to connect with
      subconscious Theta that'll be coming up)
  reduce Beta (less conscious)
  start adding Theta (more subconscious)
  pulse in some Delta (creativity)
  and then reverse the above to come up refreshed
***************************************************/
const brainwaveElement originalArduino[] PROGMEM = {
  TAG_FORMAT_CHUNKY,
  { 60, BETA_HZ }, 
  { 10, ALPHA_HZ }, 
  { 20, BETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 15, BETA_HZ },
  { 20, ALPHA_HZ }, 
  { 10, BETA_HZ }, 
  { 30, ALPHA_HZ }, 
  { 5, BETA_HZ }, 
  { 60, ALPHA_HZ }, 
  { 10, THETA_HZ },
  TAG_ALT_BLINK, 
  { 30, ALPHA_HZ },
  { 20, THETA_HZ }, 
  { 20, ALPHA_HZ }, 
  { 30, THETA_HZ },
  TAG_ALT_BLINK, 
  { 15, ALPHA_HZ },
  { 60, THETA_HZ }, 
  { 10, ALPHA_HZ }, 
  { 1, BETA_HZ }, 
  { 5, ALPHA_HZ },
  TAG_ALT_BLINK, 
  { 55, THETA_HZ },
  { 1, DELTA_HZ }, 
  { 45, THETA_HZ }, 
  { 5, DELTA_HZ },
  TAG_ALT_BLINK, 
  { 35, THETA_HZ },
  { 10, DELTA_HZ }, 
  { 25, THETA_HZ }, 
  { 15, DELTA_HZ }, 
  { 1, GAMMA_HZ },
  TAG_ALT_BLINK, 
  { 5, THETA_HZ },
  { 1, GAMMA_HZ }, 
  { 30, DELTA_HZ }, 
  { 5, GAMMA_HZ }, 
  { 60, DELTA_HZ }, 
  { 10, GAMMA_HZ },
  TAG_ALT_BLINK, 
  { 30, DELTA_HZ },
  { 5, GAMMA_HZ }, 
  { 15, DELTA_HZ }, 
  { 1, GAMMA_HZ }, 
  { 10, THETA_HZ },
  TAG_ALT_BLINK, 
  { 10, DELTA_HZ },
  { 20, THETA_HZ }, 
  { 1, ALPHA_HZ }, 
  { 10, DELTA_HZ }, 
  { 30, THETA_HZ }, 
  { 5, ALPHA_HZ },
  TAG_ALT_BLINK, 
  { 1, BETA_HZ },
  { 10, ALPHA_HZ }, 
  { 22, THETA_HZ },
  TAG_ALT_BLINK, 
  { 15, ALPHA_HZ },
  { 1, BETA_HZ }, 
  { 30, ALPHA_HZ }, 
  { 5, BETA_HZ }, 
  { 20, ALPHA_HZ },
  TAG_ALT_BLINK, 
  { 12, BETA_HZ },
  { 15, ALPHA_HZ }, 
  { 20, BETA_HZ }, 
  { 10, ALPHA_HZ }, 
  { 25, ALPHA_HZ },
  TAG_ALT_BLINK, 
  { 5, ALPHA_HZ },
  { 60, BETA_HZ }, 
  { 0, 0 }
};

const brainwaveElement originalAltman[] PROGMEM = {
  TAG_FORMAT_CHUNKY,
  { 60, BETA_HZ }, 
  { 10, ALPHA_HZ }, 
  { 20, BETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 15, BETA_HZ },
  { 20, ALPHA_HZ }, 
  { 10, BETA_HZ }, 
  { 30, ALPHA_HZ }, 
  { 5, BETA_HZ }, 
  { 60, ALPHA_HZ }, 
  { 10, THETA_HZ },
  { 30, ALPHA_HZ },
  { 20, THETA_HZ }, 
  { 30, ALPHA_HZ }, 
  { 30, THETA_HZ },
  { 15, ALPHA_HZ },
  { 60, THETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 1, BETA_HZ }, 
  { 15, ALPHA_HZ },
  { 60, THETA_HZ },
  { 1, DELTA_HZ }, 
  { 10, THETA_HZ }, 
  { 1, DELTA_HZ },
  { 10, THETA_HZ },
  { 1, DELTA_HZ }, 
  { 30, THETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 1, BETA_HZ },
  { 15, ALPHA_HZ },
  { 30, THETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 1, BETA_HZ }, 
  { 20, ALPHA_HZ }, 
  { 5, BETA_HZ },
  { 20, ALPHA_HZ },
  { 15, BETA_HZ }, 
  { 15, ALPHA_HZ }, 
  { 20, BETA_HZ }, 
  { 10, ALPHA_HZ },
  { 25, BETA_HZ },
  { 5, ALPHA_HZ }, 
  { 60, BETA_HZ }, 
  { 0, 0 }
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

#define NUM_BRAINWAVE_SESSIONS 5

const brainwaveElement *brainwaveSessions[] = {
  proteusGoodMorning04, proteusGoodNight43,
  proteusVisuals33, proteusMeditation10,
  originalAltman //change to originalArduino to go back to the one modified by LaughingOnWater
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

void setLEDPhase(int state, bool led_alt) {
  if (led_alt) {
    analogWrite(rightEyePin, state);
    analogWrite(leftEyePin, !state);
  } else {
    analogWrite(rightEyePin, state);
    analogWrite(leftEyePin, state);
  }
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
  static unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
 
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    timeout_start = millis();
#ifdef DEBUG
    Serial.println("button interrupt");
#endif
    switch (machineState) {
      case STATE_READY:
        machineState = STATE_RUNNING;
        break;
      case STATE_RUNNING:
      case STATE_SLEEPING:
        machineState = STATE_READY; // Back to the normal ready/running cycle
        break;
    }
  }else{
    
#ifdef DEBUG
    Serial.println("bounce bounce bounce");
#endif
  }
  lastDebounceTime = millis();





}

void init_from_EEPROM()
{
  // Format of EEPROM
  // 0/1 0x1234 - Magic Number
  // 2 - byte version
  // 3 - LEDIntensity

  int header = EEPROM.read(0);
  header = (header << 8) + EEPROM.read(1);
  byte version = EEPROM.read(2);

  if ((header != 0x1234) && (version != 1))// Not initialized
  {
    EEPROM.update(0, 0x12);
    EEPROM.update(1, 0x34);
    EEPROM.update(2, 0x01);
    EEPROM.update(3, LEDIntensity);
    EEPROM.update(4, ((int)centralTone)-200);
  }
  else {
    // This is an int, but I think 255 is the max value
    LEDIntensity = EEPROM.read(3);
    centralTone = (float)(EEPROM.read(4)+200);
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
  Serial.begin(115200);
#endif
  timeout_start = millis();
  tonePair.begin(rightEarPin, leftEarPin); // Set pins for right and left ears
  pinMode(rightEyePin, OUTPUT); // Pin output at rightEye
  pinMode(leftEyePin, OUTPUT); // Pin output at leftEye
  pinMode(interruptPin, INPUT_PULLUP); // User input (push button)
  pinMode(potPin, INPUT); // User input (potentiometer)
  init_from_EEPROM();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, FALLING);
}

// Get the current element information from program memory
void readBrainwaveElement(PGM_VOID_P session_ptr, int j, brainwaveElement *element)
{
    memcpy_P( element, session_ptr + j*sizeof(brainwaveElement), sizeof(brainwaveElement)  );
}

/***************************************************
   MAIN LOOP - tells our program what to do.
***************************************************/

void loop() {
  int j;

#ifdef DEBUG
  Serial.println("Waiting for session selection...");
  Serial.print("Timeout in ");
  Serial.print((TIMEOUT_DURATION_MS - (millis() - timeout_start)) / 1000);
  Serial.println(" seconds");
#endif
  while (machineState == STATE_READY) {
    currentSession = mapPot(0, NUM_SESSIONS - 1);
    blinkSessionSelection(currentSession);
    delay(50);
    if (millis() - timeout_start > TIMEOUT_DURATION_MS) //sleep timer loop
    {
#ifdef DEBUG
      Serial.println("menu timed out, going to sleep");
#endif
      goToSleep();
    }

  }
  setLEDs(LED_OFF);


#ifdef DEBUG
  Serial.print("Choose session #");
  Serial.println(currentSession);
#endif
  unsigned long startedAt = millis();
  j = 0;
  if (currentSession < NUM_BRAINWAVE_SESSIONS) {
    // The session pointer points to the current session table in program memory
    brainwaveElement *session_ptr = brainwaveSessions[currentSession];
    float lastFrequency = 0.0;
    bool do_chunky = false;
    bool alt_leds_blink = false;
    brainwaveElement element;
    readBrainwaveElement(session_ptr, j, &element);
    while (element.frequency != 0.0) { // 0.0 signifies end of table
#ifdef DEBUG
      Serial.print(j);
      Serial.print(' ');
      if (element.frequency == MODE_SPECIAL_FREQ) {
        // Special identifier
        if (element.duration == MODE_CHUNKY)
          Serial.print("MODE_CHUNKY");
        if (element.duration == MODE_CREAMY)
          Serial.print("MODE_CREAMY");
        if (element.duration == MODE_ALT_LEDS_BLINK)
          Serial.print("MODE_ALT_LEDS_BLINK");
      } else {
        if (!do_chunky) {
          Serial.print(lastFrequency);
          Serial.print(" -> ");
          Serial.print(element.frequency);
          Serial.print(" in ");
        } else {
          Serial.print(element.frequency);
          Serial.print(" for ");
        }
        Serial.print(element.duration);
      }
      Serial.print(" @");
      Serial.println(millis() - startedAt);
#endif
      if (element.frequency == MODE_SPECIAL_FREQ) {
        // Special identifier
        if (element.duration == MODE_CHUNKY)
          do_chunky = true;
        if (element.duration == MODE_CREAMY)
          do_chunky = false;
        if (element.duration == MODE_ALT_LEDS_BLINK)
          alt_leds_blink = true;
      } else if (element.duration) {
        float elapsedDms = 0; // decimilliseconds since element's start
        float totalDms = element.duration * 10000.0;
        while (elapsedDms < totalDms) {
          float intermediateFreq;
          if (!do_chunky)
            intermediateFreq = lastFrequency + (element.frequency - lastFrequency) * (elapsedDms / totalDms);
          else
            intermediateFreq = element.frequency;
          tonePair.play(centralTone, intermediateFreq);
          unsigned long halfWaveLength = round(5000.0 / intermediateFreq);
          setLEDPhase(LED_ON, alt_leds_blink);
          if (delay_decimiliseconds(halfWaveLength)) {
            break;
          }
          setLEDPhase(LED_OFF, alt_leds_blink);
          if (delay_decimiliseconds(halfWaveLength)) {
            break;
          }
          elapsedDms += (2.0 * halfWaveLength);
        };
        alt_leds_blink = false;
      };
      if (machineState != STATE_RUNNING) {
        break; // interrupt button was pressed
      };
      lastFrequency = element.frequency;
      j++;
      readBrainwaveElement(session_ptr, j, &element);
    };
  } else if (currentSession == SESSION_SETUP) {
    while (machineState == STATE_RUNNING) {
      int brightness = mapPot(BRIGHTNESS_MIN, BRIGHTNESS_MAX);
      // reverse the pot because the menu selection for brightness should be on the dim end, 
      // to keep from jarring you with max brightness
      brightness = BRIGHTNESS_MAX - brightness + BRIGHTNESS_MIN;
#ifdef LEDS_TO_GROUND
      LEDIntensity = 255 - brightness;
#else
      LEDIntensity = brightness;
#endif
      setLEDs(LED_ON);
      delay(50);
    }
    EEPROM.update(3, LEDIntensity);
#ifdef DEBUG
    Serial.print("Set intensity (0-255) to ");
    Serial.println(LEDIntensity);
#endif
    // continue to the frequency setup
    machineState = STATE_RUNNING;
    while (machineState == STATE_RUNNING) {
      tonePair.play(centralTone, 0);
      centralTone = (float)mapPot(200, 440);
      delay(50);
    }
    tonePair.stop();
    EEPROM.update(4, int(centralTone-200));
#ifdef DEBUG
    Serial.print("Set centralTone frequency to ");
    Serial.println(centralTone);
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
    goToSleep();
  }
}

/***************************************************
  This function puts the machine to sleep and wakes it up again upon interrupt
***************************************************/

void goToSleep()
{
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
