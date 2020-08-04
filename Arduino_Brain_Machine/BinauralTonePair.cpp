
//#define USE_TONE_LIBRARY

#ifdef USE_TONE_LIBRARY

#include <Tone.h> // Download from https://github.com/bhagman/Tone

class BinauralTonePair
{
  public:
    BinauralTonePair(bool phase_lock) {};
    int begin(int rightEarPin, int leftEarPin) {
      this.cb = nullptr;
      rightEarTone.begin(rightEarPin); // Tone rightEarTone begins at pin output rightEarPin
      leftEarTone.begin(leftEarPin); // Tone leftEarTone begins at pin output leftEarPin
    }
    void setBeatCallback(BEAT_CALLBACK cb) {
      this.cb = cb;
    }
    void play(float centralTone, float binauralBeat) {
      rightEarTone.play(centralTone - (binauralBeat/2));
      leftEarTone.play(centralTone + (binauralBeat/2));
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
#include <Arduino.h>

class BinauralTonePair
{
   public:
    int increment;
    int accumulator;
    int modulo;
  private:
    bool phase_lock;
  public:
    BinauralTonePair(bool p_lock) {
      phase_lock = p_lock;
    };
    void begin(int rightEarPin, int leftEarPin) {
      // Must use these pins
      if ((rightEarPin != 11) and (leftEarPin != 9)) {
        return -1;
      }
      
      pinMode(13, OUTPUT);
      
      DDRB = 0b00000000;   // set PB1,PB3 pins as outputs
      PORTB = 0x00;        // all PORTB output pins Off

      // Right ear tone will use Timer2, 8-bit resolution
      TIMSK2 = 0x00;       // no Timer interrupts enabled
      //   8-bit Timer2 OC2A (PB3, pin 11) is set up for CTC mode, toggling output on each compare
      //   Fclk = Clock = 16MHz
      //   Prescale = 256
      //   OCR0A = 154
      //   F = Fclk / (2 * Prescale * (1 + OCR0A) ) = 200.321Hz
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
      //   OCR1A = value for Beta, Alpha, Theta, or Delta (i.e., 18628, 18919, 19386, or 19750)
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
      
      int lowFreqPeriod = int(15625.0/(centralTone - binauralBeat/2) + 0.5); // on OC2A (PB3, pin 11)
      float actualLowFreq = 15625.0/lowFreqPeriod;
      float actualHighFreq = actualLowFreq + binauralBeat;
      int highFreqPeriod = int(4000000.0/actualHighFreq + 0.5);
      
      OCR2A = lowFreqPeriod - 1;  // on OC2A (PB3, pin 11)
      OCR1A = highFreqPeriod - 1; // on OC0A (PB1, pin 9)

      DDRB = 0b00001010;   // set PB1,PB3 pins as outputs

      increment = lowFreqPeriod*256 - highFreqPeriod;
      accumulator = 0;
      modulo = lowFreqPeriod*256;

      if (phase_lock)
        TIMSK1 = 0x02;
    }
    void stop() {
      DDRB = 0b00000000;   // set PB1,PB3 pins as inputs
      if (phase_lock)
        TIMSK1 = 0x00;
    }
};



#endif
