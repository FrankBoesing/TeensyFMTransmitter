/*
   Legal note

  TeensyFMTransmitter is an experimental program, designed only for experimentation.
  It is in no way intended to become a personal media center or a tool to operate a radio station,
  or even broadcast sound to one's own stereo system.

  In most countries, transmitting radio waves without a state-issued licence specific
  to the transmission modalities (frequency, power, bandwidth, etc.) is illegal.

  Therefore, always connect a shielded transmission line from the Teensy directly to a radio receiver,
  so as not to emit radio waves. Never use an antenna.

  Even if you are a licensed amateur radio operator, using TeensyFMTransmitter to transmit
  radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal
  because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

  I could not be held liable for any misuse of your own Teensy.
  Any experiment is made under your own responsibility.

*/

//  (c) Frank B

#ifndef output_fm_h_
#define output_fm_h_

#if defined(__IMXRT1062__)

#include <Arduino.h>
#include <AudioStream.h>
#include <utility/imxrt_hw.h>
#include <DMAChannel.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#define PREEMPHASIS_50        0   // use this if you are in Europe or elsewhere in the world
#define PREEMPHASIS_75        1   // use this if you are in the Americas or South Korea

#define INTERPOLATION         8

#define AUDIO_SAMPLERATE      44117.64706
#define I_TIMERVAL            ((int)(((double)F_BUS_ACTUAL) / (((double)AUDIO_SAMPLERATE) * INTERPOLATION) + 0.5))
#define I_SAMPLERATE          (((double)AUDIO_SAMPLERATE) * INTERPOLATION)

/*
  class rds : public Print
  {
  public: rds() {};

  virtual size_t write(uint8_t) {return 1;};
  virtual size_t write(const uint8_t *buffer, size_t size) {return size;};
  private:
  void begin();
  float rds_sample();
  };
*/

class AudioOutputFM : public AudioStream, public Print
{
  public:
    AudioOutputFM(uint8_t pin, unsigned MHz, int preemphasis) : AudioStream(2, inputQueueArray) {
      begin(pin, MHz, preemphasis);
    }
    unsigned long time_us(void) {
      return us;
    };

    //RDS-Data:
    void setPI(uint16_t _PI);
    void setPTY(uint8_t _PTY);
    void setTP(bool _TP);
    void setTA(bool _TA);
    void setPS(const char* PS);
    bool transmitted();
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);
  private:
    void begin(uint8_t pin, unsigned MHz, int preemphasis);
    virtual void update(void);
    static audio_block_t *block_left;
    static audio_block_t *block_right;
    static bool update_responsibility;
    audio_block_t *inputQueueArray[2];
    static DMAChannel dma;
    static unsigned long us;
    static void dmaISR(void);
};

#endif
#endif
