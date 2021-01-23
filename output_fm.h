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
  radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

  I could not be held liable for any misuse of your own Teensy.
  Any experiment is made under your own responsibility.

*/

//  (c) Frank B

#ifndef output_fm_h_
#define output_fm_h_

#if defined(__IMXRT1062__)

#include <Arduino.h>
#include <AudioStream.h>
#include <DMAChannel.h>

#define PREEMPHASIS_50        0   // use this if you are in Europe or elsewhere in the world
#define PREEMPHASIS_75        1   // use this if you are in the Americas or South Korea

class AudioOutputFM : public AudioStream
{
public:
	AudioOutputFM(uint8_t pin, unsigned MHz, int preemphasis) : AudioStream(1, inputQueueArray) { begin(pin, MHz, preemphasis); }   
  void begin(uint8_t pin, unsigned MHz, int preemphasis); 
	virtual void update(void);	
private:
	static audio_block_t *block_left;
	static bool update_responsibility;
	audio_block_t *inputQueueArray[1];

	static DMAChannel dma;	
  static void dmaISR(void);

};

#endif
#endif
