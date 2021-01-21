//(c) Frank B

#ifndef output_fm_h_
#define output_fm_h_

#if defined(__IMXRT1062__)

#include <Arduino.h>
#include <AudioStream.h>
#include <DMAChannel.h>

class AudioOutputFM : public AudioStream
{
public:
	AudioOutputFM(uint8_t pin, unsigned MHz, float preemphasis) : AudioStream(1, inputQueueArray) { begin(pin, MHz, preemphasis); }   
  void begin(uint8_t pin, unsigned MHz, float preemphasis); 
	virtual void update(void);	
private:
	static audio_block_t *block_left;
	static bool update_responsibility;
	audio_block_t *inputQueueArray[1];

	//static DMAChannel dma;
	static void isr(void);

};

#endif
#endif
