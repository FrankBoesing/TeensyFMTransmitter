/*
   Output FM for Teensy 4
   Compatible to Audio Library

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

    Data files to put on your SD card can be downloaded here:
    http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html

    2021, Frank B
*/



#include <Audio.h>
#include <SD.h>
#include "output_fm.h"

AudioPlaySdWav           playWav1;
//AudioOutputFM            audioOutput(30, 91.0, PREEMPHASIS_75); //Pin (23= I2S1, 30= I2S3, 33= I2S2) , Frequency in MHz, preemphasis
AudioOutputFM            fm(33, 91.0, PREEMPHASIS_50); //Pin (23= I2S1, 30= I2S3, 33= I2S2) , Frequency in MHz, preemphasis
AudioConnection          patchCord1(playWav1, 0, fm, 0);
AudioConnection          patchCord2(playWav1, 1, fm, 1);

void setup() {
  AudioMemory(8);

  if (!(SD.begin(BUILTIN_SDCARD))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  playWav1.play(filename);
  delay(25);
  while (playWav1.isPlaying()) {

    Serial.printf("Diagnostics AudioLib:%0.2f%% FM:%dus\n",
                  AudioProcessorUsage(), fm.time_us() );

    delay(500);
  }
}

void loop() {
  playFile("SDTEST2.WAV");  // filenames are always uppercase 8.3 format
  delay(500);
  playFile("SDTEST3.WAV");
  delay(500);
  playFile("SDTEST4.WAV");
  delay(500);
  playFile("SDTEST1.WAV");
  delay(500);
}
