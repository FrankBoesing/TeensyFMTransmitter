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
radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

I could not be held liable for any misuse of your own Teensy.
Any experiment is made under your own responsibility.

    Data files to put on your SD card can be downloaded here:
    http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html
    NUM[1-6].WAV here :: https://forum.pjrc.com/threads/64235-Teensyduino-1-54-Beta-3?p=258095&viewfull=1#post258095

    2021, Frank B
*/



#include <Audio.h>
#include <SD.h>
#include "output_fm.h"

AudioPlaySdWav           playWav1;
AudioOutputFM            audioOutput(33, 91.0, PREEMPHASIS_50); //Pin (23= I2S1, 30= I2S3, 33= I2S2) , Frequency in MHz, preemphasis: EU: 50 us -> tau = 50e-6, USA: 75 us -> tau = 75e-6

AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);

void setup() {
  AudioMemory(8);

  if (!(SD.begin(BUILTIN_SDCARD))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  for (int i = 1; i <= 6; i++) {
    playNumber(i);
    delay(50);
  }

}

elapsedMillis waitSpew;
void playFile(const char *filename)
{
  waitSpew = 10000;
  Serial.print("Playing file: ");
  Serial.println(filename);
  Serial.flush();
  playWav1.play(filename);
  delay(25);
  while (playWav1.isPlaying()) {
    if ( waitSpew > 2000 ) {
      if ( Serial.available() ) {
        while ( Serial.available() ) {
          Serial.read();
        }
        delay(50);
        playWav1.stop();
        return;
      }
      waitSpew = 0;
      Serial.print("Diagnostics: ");
      Serial.print(AudioProcessorUsage());
      Serial.print("% ");
      Serial.println(AudioMemoryUsageMax());
      delay(50);
    }
  }
  delay(500);
}

void loop() {
  playNumber(1);
  playFile("SDTEST2.WAV");  // filenames are always uppercase 8.3 format
  playNumber(2);
  playFile("SDTEST3.WAV");
  playNumber(3);
  playFile("SDTEST4.WAV");
  playNumber(4);
  playFile("SDTEST1.WAV");
}

void playNumber(int n)
{
  String filename = String("NUM") + n + ".WAV";
  playFile(filename.c_str());
}
