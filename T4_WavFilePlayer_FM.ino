/*
   Output FM for Teensy 4
   Compatible to Audio Library

     Legal note
     DO NOT CONNECT A WIRE AS ANTENNA
     Keep in mind that transmitting on certain frequencies without special permissions may be illegal in your country.

    Data files to put on your SD card can be downloaded here:
    http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html

    2021, Frank B
*/



#include <Audio.h>
#include <SD.h>
#include "output_fm.h"

AudioPlaySdWav           playWav1;
AudioOutputFM            audioOutput(30, 91.0, 50e-6f); //Pin (23= I2S1, 30= I2S3, 33= I2S2) , Frequency in MHz, preemphasis
AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);

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

    Serial.print("Diagnostics: ");
    Serial.print(AudioProcessorUsage());
    Serial.print("% ");

    Serial.println(AudioMemoryUsageMax());
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
