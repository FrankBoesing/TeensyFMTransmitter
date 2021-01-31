# TeensyFMTransmitter

* NO additional hardware needed.
* processes audio from i.e. 44.1ksps WAV files on SD card
* full integration into Audio Library
* further processing @352.8 ksps sample rate
* Pre-emphasis filter implemented
* interpolation-by-8 implemented in order to be able to accurately produce the MPX-signal
* Stereo processing implemented
* RDS

DO NOT CONNECT AN ANTENNA

Teensy 4.0 or 4.1 only

![mpx](https://upload.wikimedia.org/wikipedia/commons/thumb/1/10/UKW-Rundfunk-Basisband.svg/320px-UKW-Rundfunk-Basisband.svg.png)

## Legal note
TeensyFMTransmitter is an experimental program, designed only for experimentation. It is in no way intended to become a personal media center or a tool to operate a radio station, or even broadcast sound to one's own stereo system.

In most countries, transmitting radio waves without a state-issued licence specific to the transmission modalities (frequency, power, bandwidth, etc.) is illegal.

Therefore, always connect a shielded transmission line from the Teensy directly to a radio receiver, so as not to emit radio waves. Never use an antenna.

Even if you are a licensed amateur radio operator, using TeensyFMTransmitter to transmit radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

I could not be held liable for any misuse of your own Teensy. Any experiment is made under your own responsibility.


