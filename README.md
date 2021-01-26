# TeensyFMTransmitter

Prototype version, more a study than ready to use.

* Pre-emphasis filter implemented
* interpolation implemented in order to be able to accurately produce the MPX-signal
* Stereo implemented


DO NOT CONNECT A WIRE AS ANTENNA

Teensy 4.x only

## Legal note
TeensyFMTransmitter is an experimental program, designed only for experimentation. It is in no way intended to become a personal media center or a tool to operate a radio station, or even broadcast sound to one's own stereo system.

In most countries, transmitting radio waves without a state-issued licence specific to the transmission modalities (frequency, power, bandwidth, etc.) is illegal.

Therefore, always connect a shielded transmission line from the Teensy directly to a radio receiver, so as not to emit radio waves. Never use an antenna.

Even if you are a licensed amateur radio operator, using TeensyFMTransmitter to transmit radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

I could not be held liable for any misuse of your own Teensy. Any experiment is made under your own responsibility.


