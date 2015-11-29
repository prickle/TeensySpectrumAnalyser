# TeensySpectrumAnalyser
The Teensy Audio Spectrum Analyzer is a low-cost classic audio spectrum display with up to 128 bands.

It displays a detailed picture of what you're hearing in real-time. It shows the changing spectrum of live input signals up to 16kHz.

It takes a mono analog RCA audio input (nominal 1v pp) and presents up to 128 bands of real-time colour spectrum display with peak metering.

You can easily observe the frequency characteristics of live music in real time. The spectral composition of acoustic signals become clearly visible.

![Wiring Diagram](prickle.github.com/TeensySpectrumAnalyser/schematics/Spectrum_Wiring.png)

----
The display is constructed using low-cost red/green LED matrix modules. These modules can be daisy-chained together to make larger displays. The prototype used two modules for a 64 band spectrum but up to four modules should work ok (untested).

16x32 DIY Kit Red Green Dual-Color Dot Matrix Control Display Module
http://www.ebay.com/itm/231503508955

The code intensity modulates the LEDs to minimise power consumption while still providing good visibility. It is just possible to power a unit with two display modules from USB only but it is recommended to provide an external power supply if more than two modules are used or the brightness is increased.

----
The controller can be a Teensy 3.1 or 3.2. 

The Teensy Audio Library is at the heart of the design, providing audio capture and high resolution FFT routines. Audio frequency data is scaled and binned into logarithmic frequency response groupings and is plotted using a custom fast display driver into a spectrum that approximates the human auditory response for a balanced visual aesthetic.


Teensy 3.2 controller
https://www.pjrc.com/store/teensy32.html

Teensy Audio Library
http://www.pjrc.com/teensy/td_libs_Audio.html

----
Audio Input

An RCA audio jack takes a mono line level signal (1v peak to peak) for input. A small amount of conditioning circuitry decouples the signal from the source and adds a DC bias to shift it into the analog to digital converter's range of sensitivity.
