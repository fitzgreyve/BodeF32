<header>

# Bode Frequency Shifter (32 bit version)

</header>

This is the Teensy sketch for the code for the Fitzgreyve 1630d Frequency Shifter, intended for use with the Fitzgreyve 1630d hardware, Teensy 4.0 and Audio Shield.

Documentation is within the file comments.

The sketch uses the 32 bit [OpenAudio Arduino Library](https://github.com/chipaudette/OpenAudio_ArduinoLibrary)

## Eurorack Module Description
The Fitzgreyve  1630d is a digital interpretation of the Moog model 1630 Bode Frequency Shifter - PCBs, panels, and build information/BOM  are available from www.fitzgreyve.co.uk

The module is based on a Teensy 4.0 processor together with the Teensy Audio Board. A front panel PCB hosts these together with the controls and interface circuitry.

As configured, the module will accept +12dBm signals (as the original Moog specification)  with the output signal being nominally the same level as the input.

There are some differences between the 1630d and the original Moog 1630:
•	The 1630d includes two audio inputs; the 1630 has only has one.

•	There is no dedicated “mix” output on the 1630d – the MIX output can be assigned to “OUT A” or “OUT B” (or neither)  using the “MIXTURE” control and associated push button.

•	The 1630d includes a “MIX CV” input that can control the content of the MIX output.

•	There is no “ZERO” position on the SCALE switch, as this is not required for the digital version. The switch position has been re-used to provide a 1V/octave calibration function (CAL) for the CV inputs.

•	The FIR Hilbert filters used in the digital interpretation are not as flat in their phase and frequency responses at low frequencies when compared to the original’s original analogue dome filters.

•	There is no “hole” when using the 5Hz scale, a known issue with the original Moog module.

The PCB assembly uses though-hole components (no surface mount are necessary).

## Hardware configuration

    A0 (VR1) is MIX control
    A1 (VR3) is Squelch control
    A2 (VR2) is Shift control
    A3 is Mix CV input (note:inverted values)
    A8 is Shift CV Input (note:inverted values)

    Scale Switch digital input is D4,D3,D2

    D9 push button ditigal input
    D1 Squelch LED output
    D5 Mode LED output

## Software description

For most functions, the module operates the same as the Moog/Bode original; but there are now two signal inputs, and a Mix CV input.

Mixed output.
The Moog module has three outputs (A, B and MIX). The Teensy audio shield only has two outputs.  
Either output A or output B (or neither) can be assigned to be the mix output.
1.	SCALE must not be set to “CAL”
2.	Turn the MIXTURE control to select the mix output:
a.	MIXTURE anti-clockwise:  output A is the MIX output.
b.	MIXTURE clockwise: output B is the MIX output.
c.	MIXTURE centred:  no mix output.
3.	Press and hold the push button. 
4.	Both LEDs will initially light, then (depending on the MIXTURE control position) one or both LEDS will go off:
a.	MIXTURE anti-clockwise:  output A is the MIX output ( left “Squelch ” LED remains  on).
b.	MIXTURE clockwise: output B is the MIX output (right “mix” LED remains on).
c.	MIXTURE centred:  no mix output (no LEDs remain on).
5.	Release the push button, the new mix is saved.

If the mixed output is enabled,  with the MIXTURE control centred a Mix CV of -5V will select output “A”, +5V will select output “B”,  0V will select “A” and “B” mixed equally.

WAV file player
The module includes a basic WAV file player that can play files from the SD card. Note that the SD card outputs must be enabled in the software*.
•	The files must be named in the “8.3” format e.g. TESTFILE.WAV  , long file names are not supported.
•	All files must be in the root directory, sub-directories are currently not supported.
•	The files will play in the order loaded onto the SD card.
•	The SD card is only detected at power up.

If the “autoplay”* option is enabled the first file will play at power up, and then each file will play sequentially.
If the “autoplay” option is disabled a short press on the button will play the next file.
When a file is currently playing, a short press will skip to the next file.

* see “software user settings”
  
CV Calibration

The software has a set of default calibration values that are reasonably accurate.

The calibration procedure has two outcomes:
1.	Sets the “zero shift” position on the SHIFT panel control.
2.	Calibrates the CV1-CV3 inputs to 1 volt per octave.

Calibration procedure:
1.	Set the SCALE switch to “CAL”
2.	Set the SHIFT control to its centre (zero shift) position.
3.	Press and release the push button to start the calibration. This press also sets the SHIFT control zero position.
4.	The right (mix) LED will light: apply +1.00 volt to the CV1 input
5.	Press and release the push button (this sets the 1V calibration point).
6.	The left (squelch) LED will now light: apply +3.00 volts to the CV1 input
7.	Press and release the push button (this now attempts to calibrate to 1V per octave):
a.	If calibration is successful, both LEDs will go out.
b.	If calibration fails, both LEDs will light. A further button press will clear the LEDs
i.	If calibration fails, the default calibration values will be used.

Note that there are two software settings that control the accuracy of the calibration process, see the “Software User Settings“ section below.

## Software Settings

There are various “user” settings available in the software that can tailor the module to personal taste without having to change the actual “coding”:

All settings use define statements:
•	#define		the setting is enabled 
•	//#define	the setting is disabled (i.e. commented out)
•	Numeric settings e.g. “#define TestOscFrequency 440.0”  must have a value assigned! 	

•	Signal operating levels . Because the Squelch and peak indicator are dependent on the signal level, I have removed this option.
 
•	Input selection . You can use any combination (but some combinations don't make sense).
o	#define inputExt                   // external inputs J1 and J2
o	#define inputSDcard            // SD card audio
o	#define inputTestOsc           // internal test oscillator

•	MIXTURE controls the A and B output levels. Normally the MIXTURE control (including the “Mix CV”) only affects the mixed output if it assigned to either A or B output jacks. This allows it to also control the individual A and B output levels.
o	#define ABmixtureControl

•	Add the input signal(s) into the outputs. 
o	#define addInputToOutput

•	 Frequency of the internal test sinewave oscillator (in Hertz)
o	#define TestOscFrequency 440.0    

•	Debug mode. Sends status updates over the Teensy USB port.
o	#define DEBUG			// define to enable debug.			
o	#define debugUpdate 1500        // debug info update period in mS

•	Shift dial base frequency for exponential mode, in Hertz (normally 2.0 Hertz to match the panel).
o	#define expShiftStart 2.0        
 

•	 Squelch control values
o	#define squelchAttack 0.005      squelch attack (opening) time in seconds.
o	#define squelchRelease 0.050      squelch release (closing) time in seconds.
o	#define squelchHold 0.050            squelch hold time in seconds.

•	 Shift control options. These control what happens around the "zero" shift point. You can enable or disable any of these, however I recommended to operate with "Hysteresis" and "DeadZone" only enabled.
o	#define shiftControlHysteresis
	Enables hysteresis for shift direction change around the zero shift point. Without this setting the shift direction can oscillate at zero shift, giving audible glitches.
o	#define shiftControlDeadZone
	Enables an extended zone of “zero shift” around the zero shift point.
o	#define shiftControlVariablePhase
	Enables a gradual change of phase when passing through the "zero shift" point.  Note that this results in some amplitude modulation around the zero shift point.
o	#define shiftControlAmplitudeMod 
	Reduces modulation amplitude as we approach "zero shift". Note that this results in zero output level at zero shift! Requires shiftControlVariablePhase  to be enabled.

•	CV Slew limiter. For exponential mode only for CV changes less than 1/2 semitone.  CV changes greater then ½ semitone will jump straight to the new CV.
o	#define expCVslewLimiter 0.1           // value around 0.1 ??

•	Push button "long press" time in mS
o	#define longPressTime 1500

•	SD card Autoplay. Automatically plays the first WAV file at power up, and then plays each subsequent file.
o	#define SDautoPlay

•	Use the mode LED as a peak indicator (for the internal digital signal level).
o	#define useModeLEDasPeak

•	Peak indicator LED threshold value:  in internal digital dBfs
(-10 dBfs=0.31, -6dBfs=0.5, -3dBfs=0.707, 0 dBfs = 1.0, +2dBfs = 1.26, +3dBfs = 1.41, +6dBfs = 1.99, +12dBfs = 3.98)
o	#define peakLEDthreshold   0.707
 

•	CV calibration ADC reading stability. During calibration, accept variations in CV ADC readings up to this value.
o	#define CVcalibrationStability 2  
      
•	CV calibration Accuracy (maximum acceptable value, in ± semitones per octave).  
o	#define CVcalibrationAccuracy 0.5   

<footer>


</footer>
