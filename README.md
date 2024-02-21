<header>

# Bode Frequency Shifter (32 bit version)

</header>

This is the Teensy sketch for the code for the Fitzgreyve 1630 Frequency Shifter, intended for use with the Fitzgreyve hardare, Teensy 4.0 and Audio Shield.

Documentation is within the file comments.

The sketch uses the 32 bit [OpenAudio Arduino Library](https://github.com/chipaudette/OpenAudio_ArduinoLibrary)

##Hardware configuration

    A0 (VR1) is MIX control
    A1 (VR3) is Squelch control
    A2 (VR2) is Shift control
    A3 is Mix CV input (note:inverted values)
    A8 is Shift CV Input (note:inverted values)

    Scale Switch digital input is D4,D3,D2

    D9 push button ditigal input
    D1 Squelch LED output
    D5 Mode LED output

<footer>


</footer>
