// Teensy Bode Frequency shifter
// Copyright 2023 Mark Graves
//
// Release History
// ===============
// Release 1 	  02/03/23
// release 1-1 	13/03/23  #include AudioEffectNoiseGate_F32.h as missing from F32 library header file
// Release 1-2  13/04/23  Adjust peak indicator values, remove EURORACK/MOOG signal level options, additional comments.  
//
// Acknowledgements
// ================
// CV Calibration routine and CV processing inspired by Émilie Gillet of Mutable Instruments.
//
// Copyright, Permissions and Warranty
// ===================================
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software, to deal in the Software, including the rights to use, copy, modify,
// merge, publish, or distribute, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// Hardware configuration
// ======================
//    A0 (VR1) is MIX control
//    A1 (VR3) is Squelch control
//    A2 (VR2) is Shift control
//    A3 is Mix CV input (note:inverted values)
//    A8 is Shift CV Input (note:inverted values)
//
//    Scale Switch input is D4,D3,D2
//
//    D9 push button
//    D1 Squelch LED
//    D5 Mode LED
//
//  NOTE on signal levels
//  =====================
//  From the OpenAudio F32 design tool:
//  "With floating point, F32, audio, there is generally no maximum level that represents full scale,
//   as there is with Teensy Audio I16. The exception is at the input and output points where there
//   is an interface with an integer device. The F32 scaling at these points is set to -1.0 and +1.0 
//   corresponding to I16 -32768 and 32767."
//
//  Further note from Bob Larkin (OpenAudio F32 library author):
//  "..there is no full scale in float.  Historically, going back to the I16 Teensy Audio Library, 
//  a level of + or - 1.0 serves where a FS (full scale) value is needed. Obviously, one feature of using the floats
//  is that nothing much bad happens when we go beyond that level. The exception to all this is
//  when one converts back to I16 for I/O.  In that case, as expected, 0.999969 is 32767 and -1.0000 is -32768."
//
//  Note that how the digital value relates to the "real world" signal voltage also depends on the hardware signal scaling,  
//  and the input (and output) level settings of the SGT15000 on the Teesny Audio shield.
//
//  Squelch
//  =======
//  The squelch (noisegate) requires a threshold (in Db) in relation to Dbfs, from above Dbfs is 1.0 (peak), 2.0 peak-to-peak
//  I am assuming Moogs "0dBm" reference level to be 0.775V rms, 1.1V peak, 2.2V peak-to-peak
//  With the SGT15000 input set to 1.58V p-p maximum, setting "4" (i.e. we need a 1.58V p-p signal at the SGT input to give a ntionally "full-scale" digital signal of 2.0 p-p):
//  A "0dBm" 2.2V p-p input signal is divided by 6.3 by the input hardware, giving an actual input signal to the SGT15000 of 0.35V p-p
//  0.35/1.58 gives 0.22 - so a 0dBM signal input actually gives a digital value of 0.22 peak to peak or -13.2 dBfs.
//  We therefore need to apply a -13.2dB correction to the front panel squelch value.
//
// Peak Indicator
// ==============
// This is specified as a peak (not peak-to-peak!) signal level (not dB), against the digital 0dBfs = 1.0 (not volts):
// -10 dBfs=0.31, -6dBfs=0.5, -3dBfs=0.707, 0 dBfs = 1.0, +2dBfs = 1.26, +3dBfs = 1.41, +6dBfs = 1.99, +12dBfs = 3.98
//
// Note that all of the above only applies to signals from the external inputs, signals from the SD card etc have no reference value.

#include <Bounce2.h>                               // standard teensy debounce library
#include <Audio.h>                                 // https://www.pjrc.com/teensy/td_libs_Audio.html
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <EEPROM.h>
#include <OpenAudio_ArduinoLibrary.h>             // https://github.com/chipaudette/OpenAudio_ArduinoLibrary
#include <AudioEffectNoiseGate_F32.h>		          // missing from OpenAudio_ArduinoLibrary.h header file (added in REL1-1)

//======================================================================================
//======================================================================================
//                                     USER SETTINGS
//                                     =============
//                     Amend these as needed, you can't break anything.
//--------------------------------------------------------------------------------------

// #define to select the desired inputs
// You can use any combination (but some combinations don't make sense).
#define inputExt               // external inputs J1 and J2
#define inputSDcard            // SD card audio
//#define inputTestOsc           // test oscillator

// define if you want to use the MIXTURE control to affect the A and B output levels
// (normally the MIXTURE control only affects the mixed signal, if it is assigned to an output)
#define ABmixtureControl

// define if you want to add the input signal(s) into the outputs
//#define addInputToOutput

// Frequency of the internal test sinewave oscillator (in Hertz)
#define TestOscFrequency 440.0    

// #define to set debug mode
#define DEBUG
#define debugUpdate 2000        // debug info update period in mS

// Shift dial base frequency for exponential mode, in Hertz (normally 2.0 Hertz for standard "Moog" panel)
#define expShiftStart 2.0f        

// Squelch control (noise gate) values
#define squelchAttack 0.005      // squelch attack (opening) time in seconds
#define squelchRelease 0.050     // squelch release (closing) time in seconds
#define squelchHold 0.050          // squelch hold time in seconds

// Shift control options.
// These control what happens around the "zero" shift point
// You can enable or disable any of these however I recommended to operate with "Hysteresis" and "DeadZone" only.
#define shiftControlHysteresis
#define shiftControlDeadZone
//#define shiftControlVariablePhase     // #define enable to enable a gradual change of phase when passing through the "zero shift" point.
                                        // Note that this results in some amplitude mldulation around the zero shift point.
//#define shiftControlAmplitudeMod      // #define to reduce modulation amplitude as we approach "zero shift". Note that this results in zero output level at zero shift!
                                        // shiftControlAmplitudeMod only works if shiftControlVariablePhase is also enabled.

// CV Slew limiter when in exponential mode for CV changes less than 1/2 semitone.
#define expCVslewLimiter 0.1          // value around 0.1 ??

// push button "long press" time in mS
#define longPressTime 1500

// should the SD card autoplay all WAV files
//#define SDautoPlay

// use the mode LED as a peak indicator
#define useModeLEDasPeak

// peak indicator LED threshold value (dBfs):  
// -10 dBfs=0.31, -6dBfs=0.5, -3dBfs=0.707, 0 dBfs = 1.0, +2dBfs = 1.26, +3dBfs = 1.41, +6dBfs = 1.99, +12dBfs = 3.98
#define peakLEDthreshold   0.707

// CV calibration ADC reading stability
#define CVcalibrationStability 2        // in ADC counts

// CV calibration Accuracy (in semitones)
#define CVcalibrationAccuracy 0.75       // in semitones

// ------------------------------------------------------------------------------------
//                                  end of user settings
//======================================================================================
//======================================================================================


// enable this setting to force default calibration values into eeprom when the code is run
//
//#define eepromWIPE



//======================================================================================
//======================================================================================
//                                 INTERNAL STUFF
//               Do not change anything below unless you know what you are doing.
//--------------------------------------------------------------------------------------

// Defaults for any undefined user setting values

#ifndef TestOscFrequency 
#define TestOscFrequency 440.0   
#endif 

#ifndef debugUpdate
#define debugUpdate 2000
#endif        

#ifndef expShiftStart
#define expShiftStart 2.0
#endif 

#ifndef squelchAttack
#define squelchAttack 0.005 
#endif 

#ifndef squelchRelease    
#define squelchRelease 0.050     
#endif 

#ifndef squelchHold
#define squelchHold 0.050          
#endif 

#ifndef longPressTime
#define longPressTime 1500
#endif

#ifndef expCVslewLimiter
#define expCVslewLimiter 0.1
#endif

#ifndef CVcalibrationStability
#define CVcalibrationStability 2
#endif

#ifndef CVcalibrationAccuracy
#define CVcalibrationAccuracy 1.0 
#endif

// hardware
#define squelchLED 1
#define modeLED 5
#define buttonSwitch 9

#define default_shiftCV_scale 102.4f      // exponential scaling - ADC counts per octave
#define default_shiftCV_offset -512.0f    // shift CV offset - in ADC counts
#define default_shift_offset 512.0f       // shift panel control offset - in ADC counts
#define default_mixMode 0                 // default value = no mixing.

// SGT15000 input and output levels
#define inleveltop 4     // J1 input level
#define inlevelbottom 4  // J2 input level
#define outlevel 27


//gains for the input mixer
#ifdef  inputExt
#define inputExtGain 1.0              // external inputs J1 and J2 on
#endif 

#ifndef  inputExt
#define inputExtGain 0.0              // external inputs J1 and J2 off
#endif 

#ifdef  inputSDcard
#define inputSDcardGain 0.5              // SD card on 
#endif 

#ifndef  inputSDcard
#define inputSDcardGain 0.0              // SD Card off 
#endif 

#ifdef  inputTestOsc
#define inputTestOscGain 1.0              // Internal Test Oscillator on
#endif 

#ifndef  inputTestOsc
#define inputTestOscGain 0.0              // Internal Test Oscillator off
#endif 

//set the F32 Audio Library sample rate and block size
const float sample_rate_Hz = 44100.0f;
const int audio_block_samples = 128;         //   Must be 128 for SD .
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

// STRUCTURES AND CLASSES

enum UI_mode {                      // Control modes:
  NORMAL,                           // Normal operation
  CALIBRATION_1,                    // Calibrate Step 1: input note C1
  CALIBRATION_2,                    // Calibrate Step 2: Input note C3
  PANIC                             // problem!
  };

// calibration settings structure 
struct {
  short magic_number_1;
  short magic_number_2;
  float shift_offset;
  float shiftCV_offset;
  float shiftCV_offset_Oct;  
  float shiftCV_scale;
  int mixMode;
  } Calibration_Settings; 

// instantiate push button object
Bounce PButton=Bounce();


// OpenAudio (F32) GUItool: begin (mostly) automatically generated code
AudioInputI2S_F32        audioInI2S1;    //xy=277,191
AudioSynthWaveform_F32   InWAVE;      //xy=278,245
AudioSDPlayer_F32        playWav1;     //xy=290,318
AudioMixer8_F32          INmix;       //xy=471,243
AudioAnalyzePeak_F32     peakLED;
AudioFilterFIR_F32       HILBERT45M; //xy=686,309
AudioFilterFIR_F32       HILBERT45P;           //xy=693,203
AudioSynthSineCosine_F32 ShiftOsc;      //xy=718,464
AudioMultiply_F32        Multiply1;  //xy=940,205
AudioMultiply_F32        Multiply2; //xy=957,310
AudioMixer4_F32          Amix;       //xy=1185,210
AudioMixer4_F32          Bmix; //xy=1189,318
AudioMixer4_F32          AoutMix; //xy=1397,208
AudioMixer4_F32          BoutMix; //xy=1403,319
AudioEffectNoiseGate_F32 squelchA;     //xy=1585,202
AudioEffectNoiseGate_F32 squelchB; //xy=1586,300
AudioOutputI2S_F32       audioOutI2S1;   //xy=1771,252
AudioConnection_F32          patchCord1(audioInI2S1, 0, INmix, 0);
AudioConnection_F32          patchCord23(audioInI2S1, 1, INmix, 1);
AudioConnection_F32          patchCord2(InWAVE, 0, INmix, 2);
AudioConnection_F32          patchCord3(playWav1, 0, INmix, 3);
AudioConnection_F32          patchCord4(playWav1, 1, INmix, 4);
AudioConnection_F32          patchCord5(INmix, HILBERT45P);
AudioConnection_F32          patchCord6(INmix, HILBERT45M);
AudioConnection_F32          patchCord26(INmix, peakLED);
AudioConnection_F32          patchCord7(HILBERT45M, 0, Multiply2, 0);
AudioConnection_F32          patchCord8(HILBERT45P, 0, Multiply1, 0);
AudioConnection_F32          patchCord9(ShiftOsc, 0, Multiply1, 1);
AudioConnection_F32          patchCord10(ShiftOsc, 1, Multiply2, 1);
AudioConnection_F32          patchCord11(Multiply1, 0, Amix, 0);
AudioConnection_F32          patchCord12(Multiply1, 0, Bmix, 0);
AudioConnection_F32          patchCord13(Multiply2, 0, Bmix, 1);
AudioConnection_F32          patchCord14(Multiply2, 0, Amix, 1);
AudioConnection_F32          patchCord15(Amix, 0, AoutMix, 0);
AudioConnection_F32          patchCord16(Amix, 0, BoutMix, 1);
AudioConnection_F32          patchCord17(Bmix, 0, BoutMix, 0);
AudioConnection_F32          patchCord18(Bmix, 0, AoutMix, 1);
AudioConnection_F32          patchCord19(AoutMix, squelchA);
AudioConnection_F32          patchCord20(BoutMix, squelchB);
AudioConnection_F32          patchCord21(squelchA, 0, audioOutI2S1, 1);
AudioConnection_F32          patchCord22(squelchB, 0, audioOutI2S1, 0);
AudioConnection_F32          patchCord24(INmix, 0, AoutMix, 2);     // add input to output
AudioConnection_F32          patchCord25(INmix, 0, BoutMix, 2);     // add input to output
AudioControlSGTL5000         sgtl5000_1;     //xy=1075,426

// GUItool: end automatically generated code

// Use these with the Teensy Audio Shield SD Card
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

//
// Co-efficients for FIR Hilbert filters (+45 and -45 degress)
//

#define NUM_COEFFS 200        // max of 200 taps if using "AudioFilterFIR_F32"

#include "HilbertP45_200_Tap.h"

#include "HilbertM45_200_Tap.h"


  uint16_t readPitch() {                             // read CV input until a stable pitch voltage
    uint16_t pitch = 1023 - analogRead(A8);   
    uint16_t newpitch = 1023 - analogRead(A8);
    int16_t delta = pitch - newpitch;
    while (delta > CVcalibrationStability || delta < (-1*CVcalibrationStability)) {               // delta limits control ADC read tolerance
      pitch = newpitch;
      newpitch = 1023 - analogRead(A8);                                                           // keep reading ADC until stable CV   
      delta = pitch - newpitch;
    }
    return newpitch;
  }

  float cv_c1_;
  auto uimode = UI_mode::NORMAL;
  
  void CalibrateC1() {
    cv_c1_ = static_cast<float>(readPitch());  
  }

 
  bool CalibrateC3() {
    float c3 = static_cast<float>(readPitch());             // approx 820 for +3V 
    float c1 = cv_c1_;                                      // approx 614 for +1V  
    float delta = c3 - c1;                                  // expected delta is approx 206 (2 octaves)
    #ifdef DEBUG
    Serial.println("====================================");
    Serial.print("calibration C1 value: ");    
    Serial.println(c1, 2);
    Serial.print("calibration C3 value: ");    
    Serial.println(c3, 2);
    Serial.print("calibration delta: ");    
    Serial.println(delta, 2);
    Serial.println("====================================");
    #endif

    float CVcalibration_ = 8.5333 * CVcalibrationAccuracy;                   // ADC counts per semitone   
    
    if (delta < (206.0 + CVcalibration_) && delta > (206.0 - CVcalibration_)) {                 // valid tolerance for expected delta   +/- 
      Calibration_Settings.shiftCV_scale = delta / 2.0f;      // ADC count value per OCTAVE (per octave, not per semitone !!!). expected value approx 102.4
      Calibration_Settings.shiftCV_offset = -1.0 *(c1 - Calibration_Settings.shiftCV_scale);     // expected ADC value  when CV input  = 0V   
      Calibration_Settings.shiftCV_offset_Oct = Calibration_Settings.shiftCV_offset / Calibration_Settings.shiftCV_scale;  // ADC offset value in octaves when input  = 0V  
      return true;
    }
    return false;
  }

//
// Configure the  A and B output mixers   
//
//  mixControl: Mixer Modes:
//      0:        Normal (UP to A, DOWN to B)   Amix.gain(0, 1.0f);   Amix.gain(1, 0.0f);   Bmix.gain(0, 1.0f);   Bmix.gain(1, 0.0f);
//      1:        A output = mix  (UP and DOWN to A, DOWN to B)   Amix.gain(0, 0.5f);   Amix.gain(1, 0.5f);   Bmix.gain(0, 1.0f);   Bmix.gain(1, 0.0f);
//      2:        B output = mix  (UP to A, DOWN and UP to B)   Amix.gain(0, 1.0f);   Amix.gain(1, 0.0f);   Bmix.gain(0, 0.5f);   Bmix.gain(1, 0.5f);
//
// mixCV: Mixer CV control amount (normally the total of the front panel mix control and the Mix CV input):
//      0.0         Full A channel
//      1.0         Full B channel
//      0.5         A+B
//      (i.e. channel A uses inverse value)
//
// We could do somtehing clever here like equal power mixing. 
// However, the actual Bode 1630 "mix" control is just a 5K linear potentiometer between the A and B outputs!
//
  void setOutMix(short mixCONTROL, float mixCV) {
    
    #ifdef addInputToOutput
    float maxVol= 0.7;                  // reduced volume if input added to output
    #endif

    #ifndef addInputToOutput
    float maxVol= 1.0;
    #endif

    float CV = mixCV * maxVol;        // adjust for any volume other than 1.0
    
    switch (mixCONTROL) {
      case 1: {
        AoutMix.gain(0, maxVol - CV);   // mix A into A output  
        AoutMix.gain(1, CV);          // mix B into A output 
        
        BoutMix.gain(0, maxVol);           //  B direct into B output  
        BoutMix.gain(1, 0.0f);           // no mix A into B out               
        break;
      }
      case 2: {
        AoutMix.gain(0, maxVol);           // A direct into A output  
        AoutMix.gain(1, 0.0f);           // no B into A output  
        
        BoutMix.gain(0, CV);           // mix B into B output  
        BoutMix.gain(1, maxVol - CV);    // mix A into B output  
        break;
      }
      default: {
        #ifndef ABmixtureControl
        AoutMix.gain(0, maxVol);           // A direct into A output, no mixture control  
        BoutMix.gain(0, maxVol);           // B direct into B output, no mixture control  
        
        AoutMix.gain(1, 0.0f);           // no mix B into A output  
        BoutMix.gain(1, 0.0f);           // no mix A into B output 
        #endif

        #ifdef ABmixtureControl
        AoutMix.gain(0, maxVol-CV);        // A direct into A output, WITH mixture control  
        BoutMix.gain(0, CV);               // B direct into B output, WITH mixture control  
        
        AoutMix.gain(1, 0.0f);           // no mix B into A output  
        BoutMix.gain(1, 0.0f);           // no mix A into B output 
        #endif
         
        break;    
      }
    }
  }

bool validSDcard; 
File root;


void playNextFile() {
    while(true) {                             // loop until non-directory file or end of files
       File entry =  root.openNextFile();
       if (! entry) {
         Serial.println("** rewind SD directory ** ");
         root.rewindDirectory();
         playFile(entry.name());
       }
       String curfile = entry.name();
       if (!entry.isDirectory() && curfile.lastIndexOf(".WAV")) {
          playFile(entry.name());
          break;
       } 
       entry.close();
   }
}   

void playFile(const char *filename) {

#ifdef DEBUG  
  Serial.println("--------------------------------------------");
  Serial.print("Playing file: ");
  Serial.println(filename);
  Serial.println("--------------------------------------------");
#endif
  
  playWav1.play(filename);

  delay(25);            // A brief delay for the library read WAV info
}


void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(4);
  AudioMemory_F32(16, audio_settings);
  
  Serial.begin(115200);

  // Enable the audio shield, select input, and enable output
  sgtl5000_1.enable();
//  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.muteHeadphone();           // not used - mute to save power ??
//
// Actual measured full-scale peak-to-peak sine wave input for max signal
// hardware divides signal by 6.3
//  0: 3.12 Volts p-p
//  1: 2.63 Volts p-p
//  2: 2.22 Volts p-p
//  3: 1.87 Volts p-p
//  4: 1.58 Volts p-p ** This value used to match Moog levels - any change will affect squelch and peak indicator thresholds !!**
//  5: 1.33 Volts p-p DEFAULT 
//  6: 1.11 Volts p-p
//  7: 0.94 Volts p-p
//  8: 0.79 Volts p-p
//  9: 0.67 Volts p-p
// 10: 0.56 Volts p-p
// 11: 0.48 Volts p-p 
// 12: 0.40 Volts p-p
// 13: 0.34 Volts p-p
// 14: 0.29 Volts p-p
// 15: 0.24 Volts p-p
//  sgtl5000_1.lineInLevel(inlevel);
  sgtl5000_1.lineInLevel(inleveltop, inlevelbottom);        // left channel (J1), right channel (J2)

// sgtl5000_1::lineOutLevel
// hardware multiplies the output signal by 7
//  Actual measured full-scale peak-to-peak sine wave output voltage:
//  0-12: output has clipping
//  13: 3.16 Volts p-p
//  14: 2.98 Volts p-p
//  15: 2.83 Volts p-p
//  16: 2.67 Volts p-p
//  17: 2.53 Volts p-p
//  18: 2.39 Volts p-p
//  19: 2.26 Volts p-p
//  20: 2.14 Volts p-p
//  21: 2.02 Volts p-p
//  22: 1.91 Volts p-p
//  23: 1.80 Volts p-p
//  24: 1.71 Volts p-p
//  25: 1.62 Volts p-p
//  26: 1.53 Volts p-p
//  27: 1.44 Volts p-p  ** this value used **
//  28: 1.37 Volts p-p
//  29: 1.29 Volts p-p   
//  30: 1.22 Volts p-p
//  31: 1.16 Volts p-p  
  sgtl5000_1.lineOutLevel(outlevel, outlevel);
  
//  SGT15000 DSP signal processing, if required:

//  sgtl5000_1.eqSelect(0);
//  sgtl5000_1.audioPostProcessorEnable();
//  sgtl5000_1.audioPreProcessorEnable();
//  sgtl5000_1.autoVolumeControl(2,1,0,-5,0.5,0.5);
//  sgtl5000_1.enhanceBassDisable();

  // set up push button
  PButton.attach(buttonSwitch, INPUT_PULLUP);
  PButton.interval(10);              // debounce interval in mS 
                                     // possible states: pressed, changed,fell, rose 

  pinMode(squelchLED, OUTPUT);         // LED
  pinMode(modeLED, OUTPUT);         // LED

  // set up the sine/cos LFO
  // cos = sin + 90 degrees (the absolute phases do not matter, but they must be 90 degrees apart)
  ShiftOsc.amplitude(1.0f); 
  ShiftOsc.frequency(0.1f);
  ShiftOsc.phaseS_C_r(1.57079633f);           // COSINE output leads SINE output by 90 degrees (in radians)    

//
// set up the shift-UP and Shift-DOWN mixers
// Do not change these otherwise the frequencey shifter will not work!!
//
  Amix.gain(0, 1.0f);         //  Hilbert45+ * Sin 
  Amix.gain(1, -1.0f);        // -1 * Hilbert45- * Cos (!! the gain here must be NEGATIVE for the shifter to work !!)
  Bmix.gain(0, 1.0f);         // Hilbert45+ * Sin 
  Bmix.gain(1, 1.0f);         // Hilbert45- * Cos


//
// set up the squelch
//
  squelchA.setOpeningTime(squelchAttack);
  squelchB.setOpeningTime(squelchAttack);  
  squelchA.setClosingTime(squelchRelease); 
  squelchB.setClosingTime(squelchRelease);   
  squelchA.setHoldTime(squelchHold);        
  squelchB.setHoldTime(squelchHold);
  squelchA.setThreshold(0.0);        
  squelchB.setThreshold(0.0);
  
 // the input mixer selects the sound source(s)
  INmix.gain(0, inputExtGain);   	// ch 0 is J1 input (left)    
  INmix.gain(1, inputExtGain);     // ch 1 is J2 (input (right)
  INmix.gain(2, inputTestOscGain);    // ch 2 is internal test tone     
  INmix.gain(3, inputSDcardGain);    	// ch 3,4 are internal SD card player    
  INmix.gain(4, inputSDcardGain);


  // initialise the Hilbert filters	 
  HILBERT45P.begin(inCoeffsplus, NUM_COEFFS);
  HILBERT45M.begin(inCoeffsminus, NUM_COEFFS);
  

  InWAVE.begin(1.0f, TestOscFrequency, WAVEFORM_SINE);   // set up internal test oscillator 

  analogReadResolution(10);

  // try and open an SD Card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    validSDcard = false;
    #ifdef DEBUG
    Serial.println("Unable to access the SD card");
    #endif
  } else {
    validSDcard=true;    
    #ifdef DEBUG
    Serial.println("SD card initialised");
    #endif
    root = SD.open("/");                // SD card root directory
    // autoplay first SD card file ?
    #ifdef SDautoPlay
    playNextFile();
    #endif
  }


  #ifdef eepromWIPE
      Calibration_Settings.magic_number_1=170;
      Calibration_Settings.magic_number_2=85;
      Calibration_Settings.shift_offset = default_shift_offset;          
      Calibration_Settings.shiftCV_scale = default_shiftCV_scale;        
      Calibration_Settings.shiftCV_offset = default_shiftCV_offset;  
      Calibration_Settings.shiftCV_offset_Oct = default_shiftCV_offset / default_shiftCV_scale;
      Calibration_Settings.mixMode = default_mixMode; 
      EEPROM.put(0, Calibration_Settings);
  #endif

  
  EEPROM.get(0, Calibration_Settings);
  if (Calibration_Settings.magic_number_1 != 170 || Calibration_Settings.magic_number_2 != 85) {    // no valid calibration is stored, so use the default values
      Calibration_Settings.magic_number_1=170;
      Calibration_Settings.magic_number_2=85;
      Calibration_Settings.shift_offset = default_shift_offset;          
      Calibration_Settings.shiftCV_scale = default_shiftCV_scale;        
      Calibration_Settings.shiftCV_offset = default_shiftCV_offset;  
      Calibration_Settings.shiftCV_offset_Oct = default_shiftCV_offset / default_shiftCV_scale;
      Calibration_Settings.mixMode = default_mixMode; 
  }

//
// set up the A and B output mixers   
//
  setOutMix(0, 0.5f);                     // default output mode (A to A out , B to B out), centred mix.

//
// add the input signal to the output ?
//
#ifndef addInputToOutput
      AoutMix.gain(2, 0.0f);           // no mix of input into A output  
      BoutMix.gain(2, 0.0f);           // no mix of input into B output 
#endif       

#ifdef addInputToOutput
      AoutMix.gain(2, 0.7f);           // no mix of input into A output  
      BoutMix.gain(2, 0.7f);           // no mix of input into B output 
#endif 

// Set up the scale switch I/O pins 
  pinMode(4, INPUT_PULLUP);   
  pinMode(3, INPUT_PULLUP);  
  pinMode(2, INPUT_PULLUP);
  
}


unsigned long last_time = millis();
float shiftAmount, oldShiftAmount, mixCV, mixControl, squelchControl, pitchCV, previousPitchCV, squelchSignal, mixTotal, mixTotalOld;
float shiftControl, shiftCVtotal, hysteresis, deadzone, shiftPhase, peakValue;
bool squelchLEDsuppress=false, longPressActive=false;
int mixMode;
long buttonTimer;


void loop() {

//
// Set the "scale" switch value from D4 D3 D2
//

  uint8_t scale =  0;
  if (digitalRead(2) == LOW) scale = scale + 1;
  if (digitalRead(3) == LOW) scale = scale + 2;  
  if (digitalRead(4) == LOW) scale = scale + 4;    
  
  PButton.update();

  //
  // MixCV controls the balance at the selected Mix output:
  // 0.0  100% channel A, 0% channel B 
  // 1.0  0% channel A, 100% channel B
  mixCV = static_cast <float> (1023 - analogRead(A3)) /1023.0;  // Mix CV input, range 0.0 (-5V input) to 1.0 (+5V input)
  mixControl =  static_cast <float> (analogRead(A0)) /1023.0;  // Mix panel control 0.0 to 1.0
  
  mixTotal = (mixControl + mixCV - 0.5f);                      // Total of mixCV plus mixControl, range 0.0 (output A) - 1.0 (output B)
  if (mixTotal > 1.0f) mixTotal  = 1.0f;
  if (mixTotal < 0.0f) mixTotal  = 0.0f;  

  if (mixTotal != mixTotalOld) setOutMix(mixMode, mixTotal);   // only update the mix if a change  of control (or CV)
  mixTotalOld = mixTotal;
  
  // Squelch control
  // the threshold setting value relates to the external inputs.
  // Threshold value may not be correct for SD card or internal test osc signals!
  squelchControl = static_cast <float> (analogRead(A1)) /1023.0;                                               // Squelch panel control, range 0.0 to 1.0
  float squelchValue= (-31.57f * squelchControl * squelchControl) + (93.586f * squelchControl) - 61.258;       // Moog control scale mapped to Squelch value in dBm
  if (squelchValue > 0.0f )  squelchValue = 0.0f ;                                                             // constrain to max panel value
  if (squelchValue < -60.0f )  squelchValue = -60.0f ;                                                         // constrain to min panel value
  squelchValue = squelchValue - 13.2;                                                                          // conversion factor for "real world" dBm value to F32 Noisegate dBfs value.
  squelchA.setThreshold(squelchValue);        
  squelchB.setThreshold(squelchValue);

  // squelch indicator
  if (!squelchLEDsuppress) {
      if (squelchA.infoIsOpen()) {
        digitalWrite(squelchLED, 1); 
      } else {
        digitalWrite(squelchLED, 0); 
      }    
  }


//
//  if the button is pressed when scale != "CAL"
//  for a long press: we set the output mixing mode from the Mix Control setting
//  for a short press: we start the next SD card file (if SD card is present)
//
   if (scale != 0 && uimode == UI_mode::NORMAL )  { 
      if(PButton.fell()) {                                   // button first pressed 
          squelchLEDsuppress = true;   
          digitalWrite(squelchLED, 1);                      // both buttons on    
          digitalWrite(modeLED, 1);      
          longPressActive = false;  
          buttonTimer = millis();  
      }

      if (!longPressActive && (millis() - buttonTimer) > longPressTime) {
        longPressActive = true;
        digitalWrite(modeLED, 0);                         // clear both LEDs 
        digitalWrite(squelchLED, 0);  
      }     

      if(PButton.rose()) {                                      // button released 
          if (longPressActive == true ) {                       // long press release:save mix mode
              Calibration_Settings.mixMode = mixMode;  
              EEPROM.put(0, Calibration_Settings);              // we write the mix mode to eeprom, even if the rest of the calibration settings are still set to "default" values.
          } else {                                              // short press released: next SD card track
              if (validSDcard) {
                   playNextFile();
              }
          }
          digitalWrite(modeLED, 0);                             // clear any mix mode LEDs 
          digitalWrite(squelchLED, 0);  
          squelchLEDsuppress = false;   
          longPressActive = false;       
      }

     
      if(PButton.read()==0 && longPressActive) {                               // button long press active: adjust mix mode
          if (mixControl < 0.25) {
                  mixMode = 1;
                  digitalWrite(squelchLED, 1);     
                  digitalWrite(modeLED, 0);     
          } else {
            if (mixControl > 0.75) {
                  mixMode = 2;
                  digitalWrite(squelchLED, 0);     
                  digitalWrite(modeLED, 1);    
            } else {
                  mixMode = 0;
                  digitalWrite(squelchLED, 0);     //            
                  digitalWrite(modeLED, 0);     // 
            }
          } 

          setOutMix(mixMode, mixTotal);         // reset the output mix in the new mix mode
      }
      
   }

//
// Instigate CV calibration procedure if any press of mode button when scale is set to "CAL" (scale 0)
//
  if (scale == 0 && PButton.fell())  {  

 //
 //    Check if/where we are in any part of the calibration process:
 //
      switch ( uimode ) { 
         case UI_mode::NORMAL: {                //  move to state CALIBRATION_1
                  digitalWrite(squelchLED, 0);     
                  digitalWrite(modeLED, 1);     
                  squelchLEDsuppress=true;
                  uimode = UI_mode::CALIBRATION_1;
                  break;
                  }          
        case UI_mode::CALIBRATION_1: {           // get C1 value, move to state CALIBRATION_2
                  CalibrateC1();
                  Calibration_Settings.shift_offset = analogRead(A2);   // read the current setting of the shift control input 
                  digitalWrite(squelchLED, 1);     // 
                  digitalWrite(modeLED, 0);     // 
                  uimode = UI_mode::CALIBRATION_2;
                  break;
                  }   
        case UI_mode::CALIBRATION_2: {        // get C3 value, calibrate, then move to  states NORMAL or PANIC
                  bool success = CalibrateC3();
                  if (success) {
                      uimode = UI_mode::NORMAL;
                      digitalWrite(squelchLED, 0);     // 
                      digitalWrite(modeLED, 0);     // 
                      EEPROM.put(0, Calibration_Settings);                        // We don't need to use any form of eeprom levelling: if we write once a day, the
                                                                                  // eeprom will last about 270 years!
                    
                  } else {
                      uimode = UI_mode::PANIC;
                      digitalWrite(squelchLED, 1);     // 
                      digitalWrite(modeLED, 1);     // 
                      // Calibration_Settings.shift_offset = default_shift_offset;          // use default calibration settings
                      Calibration_Settings.shiftCV_scale = default_shiftCV_scale;        
                      Calibration_Settings.shiftCV_offset = default_shiftCV_offset;  
                      Calibration_Settings.shiftCV_offset_Oct = default_shiftCV_offset / default_shiftCV_scale;
                      EEPROM.put(0, Calibration_Settings);
                  }
                  break;
                  }           
        case UI_mode::PANIC: {                 // if in state PANIC, another button press clears it.              
                  digitalWrite(squelchLED, 0);      
                  digitalWrite(modeLED, 0);      
                  uimode = UI_mode::NORMAL;     
                  squelchLEDsuppress=false;
                  break;        
                  }
     } // end of switch (UIMode)
  } // end of   if (scale == 0 && PButton.fell())
  
 //
 //  Shift Control and CV scaling for each Scale setting.
 //
  switch (scale) {
        case 0: {
           shiftAmount = 0.0f;                                                  // Calibration mode : no shift
           hysteresis=0.0f;                                                     // no phase change hysteresis
           deadzone=0.0f;                                                       // no zero shift deadzone        
        break;
        }
        
        case 1: {                                                                                          // exponential, CV's etc are all measured in OCTAVES
          shiftControl = static_cast <float> (analogRead(A2)) / 102.4;                                     // Shift CV panel input, 10 octave range with no offset in EXP mode
          pitchCV = Calibration_Settings.shiftCV_offset_Oct;                                               // start with offset - ** in octaves **
          pitchCV += static_cast <float> (1023 - analogRead(A8)) / Calibration_Settings.shiftCV_scale;     // add incoming CV value, in octaves  
           
          float interval = pitchCV - previousPitchCV;
          // When a pitch difference of more than 1 quartertone is observed, jump straight to the right pitch.
          // otherwise, slew limit the CV change. 
          // (Thanks to Émilie Gillet of Mutable Instruments)
          if (interval < -0.042f || interval > 0.042f) {                                                  // 0.042 = 1/24  (octave/quartertone)      
            previousPitchCV = pitchCV;
          } else {
            previousPitchCV += expCVslewLimiter * interval;                                              // slew limiter 
          }
                                                                    
           shiftCVtotal = shiftControl +  previousPitchCV ;                                              // sum incoming CV with front panel
           shiftAmount = expShiftStart * pow(2.0, shiftCVtotal) ;                                        // exponential shift frequency
           hysteresis=0.0f;                                                                              // no phase change hysteresis
           deadzone=0.0f;                                                                                // no zero shift deadzone
          
        break;
        }
        
        case 2: {                                                                                           // Linear 5 Hz   
           shiftAmount = ((static_cast <float> (analogRead(A2)) - Calibration_Settings.shift_offset) + (static_cast <float> (1023 - analogRead(A8))+Calibration_Settings.shiftCV_offset)) /102.4;
           hysteresis=0.2f;            
           deadzone=0.2f;   
           break;
        }
           
        case 3: {                                                                                           // Linear 50 Hz
           shiftAmount = ((static_cast <float> (analogRead(A2)) - Calibration_Settings.shift_offset) + (static_cast <float> (1023 - analogRead(A8))+Calibration_Settings.shiftCV_offset)) /10.24;  
           hysteresis=2.5f;  
           deadzone=2.5f;  
           break;
          }
           
        case 4: {                                                                                           // Linear 500 Hz
           shiftAmount = ((static_cast <float> (analogRead(A2)) - Calibration_Settings.shift_offset) + (static_cast <float> (1023 - analogRead(A8))+Calibration_Settings.shiftCV_offset)) /1.024;     
           hysteresis=25.0f;   
           deadzone=25.0f;  
           break;
        }
           
        case 5: {                                                                                         // Linear 5000 Hz
           shiftAmount = ((static_cast <float> (analogRead(A2)) - Calibration_Settings.shift_offset) + (static_cast <float> (1023 - analogRead(A8))+Calibration_Settings.shiftCV_offset)) * 9.96;  
           hysteresis=300.0f; 
           deadzone=250.0f; 
           break;
        }   
     } // end of switch(scale)

//
// Process changes of shift frequency and direction (modulator phase).
// (by definition, we can only change shift direction if the frequency has also changed!)
//
     
  #ifdef shiftControlDeadZone
  if (shiftAmount < deadzone && shiftAmount > (-1.0 * deadzone)) shiftAmount=0.0;       // deadzone around zero shift
  #endif

  #ifndef shiftControlHysteresis
  hysteresis=0.0;                                      // disable hysteresis if not selected in user settings 
  #endif
  
  if (shiftAmount != oldShiftAmount) {
    
    ShiftOsc.frequency(fabs(shiftAmount));              // set the new shift frequency, value is always positive.

    if (shiftAmount > hysteresis) {                   // switch phase at upper hysteresis point
          shiftPhase=1.57079633f;                     // COSINE output leads SINE output by 90 degrees (in radians) 
          ShiftOsc.phaseS_C_r(shiftPhase);
          ShiftOsc.amplitude(1.0f);                   // maximum modulation amplitude 
    } else {
    if (shiftAmount < (-1.0f * hysteresis)) {         // switch phase at lower hysteresis point
          shiftPhase=-1.57079633f;                    // COSINE output lags SINE output by 90 degrees (in radians)               
          ShiftOsc.phaseS_C_r(shiftPhase); 
          ShiftOsc.amplitude(1.0f);                   // maximum modulation amplitude     
    }
  
    #ifdef shiftControlVariablePhase
    else {
    //
    // This code gradually changes the modulator phase over the hysteresis zone around zero shift.
    // if omitted we get an instantaneous change of phase at the hysteresis points.
    // However, note that it results in amplitude modulation in the hysteresis zone.
    //
    if (shiftAmount < 0.0f && shiftAmount >= (-1.0f * hysteresis)) {                    // moving from negative shift towards positive shift
          shiftPhase=-1.57079633f * (1.0f - (shiftAmount + hysteresis) / hysteresis);   // reduce shift phase as we approach zero 
          ShiftOsc.phaseS_C_r(shiftPhase); 
          #ifdef shiftControlAmplitudeMod                           
          ShiftOsc.amplitude(shiftPhase / -1.57079633f);                                // reduce modulation amplitude as we approach zero shift ( Note: gives zero output level at zero shift!)  
          #endif                        
    } else {
       if (shiftAmount > 0.0f && shiftAmount <= hysteresis) {                           // moving from positive shift towards negative shift
          shiftPhase=1.57079633f * (1.0f - (hysteresis - shiftAmount) / hysteresis);    // reduce shift phase as we approach zero 
          ShiftOsc.phaseS_C_r(shiftPhase);  
          #ifdef shiftControlAmplitudeMod
          ShiftOsc.amplitude(shiftPhase / 1.57079633f);                                  // reduce modulation amplitude as we approach zero shift  ( Note: zero output level at zero shift!)  
          #endif               
          }
    }
    }
    #endif

    }
    
    oldShiftAmount = shiftAmount;                    
 }  

  #ifdef SDautoPlay 
  if (validSDcard && !playWav1.isPlaying()) {
      playNextFile();   
  }
  #endif
     
  // peak LED
  #ifdef useModeLEDasPeak
  if (!squelchLEDsuppress && peakLED.available()) {
    peakValue = peakLED.read();
    if (peakValue > peakLEDthreshold) {                        
      digitalWrite(modeLED, 1); 
    } else {
      digitalWrite(modeLED, 0); 
    }    
  }
  #endif
  
  // print information about resource usage 
  #ifdef DEBUG
  if (millis() - last_time >= debugUpdate) {
    
    Serial.print("Proc = ");
    Serial.print(audio_settings.processorUsage());
    Serial.print(" (");    
    Serial.print(audio_settings.processorUsageMax());
    Serial.print("),  Mem = ");
    Serial.print(AudioMemoryUsage());
    Serial.print(" (");    
    Serial.print(AudioMemoryUsageMax());
    Serial.print("),  Mem F32 = ");
    Serial.print(AudioMemoryUsage_F32());
    Serial.print(" (");    
    Serial.print(AudioMemoryUsageMax_F32());
    Serial.println(")");
    Serial.print("Scale: ");    
    Serial.println(scale, DEC);
    if (!squelchLEDsuppress) {
       Serial.print("Input Peak: ");    
       Serial.println(peakValue, 2);        
      }
    if (scale == 1 ) {
        Serial.print("EXP: Pitch CV: ");    
        Serial.print(previousPitchCV, 2);
        Serial.print("   Shift Control: ");    
        Serial.print(shiftControl, 2);    
        Serial.print("   Shift CV Total: "); 
        Serial.println(shiftCVtotal, 2);       
    }
    Serial.print("Shift Frequency: ");    
    Serial.print(shiftAmount, 2);
    Serial.print("  Shift Phase: ");    
    Serial.println(shiftPhase, 2);
    Serial.print("Mix Control: ");    
    Serial.print(mixControl, 2);
    Serial.print("   Mix CV: ");    
    Serial.println(mixCV, 2);
    Serial.print("Squelch Control: ");    
    Serial.print(squelchControl, 2);
    Serial.print("   Squelch (dB): ");    
    Serial.println(squelchValue, 2);
    Serial.print("   Peak Value: ");    
    Serial.println(    peakValue, 3);
        
    last_time = millis();
  }
  #endif  

}
