 /********************************************************************************************************************************************************
 *                                                                                                                                                       *
 *  Project:         FFT Spectrum Analyzer                                                                                                               *
 *  Target Platform: ESP32                                                                                                                               *
 *                                                                                                                                                       * 
 *  Version: 1.0                                                                                                                                         *
 *  Hardware setup: See github                                                                                                                           *
 *  Spectrum analyses done with analog chips MSGEQ7                                                                                                      *
 *                                                                                                                                                       * 
 *  Mark Donners                                                                                                                                         *
 *  The Electronic Engineer                                                                                                                              *
 *  Website:   www.theelectronicengineer.nl                                                                                                              *
 *  facebook:  https://www.facebook.com/TheelectronicEngineer                                                                                            *
 *  youtube:   https://www.youtube.com/channel/UCm5wy-2RoXGjG2F9wpDFF3w                                                                                  *
 *  github:    https://github.com/donnersm                                                                                                               *
 *                                                                                                                                                       *  
 ********************************************************************************************************************************************************* 
 * Version History                                                                                                                                       *
 *  1.0 First release, code extraced from 14 band spectrum analyzer 3.00 and modified to by used with FFT on a ESP32. No need for frequency board or     *
 *      MCGEQ7 chips.                                                                                                                                    *
 *      - HUB75 interface or                                                                                                                             *
 *      - WS2812 leds ( matrix/ledstrips)                                                                                                                *
 *      - 8/16/32 or 64 channel analyzer                                                                                                                 *
 *      - calibration for White noise, pink noise, brown noise sensitivity included and selectable                                                         *
 *      - Fire screensaver                                                                                                                               *
 *      - Display of logo and interface text when used with HUB75                                                                                        *
 *                                                                                                                                                       *
 *********************************************************************************************************************************************************
 * Version FFT 1.0 release July 2021                                                                                                                     *
 *********************************************************************************************************************************************************
 *  Status   | Description                                                                                                                               *
 *  Open     | Some Hub75 displays use a combination of chipsets of are from a different productions batch which will not work with this libary          *
 *  Open     | Sometime the long press for activating/de-activating the autoChange Pattern mode doesn't work                                             *
 *  Solved   | When using 64 bands, band 0 is always at max value. This was caused by the array dize [64]-> solved by chnaging it to 65                  *
 * Not a bug | Different types of HUB75 displays require different libary settings.It is what it is and it all depends on what the distributer sends you.*
 *           | For into on the libary settings, see the library documentation on Github: https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA  *
 * Wish      | Web interface. not possible without some heavy workaround cant use WIFI and ADC at same time                                              *
 * *******************************************************************************************************************************************************          
 * People who inspired me to do this build and figure out how stuff works:
 * Dave Plummer         https://www.youtube.com/channel/UCNzszbnvQeFzObW0ghk0Ckw
 * Mrfaptastic          https://github.com/mrfaptastic
 * Scott Marley         https://www.youtube.com/user/scottmarley85
 * Brian Lough          https://www.youtube.com/user/witnessmenow
 * atomic14             https://www.youtube.com/channel/UC4Otk-uDioJN0tg6s1QO9lw
 * 
 * Make sure your arduino IDE settings: Compiler warnings is set to default to make sure the code will compile                                           */





#define VERSION     "V1.0"

#include <FastLED.h>
#include <arduinoFFT.h>
#include "I2SPLUGIN.h"
#include <math.h>
#include "FFT.h"
#include "LEDDRIVER.H"
#include "Settings.h"




int skip=true;
int ButtonOnTimer=0;
int ButtonStarttime=0;
int ButtonSequenceCounter=0;
int ButtonOffTimer=0;
int ButtonStoptime=0;

int ButtonPressedCounter=0;
int ButtonReleasedCounter=0;
int ShortPressFlag=0;
int LongPressFlag=0;
int LongerPressFlag=0;
boolean Next_is_new_pressed= true;
boolean Next_is_new_release = true;
int PreviousPressTime=0;
#define up  1
#define down 0
int PeakDirection=0;

long LastDoNothingTime = 0;                       // only needed for screensaver
int DemoModeMem=0;                                   // to remember what mode we are in when going to demo, in order to restore it after wake up
bool AutoModeMem=false;                                // same story
bool DemoFlag=false;                               // we need to know if demo mode was manually selected or auto engadged. 


void setup() {
 Serial.begin(115200);
 Serial.println("Setting up Audio Input I2S");
 setupI2S();
 Serial.println("Audio input setup completed");
 delay(1000);
 SetupLEDSTRIP();

}

void loop() {
  size_t bytesRead = 0;
  int TempADC=0;
  if (skip==false)i2s_adc_disable(I2S_NUM_0);
  skip=false; // we only want to skip this the very first loop run.
  FastLED.setBrightness(100);
    
    //############ Step 1: read samples from the I2S Buffer ##################
  i2s_adc_enable(I2S_NUM_0);

  i2s_read(I2S_PORT, 
          (void*)samples, 
            sizeof(samples),  
            &bytesRead,   // workaround This is the actual buffer size last half will be empty but why?
            portMAX_DELAY); // no timeout

  if (bytesRead != sizeof(samples)){
    Serial.printf("Could only read %u bytes of %u in FillBufferI2S()\n", bytesRead, sizeof(samples));
    // return;
  }

  //############ Step 2: compensate for Channel number and offset, safe all to vReal Array   ############
  for (uint16_t i = 0; i < ARRAYSIZE(samples); i++) {
    vReal[i] = offset-samples[i];
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
 

 //############ Step 3: Do FFT on the VReal array  ############
  // compute FFT
  FFT.DCRemoval();
  FFT.Windowing(vReal, SAMPLEBLOCK, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLEBLOCK, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLEBLOCK);
  FFT.MajorPeak(vReal, SAMPLEBLOCK, samplingFrequency); 
  for (int i = 0; i < numBands; i++) {
    FreqBins[i] = 0;
  }
 //############ Step 4: Fill the frequency bins with the FFT Samples ############
 float averageSum = 0.0f;
 for (int i = 2; i < SAMPLEBLOCK / 2; i++){ 
   averageSum+=vReal[i];
   if (vReal[i] > NoiseTresshold){
     int freq = BucketFrequency(i);
     int iBand = 0;
     while (iBand < numBands){
       if (freq < BandCutoffTable[iBand])break;
       iBand++;
     }
     if (iBand > numBands)iBand = numBands;
     FreqBins[iBand]+= vReal[i]; 
     //  float scaledValue = vReal[i];
     //  if (scaledValue > peak[iBand])
     //    peak[iBand] = scaledValue;
   }
 }


 //############ Step 5: Determine the VU value  and mingle in the readout...( cheating the bands ) ############ Step 
 float t=averageSum / (SAMPLEBLOCK / 2);
 gVU = max(t, (oldVU * 3 + t) / 4);
 oldVU = gVU; 
 if(gVU>DemoTreshold)LastDoNothingTime = millis(); // if there is signal in any off the bands[>2] then no demo mode

  // Serial.printf("gVu: %d\n",(int) gVU);

 for(int j=0;j<numBands;j++){
   if (CalibrationType==1)FreqBins[j]*= BandCalibration_Pink[j];
   else if (CalibrationType==2)FreqBins[j]*= BandCalibration_White[j];
    else if (CalibrationType==3)FreqBins[j]*= BandCalibration_Brown[j];

 }

 //*
 //############ Step 6: Averaging and making it all fit on screen 
 //for (int i = 0; i < numBands; i++) {
   //Serial.printf ("Chan[%d]:%d",i,(int)FreqBins[i]);
    //FreqBins[i] = powf(FreqBins[i], gLogScale); // in case we want log scale..i leave it in here as reminder
   //  Serial.printf( " - log: %d \n",(int)FreqBins[i]);
// }
 static float lastAllBandsPeak = 0.0f;
 float allBandsPeak = 0;
 //bufmd[1]=FreqBins[13];
 //bufmd[2]=FreqBins[1];
 for (int i = 0; i < numBands; i++){
   //allBandsPeak = max (allBandsPeak, FreqBins[i]);
   if (FreqBins[i]> allBandsPeak){
     allBandsPeak = FreqBins[i];
   }
 }   
 if (allBandsPeak < 1)allBandsPeak = 1;
 //  The followinf picks allBandsPeak if it's gone up.  If it's gone down, it "averages" it by faking a running average of GAIN_DAMPEN past peaks
 allBandsPeak = max(allBandsPeak, ((lastAllBandsPeak * (GAIN_DAMPEN-1)) + allBandsPeak) / GAIN_DAMPEN);  // Dampen rate of change a little bit on way down
 lastAllBandsPeak = allBandsPeak;


 if (allBandsPeak < 80000)allBandsPeak = 80000;
 for (int i = 0; i < numBands; i++){ 
   FreqBins[i] /= (allBandsPeak * 1.0f);
 }

 // Process the FFT data into bar heights
 for (int band = 0; band < numBands; band++) {
   int barHeight = FreqBins[band]*kMatrixHeight-1;  //(AMPLITUDE);
   if (barHeight > TOP-2) barHeight = TOP-2;
  
   // Small amount of averaging between frames
   barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;

   // Move peak up
   if (barHeight > peak[band]) {
     peak[band] = min(TOP, barHeight);
     PeakFlag[band]=1;
   }
    bndcounter[band]+=barHeight; // ten behoeve calibratie

  // Now visualize those bar heights
  switch (buttonPushCounter) {
    case 0:
      changingBarsLS(band, barHeight);
     break;
     
    case 1: 

     TriBarLS(band, barHeight);
     TriPeakLS(band);

     break;
    case 2:
      rainbowBarsLS(band, barHeight);
      NormalPeakLS(band, PeakColor1);
      break;
    case 3:
      purpleBarsLS(band, barHeight);
      NormalPeakLS(band, PeakColor2);
      break;
  } 
  
  // Save oldBarHeights for averaging later
  oldBarHeights[band] = barHeight;
 }
// for calibration
 //bndcounter[h]+=barHeight;
 if (loopcounter==256){
  loopcounter=0;
 #if CalibratieLog 
   Calibration();
   for(int g=0;g<numBands;g++)bndcounter[g]=0;
 #endif
 
 }
 loopcounter++;

   // Decay peak
 EVERY_N_MILLISECONDS(Fallingspeed){
   for (byte band = 0; band < numBands; band++){
     if(PeakFlag[band]==1){
       PeakTimer[band]++;
       if (PeakTimer[band]> Peakdelay){PeakTimer[band]=0;PeakFlag[band]=0;}
     }
     else if ((peak[band] > 0) &&(PeakDirection==up)){ 
       peak[band] += 1;
       if (peak[band]>(kMatrixHeight+10))peak[band]=0;
       } // when to far off screen then reset peak height
     else if ((peak[band] > 0)&&(PeakDirection==down)){ peak[band] -= 1;}
   }   
     colorTimer++;
 }

 
 EVERY_N_MILLISECONDS(10)colorTimer++; // Used in some of the patterns
 

  delay(1); // needed to give fastled a minimum recovery time
  FastLED.show();

  
} // loop end






  // BucketFrequency
  //
  // Return the frequency corresponding to the Nth sample bucket.  Skips the first two 
  // buckets which are overall amplitude and something else.

int BucketFrequency(int iBucket){
 if (iBucket <= 1)return 0;
 int iOffset = iBucket - 2;
 return iOffset * (samplingFrequency / 2) / (SAMPLEBLOCK / 2);
}


void Calibration(void){
  Serial.printf("BandCalibration_XXXX[%1d]=\n{",numBands);
  long Totalbnd=0;
  
  for (int g=0;g<numBands;g++){
    if (bndcounter[g]>Totalbnd)Totalbnd=bndcounter[g];
  }
  
  
  for (int g=0;g<numBands;g++){
    bndcounter[g]=Totalbnd/bndcounter[g];
    Serial.printf(" %2.2f",bndcounter[g]);
    if(g<numBands-1)Serial.printf(",");
    else Serial.print(" };\n");
  }
}