#include <arduinoFFT.h>
#include "NotchFilter.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char *ssid = "RobotDungeon24G";
const char *password = "helloworld";

#include <driver/i2s.h>
// you shouldn't need to change these settings
#define SAMPLE_RATE 44100 / 8
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0

const int BUFFER_SIZE = 1024;
const int FFT_SIZE = 1024;
const int MIN_FREQUENCY = 200;
const int MAX_FREQUENCY = 500;
const int FREQUENCY_RESOLUTION = 1;

int32_t sampleBuffer[FFT_SIZE];

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];

/* Create FFT object */
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_SIZE, SAMPLE_RATE / 2);

// Define the frequencies of the C major scale notes (in Hz) tuned with 432Hz
const float cMajorScale[] = {256.87, 288.33, 323.63, 342.88, 384.87, 432.00, 484.90, 513.74};

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Setup I2S ...");

  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
}

bool MODE_FFT = true;
bool MODE_NOTCH = false;

const int READ_LEN = 2;
NotchFilter filter = NotchFilter(cMajorScale[1], 44100 / READ_LEN, 10.0);
// NotchFilter(float centerFreq, float sampleRate, float bandwidth) {
bool memBit = false;
float maxVal = 0.0;
void loop()
{
  ArduinoOTA.handle();
  if (MODE_FFT && getSamples())
  {
    performFFT();
    performNoteDetection();
  }

  if (MODE_NOTCH)
  {
    float newSample = getSampleForNotchFilter();
    float filteredOutput = filter.filter(newSample);
    // Serial.print("-500, 500, ");
    // Serial.println(filteredOutput);
    if (filteredOutput > maxVal)
    {
      maxVal = filteredOutput;
      memBit = true;
    }
    else if (memBit)
    {
      memBit = false;
      Serial.print("0, 500, ");
      Serial.println(maxVal);
    }
  }
}

float getSampleForNotchFilter()
{
  size_t bytes_read = 0;

  float output = 0.0;

  i2s_read(I2S_NUM_0, sampleBuffer, sizeof(int32_t) * READ_LEN, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  // dump the samples out to the serial channel.
  if (samples_read != READ_LEN)
  {
    Serial.print("samples read don't equal buffer size,");
    Serial.println(samples_read);
  }
  else
  {
    for (int i = 0; i < samples_read; i++)
    {
      output += (float)sampleBuffer[i] / 1000000.0;
    }
  }
  return output / (float)READ_LEN;
}

// stores the I2C Samples into sBuffer
bool getSamples()
{

  // read from the I2S device
  size_t bytes_read = 0;

  i2s_read(I2S_NUM_0, sampleBuffer, sizeof(int32_t) * BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  // dump the samples out to the serial channel.
  if (samples_read != BUFFER_SIZE)
  {
    Serial.print("samples read don't equal buffer size,");
    Serial.println(samples_read);
    return false;
  }
  else
  {
    for (int i = 0; i < samples_read; i++)
    {
      // Serial.printf("%ld\n", sampleBuffer[i]);
      vReal[i] = (double)sampleBuffer[i] / 100000.0;
    }
    return true;
  }
}

void zeroImaginary()
{
  for (int i = 0; i < FFT_SIZE; i++)
  {
    vImag[i] = 0.0; // Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
}

void i2s_install()
{

  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 44100,
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = BUFFER_SIZE,
      .use_apll = false};

  /*
   i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
     .sample_rate = SAMPLE_RATE,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
     .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
     .communication_format = I2S_COMM_FORMAT_I2S,
     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
     .dma_buf_count = 4,
     .dma_buf_len = BUFFER_SIZE,
     .use_apll = false,
     .tx_desc_auto_clear = false,
     .fixed_mclk = 0};
   */

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin()
{
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
}

void performFFT()
{
  zeroImaginary(); // initialize the imaginary part of the FFT
  // Serial.println("Data:");
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
  // Serial.println("Weighed data:");
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  // Serial.println("Computed Real values:");
  // Serial.println("Computed Imaginary values:");
  FFT.complexToMagnitude(); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  float peakFrequency;
  float peakAmp;
  FFT.majorPeak(&peakFrequency, &peakAmp);
  // Serial.print("Peak frequency:");
  // Serial.print(peakFrequency);
  // Serial.print(", amp: ");
  // Serial.println(peakAmp);
  if (false && abs(peakFrequency - cMajorScale[1]) < 3.0)
  {
    Serial.print("0, 10000, ");
    Serial.println(peakAmp);
  }

  // putDataIntoFrequencyBands();
}

void putDataIntoFrequencyBands()
{
  const int NUM_BANDS = (MAX_FREQUENCY - MIN_FREQUENCY) / 10 + 1;
  float bands[NUM_BANDS];

  // Initialize bands to zero
  for (int i = 0; i < NUM_BANDS; i++)
  {
    bands[i] = 0.0;
  }

  // Accumulate the magnitudes into the corresponding bands
  for (int i = MIN_FREQUENCY; i <= MAX_FREQUENCY; i++)
  {
    int bandIndex = (i - MIN_FREQUENCY) / 10;
    bands[bandIndex] += vReal[i];
  }

  // Print the frequency bands and their accumulated magnitudes
  for (int i = 0; i < NUM_BANDS; i++)
  {
    int bandMinFrequency = MIN_FREQUENCY + i * 10;
    int bandMaxFrequency = bandMinFrequency + 10;
    Serial.print("Frequency Band ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(bandMinFrequency);
    Serial.print(" Hz - ");
    Serial.print(bandMaxFrequency);
    Serial.print(" Hz, Magnitude: ");
    Serial.println(bands[i]);
  }
}

void getTopFrequencies()
{
  const int NUM_FREQUENCIES = 4;
  float frequencies[NUM_FREQUENCIES];
  float amplitudes[NUM_FREQUENCIES];

  // Find the top frequencies and their amplitudes
  for (int i = 0; i < NUM_FREQUENCIES; i++)
  {
    float maxAmplitude = 0.0;
    int maxIndex = 0;

    // Find the maximum amplitude and its index
    for (int j = MIN_FREQUENCY; j <= MAX_FREQUENCY; j += FREQUENCY_RESOLUTION)
    {
      if (vReal[j] > maxAmplitude)
      {
        maxAmplitude = vReal[j];
        maxIndex = j;
      }
    }

    // Store the frequency and amplitude
    frequencies[i] = maxIndex;
    amplitudes[i] = maxAmplitude;

    // Set the amplitude of the maximum frequency to zero
    vReal[maxIndex] = 0.0;
  }

  // Print the top frequencies and their amplitudes
  for (int i = 0; i < NUM_FREQUENCIES; i++)
  {
    Serial.print("Frequency ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(frequencies[i]);
    Serial.print(" Hz, Amplitude: ");
    Serial.println(amplitudes[i]);
  }
}

const float THRESHOLD = 100.0;
void performNoteDetection()
{

  // Define the frequency bins corresponding to each note in the C major scale
  // const int noteFrequencies[7] = {256, 288, 323, 342, 384, 432, 484}; // Adjusted for 432 Hz tuning

  // Initialize variables to store detected notes and their amplitudes
  int detectedNotes[8]; // Maximum 8 simultaneous notes
  double detectedAmplitudes[8];
  int numDetectedNotes = 0;

  // Iterate through the FFT bins
  for (int i = 0; i < FFT_SIZE / 2; i++)
  {
    double frequency = i * (SAMPLE_RATE / FFT_SIZE);
    float frequencyBandHz = (SAMPLE_RATE / FFT_SIZE);
    if (abs(frequency + frequencyBandHz / 2 - cMajorScale[1]) < frequencyBandHz / 2)
    {
      // Serial.println(vReal[i]);
    }
    if (frequency < 500)
    {
      Serial.print(vReal[i]);
      Serial.print(", ");
    }

    // Check if the magnitude in this bin exceeds a certain threshold

    if (vReal[i] > THRESHOLD && false)
    {
      // Calculate the corresponding frequency for this bin

      // Check if the frequency is close to any note in the C major scale
      bool wasFound = false;
      for (int j = 0; j < 8; j++)
      {
        if (abs(frequency - cMajorScale[j]) <= frequencyBandHz)
        { //
          // Add the detected note and its amplitude to the lists
          detectedNotes[numDetectedNotes] = cMajorScale[j];
          detectedAmplitudes[numDetectedNotes] = vReal[i];
          numDetectedNotes++;

          // Break out of the inner loop once a note is detected
          break;
        }
      }
      if (!wasFound && vReal[i] > THRESHOLD)
      {
        Serial.print("freq: ");
        Serial.print(frequency);
        Serial.print("amp: ");
        Serial.print(vReal[i]);
      }

      // Break out of the outer loop if we've reached the maximum number of detected notes
      if (numDetectedNotes == 8)
      {
        break;
      }
    }
  }

  // Print information about the detected notes
  // Serial.print("Detected notes: ");
  /*
  for (int i = 0; i < numDetectedNotes; i++) {
    if(detectedNotes[i] == cMajorScale[0]){
      Serial.print("C4 Root Chakra");
    }
    else if(detectedNotes[i] == cMajorScale[1]){
      Serial.print("D4 Sacral Chakra");
    }
    else if(detectedNotes[i] == cMajorScale[2]){
      Serial.print("E4");
    }
    else if(detectedNotes[i] == cMajorScale[3]){
      Serial.print("F4");
    }
    else if(detectedNotes[i] == cMajorScale[4]){
      Serial.println("G4");
    }
    else if(detectedNotes[i] == cMajorScale[5]){
      Serial.print("A4");
    }
    else if(detectedNotes[i] == cMajorScale[6]){
      Serial.print("B4");
    }
    else{
      Serial.print("C5");
    }
    //Serial.print(detectedNotes[i]);
    Serial.print(" , Amplitude: ");
    Serial.print(detectedAmplitudes[i]);
  }
  */
  Serial.println();
}
