#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "RobotDungeon24G";
const char* password = "helloworld";

// Include I2S driver
#include <driver/i2s.h>

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Define input buffer length
#define bufferLen 64
int16_t sBuffer[bufferLen];

void i2s_install() {
// Set up I2S Processor configuration
const i2s_config_t i2s_config = {
.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
.sample_rate = 44100,
//.sample_rate = 11025, if you like
.bits_per_sample = i2s_bits_per_sample_t(16),
.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
//.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
.intr_alloc_flags = 0,
.dma_buf_count = 8,
.dma_buf_len = bufferLen,
.use_apll = false
};

i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
// Set I2S pin configuration
const i2s_pin_config_t pin_config = {
.bck_io_num = I2S_SCK,
.ws_io_num = I2S_WS,
.data_out_num = -1,
.data_in_num = I2S_SD
};

i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
}

void loop() {
  ArduinoOTA.handle();

  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }

      // Average the data reading
      mean /= samples_read;

      // Print to serial plotter
      Serial.println(mean);
    }
  }
}