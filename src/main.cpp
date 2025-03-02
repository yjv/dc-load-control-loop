#include <Arduino.h>
#include <SPI.h>
#include <adc.h>
#include <QuickPID.h>
// #include "soc/rtc.h"
// #include "hal/wdt_hal.h"
// #include "rtc_wdt.h"

// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 5
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

static constexpr uint8_t GATE_DRIVE_SCLK = D5;
static constexpr uint8_t GATE_DRIVE_COPI = D6;
static constexpr uint8_t GATE_DRIVE_CIPO = D4;
static constexpr uint8_t SENSE_ERROR = D7;
static constexpr uint8_t GATE_DRIVE_LDAC = D8;
static constexpr uint8_t GATE_DRIVE_CS = D9;
static constexpr uint8_t SENSE_CS = D10;
static constexpr uint8_t SENSE_COPI = D11;
static constexpr uint8_t SENSE_CIPO = D12;
static constexpr uint8_t SENSE_SCLK = D13;

uint32_t gateCode = 0;
float gate = 0.0f;

// SPIClass senseSPI(HSPI);
SPIClass gateSPI(FSPI);

ADC adc(HSPI, SENSE_SCLK, SENSE_CIPO, SENSE_COPI, SENSE_CS);

uint32_t currentCode = 0;
uint32_t voltageCode = 0;

float current;
float voltage;
float power;
float resistance;
float setpoint = 0.25f;

QuickPID pid(&current, &gate, &setpoint, 8.0f, 3.0f, 0.0f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnError, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

static QueueHandle_t reading_queue;
static const int reading_queue_len = 200;

SemaphoreHandle_t semaphore;
#define TAG "main"

boolean updated = false;

void IRAM_ATTR readADC()
{
  Reading reading = adc.read();

  xQueueSendFromISR(reading_queue, &reading, NULL);
  // if (reading.getStatus().currentConversionChannel() == Channel::CH1)
  // {
  //   currentCode = reading.getData();
  // }

  // if (reading.getStatus().currentConversionChannel() == Channel::CH0)
  // {
  //   voltageCode = reading.getData();
  // }

  // ESP_LOGI(TAG, "Reading: %07d, Status: 0x%02x, Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV, Gate Code: %d\n", 
  //   reading.getData(), 
  //   reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance,
  //   gate,
  //   gateCode
  // );

  // current = (float)currentCode / 16777216.0 * 5.0 / 50 / 0.0047;
  // voltage = static_cast<float>(voltageCode) / 16777216.0 * 5.0 / (16e-3/2.13);
  // power = current * voltage;
  // resistance = voltage / max(current, 0.000001f);

  // ESP_LOGI(TAG, "Reading: %07d, Status: 0x%02x, Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV, Gate Code: %d\n", 
  //   reading.getData(), 
  //   reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance,
  //   gate,
  //   gateCode
  // );
    // ESP_LOGI(TAG, "taking semaphore in interupt");

  // do {} while (xSemaphoreTakeFromISR(semaphore, NULL) != pdPASS);
  // ESP_LOGI(TAG, "got semaphore in interupt");
  // current = static_cast<float>(currentCode) / 16777216.0 * 5.0 / 50 / 0.0047;
  // voltage = static_cast<float>(voltageCode) / 16777216.0 * 5.0 / (16e-3/2.13);
  // power = current * voltage;
  // resistance = voltage / max(current, 0.000001f);

  // xSemaphoreGiveFromISR(semaphore, NULL);
  // updateDac();
}

void updateDac()
{
  // Serial.printf("Reading: %07d, Status: 0x%02x, Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV, Gate Code: %d\n", 
  //   reading.getData(), 
  //   reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance,
  //   gate,
  //   gateCode
  // );

  gateCode = static_cast<uint32_t>(gate / 5.0 * 1048576.0f);

  gateSPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  gateSPI.transferBits(gateCode << 4, NULL, 24);
  gateSPI.endTransaction();
 
  digitalWrite(GATE_DRIVE_LDAC, LOW);
  digitalWrite(GATE_DRIVE_LDAC, HIGH);
}

TaskHandle_t adcTask;

// wdt_hal_context_t rwdt_ctx;

void runAdc( void * pvParameters ) {

  ESP_LOGI(TAG, "Task2 running on core ");
  ESP_LOGI(TAG, "%d", xPortGetCoreID());
  // wdt_hal_disable();

  for(;;){
    delay(10);
  } 
}

void setup() {
  esp_reset_reason_t reason = esp_reset_reason();

  esp_log_level_set("*", ESP_LOG_DEBUG);
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(5000);
  Serial.printf("LOG_LOCAL_LEVEL %d\n", LOG_LOCAL_LEVEL);
  Serial.setDebugOutput(true);
  ESP_LOGE(TAG, "Reset reason: %d", reason);
  ESP_LOGE(TAG, "ESP_LOGE, level 1 = ARDUHAL_LOG_LEVEL_ERROR   = ESP_LOG_ERROR");
  ESP_LOGW(TAG, "ESP_LOGW, level 2 = ARDUHAL_LOG_LEVEL_WARN    = ESP_LOG_WARN");    
  ESP_LOGI(TAG, "ESP_LOGI, level 3 = ARDUHAL_LOG_LEVEL_INFO    = ESP_LOG_INFO");
  ESP_LOGD(TAG, "ESP_LOGD, level 4 = ARDUHAL_LOG_LEVEL_DEBUG   = ESP_LOG_DEBUG");
  ESP_LOGV(TAG, "ESP_LOGV, level 5 = ARDUHAL_LOG_LEVEL_VERBOSE = ESP_LOG_VERBOSE");
  pinMode(SENSE_ERROR, INPUT);
  pinMode(GATE_DRIVE_LDAC, OUTPUT);
  pinMode(GATE_DRIVE_COPI, OUTPUT);
  pinMode(GATE_DRIVE_CIPO, INPUT);
  pinMode(GATE_DRIVE_SCLK, OUTPUT);
  pinMode(GATE_DRIVE_CS, OUTPUT);
  pinMode(SENSE_CS, OUTPUT);
  pinMode(SENSE_COPI, OUTPUT);
  pinMode(SENSE_CIPO, INPUT);
  pinMode(SENSE_SCLK, OUTPUT);

  digitalWrite(GATE_DRIVE_LDAC, HIGH);
  digitalWrite(GATE_DRIVE_CS, LOW);

  Serial.println("Starting SPI...");

  gateSPI.begin(GATE_DRIVE_SCLK, GATE_DRIVE_CIPO, GATE_DRIVE_COPI, GATE_DRIVE_CS);
  gateSPI.setHwCs(true);

  Serial.println("Starting ADC...");

  reading_queue = xQueueCreate(reading_queue_len, sizeof(Reading));
  adc.initialize();
  adc.reset();
  adc.configureMode(Mode::CONTINUOUS, false, SettleDelay::DELAY_0_US);
  adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, FilterSampleRate::SPS_1000);
  adc.configureSetup(Setup::SETUP1, Filter::Sinc5Sinc1, FilterSampleRate::SPS_1000);
  adc.configureChannel(Channel::CH0, Setup::SETUP0, ChannelInput::AIN1, ChannelInput::AIN0);
  adc.configureChannel(Channel::CH1, Setup::SETUP1, ChannelInput::AIN3, ChannelInput::AIN2);
  adc.disableChannel(Channel::CH2);
  adc.disableChannel(Channel::CH3);
  adc.configureInterface(true, true, true, ChecksumMode::CRC);
  delay(5000);

  attachInterrupt(digitalPinToInterrupt(SENSE_CIPO), readADC, FALLING);
  adc.startReading();  

  pid.SetSampleTimeUs(100);
  pid.SetOutputLimits(0, 5.0);
  pid.SetMode(QuickPID::Control::automatic);
}

uint8_t ready = 0;

void loop() {

  Reading reading(false, Status(0), 0);
  uint8_t reading_count = 0;

  while (xQueueReceive(reading_queue, (void *)&reading, 0) == pdTRUE) {

    reading_count++;

    if (reading.getStatus().currentConversionChannel() == Channel::CH1)
    {
      currentCode = reading.getData();
      current = static_cast<float>(currentCode) / 16777216.0 * 5.0 / 50 / 0.0047;
    }

    if (reading.getStatus().currentConversionChannel() == Channel::CH0)
    {
      voltageCode = reading.getData();
      voltage = static_cast<float>(voltageCode) / 16777216.0 * 5.0 / (16e-3/2.13);
    }

    power = current * voltage;
    resistance = voltage / max(current, 0.000001f);

    if (pid.Compute())
    {
      updateDac();
    }
  }

  ESP_LOGI(TAG, "Reading count: %d, Reading: %07d, Status: 0x%02x, Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV, Gate Code: %d\n", 
    reading_count,
    reading.getData(), 
    reading.getStatus().currentConversionChannel().getNumber(),
    current,
    voltage,
    power,
    resistance,
    gate,
    gateCode
  );

  // while (!updated) {
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }

  // xSemaphoreGive(semaphore);

  // ESP_LOGI("adc", "Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm", 
  //   // reading.getData(), 
  //   // reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance
  // );

  // uint8_t newReady = digitalRead(CIPO);

  // if ( newReady!= ready)
  // {
  //   ready = newReady;
  //   // Serial.printf("Ready: %d\n", ready);
  // }

  // Serial.printf("Id: 0x%02x\n", adc.getId());

  // if (adc.check())
  // {
  //   // Serial.println("Reading ADC...");
  //   Reading reading = adc.read();

  //   if (reading.getStatus().currentConversionChannel() == Channel::CH1)
  //   {
  //     currentCode = reading.getData();
  //   }

  //   if (reading.getStatus().currentConversionChannel() == Channel::CH0)
  //   {
  //     voltageCode = reading.getData();
  //   }
  // ESP_LOGI("adc", "Task2 running on core ");
  // ESP_LOGI("adc", "%d", xPortGetCoreID());

  // Serial.printf("Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV, Gate Code: %d\n", 
  //   reading.getData(), 
  //   reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance,
  //   gate,
  //   gateCode
  // );

    // Serial.printf("ADCMODE: 0x%04x\n", adc.readRegister(Register::ADCMODE));
    // Serial.printf("IFMODE: 0x%04x\n", adc.readRegister(Register::IFMODE));
    // Serial.printf("CH0: 0x%04x\n", adc.readRegister(Register::CH0));
    // Serial.printf("CH1: 0x%04x\n", adc.readRegister(Register::CH1));
    // Serial.printf("SETUPCON0: 0x%04x\n", adc.readRegister(Register::SETUPCON0));
    // Serial.printf("SETUPCON1: 0x%04x\n", adc.readRegister(Register::SETUPCON1));
    // Serial.printf("FILTCON0: 0x%04x\n", adc.readRegister(Register::FILTCON0));
    // Serial.printf("FILTCON1: 0x%04x\n", adc.readRegister(Register::FILTCON1));

    // adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, FilterSampleRate::SPS_1000);
  // }

}




