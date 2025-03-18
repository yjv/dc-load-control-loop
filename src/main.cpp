#include <Arduino.h>
#include <SPI.h>
#include <adc.h>
#include <QuickPID.h>
#include <PID.h>
#include <vector>
#include <cmath>

static constexpr uint8_t GATE_DRIVE_LDAC = D5;
static constexpr uint8_t GATE_DRIVE_CS = D6;
static constexpr uint8_t GATE_DRIVE_COPI = D7;
static constexpr uint8_t GATE_DRIVE_CIPO = D8;
static constexpr uint8_t GATE_DRIVE_SCLK = D9;
static constexpr uint8_t SENSE_CS = D10;
static constexpr uint8_t SENSE_COPI = D11;
static constexpr uint8_t SENSE_CIPO = D12;
static constexpr uint8_t SENSE_SCLK = D13;

uint32_t gateCode = 0;
float gate = 0.0f;

SPIClass gateSPI(FSPI);

ADC adc(HSPI, SENSE_SCLK, SENSE_CIPO, SENSE_COPI, SENSE_CS);

uint32_t currentCode = 0;
uint32_t voltageCode = 0;

float current = 0;
float voltage = 0;
float power = 0;
float resistance = 0;

enum class LoadMode: uint8_t {
  CONSTANT_CURRENT,
  CONSTANT_VOLTAGE,
  CONSTANT_POWER,
  CONSTANT_RESISTANCE
};

LoadMode mode = LoadMode::CONSTANT_CURRENT;
float setpoint = 0.0f;
uint32_t setpointCode = 0;

arc::PID<float> gatePid(0.0f, 0.0f, 0.0f);

static QueueHandle_t reading_queue;
static const int reading_queue_len = 200;

SemaphoreHandle_t semaphore;
#define TAG "main"

void IRAM_ATTR readADC()
{
  Reading reading = adc.read();

  xQueueSendFromISR(reading_queue, &reading, NULL);
}

void updateDac()
{
  gateCode = static_cast<uint32_t>(gate / 5.0 * 1048576.0f);

  gateSPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  gateSPI.transferBits(gateCode << 4, NULL, 24);
  gateSPI.endTransaction();
 
  digitalWrite(GATE_DRIVE_LDAC, LOW);
  digitalWrite(GATE_DRIVE_LDAC, HIGH);
}

uint32_t reading_count = 0;
uint32_t invalid_count = 0;
uint32_t updateDacCount = 0;

void runLoad(void * pvParameters) {

  uint8_t readBoth = 0;

  while (true) {

    Reading reading(false, Status(0), 0, 0);

    while (xQueueReceive(reading_queue, (void *)&reading, 1) == pdTRUE) {
    
      reading_count++;

      if (!reading.isValid() || reading.getStatus().hasADCError())
      {
        invalid_count++;
        continue;
      }

      if (reading.getStatus().currentConversionChannel() == Channel::CH1)
      {
        currentCode = reading.getData();
        current = static_cast<float>(currentCode) / 16777216.0 * 5.0 / 50 / 0.0047; //167.77216
        readBoth |= 1;
      }

      if (reading.getStatus().currentConversionChannel() == Channel::CH0)
      {
        voltageCode = reading.getData();
        voltage = static_cast<float>(voltageCode) / 16777216.0 * 5.0 / (22.9e-3 / 3.079); //1,677.7216
        readBoth |= 2;
      }

      power = current * voltage;
      resistance = voltage / max(current, 0.000001f);

      float input = 0.0f;

      if (mode == LoadMode::CONSTANT_CURRENT)
      {
        input = current;
      }
      else if (mode == LoadMode::CONSTANT_VOLTAGE)
      {
        input = voltage;
      }
      else if (mode == LoadMode::CONSTANT_POWER)
      {
        input = power;
      }
      else if (mode == LoadMode::CONSTANT_RESISTANCE)
      {
        input = resistance;
      }

      gatePid.setInput(input, reading.getTime());
    }

    if (readBoth == 3)
    {
      readBoth = 0;

      float gatePidOutput = gatePid.getOutput();

      if (mode == LoadMode::CONSTANT_CURRENT || mode == LoadMode::CONSTANT_POWER)
      {
        gate += gatePidOutput;
      }
      else
      {
        gate -= gatePidOutput;
      }

      gate = min(max(gate, 0.0f), 4.9999952316f);

      updateDacCount++;
      updateDac();  
    }
  }
}

TaskHandle_t runLoadTaskHandle;
TaskHandle_t runCommunicationTaskHandle;

unsigned long previous_time = 0;

void runCommunication( void* pvParameters ) {
  
  while (true)
  {
    while (Serial.available() > 0)
    {
      uint8_t command = Serial.read();

      if (command == 0x00)
      {
        float maxSetpoint;
        uint8_t * setpointData = new uint8_t[5];

        Serial.read(setpointData, 5);
        std::reverse(setpointData + 1, setpointData + 5);
        mode = static_cast<LoadMode>(setpointData[0]);

        ESP_LOGD(TAG, "setpointData: %d, %d, %d, %d, %d", setpointData[0], setpointData[1], setpointData[2], setpointData[3], setpointData[4]);
        ESP_LOGD(TAG, "Mode: %d, Setpoint: %d", mode, setpointCode);

        if (mode == LoadMode::CONSTANT_CURRENT)
        {
          maxSetpoint = 20.0;
        }
        else if (mode == LoadMode::CONSTANT_VOLTAGE)
        {
          maxSetpoint = 600.0;
        }
        else if (mode == LoadMode::CONSTANT_POWER)
        {
          maxSetpoint = 200.0;
        }
        else if (mode == LoadMode::CONSTANT_RESISTANCE)
        {
          maxSetpoint = 20e3;
        }
      
        setpointCode = *(uint32_t *)(setpointData + 1);
        setpoint = maxSetpoint * (float)setpointCode / 16777216.0 * 5.0 / 4.7;
        gatePid.setTarget(setpoint);
      }
    }

    // float kp = 0.085 * 0.6;
    float kp = (float)analogRead(A0) / 4096.0 * 0.75;
    // float ki = 1.2 * 0.085 / 20;
    float ki = (float)analogRead(A1) / 4096.0 * 0.75 / 1000;
    // float kd = 0.075 * 0.085 / 20e-6;
    float kd = (float)analogRead(A2) / 4096.0 * 0.75 * 1000;

    gatePid.setKp(kp);
    gatePid.setKi(ki);
    gatePid.setKd(kd);

    previous_time = millis();
    ESP_LOGD(TAG, "Reading count: %d, Invalid count: %d, Setpoint: %f, Kp: %f, Ki: %f, Gate: %fV, Gate PID Output: %f\n", 
      reading_count,
      invalid_count,
      setpoint,
      kp,
      ki,
      gate,
      gatePidOutput
    );

    uint32_t voltageCodeSent = (uint32_t)((float)voltage / 638.2978723404 * 16777216.0);
    uint8_t dataSize = sizeof(uint32_t) * 7 + 1;

    uint8_t * data = new uint8_t[dataSize];
    data[0] = 0xFF;

    uint32_t modeCode = (uint32_t)mode;

    uint8_t *dataPtr = &data[1];
    memcpy(dataPtr, &updateDacCount, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &invalid_count, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &reading_count, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &modeCode, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &setpointCode, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &currentCode, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &voltageCodeSent, sizeof(uint32_t));

    std::reverse(data + 1, data + dataSize);
    Serial.write(data, dataSize);

    reading_count = 0;
    invalid_count = 0;
    updateDacCount = 0;

    vTaskDelay(500);
  }
}

void setup() {
  esp_reset_reason_t reason = esp_reset_reason();

  // esp_task_wdt_add(NULL);

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

  FilterSampleRate sampleRate = FilterSampleRate::SPS_1000;

  reading_queue = xQueueCreate(reading_queue_len, sizeof(Reading));
  adc.initialize();
  adc.reset();
  adc.configureMode(Mode::CONTINUOUS, false, SettleDelay::DELAY_0_US);
  adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, sampleRate);
  adc.configureSetup(Setup::SETUP1, Filter::Sinc5Sinc1, sampleRate);
  adc.configureChannel(Channel::CH0, Setup::SETUP0, ChannelInput::AIN1, ChannelInput::AIN0);
  adc.configureChannel(Channel::CH1, Setup::SETUP1, ChannelInput::AIN3, ChannelInput::AIN2);
  adc.disableChannel(Channel::CH2);
  adc.disableChannel(Channel::CH3);
  adc.configureInterface(true, true, true, ChecksumMode::CRC);
  delay(5000);

  xTaskCreatePinnedToCore(
    runLoad,
    "runLoad",
    8192,
    NULL,
    1,
    &runLoadTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    runCommunication,
    "runCommunication",
    8192,
    NULL,
    1,
    &runCommunicationTaskHandle,
    1
  );
  attachInterrupt(digitalPinToInterrupt(SENSE_CIPO), readADC, FALLING);
  adc.startReading();  
}

void loop() {
  
}




