#include <Arduino.h>
#include <SPI.h>
#include <adc.h>
#include <dac.h>
#include <cmath>
#include <control/control_algorithm.h>
#include <control/pid_algorithm.h>
#include <load.h>
#include <Wire.h>
#include <communication/control_loop.h>

static constexpr uint8_t GATE_DRIVE_LDAC = D5;
static constexpr uint8_t GATE_DRIVE_CS = D6;
static constexpr uint8_t GATE_DRIVE_COPI = D7;
static constexpr uint8_t GATE_DRIVE_SCLK = D9;
static constexpr uint8_t SENSE_CS = D10;
static constexpr uint8_t SENSE_COPI = D11;
static constexpr uint8_t SENSE_CIPO = D12;
static constexpr uint8_t SENSE_SCLK = D13;

ADC adc(HSPI, SENSE_SCLK, SENSE_CIPO, SENSE_COPI, SENSE_CS);
DAC dac(FSPI, GATE_DRIVE_SCLK, GATE_DRIVE_COPI, GATE_DRIVE_CS, GATE_DRIVE_LDAC);

// ControlAlgorithm * control_algorithm = new RelativeAlgorithm();
ControlAlgorithm * control_algorithm = new PidControlAlgorithm();

State state;

ControlLoop control_loop(&Wire, state);

static QueueHandle_t reading_queue;
static constexpr int reading_queue_len = 10000;

#define TAG "main"

void IRAM_ATTR readADC()
{
  const Reading reading = adc.read();
  // Reading reading(false, Status(0), 0, 0);
  xQueueSendFromISR(reading_queue, &reading, NULL);
}

[[noreturn]] void runLoad(void *) {

  while (true) {

    uint32_t current_count = 0;
    uint64_t current_codes = 0;
    uint32_t voltage_count = 0;
    uint64_t voltage_codes = 0;
    unsigned long latest_time = 0;

    Reading reading(false, Status(0), 0, 0);

    while (xQueueReceive(reading_queue, &reading, 1) == pdTRUE) {

      ValueType value_type;

      if (reading.getStatus().currentConversionChannel() == Channel::CH1) {
        value_type = ValueType::CURRENT;
      }
      else if (reading.getStatus().currentConversionChannel() == Channel::CH0) {
        value_type = ValueType::VOLTAGE;
      }
      else
      {
        ESP_LOGE("main", "Invalid conversion channel: %d", reading.getStatus().currentConversionChannel());
        abort(); // Abort if there are no matches
      }

      const bool valid = reading.isValid() && !reading.getStatus().hasADCError();
      state.incrementReadingCount(valid, value_type);

      if (!valid)
      {
        continue;
      }

      if (value_type == ValueType::CURRENT)
      {
        current_count++;
        current_codes += reading.getData();
        latest_time = reading.getTime();
      }

      if (value_type == ValueType::VOLTAGE)
      {
        voltage_count++;
        voltage_codes += reading.getData();
        latest_time = reading.getTime();
      }
    }

    if (voltage_count == 0 && current_count == 0)
    {
      continue;
    }

    if (current_count > 0)
    {
      state.setCurrent(new Current(static_cast<uint32_t>(current_codes/current_count)));
    }

    if (voltage_count > 0)
    {
      state.setVoltage(new Voltage(static_cast<uint32_t>(voltage_codes/voltage_count)));
    }

    control_algorithm->setTarget(*state.getSetpoint());
    control_algorithm->addReading(state.getCurrent(), state.getVoltage(), latest_time);

    auto gate_control_voltage = control_algorithm->updateOutput(state.getGateControlVoltage());
    state.setGateControlVoltage(gate_control_voltage);

    dac.write(state.getGateControlVoltage()->getCode());
  }
}

TaskHandle_t runLoadTaskHandle;
TaskHandle_t runCommunicationTaskHandle;

[[noreturn]] void runCommunication( void* ) {
  
  while (true)
  {
    while (Serial.available() > 0)
    {
      const uint8_t command = Serial.read();

      if (command == 0x00)
      {
        const auto setpoint_data = new uint8_t[5];

        Serial.read(setpoint_data, 5);
        std::reverse(setpoint_data + 1, setpoint_data + 5);
        auto mode = static_cast<LoadMode>(setpoint_data[0]);

        ESP_LOGD(TAG, "setpoint_data: %d, %d, %d, %d, %d", setpoint_data[0], setpoint_data[1], setpoint_data[2], setpoint_data[3], setpoint_data[4]);
        ESP_LOGD(TAG, "Mode: %d, Setpoint: %d", mode, state.getSetpoint()->getCode());

        const uint32_t setpoint_code = *reinterpret_cast<uint32_t*>(setpoint_data + 1);

        Value * setpoint = nullptr;

        switch (mode)
        {
        case LoadMode::OFF:
          setpoint = new Current(0u);
          break;
        case LoadMode::CONSTANT_CURRENT:
          setpoint = new Current(setpoint_code);
          break;
        case LoadMode::CONSTANT_VOLTAGE:
          setpoint = new Voltage(setpoint_code);
          break;
        case LoadMode::CONSTANT_POWER:
          setpoint = new Power(setpoint_code);
          break;
        case LoadMode::CONSTANT_RESISTANCE:
          setpoint = new Resistance(setpoint_code);
          break;
        default:
          ESP_LOGE(TAG, "Unhandled LoadMode: %d", static_cast<int>(mode));
          std::terminate(); // Terminate the program if an unhandled enum value is encountered
        }

        state.setMode(mode);
        state.setSetpoint(setpoint);
        control_algorithm->setTarget(*setpoint);
      }
    }

    // kp for CC: 0.19
    // kp for CV: 0.01
    // float kp = 0.085 * 0.6;
    float kp = static_cast<float>(analogRead(A0)) / 4096.0f * 1.0f;
    float ki = 0.0f;
    // float ki = static_cast<float>(analogRead(A1)) / 4096.0f * 0.75f;
    float kd = 0.0f;
    // float kd = static_cast<float>(analogRead(A2)) / 4096.0f * 0.75f;

    ((PidControlAlgorithm*)control_algorithm)->setCoefficients(kp, ki, kd);

    ESP_LOGD(TAG, "Reading count: %d, Invalid count: %d, Invalid current count: %d, Invalid voltage count: %d, Setpoint: %f, Kp: %f, Ki: %f, Gate: %fV, Gate PID Output: %u, Voltage: %u, Current: %f\n",
      state.getReadingCount(),
      state.getInvalidCurrentReadingCount() + state.getInvalidVoltageReadingCount(),
      state.getInvalidCurrentReadingCount(),
      state.getInvalidVoltageReadingCount(),
      state.getSetpoint()->getValue(),
      kp,
      ki,
      state.getGateControlVoltage()->getValue(),
      state.getGateControlVoltage()->getCode(),
      (*state.getVoltage() / *state.getCurrent()).getValue(),
      state.getCurrent()->getValue()
    );

    auto mode_code = static_cast<uint32_t>(state.getMode());
    uint32_t setpoint_code = state.getSetpoint()->getCode();
    uint32_t current_code = state.getCurrent()->getCode();
    uint32_t voltage_code = state.getVoltage()->getCode();
    uint32_t reading_count = state.getReadingCount();
    uint32_t invalid_count = state.getInvalidCurrentReadingCount() + state.getInvalidVoltageReadingCount();
    uint32_t update_dac_count = state.getUpdateGateVoltageCount();

    constexpr uint8_t data_size = sizeof(uint32_t) * 7 + 1;

    const auto data = new uint8_t[data_size];
    data[0] = 0xFF;

    uint8_t *dataPtr = &data[1];
    memcpy(dataPtr, &update_dac_count, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &invalid_count, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &reading_count, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &mode_code, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &setpoint_code, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &current_code, sizeof(uint32_t));
    dataPtr += sizeof(uint32_t);
    memcpy(dataPtr, &voltage_code, sizeof(uint32_t));

    std::reverse(data + 1, data + data_size);
    Serial.write(data, data_size);

    state.clearReadingCounts();
    state.clearUpdateGateVoltageCount();
    vTaskDelay(500);
  }
}

void onWireReceive(int num_bytes) {
  control_loop.onReceive(num_bytes);
}

void onWireRequest() {
  control_loop.onRequest();
}

void setup() {
  esp_reset_reason_t reason = esp_reset_reason();

  // esp_task_wdt_add(NULL);

  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("Value", ESP_LOG_DEBUG);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(2);
  Wire.onReceive(onWireReceive);
  Wire.onRequest(onWireRequest);
  delay(5000);
  Serial.printf("LOG_LOCAL_LEVEL %d\n", LOG_LOCAL_LEVEL);
  Serial.setDebugOutput(true);
  ESP_LOGE(TAG, "Reset reason: %d", reason);
  ESP_LOGE(TAG, "ESP_LOGE, level 1 = ARDUHAL_LOG_LEVEL_ERROR   = ESP_LOG_ERROR");
  ESP_LOGW(TAG, "ESP_LOGW, level 2 = ARDUHAL_LOG_LEVEL_WARN    = ESP_LOG_WARN");    
  ESP_LOGI(TAG, "ESP_LOGI, level 3 = ARDUHAL_LOG_LEVEL_INFO    = ESP_LOG_INFO");
  ESP_LOGD(TAG, "ESP_LOGD, level 4 = ARDUHAL_LOG_LEVEL_DEBUG   = ESP_LOG_DEBUG");
  ESP_LOGV(TAG, "ESP_LOGV, level 5 = ARDUHAL_LOG_LEVEL_VERBOSE = ESP_LOG_VERBOSE");
  
  digitalWrite(GATE_DRIVE_LDAC, HIGH);
  digitalWrite(GATE_DRIVE_CS, LOW);

  Serial.println("Starting SPI...");

  dac.initialize();

  Serial.println("Starting ADC...");

  FilterSampleRate sampleRate = FilterSampleRate::SPS_5000;

  reading_queue = xQueueCreate(reading_queue_len, sizeof(Reading));
  adc.initialize();
  adc.reset();
  adc.internalZeroScaleCalibration();
  adc.configureMode(Mode::CONTINUOUS, false, SettleDelay::DELAY_0_US);
  adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, sampleRate);
  adc.configureSetup(Setup::SETUP1, Filter::Sinc5Sinc1, sampleRate);
  adc.configureSetup(Setup::SETUP2, Filter::Sinc5Sinc1, sampleRate);
  adc.configureChannel(Channel::CH0, Setup::SETUP0, ChannelInput::AIN1, ChannelInput::AIN0);
  adc.configureChannel(Channel::CH1, Setup::SETUP1, ChannelInput::AIN3, ChannelInput::AIN2);
  // adc.configureChannel(Channel::CH2, Setup::SETUP2, ChannelInput::AIN0, ChannelInput::AIN4);
  adc.disableChannel(Channel::CH2);
  adc.disableChannel(Channel::CH3);
  adc.configureInterface(true, true, true, ChecksumMode::CRC);

  xTaskCreatePinnedToCore(
    runLoad,
    "runLoad",
    8192,
    nullptr,
    1,
    &runLoadTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    runCommunication,
    "runCommunication",
    8192,
    nullptr,
    1,
    &runCommunicationTaskHandle,
    1
  );
  attachInterrupt(digitalPinToInterrupt(SENSE_CIPO), readADC, FALLING);
  adc.startReading();
}

void loop() {
  
}




