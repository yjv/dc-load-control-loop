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
#include <communication/register.h>

// Hardware pin definitions for gate drive control (DAC)
static constexpr uint8_t GATE_DRIVE_LDAC = D6;  // Load DAC pin for gate control voltage
static constexpr uint8_t GATE_DRIVE_CS = D9;    // Chip select for gate drive DAC
static constexpr uint8_t GATE_DRIVE_COPI = D7;  // Controller out, peripheral in for gate drive
static constexpr uint8_t GATE_DRIVE_SCLK = D8;  // Serial clock for gate drive DAC

// Hardware pin definitions for current/voltage sensing (ADC)
static constexpr uint8_t SENSE_CS = D5;         // Chip select for sense ADC
static constexpr uint8_t SENSE_CS_GPIO = 8;     // GPIO number for sense CS (for direct register access)
static constexpr uint8_t SENSE_SCLK = D4;       // Serial clock for sense ADC
static constexpr uint8_t SENSE_COPI = D3;       // Controller out, peripheral in for sense ADC
static constexpr uint8_t SENSE_CIPO = D2;       // Controller in, peripheral out for sense ADC

// I2C communication address for control loop
static constexpr uint8_t I2C_ADDR = 0x10;

// Initialize ADC for current/voltage sensing using high-speed SPI
ADC adc(HSPI, SENSE_SCLK, SENSE_CIPO, SENSE_COPI, SENSE_CS);

// Initialize DAC for gate control voltage output using fast SPI
DAC dac(FSPI, GATE_DRIVE_SCLK, GATE_DRIVE_COPI, GATE_DRIVE_CS, GATE_DRIVE_LDAC);

// Control algorithm selection - currently using PID control
// ControlAlgorithm * control_algorithm = new RelativeAlgorithm();
ControlAlgorithm * control_algorithm = new PidControlAlgorithm();

// Load object that manages the electronic load state and control
load::Load loadObject(control_algorithm);

// Communication interface for I2C control of the load
communication::ControlLoop control_loop(&Wire, loadObject);

// FreeRTOS queue for ADC readings from interrupt context
static QueueHandle_t reading_queue;
static constexpr int reading_queue_len = 10000;  // Maximum queue size for ADC readings

#define TAG "main"  // Logging tag for this module

/**
 * Interrupt service routine for ADC data ready signal
 * This function is called when the ADC has new data available.
 * It reads the ADC data and queues it for processing by the main control loop.
 * 
 * Note: This function runs in interrupt context, so it must be fast and not block.
 */
void IRAM_ATTR readADC()
{
  // Read the latest ADC conversion result
  const Reading reading = adc.read();
  // Reading reading(false, Status(0), 0, 0);  // Debug/test reading

  // Queue the reading for processing by the main control loop task
  xQueueSendFromISR(reading_queue, &reading, NULL);

  // Toggle CS to prepare for next conversion (alternative to direct GPIO register access)
  digitalWrite(SENSE_CS, HIGH);
  digitalWrite(SENSE_CS, LOW);
  // GPIO.out_w1ts = (uint32_t)1 << SENSE_CS_GPIO;  // Direct register access (commented)
  // GPIO.out_w1tc = (uint32_t)1 << SENSE_CS_GPIO;  // Direct register access (commented)
}

/**
 * Main control loop task that processes ADC readings and updates the load control
 * This task runs continuously on core 0 and handles:
 * - Processing ADC readings from the interrupt queue
 * - Averaging current and voltage readings
 * - Updating the control algorithm with new readings
 * - Computing and applying new gate control voltage
 * 
 * @param parameter Unused task parameter
 */
[[noreturn]] void runLoad(void *) {

  while (true) {
    // Accumulators for averaging multiple ADC readings
    uint32_t current_count = 0;      // Number of current readings received
    uint64_t current_codes = 0;      // Sum of current ADC codes for averaging
    uint32_t voltage_count = 0;      // Number of voltage readings received
    uint64_t voltage_codes = 0;      // Sum of voltage ADC codes for averaging
    unsigned long latest_time = 0;   // Timestamp of most recent reading

    Reading reading(false, Status(0), 0, 0);  // Memory location to load readings from queue

    // Process all available readings from the ADC interrupt queue
    while (xQueueReceive(reading_queue, &reading, 1) == pdTRUE) {

      ValueType value_type;

      // Determine the type of reading based on ADC channel
      if (reading.getStatus().currentConversionChannel() == Channel::CH1) {
        value_type = ValueType::CURRENT;   // Channel 1 measures current
      }
      else if (reading.getStatus().currentConversionChannel() == Channel::CH0) {
        value_type = ValueType::VOLTAGE;   // Channel 0 measures voltage
      }
      else
      {
        ESP_LOGE("main", "Invalid conversion channel: %d", reading.getStatus().currentConversionChannel());
        abort(); // Abort if there are no matches - this should never happen
      }

      // Update reading statistics and validate the reading
      const bool valid = reading.isValid();
      loadObject.getState()->incrementReadingCount(valid, value_type);

      // Skip invalid readings but continue processing
      if (!valid)
      {
        continue;
      }

      // Accumulate current readings for averaging
      if (value_type == ValueType::CURRENT)
      {
        current_count++;
        current_codes += reading.getData();
        latest_time = reading.getTime();
      }

      // Accumulate voltage readings for averaging
      if (value_type == ValueType::VOLTAGE)
      {
        voltage_count++;
        voltage_codes += reading.getData();
        latest_time = reading.getTime();
      }
    }

    // Skip control loop iteration if no valid readings were received
    if (voltage_count == 0 && current_count == 0)
    {
      continue;
    }

    // Debug logging for ADC reading statistics (commented for performance)
    // ESP_LOGD(TAG, "Current count: %lu, Current Codes: %llu, Current Code: %lu, Voltage count: %lu, Voltage Codes: %llu, Voltage Code: %lu, Latest time: %lu", current_count, current_codes, static_cast<uint32_t>(current_codes/current_count), voltage_count, voltage_codes, static_cast<uint32_t>(voltage_codes/voltage_count), latest_time);

    // Update current measurement if we have valid current readings
    if (current_count > 0)
    {
      // Calculate average current code from accumulated readings
      auto current = new Current(static_cast<uint32_t>(current_codes/current_count));
      // ESP_LOGD(TAG, "Current Code: %lu, Current Value: %fA", current->getCode(), current->getValue());
      loadObject.getState()->setCurrent(current);
    }

    // Update voltage measurement if we have valid voltage readings
    if (voltage_count > 0)
    {
      // Calculate average voltage code from accumulated readings
      auto voltage = new Voltage(static_cast<uint32_t>(voltage_codes/voltage_count));
      // ESP_LOGD(TAG, "Voltage Code: %lu, Voltage Value: %fV", voltage->getCode(), voltage->getValue());
      loadObject.getState()->setVoltage(voltage);
    }

    // Update control algorithm with current setpoint and readings
    control_algorithm->setTarget(*loadObject.getState()->getSetpoint());
    control_algorithm->addReading(loadObject.getState()->getCurrent().get(), loadObject.getState()->getVoltage().get(), latest_time);

    // Calculate new gate control voltage using the control algorithm
    auto gate_control_voltage = control_algorithm->updateOutput(loadObject.getState()->getGateControlVoltage().get());
    loadObject.getState()->setGateControlVoltage(gate_control_voltage);

    // Apply the new gate control voltage to the DAC
    dac.write(loadObject.getState()->getGateControlVoltage()->getCode());
  }
}

// Task handles for FreeRTOS tasks
TaskHandle_t runLoadTaskHandle;           // Handle for the main control loop task
TaskHandle_t runCommunicationTaskHandle;  // Handle for the communication task

// Buffer for debug output data (status + mode + debug data + sync byte)
uint8_t debug_output_buffer[communication::StatusData::SIZE + communication::ModeData::SIZE + communication::DebugData::SIZE + 1];

/**
 * Communication task that handles serial commands and debug output
 * This task runs continuously on core 1 and handles:
 * - Processing serial commands for load control
 * - Sending debug data via serial output
 * - Updating PID coefficients
 * 
 * @param parameter Unused task parameter
 */
[[noreturn]] void runCommunication( void* ) {
  
  while (true)
  {
    // Alternative CS control using direct GPIO register access (commented)
    // GPIO.out_w1ts = (uint32_t)1 << SENSE_CS_GPIO;
    // GPIO.out_w1tc = (uint32_t)1 << SENSE_CS_GPIO;

    // Process incoming serial commands
    while (Serial.available() > 0)
    {
      const uint8_t command = Serial.read();

      // Command 0x00: Set load mode and setpoint
      if (command == 0x00)
      {
        const auto setpoint_data = new uint8_t[5];  // 1 byte mode + 4 bytes setpoint

        // Read the complete command data
        Serial.read(setpoint_data, 5);
        std::reverse(setpoint_data + 1, setpoint_data + 5);  // Convert from big-endian to little-endian
        auto mode = static_cast<LoadMode>(setpoint_data[0]);

        ESP_LOGD(TAG, "setpoint_data: %d, %d, %d, %d, %d", setpoint_data[0], setpoint_data[1], setpoint_data[2], setpoint_data[3], setpoint_data[4]);
        ESP_LOGD(TAG, "Mode: %d, Setpoint: %d", mode, loadObject.getState()->getSetpoint()->getCode());

        // Extract 32-bit setpoint code from the data
        const uint32_t setpoint_code = *reinterpret_cast<uint32_t*>(setpoint_data + 1);

        Value * setpoint = nullptr;

        // Create appropriate Value object based on load mode
        switch (mode)
        {
        case LoadMode::OFF:
          setpoint = new Current(0u);  // Turn off load (0 current)
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

        // Update load state and control algorithm with new setpoint
        loadObject.getState()->setMode(mode);
        loadObject.getState()->setSetpoint(setpoint);
        control_algorithm->setTarget(*setpoint);
      }
    }

    // PID control coefficients (can be adjusted for different load modes)
    // kp for CC: 0.19, kp for CV: 0.01
    float kp = 0.2f;  // Proportional gain
    // float kp = static_cast<float>(analogRead(A0)) / 4096.0f * 1.0f;  // Potentiometer control
    float ki = 0.0f;  // Integral gain
    // float ki = static_cast<float>(analogRead(A1)) / 4096.0f * 0.75f;  // Potentiometer control
    float kd = 0.0f;  // Derivative gain
    // float kd = static_cast<float>(analogRead(A2)) / 4096.0f * 0.75f;  // Potentiometer control

    // Update PID coefficients in the control algorithm
    ((PidControlAlgorithm*)control_algorithm)->setCoefficients(kp, ki, kd);

    // Debug logging of system status and performance metrics
    ESP_LOGD(TAG, "Reading count: %d, Invalid count: %d, Invalid current count: %d, Invalid voltage count: %d, Setpoint: %f, Kp: %f, Ki: %f, Gate: %u %fV, Voltage: %u %fV, Current: %u %fA\n",
      loadObject.getState()->getReadingCount(),
      loadObject.getState()->getInvalidCurrentReadingCount() + loadObject.getState()->getInvalidVoltageReadingCount(),
      loadObject.getState()->getInvalidCurrentReadingCount(),
      loadObject.getState()->getInvalidVoltageReadingCount(),
      loadObject.getState()->getSetpoint()->getValue(),
      kp,
      ki,
      loadObject.getState()->getGateControlVoltage()->getCode(),
      loadObject.getState()->getGateControlVoltage()->getValue(),
      loadObject.getState()->getVoltage()->getCode(),
      loadObject.getState()->getVoltage()->getValue(),
      loadObject.getState()->getCurrent()->getCode(),
      loadObject.getState()->getCurrent()->getValue()
    );

    // Prepare debug data for transmission
    communication::StatusData status_data = communication::ControlLoopRegisters::STATUS.getData(&loadObject);
    communication::ModeData mode_data = communication::ControlLoopRegisters::MODE.getData(&loadObject);
    communication::DebugData debug_data = communication::ControlLoopRegisters::DEBUG.getData(&loadObject);

    // Build debug output packet with sync byte
    debug_output_buffer[0] = 0xff;  // Sync byte for packet identification
    uint8_t * buffer_ptr = debug_output_buffer;
    buffer_ptr++;

    // Serialize data structures into network byte order
    status_data.toNetwork(communication::Buffer(buffer_ptr, communication::StatusData::SIZE));
    buffer_ptr += communication::StatusData::SIZE;
    mode_data.toNetwork(communication::Buffer(buffer_ptr, communication::ModeData::SIZE));
    buffer_ptr += communication::ModeData::SIZE;
    debug_data.toNetwork(communication::Buffer(buffer_ptr, communication::DebugData::SIZE));

    // Send debug data via serial port
    Serial.write(debug_output_buffer, sizeof(debug_output_buffer));

    // Clear reading statistics for next cycle
    loadObject.getState()->clearCounts();
    
    // Wait 500ms before next iteration
    vTaskDelay(500);
  }
}

/**
 * I2C receive callback function
 * Called when data is received via I2C from an external controller
 * 
 * @param num_bytes Number of bytes received
 */
void onWireReceive(int num_bytes) {
  control_loop.onReceive(num_bytes);
}

/**
 * I2C request callback function
 * Called when an external controller requests data via I2C
 */
void onWireRequest() {
  control_loop.onRequest();
}

/**
 * Arduino setup function - initializes the electronic load control system
 * This function performs all necessary initialization including:
 * - Serial communication setup
 * - I2C communication setup
 * - ADC configuration for current/voltage sensing
 * - DAC configuration for gate control
 * - FreeRTOS task creation
 * - Interrupt setup for ADC data ready
 */
void setup() {
  esp_reset_reason_t reason = esp_reset_reason();

  // esp_task_wdt_add(NULL);  // Watchdog timer (commented out)

  // Configure logging levels for debugging
  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("Value", ESP_LOG_DEBUG);

  // Initialize serial communication for debugging and control
  Serial.begin(115200);
  
  // Initialize I2C communication for external control
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onWireReceive);  // Set up I2C receive callback
  Wire.onRequest(onWireRequest);  // Set up I2C request callback
  
  delay(5000);  // Allow time for serial connection to establish so serial monitor can see logs
  
  // Debug logging setup
  Serial.printf("LOG_LOCAL_LEVEL %d\n", LOG_LOCAL_LEVEL);
  Serial.setDebugOutput(true);
  ESP_LOGE(TAG, "Reset reason: %d", reason);
  ESP_LOGE(TAG, "ESP_LOGE, level 1 = ARDUHAL_LOG_LEVEL_ERROR   = ESP_LOG_ERROR");
  ESP_LOGW(TAG, "ESP_LOGW, level 2 = ARDUHAL_LOG_LEVEL_WARN    = ESP_LOG_WARN");    
  ESP_LOGI(TAG, "ESP_LOGI, level 3 = ARDUHAL_LOG_LEVEL_INFO    = ESP_LOG_INFO");
  ESP_LOGD(TAG, "ESP_LOGD, level 4 = ARDUHAL_LOG_LEVEL_DEBUG   = ESP_LOG_DEBUG");
  ESP_LOGV(TAG, "ESP_LOGV, level 5 = ARDUHAL_LOG_LEVEL_VERBOSE = ESP_LOG_VERBOSE");
  
  // Initialize DAC control pins
  digitalWrite(GATE_DRIVE_LDAC, HIGH);  // Set LDAC high initially
  digitalWrite(GATE_DRIVE_CS, LOW);     // Set CS low for DAC

  Serial.println("Starting SPI...");

  // Initialize DAC for gate control voltage output
  dac.initialize();

  Serial.println("Starting ADC...");

  // Configure ADC sample rate (5000 samples per second)
  FilterSampleRate sampleRate = FilterSampleRate::SPS_5000;

  // Create FreeRTOS queue for ADC readings
  reading_queue = xQueueCreate(reading_queue_len, sizeof(Reading));
  
  // Initialize and configure ADC
  adc.initialize();
  Serial.println("Starting ADC...");
  adc.reset();  // Reset ADC to known state
  Serial.println("Starting ADC...");
  
  // Configure ADC interface: I/O strength enabled, continuous read disabled, 
  // status append enabled, CRC error checking enabled, DOUT/RDY pin enabled
  // Note: continuous read will be enabled later after channel configuration
  adc.configureInterface(true, false, true, ChecksumMode::CRC, true);
  Serial.println("Starting ADC...");
  
  // Internal calibration (commented out for now)
  // adc.internalZeroScaleCalibration();
  // Serial.println("Starting ADC...");
  
  // Configure ADC mode: continuous conversion, single cycle settling disabled, no settle delay
  adc.configureMode(Mode::CONTINUOUS, false, SettleDelay::DELAY_0_US);
  Serial.println("Starting ADC...");
  
  // Configure ADC setup registers for different channels
  adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, sampleRate);
  Serial.println("Starting ADC...");
  adc.configureSetup(Setup::SETUP1, Filter::Sinc5Sinc1, sampleRate);
  Serial.println("Starting ADC...");
  adc.configureSetup(Setup::SETUP2, Filter::Sinc5Sinc1, sampleRate);
  Serial.println("Starting ADC...");
  
  // Configure ADC channels for voltage and current sensing
  adc.configureChannel(Channel::CH0, Setup::SETUP0, ChannelInput::AIN0, ChannelInput::AIN1);  // Voltage sensing
  Serial.println("Starting ADC...");
  adc.configureChannel(Channel::CH1, Setup::SETUP1, ChannelInput::AIN3, ChannelInput::AIN2);  // Current sensing
  Serial.println("Starting ADC...");
  
  // Disable unused channels
  // adc.configureChannel(Channel::CH2, Setup::SETUP2, ChannelInput::AIN0, ChannelInput::AIN4);
  adc.disableChannel(Channel::CH2);
  Serial.println("Starting ADC...");
  adc.disableChannel(Channel::CH3);
  Serial.println("Starting ADC...");
  
  // Reconfigure ADC interface: I/O strength enabled, continuous read enabled,
  // status append enabled, CRC error checking enabled, DOUT/RDY pin enabled
  // Key difference from earlier call: continuous read is now enabled for data streaming
  adc.configureInterface(true, true, true, ChecksumMode::CRC, true);
  Serial.println("Starting ADC...");

  // Create FreeRTOS tasks for control loop and communication
  xTaskCreatePinnedToCore(
    runLoad,              // Task function
    "runLoad",            // Task name
    8192,                 // Stack size
    nullptr,              // Task parameter
    1,                    // Priority
    &runLoadTaskHandle,   // Task handle
    0                     // Core 0
  );

  xTaskCreatePinnedToCore(
    runCommunication,              // Task function
    "runCommunication",            // Task name
    8192,                          // Stack size
    nullptr,                       // Task parameter
    1,                             // Priority
    &runCommunicationTaskHandle,   // Task handle
    1                              // Core 1
  );
  
  // Attach interrupt for ADC data ready signal
  attachInterrupt(digitalPinToInterrupt(SENSE_CIPO), readADC, FALLING);
  
  // Start ADC continuous reading
  adc.startReading();
}

/**
 * Arduino loop function - not used in this implementation
 * All functionality is handled by FreeRTOS tasks
 */
void loop() {
  // Empty - all functionality is handled by FreeRTOS tasks
}




