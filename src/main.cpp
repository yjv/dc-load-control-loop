#include <Arduino.h>
#include <SPI.h>
#include <adc.h>
#include <QuickPID.h>

static constexpr uint8_t SENSE_ERROR = D7;
static constexpr uint8_t GATE_DRIVE_LDAC = D8;
static constexpr uint8_t GATE_DRIVE_CS = D9;
static constexpr uint8_t SENSE_CS = D10;
static constexpr uint8_t COPI = D11;
static constexpr uint8_t CIPO = D12;
static constexpr uint8_t SCLK = D13;

uint32_t gateCode = 0;
float gate = 0.0f;

// SPIClass senseSPI(HSPI);
SPIClass gateSPI(FSPI);

ADC adc(HSPI, SCLK, CIPO, COPI, SENSE_CS);

uint32_t currentCode;
uint32_t voltageCode;

float current;
float voltage;
float power;
float resistance;
float setpoint = 0.5f;

Reading reading(Status(0), 0);
QuickPID pid(&current, &gate, &setpoint, 1.0f, 1.0f, 0.0f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnError, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

void IRAM_ATTR readADC(Reading reading)
{
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

  pid.Compute();

  gateCode = static_cast<uint32_t>(gate / 5.0 * 1048576.0f);

  gateSPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  gateSPI.transferBits(gateCode << 4, NULL, 24);
  gateSPI.endTransaction();
 
  digitalWrite(GATE_DRIVE_LDAC, LOW);
  digitalWrite(GATE_DRIVE_LDAC, HIGH);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(SENSE_ERROR, INPUT);
  pinMode(GATE_DRIVE_LDAC, OUTPUT);
  pinMode(GATE_DRIVE_CS, OUTPUT);
  pinMode(SENSE_CS, OUTPUT);
  pinMode(COPI, OUTPUT);
  pinMode(CIPO, INPUT);
  pinMode(SCLK, OUTPUT);

  digitalWrite(GATE_DRIVE_LDAC, HIGH);
  digitalWrite(GATE_DRIVE_CS, HIGH);

  delay(100);

  Serial.println("Starting SPI...");

  // gateSPI.begin(SCLK, CIPO, COPI, GATE_DRIVE_CS);
  // gateSPI.setHwCs(true);

  Serial.println("Starting ADC...");

  adc.initialize();
  adc.reset();
  adc.configureMode(Mode::CONTINUOUS, false, SettleDelay::DELAY_0_US);
  adc.configureSetup(Setup::SETUP0, Filter::Sinc5Sinc1, FilterSampleRate::SPS_15625);
  adc.configureSetup(Setup::SETUP1, Filter::Sinc5Sinc1, FilterSampleRate::SPS_15625);
  adc.configureChannel(Channel::CH0, Setup::SETUP0, ChannelInput::AIN1, ChannelInput::AIN0);
  adc.configureChannel(Channel::CH1, Setup::SETUP1, ChannelInput::AIN3, ChannelInput::AIN2);
  adc.disableChannel(Channel::CH2);
  adc.disableChannel(Channel::CH3);
  adc.configureInterface(true, true, CrcMode::CRC_DISABLED);
  adc.startReading(readADC);

  pid.SetSampleTimeUs(100);
  pid.SetOutputLimits(0, 5.0);
  pid.SetMode(QuickPID::Control::automatic);
}

uint8_t ready = 0;

void loop() {
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
  
  // Serial.printf("Reading: %07d, Status: 0x%02x, Current: %fA, Voltage: %fV, Power: %fW, Resistance: %fOhm, Gate: %fV\n", 
  //   reading.getData(), 
  //   reading.getStatus().currentConversionChannel().getNumber(),
  //   current,
  //   voltage,
  //   power,
  //   resistance,
  //   gate
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




