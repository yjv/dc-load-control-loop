#include <Arduino.h>
#include <SPI.h>
#include <adc.h>

static constexpr uint8_t SENSE_ERROR = D7;
static constexpr uint8_t GATE_DRIVE_LDAC = D8;
static constexpr uint8_t GATE_DRIVE_CS = D9;
static constexpr uint8_t SENSE_CS = D10;
static constexpr uint8_t COPI = D11;
static constexpr uint8_t CIPO = D12;
static constexpr uint8_t SCLK = D13;

uint32_t gate = 0;

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
  digitalWrite(SENSE_CS, HIGH);

  SPI.begin();
}

void loop() {
  // Serial.println("Sending SPI data...");

  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GATE_DRIVE_CS, LOW);

  SPI.transferBits(gate << 4, NULL, 24);

  digitalWrite(GATE_DRIVE_CS, HIGH);
  digitalWrite(GATE_DRIVE_LDAC, LOW);
  digitalWrite(GATE_DRIVE_LDAC, HIGH);

  SPI.endTransaction();
  // gate = 1048575;
  gate = (gate + 64) % 1048576;
  // gate = gate == 1048575 ? 0 : 1048575;

  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SENSE_CS, LOW);

  uint8_t data[2];

  SPI.write(Registers::READ|Registers::ID);
  SPI.transferBytes(NULL, data, 2);

  digitalWrite(SENSE_CS, HIGH);

  SPI.endTransaction();

  Serial.printf("Gate: %d, Data: 0x%02x%02x\n", gate, data[0], data[1]);
}




