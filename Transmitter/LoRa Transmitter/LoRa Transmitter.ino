
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// =====================================
// LoRa hardware configuration
// =====================================

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  923E6

// GY-271 (QMC5883L)
#define QMC_ADDR 0x0D

// =====================================
// Experimental parameters
// =====================================

const uint32_t ACTIVE_MS = 60000UL;
const uint32_t IDLE_MS = 60000UL;
const uint32_t CYCLE_MS = ACTIVE_MS + IDLE_MS;

const uint16_t PACKETS_PER_CYCLE = 100;
const uint32_t INTERVAL_MS =
    ACTIVE_MS / PACKETS_PER_CYCLE;  // 600 ms

uint32_t experimentStart;
uint32_t currentCycle = UINT32_MAX;

int lastSentSeq = -1;
uint16_t successfulTx = 0;
bool idleAnnounced = false;

// =====================================
// GY-271 functions
// =====================================

bool writeRegister(byte reg, byte value) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMagnetometer(int16_t &x,
                      int16_t &y,
                      int16_t &z) {

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((uint8_t)QMC_ADDR,
                       (uint8_t)6) != 6)
    return false;

  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();

  x = (int16_t)(((uint16_t)xh << 8) | xl);
  y = (int16_t)(((uint16_t)yh << 8) | yl);
  z = (int16_t)(((uint16_t)zh << 8) | zl);

  return true;
}

// =====================================
// Transmit one numbered packet
// =====================================

void sendPacket(uint32_t cycle, uint16_t seq) {

  int16_t x = 0, y = 0, z = 0;

  bool sensorOK = readMagnetometer(x, y, z);

  // Format:
  // L,cycle,sequence,X,Y,Z,sensor_valid

  char payload[100];

  snprintf(payload, sizeof(payload),
           "L,%lu,%u,%d,%d,%d,%u",
           (unsigned long)cycle,
           (unsigned int)seq,
           (int)x, (int)y, (int)z,
           sensorOK ? 1U : 0U);

  if (!LoRa.beginPacket()) {
    Serial.println("Packet start failed");
    return;
  }

  LoRa.print(payload);

  if (LoRa.endPacket() == 1) {
    successfulTx++;

    Serial.print("TX: ");
    Serial.println(payload);
  } else {
    Serial.println("Packet transmission failed");
  }
}

// =====================================
// Setup
// =====================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting long-range LoRa test");

  // GY-271 setup
  Wire.begin(21, 22);
  Wire.setClock(100000);

  bool sensorReady =
      writeRegister(0x0B, 0x01) &&
      writeRegister(0x09, 0x15);

  if (!sensorReady) {
    Serial.println("GY-271 setup failed!");
    while (true) delay(1000);
  }

  // LoRa setup
  SPI.begin(18, 19, 23, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa initialization failed!");
    while (true) delay(1000);
  }

  // Modtronix inAir9 RFO output
  LoRa.setTxPower(5, PA_OUTPUT_RFO_PIN);

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

  Serial.println("Transmitter ready");

  // Begin the first burst after 3 seconds
  experimentStart = millis() + 3000UL;
}

// =====================================
// Main loop
// =====================================

void loop() {

  // Wait for initial experiment start
  if ((int32_t)(millis() - experimentStart) < 0)
    return;

  uint32_t elapsed = millis() - experimentStart;

  uint32_t cycle = elapsed / CYCLE_MS;
  uint32_t phase = elapsed % CYCLE_MS;

  // Detect a new cycle
  if (cycle != currentCycle) {

    currentCycle = cycle;
    lastSentSeq = -1;
    successfulTx = 0;
    idleAnnounced = false;

    Serial.println();
    Serial.print("===== CYCLE ");
    Serial.print(cycle);
    Serial.println(" START =====");
  }

  // First 60 seconds: transmission
  if (phase < ACTIVE_MS) {

    int targetSeq = phase / INTERVAL_MS;

    if (targetSeq < PACKETS_PER_CYCLE &&
        targetSeq > lastSentSeq) {

      // If a slot was skipped, it is not retransmitted
      lastSentSeq = targetSeq;

      sendPacket(cycle, targetSeq);
    }
  }

  // Next 60 seconds: idle
  else {

    if (!idleAnnounced) {

      idleAnnounced = true;

      // Put radio into standby during idle
      LoRa.idle();

      Serial.println();
      Serial.print("Cycle ");
      Serial.print(cycle);
      Serial.println(" transmission finished");

      Serial.print("Successful local TX: ");
      Serial.print(successfulTx);
      Serial.print("/");
      Serial.println(PACKETS_PER_CYCLE);

      Serial.println("60-second idle period");
      Serial.println("Move receiver now.");
    }
  }

  delay(2);
}
