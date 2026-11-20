
#include <SPI.h>
#include <LoRa.h>

// =====================================
// LoRa Hardware Configuration
// =====================================

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  923E6

// =====================================
// Experiment Configuration
// =====================================

const uint32_t ACTIVE_MS = 60000UL;
const uint32_t IDLE_MS = 60000UL;
const uint32_t CYCLE_MS = ACTIVE_MS + IDLE_MS;

const uint16_t PACKETS_PER_CYCLE = 100;
const uint32_t INTERVAL_MS = 600UL;

// =====================================
// Synchronization
// =====================================

bool synchronized = false;

uint32_t baseTimeMs = 0;
uint32_t currentCycle = 0;

bool summaryPrinted = false;

// =====================================
// Measurement Storage
// =====================================

bool seen[PACKETS_PER_CYCLE] = {false};

uint16_t received = 0;
uint16_t sensorErrors = 0;

double sumRSSI = 0;
double sumSNR = 0;

int minRSSI = 999;
int maxRSSI = -999;

float minSNR = 999;
float maxSNR = -999;

// =====================================
// Display Formatted Table Header
// =====================================

void printHeader() {

  Serial.println();

  Serial.printf(
    "%-6s %-12s %-10s %-8s %-8s %-8s\n",
    "SEQ",
    "RSSI (dBm)",
    "SNR (dB)",
    "X",
    "Y",
    "Z"
  );

  Serial.println(
    "-----------------------------------------------------------"
  );
}

// =====================================
// Reset Statistics
// =====================================

void resetStatistics() {

  for (int i = 0; i < PACKETS_PER_CYCLE; i++) {
    seen[i] = false;
  }

  received = 0;
  sensorErrors = 0;

  sumRSSI = 0;
  sumSNR = 0;

  minRSSI = 999;
  maxRSSI = -999;

  minSNR = 999;
  maxSNR = -999;

  summaryPrinted = false;
}

// =====================================
// Display Cycle Summary
// =====================================

void printSummary() {

  if (summaryPrinted) return;

  summaryPrinted = true;

  int lost = PACKETS_PER_CYCLE - received;

  float pdr =
      100.0f * received / PACKETS_PER_CYCLE;

  float plr =
      100.0f * lost / PACKETS_PER_CYCLE;

  Serial.println();
  Serial.println("================================");
  Serial.print("CYCLE ");
  Serial.print(currentCycle);
  Serial.println(" - RESULTS");
  Serial.println("================================");

  Serial.print("Expected packets : ");
  Serial.println(PACKETS_PER_CYCLE);

  Serial.print("Received packets : ");
  Serial.println(received);

  Serial.print("Lost packets     : ");
  Serial.println(lost);

  Serial.println();

  Serial.print("PDR              : ");
  Serial.print(pdr, 2);
  Serial.println(" %");

  Serial.print("PLR              : ");
  Serial.print(plr, 2);
  Serial.println(" %");

  Serial.println();

  if (received > 0) {

    Serial.print("Average RSSI     : ");
    Serial.print(sumRSSI / received, 2);
    Serial.println(" dBm");

    Serial.print("Minimum RSSI     : ");
    Serial.print(minRSSI);
    Serial.println(" dBm");

    Serial.print("Maximum RSSI     : ");
    Serial.print(maxRSSI);
    Serial.println(" dBm");

    Serial.println();

    Serial.print("Average SNR      : ");
    Serial.print(sumSNR / received, 2);
    Serial.println(" dB");

    Serial.print("Minimum SNR      : ");
    Serial.print(minSNR, 2);
    Serial.println(" dB");

    Serial.print("Maximum SNR      : ");
    Serial.print(maxSNR, 2);
    Serial.println(" dB");

  } else {

    Serial.println("RSSI             : N/A");
    Serial.println("SNR              : N/A");
  }

  Serial.println();

  Serial.print("Sensor errors    : ");
  Serial.println(sensorErrors);

  Serial.println("================================");
  Serial.println("MOVE TO NEXT LOCATION");
  Serial.println("TX IDLE - SUMMARY HELD");
  Serial.println("================================");
}

// =====================================
// Update Active/Idle Schedule
// =====================================

void updateSchedule() {

  if (!synchronized) return;

  uint32_t elapsed = millis() - baseTimeMs;

  uint32_t expectedCycle = elapsed / CYCLE_MS;
  uint32_t phase = elapsed % CYCLE_MS;

  // Detect beginning of new cycles
  while (currentCycle < expectedCycle) {

    if (!summaryPrinted) {
      printSummary();
    }

    currentCycle++;

    resetStatistics();

    Serial.println();
    Serial.println("================================");

    Serial.print("STARTING CYCLE ");
    Serial.println(currentCycle);

    Serial.println("================================");

    printHeader();
  }

  // End of the 60-second receiving window
  if (phase >= ACTIVE_MS && !summaryPrinted) {
    printSummary();
  }
}

// =====================================
// Setup
// =====================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting LoRa Receiver...");

  // Initialize SPI
  SPI.begin(18, 19, 23, LORA_SS);

  // Configure LoRa pins
  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );

  // Initialize LoRa module
  if (!LoRa.begin(LORA_FREQ)) {

    Serial.println("LoRa initialization failed!");

    while (true) {
      delay(1000);
    }
  }

  // LoRa settings
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

  Serial.println("LoRa Receiver Ready!");
  Serial.println("Waiting for synchronization...");
}

// =====================================
// Main Loop
// =====================================

void loop() {

  updateSchedule();

  // Check for an incoming packet
  int packetSize = LoRa.parsePacket();

  if (!packetSize) return;

  // =================================
  // Read Received Packet
  // =================================

  char payload[100];
  int index = 0;

  while (LoRa.available()) {

    char c = (char)LoRa.read();

    if (index < (int)sizeof(payload) - 1) {
      payload[index++] = c;
    }
  }

  payload[index] = '\0';

  // =================================
  // Extract Packet Information
  // =================================

  unsigned long cycle;
  unsigned int seq;

  int x, y, z, sensorValid;

  // Expected packet:
  // L,cycle,sequence,X,Y,Z,sensor_valid

  int fields = sscanf(
    payload,
    "L,%lu,%u,%d,%d,%d,%d",
    &cycle,
    &seq,
    &x,
    &y,
    &z,
    &sensorValid
  );

  if (fields != 6) return;

  if (seq >= PACKETS_PER_CYCLE) return;

  if (sensorValid != 0 && sensorValid != 1)
    return;

  // =================================
  // Establish Synchronization
  // =================================

  if (!synchronized) {

    synchronized = true;

    currentCycle = (uint32_t)cycle;

    uint32_t txOffset =
        (uint32_t)cycle * CYCLE_MS +
        (uint32_t)seq * INTERVAL_MS;

    baseTimeMs = millis() - txOffset;

    resetStatistics();

    Serial.println();
    Serial.println("Synchronization established!");

    Serial.print("Current Cycle: ");
    Serial.println(currentCycle);

    printHeader();
  }

  updateSchedule();

  uint32_t phase =
      (millis() - baseTimeMs) % CYCLE_MS;

  // =================================
  // Validate Packet Cycle
  // =================================

  if ((uint32_t)cycle != currentCycle ||
      phase >= ACTIVE_MS ||
      summaryPrinted) {

    return;
  }

  // Ignore duplicate packets
  if (seen[seq]) return;

  seen[seq] = true;

  received++;

  // =================================
  // Measure RSSI and SNR
  // =================================

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  sumRSSI += rssi;
  sumSNR += snr;

  if (rssi < minRSSI) minRSSI = rssi;
  if (rssi > maxRSSI) maxRSSI = rssi;

  if (snr < minSNR) minSNR = snr;
  if (snr > maxSNR) maxSNR = snr;

  if (!sensorValid) {
    sensorErrors++;
  }

  // =================================
  // Display Formatted Packet Data
  // =================================

  Serial.printf(
    "%-6u %-12d %-10.2f %-8d %-8d %-8d\n",
    seq,
    rssi,
    snr,
    x,
    y,
    z
  );
}
