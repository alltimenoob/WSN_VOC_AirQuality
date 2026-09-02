/*
  03_espnow_receiver.ino

  Board role:
    /dev/ttyUSB0 receives simulated sensor readings from /dev/ttyUSB1.

  Expected sender:
    Sender STA MAC:   F4:65:0B:54:69:4C

  Receiver:
    Receiver STA MAC: F4:65:0B:55:08:2C

  Both:
    Classic ESP32-D0WD-V3
    Arduino-ESP32 core 3.3.11

  What this teaches:
    - ESP-NOW receives packets directly from another ESP32.
    - Both radios must use the same Wi-Fi channel.
    - The packet contains a version, sequence number and sensor values.
    - CRC-16 detects accidental packet corruption.
    - The receiver can detect missing/out-of-order packets.
    - The sender's delivery callback is not the same as an application-level reply.

  Serial Monitor: 115200 baud
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <stddef.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

constexpr uint8_t ESPNOW_CHANNEL = 9;

constexpr uint8_t EXPECTED_SENDER_MAC[ESP_NOW_ETH_ALEN] = {
  0xF4, 0x65, 0x0B, 0x59, 0x16, 0x94
};

constexpr uint8_t RECEIVER_LOCAL_MAC[ESP_NOW_ETH_ALEN] = {
  0xF4, 0x65, 0x0B, 0x56, 0x1E, 0x88
};

constexpr uint8_t PACKET_MAGIC[4] = {'E', 'S', 'P', '1'};
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t PACKET_TYPE_TELEMETRY = 1;

// -----------------------------------------------------------------------------
// Packet format - MUST match the sender
// -----------------------------------------------------------------------------

struct __attribute__((packed)) SensorPacket {
  uint8_t magic[4];
  uint8_t version;
  uint8_t type;
  uint16_t packetBytes;
  uint32_t sessionId;
  uint32_t sequence;
  uint32_t uptimeMs;
  int16_t temperatureCentiC;
  uint16_t humidityCentiPct;
  uint16_t lightLux;
  uint16_t batteryMillivolts;
  uint16_t crc16;
};

static_assert(sizeof(SensorPacket) == 30,
              "Unexpected SensorPacket size");

static_assert(
  offsetof(SensorPacket, crc16) == sizeof(SensorPacket) - sizeof(uint16_t),
  "crc16 must be the final field"
);

// -----------------------------------------------------------------------------
// Statistics
// -----------------------------------------------------------------------------

uint32_t receivedCount = 0;
uint32_t validCount = 0;
uint32_t invalidCount = 0;
uint32_t crcErrorCount = 0;
uint32_t sequenceGapCount = 0;

bool havePreviousPacket = false;
uint32_t previousSessionId = 0;
uint32_t previousSequence = 0;

// -----------------------------------------------------------------------------
// Logging helpers
// -----------------------------------------------------------------------------

void printLogPrefix(const char *level, const char *area) {
  Serial.printf(
    "[%10lu ms] [%-5s] [%-7s] ",
    static_cast<unsigned long>(millis()),
    level,
    area
  );
}

void printMac(const uint8_t mac[ESP_NOW_ETH_ALEN]) {
  Serial.printf(
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0],
    mac[1],
    mac[2],
    mac[3],
    mac[4],
    mac[5]
  );
}

// -----------------------------------------------------------------------------
// CRC-16/CCITT-FALSE
// Must match sender exactly
// -----------------------------------------------------------------------------

uint16_t crc16CcittFalse(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

// -----------------------------------------------------------------------------
// Packet validation
// -----------------------------------------------------------------------------

bool validatePacket(const SensorPacket &packet) {

  // Check packet magic
  if (memcmp(packet.magic, PACKET_MAGIC, sizeof(PACKET_MAGIC)) != 0) {
    printLogPrefix("ERROR", "PACKET");
    Serial.println("Invalid magic bytes.");
    return false;
  }

  // Check protocol version
  if (packet.version != PROTOCOL_VERSION) {
    printLogPrefix("ERROR", "PACKET");
    Serial.printf(
      "Unsupported protocol version: %u\n",
      static_cast<unsigned>(packet.version)
    );
    return false;
  }

  // Check packet type
  if (packet.type != PACKET_TYPE_TELEMETRY) {
    printLogPrefix("ERROR", "PACKET");
    Serial.printf(
      "Unexpected packet type: %u\n",
      static_cast<unsigned>(packet.type)
    );
    return false;
  }

  // Check packet length
  if (packet.packetBytes != sizeof(SensorPacket)) {
    printLogPrefix("ERROR", "PACKET");
    Serial.printf(
      "Unexpected packet size field: %u\n",
      static_cast<unsigned>(packet.packetBytes)
    );
    return false;
  }

  // Calculate CRC over everything except the received CRC field
  const uint16_t calculatedCrc = crc16CcittFalse(
    reinterpret_cast<const uint8_t *>(&packet),
    offsetof(SensorPacket, crc16)
  );

  if (calculatedCrc != packet.crc16) {
    ++crcErrorCount;

    printLogPrefix("ERROR", "CRC");
    Serial.printf(
      "CRC mismatch: received=0x%04X calculated=0x%04X\n",
      static_cast<unsigned>(packet.crc16),
      static_cast<unsigned>(calculatedCrc)
    );

    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Print decoded sensor packet
// -----------------------------------------------------------------------------

void printSensorPacket(const SensorPacket &packet) {

  printLogPrefix("DATA", "SENSOR");

  Serial.printf(
    "session=0x%08lX seq=%lu uptime=%lu ms "
    "temp=%.2f C humidity=%.2f %% "
    "light=%u lx battery=%u mV\n",

    static_cast<unsigned long>(packet.sessionId),

    static_cast<unsigned long>(packet.sequence),

    static_cast<unsigned long>(packet.uptimeMs),

    packet.temperatureCentiC / 100.0f,

    packet.humidityCentiPct / 100.0f,

    static_cast<unsigned>(packet.lightLux),

    static_cast<unsigned>(packet.batteryMillivolts)
  );
}

// -----------------------------------------------------------------------------
// ESP-NOW receive callback
// Arduino-ESP32 core 3.3.x / ESP-IDF 5.x
// -----------------------------------------------------------------------------

void onDataReceive(
  const esp_now_recv_info_t *recvInfo,
  const uint8_t *data,
  int dataLength
) {
  ++receivedCount;

  if (recvInfo == nullptr || recvInfo->src_addr == nullptr) {
    printLogPrefix("ERROR", "RX");
    Serial.println("Receive callback had no source address.");
    ++invalidCount;
    return;
  }

  // Check that this packet came from the expected sender
  if (memcmp(
        recvInfo->src_addr,
        EXPECTED_SENDER_MAC,
        ESP_NOW_ETH_ALEN
      ) != 0) {

    printLogPrefix("WARN", "RX");

    Serial.print("Packet from unexpected sender: ");
    printMac(recvInfo->src_addr);
    Serial.println();

    ++invalidCount;
    return;
  }

  printLogPrefix("INFO", "RX");

  Serial.print("Packet received from ");
  printMac(recvInfo->src_addr);
  Serial.printf(" (%d bytes)\n", dataLength);

  // Check packet size
  if (dataLength != sizeof(SensorPacket)) {
    printLogPrefix("ERROR", "PACKET");

    Serial.printf(
      "Expected %u bytes but received %d bytes.\n",
      static_cast<unsigned>(sizeof(SensorPacket)),
      dataLength
    );

    ++invalidCount;
    return;
  }

  // Copy into our packet structure
  SensorPacket packet = {};
  memcpy(&packet, data, sizeof(packet));

  // Validate packet
  if (!validatePacket(packet)) {
    ++invalidCount;
    return;
  }

  ++validCount;

  // Detect a new sender session
  if (!havePreviousPacket ||
      packet.sessionId != previousSessionId) {

    printLogPrefix("INFO", "SESSION");

    if (havePreviousPacket) {
      Serial.printf(
        "New sender session detected: 0x%08lX -> 0x%08lX\n",
        static_cast<unsigned long>(previousSessionId),
        static_cast<unsigned long>(packet.sessionId)
      );
    } else {
      Serial.printf(
        "First sender session: 0x%08lX\n",
        static_cast<unsigned long>(packet.sessionId)
      );
    }

    havePreviousPacket = true;
    previousSessionId = packet.sessionId;
    previousSequence = packet.sequence;
  }
  else {
    // Sequence numbers should normally increase by exactly 1
    if (packet.sequence != previousSequence + 1U) {

      if (packet.sequence > previousSequence + 1U) {

        const uint32_t missing =
          packet.sequence - previousSequence - 1U;

        sequenceGapCount += missing;

        printLogPrefix("WARN", "SEQUENCE");

        Serial.printf(
          "Missing %lu packet(s): previous=%lu current=%lu\n",
          static_cast<unsigned long>(missing),
          static_cast<unsigned long>(previousSequence),
          static_cast<unsigned long>(packet.sequence)
        );
      }
      else {

        printLogPrefix("WARN", "SEQUENCE");

        Serial.printf(
          "Out-of-order/duplicate packet: previous=%lu current=%lu\n",
          static_cast<unsigned long>(previousSequence),
          static_cast<unsigned long>(packet.sequence)
        );
      }
    }

    previousSequence = packet.sequence;
  }

  // Display the sensor data
  printSensorPacket(packet);
}

// -----------------------------------------------------------------------------
// Print receiver statistics
// -----------------------------------------------------------------------------

void printSummary() {

  static uint32_t lastSummaryMs = 0;
  const uint32_t now = millis();

  if (now - lastSummaryMs < 15000) {
    return;
  }

  lastSummaryMs = now;

  printLogPrefix("INFO", "SUMMARY");

  Serial.printf(
    "received=%lu valid=%lu invalid=%lu "
    "crc_errors=%lu sequence_gaps=%lu\n",

    static_cast<unsigned long>(receivedCount),

    static_cast<unsigned long>(validCount),

    static_cast<unsigned long>(invalidCount),

    static_cast<unsigned long>(crcErrorCount),

    static_cast<unsigned long>(sequenceGapCount)
  );
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  delay(700);

  Serial.println();
  Serial.println("=== ESP-NOW lesson: simulated sensor receiver ===");

  // Put ESP32 into station mode
  if (!WiFi.mode(WIFI_STA)) {
    printLogPrefix("FATAL", "SETUP");
    Serial.println("Could not start Wi-Fi station mode.");

    while (true) {
      delay(1000);
    }
  }

  // Wait for station interface
  const uint32_t wifiStartMs = millis();

  while (!WiFi.STA.started() &&
         millis() - wifiStartMs < 3000) {
    delay(10);
  }

  if (!WiFi.STA.started()) {
    printLogPrefix("FATAL", "SETUP");
    Serial.println("Wi-Fi station interface did not start.");

    while (true) {
      delay(1000);
    }
  }

  // Set same channel as sender
  const int channelResult = WiFi.setChannel(ESPNOW_CHANNEL);

  if (channelResult != ESP_OK) {
    printLogPrefix("FATAL", "RADIO");
    Serial.println("Could not set ESP-NOW channel 9.");

    while (true) {
      delay(1000);
    }
  }

  // Read this board's station MAC
  uint8_t localMac[ESP_NOW_ETH_ALEN] = {};

  if (esp_read_mac(localMac, ESP_MAC_WIFI_STA) != ESP_OK) {
    printLogPrefix("FATAL", "RADIO");
    Serial.println("Could not read station MAC.");

    while (true) {
      delay(1000);
    }
  }

  printLogPrefix("INFO", "RADIO");

  Serial.print("receiver_sta_mac=");
  printMac(localMac);

  Serial.printf(
    " channel=%ld\n",
    static_cast<long>(WiFi.channel())
  );

  // Make sure this is the correct receiver board
  if (memcmp(
        localMac,
        RECEIVER_LOCAL_MAC,
        ESP_NOW_ETH_ALEN
      ) != 0) {

    printLogPrefix("FATAL", "BOARD");

    Serial.println(
      "Wrong board for receiver role. Expected "
      "F4:65:0B:55:08:2C."
    );

    while (true) {
      delay(1000);
    }
  }

  // Initialise ESP-NOW
  esp_err_t result = esp_now_init();

  if (result != ESP_OK) {

    printLogPrefix("FATAL", "ESPNOW");

    Serial.printf(
      "esp_now_init failed: %s (0x%04X)\n",
      esp_err_to_name(result),
      static_cast<unsigned>(result)
    );

    while (true) {
      delay(1000);
    }
  }

  // Register receive callback
  result = esp_now_register_recv_cb(onDataReceive);

  if (result != ESP_OK) {

    printLogPrefix("FATAL", "ESPNOW");

    Serial.printf(
      "Register receive callback failed: %s (0x%04X)\n",
      esp_err_to_name(result),
      static_cast<unsigned>(result)
    );

    while (true) {
      delay(1000);
    }
  }

  printLogPrefix("OK", "SETUP");

  Serial.printf(
    "Receiver ready. Listening on channel %u for %u-byte packets.\n",
    static_cast<unsigned>(ESPNOW_CHANNEL),
    static_cast<unsigned>(sizeof(SensorPacket))
  );

  Serial.print("Expected sender: ");
  printMac(EXPECTED_SENDER_MAC);
  Serial.println();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {

  // ESP-NOW reception happens through the callback.
  // The loop is mainly used for statistics and housekeeping.

  printSummary();

  delay(10);
}
