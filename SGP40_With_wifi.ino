#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SGP40.h>

// ============================================================
// WiFi / MQTT CONFIGURATION
// ============================================================
const char* ssid = "WIN-B911Q5MBH7G 5381";           // Wi-Fi name
const char* password = "-46E75s5";                    // Wi-Fi password
const char* mqtt_server = "192.168.137.102";           // Raspberry Pi broker IP

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_SGP40 sgp;

// ============================================================
// LIVE AMBIENT CONDITIONS (updated from MQTT, used for SGP40 compensation)
// ============================================================
// Sensible defaults in case a measurement happens before the first MQTT
// message arrives -- 0.0 would badly skew the humidity compensation.
float temperatureC = 25.0;
float humidityRH   = 50.0;

// Track whether we've received at least one real reading of each, just so
// the Serial output makes it obvious when it's still on defaults.
bool haveTemperature = false;
bool haveHumidity    = false;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Wi-Fi connected successfully.");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
}

// This function runs every time a new MQTT message is received
void callback(char* topic, byte* message, unsigned int length) {
  String payload = "";
  for (unsigned int i = 0; i < length; i++) {
    payload += (char)message[i];
  }

  String topicStr = String(topic);

  if (topicStr == "home/sensors/temperature") {
    temperatureC = payload.toFloat();
    haveTemperature = true;
    Serial.print("[Temperature] ");
    Serial.print(payload);
    Serial.println(" C");
  }
  else if (topicStr == "home/sensors/humidity") {
    humidityRH = payload.toFloat();
    haveHumidity = true;
    Serial.print("[Humidity] ");
    Serial.print(payload);
    Serial.println(" %RH");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to Raspberry Pi...");

    // Unique client ID -- keep this different from any other ESP32 sketch
    // subscribing to the same broker, or the broker will kick whichever
    // client connected first.
    if (client.connect("ESP32_SGP40")) {
      Serial.println("connected!");
      client.subscribe("home/sensors/temperature");
      client.subscribe("home/sensors/humidity");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // ESP32 I2C pins
  Wire.begin(21, 22);

  Serial.println();
  Serial.println("=== SGP40 Humidity-Compensated Test ===");

  if (!sgp.begin(&Wire)) {
    Serial.println("SGP40 not found!");
    Serial.println("Check VCC, GND, SDA and SCL.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("SGP40 detected.");
  Serial.println("Waiting for first MQTT reading before using live temp/humidity...");
  Serial.println();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Non-blocking timer instead of delay(1000): a blocking delay here would
  // stall client.loop(), which handles incoming MQTT messages and keep-alive
  // pings -- that would make the connection laggy or drop entirely.
  static unsigned long lastMeasurement = 0;
  unsigned long now = millis();

  // Sensirion recommends approximately 1 measurement/second
  if (now - lastMeasurement >= 1000) {
    lastMeasurement = now;

    // ----------------------------------------------------------
    // Humidity-compensated raw SGP40 measurement
    // ----------------------------------------------------------
    uint16_t rawVOC = sgp.measureRaw(temperatureC, humidityRH);

    Serial.print("Temperature: ");
    Serial.print(temperatureC, 1);
    Serial.print(" \xC2\xB0C");
    Serial.print(haveTemperature ? " (live)" : " (default)");
    Serial.print(" | Humidity: ");
    Serial.print(humidityRH, 1);
    Serial.print(" %RH");
    Serial.print(haveHumidity ? " (live)" : " (default)");
    Serial.print(" | Raw compensated VOC: ");
    Serial.println(rawVOC);

    // ----------------------------------------------------------
    // VOC Index
    // ----------------------------------------------------------
    int32_t vocIndex = sgp.measureVocIndex(temperatureC, humidityRH);

    Serial.print("VOC Index: ");
    Serial.println(vocIndex);

    Serial.println("-----------------------------");
  }
}
