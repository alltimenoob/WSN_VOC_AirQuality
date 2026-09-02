#include <WiFi.h>
#include <PubSubClient.h>

// --- CONFIGURATION ---
const char* ssid = "WIN-B911Q5MBH7G 5381";           // Replace with your Wi-Fi name
const char* password = "-46E75s5";   // Replace with your Wi-Fi password

// The local IPv4 address of your Raspberry Pi (e.g., "192.168.1.50")
const char* mqtt_server = "192.168.137.102"; 

WiFiClient espClient;
PubSubClient client(espClient);

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
  for (int i = 0; i < length; i++) {
    payload += (char)message[i];
  }
  
  // Check which topic the message belongs to and print accordingly
  if (String(topic) == "home/sensors/temperature") {
    Serial.print("[Temperature] ");
    Serial.print(payload);
    Serial.println(" C");
  } 
  else if (String(topic) == "home/sensors/humidity") {
    Serial.print("[Humidity] ");
    Serial.print(payload);
    Serial.println(" %RH");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to Raspberry Pi...");
    
    // Attempt to connect with a unique client ID
    if (client.connect("ESP32_SensorDisplay")) {
      Serial.println("connected!");
      
      // Subscribe to both topics once connected
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
  
  setup_wifi();
  
  // Configure the MQTT server and port (1883 is default)
  client.setServer(mqtt_server, 1883);
  
  // Set the function to call when a message arrives
  client.setCallback(callback);
}

void loop() {
  // Ensure the MQTT connection stays alive
  if (!client.connected()) {
    reconnect();
  }
  
  // Process incoming messages and maintain connection
  client.loop();
}
