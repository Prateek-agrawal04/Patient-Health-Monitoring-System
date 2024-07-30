#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define MQTT_CLIENT_NAME "mymqttclient"

// WiFi credentials
const char* ssid = "Prateek";
const char* password = "11225678";

// Ubidots details
const char* mqttBroker = "industrial.api.ubidots.com";
const int mqttPort = 1883;  // Use 8883 for SSL/TLS
const char* mqttToken = "BBUS-4IzMzZigF2aFiBqPMVvsTs6o3W0RSY";  // Replace with your Ubidots token
const char* deviceLabel = "esp32";  // Replace with your device label
const char* tempVarLabel = "dhtemp";
const char* humVarLabel = "humidity";
const char* ecgVarLabel = "ecg";
const char* mlxAmbientTempVarLabel = "ambient";
const char* mlxObjectTempVarLabel = "temperature";

// Initialize MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Initialize DHT sensor
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT11 or DHT22
DHT dht(DHTPIN, DHTTYPE);

// Initialize WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Pin for ECG sensor
const int ecgPin = 34;

void setup() {
  Serial.begin(115200);
  mlx.begin();
  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  client.setServer(mqttBroker, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttToken, "")) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read temperature and humidity from DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read temperature from MLX90614
  float ambientTemp = mlx.readAmbientTempC();
  float objectTemp = mlx.readObjectTempC();

  // Read ECG value
  int ecgValue = analogRead(ecgPin);

  // Prepare JSON payload
  char payload[300];
  snprintf(payload, sizeof(payload), "{\"%s\": %.2f, \"%s\": %.2f, \"%s\": %.2f, \"%s\": %.2f, \"%s\": %d}", 
           tempVarLabel, temperature, humVarLabel, humidity, 
           mlxAmbientTempVarLabel, ambientTemp, mlxObjectTempVarLabel, objectTemp, 
           ecgVarLabel, ecgValue);

  // Topic
  char topic[150];
  snprintf(topic, sizeof(topic), "/v1.6/devices/%s", deviceLabel);

  // Publish data
  client.publish(topic, payload);
  Serial.print("Published: ");
  Serial.println(payload);

  delay(5000);  // Publish every 5 seconds
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT...");
    if (client.connect("ESP32Client", mqttToken, "")) {
      Serial.println("Reconnected to MQTT");
    } else {
      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}