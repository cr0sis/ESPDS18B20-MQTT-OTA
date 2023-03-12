#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

// Data wire is plugged into D1 on the Wemos D1 Mini
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with the DS18B20 sensors
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to Dallas Temperature library
DallasTemperature sensors(&oneWire, 2);

// Variables to track timing for temperature readings
unsigned long previousTempMillis = 0; // Track timing for temperature readings
const unsigned long tempInterval = 3000; // Read temperature every 3 seconds

// TCP/IP server port number
const int serverPort = 8888;

// WiFi credentials
const char* ssid = "ssid";
const char* password = "password";

// MQTT credentials
#define MQTT_HOST "broker-ip"
#define MQTT_PORT 1883


// Create a web server object
WiFiServer server(serverPort);

// Client instance
WiFiClient client;

// Temperature variables
float temperature_1;
float temperature_2;

// Flag to indicate if the client is connected
bool clientConnected = false;

// Maximum number of retries to connect to MQTT broker
const int maxMqttConnectRetries = 5;

// Counter for number of MQTT connection attempts
int mqttConnectRetries = 0;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  // Return if maximum number of retries has been reached
  if (mqttConnectRetries >= maxMqttConnectRetries) {
    Serial.println("Failed to connect to MQTT broker after 5 attempts");
    return;
  }

  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  // Reset the MQTT connection retry counter on successful connection
  mqttConnectRetries = 0;

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT");

  // Attempt to reconnect to the MQTT broker
  if (WiFi.isConnected()) {
    mqttConnectRetries++;
    connectToMqtt();
  }
}

void setup() {
  // Start the serial communication
  Serial.begin(9600);

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start OTA update
  ArduinoOTA.begin();
  Serial.println("OTA update started");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Set the sensor resolution to 12 bits
  sensors.begin();
  sensors.setResolution(12);

  // Set up MQTT client
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToMqtt(); // Call connectToMqtt() only once
}

void loop() {
  // Check for OTA update
  ArduinoOTA.handle();

  // Check for a new client connection
  if (server.hasClient()) {
    if (!clientConnected) {
      client = server.available();
      Serial.println("New client connected");
      clientConnected = true;
    }
  }

  // Check for data from the client
  if (clientConnected && client.available()) {
    String request = client.readStringUntil('\r');
    Serial.println(request);
    if (request == "get_temperatures") {
      sensors.requestTemperatures();
      temperature_1 = sensors.getTempCByIndex(0);
      temperature_2 = sensors.getTempCByIndex(1);
      String response = String(temperature_1) + "," + String(temperature_2);
      client.println(response);
    } else {
      client.println("Invalid request");
    }
    clientConnected = false;
    client.stop();
    Serial.println("Client disconnected");
  }

  // Check for temperature reading
  unsigned long currentTempMillis = millis();
  if (currentTempMillis - previousTempMillis >= tempInterval) {
    sensors.requestTemperatures();
    previousTempMillis = currentTempMillis;
    temperature_1 = sensors.getTempCByIndex(0);
    temperature_2 = sensors.getTempCByIndex(1);

    // Publish temperature readings to MQTT broker
    mqttClient.publish("esp/ds18b20/temp1", 0, true, String(temperature_1).c_str());
    mqttClient.publish("esp/ds18b20/temp2", 0, true, String(temperature_2).c_str());
  }
}
