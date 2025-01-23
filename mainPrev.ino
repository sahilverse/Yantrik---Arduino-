#include "arduino_secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "thingProperties.h"

// Pin Definitions
#define PIR_PIN 21
#define DHTPIN 5
#define DHTTYPE DHT11
#define MQ2_PIN 34
#define BUZZER_PIN 23
#define RELAY_PIN1 4
#define RELAY_PIN2 18

// Thresholds
#define GAS_THRESHOLD 500

// Timing Intervals
const unsigned long DHT_INTERVAL = 5000;
const unsigned long GAS_CHECK_INTERVAL = 100;
const unsigned long BUZZER_BLINK_INTERVAL = 200;
const unsigned long MOTION_CHECK_INTERVAL = 200;

// Sensor Objects
DHT dht(DHTPIN, DHTTYPE);

// Timing Variables
unsigned long previousDhtMillis = 0;
unsigned long previousGasMillis = 0;
unsigned long previousBuzzerMillis = 0;
unsigned long previousMotionMillis = 0;

// States
bool gasAlarmActive = false;
bool buzzerState = false;

// MQTT
const char *mqtt_server = "15bc3a94d4d3431aa3957b6dc67860f0.s1.eu.hivemq.cloud";
const char *clientId = "Yantrik_home";

// HiveMQ authentication
const char *mqtt_username = SECRET_MQTT_USERNAME;
const char *mqtt_password = SECRET_MQTT_PASS;

// MQTT TOPICS
const char *lightState = "light/state";

WiFiClientSecure espClient;
PubSubClient *client;

void setup()
{
  Serial.begin(115200);
  delay(1500);
  espClient.setInsecure();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  initProperties(); // Initialize IoT properties
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  dht.begin();

  // MQTT
  client = new PubSubClient(espClient);
  client->setServer(mqtt_server, 8883);
  client->setCallback(mqttCallback);
  Serial.println("System Initialized");
}

void loop()
{
  ArduinoCloud.update(); // Handle IoT Cloud updates

  unsigned long currentMillis = millis();

  // Handle MQTT communication and try reconnecting if needed
  if (!client->connected())
  {
    reconnectMQTT();
  }

  // Handle MQTT communication
  client->loop();

  // Gas And Alarm
  checkGasSensor(currentMillis);
  handleGasAlarm(currentMillis);

  // Temperature Sensor
  if (currentMillis - previousDhtMillis >= DHT_INTERVAL)
  {
    previousDhtMillis = currentMillis;
    sendDhtData();
  }

  // Motion Sensor
  if (currentMillis - previousMotionMillis >= MOTION_CHECK_INTERVAL)
  {
    checkMotionSensor();
    previousMotionMillis = currentMillis;
  }

  delay(100);
}

// Function to check gas levels
void checkGasSensor(unsigned long currentMillis)
{
  static bool lastGasAlarmState = false;
  if (currentMillis - previousGasMillis >= GAS_CHECK_INTERVAL)
  {
    previousGasMillis = currentMillis;

    int gasLevel = analogRead(MQ2_PIN);
    gasAlarmActive = (gasLevel > GAS_THRESHOLD);
    bool gasStateChanged = (gasAlarmActive != lastGasAlarmState);

    if (gasStateChanged)
    {
      const char gasTopic[] = "gas/detected";
      const char *gasMessage = gasAlarmActive ? "Yes" : "No";
      lastGasAlarmState = gasAlarmActive; // Update the last state
      if (client->connected())
      {
        client->publish(gasTopic, gasMessage, true);
      }
    }
  }
}

// Function to manage gas alarm
void handleGasAlarm(unsigned long currentMillis)
{
  if (gasAlarmActive)
  {
    if (currentMillis - previousBuzzerMillis >= BUZZER_BLINK_INTERVAL)
    {
      previousBuzzerMillis = currentMillis;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState);
    }
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Function to check for motion
void checkMotionSensor()
{
  if (digitalRead(PIR_PIN) == HIGH)
  {
    digitalWrite(RELAY_PIN2, LOW);
  }
  else
  {
    digitalWrite(RELAY_PIN2, HIGH);
  }
}

// Function to get Temperature Readings
void sendDhtData()
{
  temperature = dht.readTemperature();

  if (isnan(temperature))
  {
    Serial.println("Failed to read temperature!!!");
    return;
  }

  char tempStr[8];
  dtostrf(temperature, 1, 2, tempStr);

  // Publish the temperature data to the MQTT broker
  const char temperatureTopic[] = "home/temperature";
  if (client->publish(temperatureTopic, tempStr))
  {
    Serial.println("Temperature data sent successfully!");
  }
  else
  {
    Serial.println("Failed to send temperature data.");
  }
}

void onLedChange()
{
  if (led == 1)
  {
    client->publish(lightState, "ON", true);
    digitalWrite(RELAY_PIN1, LOW);
  }
  else
  {
    client->publish(lightState, "OFF", true);
    digitalWrite(RELAY_PIN1, HIGH);
  }
}

// Function to handle MQTT connection
void reconnectMQTT()
{
  if (!client->connected())
  {
    Serial.print("Attempting MQTT connection...");

    if (client->connect(clientId, mqtt_username, mqtt_password))
    {

      Serial.println("Connected to MQTT Broker");
      client->subscribe(lightState);
    }
    else
    {
      Serial.print("Failed to connect. Retry in 5 seconds... \n");
      delay(5000);
    }
  }
}

// Receiving from MQTT Topics
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String message = "";

  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  // Light Control
  if (message == "ON")
  {
    digitalWrite(RELAY_PIN1, LOW);
    led = 1;
  }
  else if (message == "OFF")
  {
    digitalWrite(RELAY_PIN1, HIGH);
    led = 0;
  }
}
