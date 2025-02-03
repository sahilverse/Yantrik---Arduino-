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

// MQTT Topics
const char *lightState = "light/state";

WiFiClientSecure espClient;
PubSubClient *client;

// FreeRTOS Task Handles
TaskHandle_t mqttTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t alarmTaskHandle;
TaskHandle_t motionTaskHandle;

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

    // MQTT Setup
    client = new PubSubClient(espClient);
    client->setServer(mqtt_server, 8883);
    client->setCallback(mqttCallback);
    Serial.println("System Initialized");

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, &mqttTaskHandle, 0);       // Core 0
    xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 1, &sensorTaskHandle, 1); // Core 1
    xTaskCreatePinnedToCore(alarmTask, "Alarm Task", 2048, NULL, 1, &alarmTaskHandle, 1);    // Core 1
    xTaskCreatePinnedToCore(motionTask, "Motion Task", 2048, NULL, 1, &motionTaskHandle, 1); // Core 1
}

void loop()
{
    ArduinoCloud.update(); // Handle IoT Cloud updates
    delay(100);
}

// MQTT Task
void mqttTask(void *pvParameters)
{
    while (true)
    {
        if (!client->connected())
        {
            reconnectMQTT();
        }
        client->loop();                      // Keep MQTT communication alive
        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield some CPU time
    }
}

// Sensor Task
void sensorTask(void *pvParameters)
{
    unsigned long previousDhtMillis = 0;
    const unsigned long DHT_INTERVAL = 5000;

    while (true)
    {
        unsigned long currentMillis = millis();

        // Gas Sensor Check
        static bool lastGasAlarmState = false;
        int gasLevel = analogRead(MQ2_PIN);
        gasAlarmActive = (gasLevel > GAS_THRESHOLD);
        bool gasStateChanged = (gasAlarmActive != lastGasAlarmState);

        // Publish gas state to MQTT
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

        // DHT Sensor Check
        if (currentMillis - previousDhtMillis >= DHT_INTERVAL)
        {
            previousDhtMillis = currentMillis;
            temperature = dht.readTemperature();

            if (!isnan(temperature))
            {
                char tempStr[8];
                dtostrf(temperature, 1, 2, tempStr);
                client->publish("home/temperature", tempStr);
                Serial.println("Temperature data sent.");
            }
            else
            {
                Serial.println("Failed to read temperature!");
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Alarm Task
void alarmTask(void *pvParameters)
{
    while (true)
    {
        if (gasAlarmActive)
        {
            buzzerState = !buzzerState;
            digitalWrite(BUZZER_PIN, buzzerState);
            vTaskDelay(200 / portTICK_PERIOD_MS); // Buzzer blink interval
        }
        else
        {
            digitalWrite(BUZZER_PIN, LOW);
            vTaskDelay(100 / portTICK_PERIOD_MS); // Check again after 100ms
        }
    }
}

// Motion Detection Task
void motionTask(void *pvParameters)
{
    while (true)
    {
        if (digitalRead(PIR_PIN) == HIGH)
        {
            digitalWrite(RELAY_PIN2, LOW); // Motion detected
        }
        else
        {
            digitalWrite(RELAY_PIN2, HIGH); // No motion
        }
        vTaskDelay(MOTION_CHECK_INTERVAL / portTICK_PERIOD_MS);
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
    while (!client->connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client->connect(clientId, mqtt_username, mqtt_password))
        {
            Serial.println("Connected to MQTT Broker");
            client->subscribe(lightState);
        }
        else
        {
            Serial.println("Failed. Retrying...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}



// MQTT Callback
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
