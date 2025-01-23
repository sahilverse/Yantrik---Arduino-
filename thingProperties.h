#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

const char DEVICE_LOGIN_NAME[] = "1c1135bd-a53b-4164-8b35-e868f2b17671";

const char SSID[] = SECRET_SSID;             // Network SSID (name)
const char PASS[] = SECRET_WIFI_PASS;        // Network password
const char DEVICE_KEY[] = SECRET_DEVICE_KEY; // Secret device password

void onLedChange();

CloudLight led;
CloudTemperatureSensor temperature;

void initProperties()
{

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(led, READWRITE, ON_CHANGE, onLedChange);
  ArduinoCloud.addProperty(temperature, READ, ON_CHANGE, NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
