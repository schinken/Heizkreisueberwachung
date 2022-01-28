#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "settings.h"

OneWire oneWire(D2);
DallasTemperature sensors(&oneWire);

WiFiClient wifiClient;
PubSubClient mqttClient;

char buffer[64];
char topic[64], value[12];

t_TemperatureSensor Sensors[] = {
  { {0x28, 0xFF, 0x28, 0xCB, 0x43, 0x16, 0x04, 0xD1}, "flow",          98.500, -0.225, 0.000, 0.000 },
  { {0x28, 0xFF, 0xE4, 0xCE, 0x43, 0x16, 0x04, 0x53}, "kitchen",       98.500, -0.125, 0.000, 0.000 },
  { {0x28, 0xFF, 0x02, 0x37, 0x44, 0x16, 0x03, 0x0E}, "dining",        98.675, -0.338, 0.000, 0.000 },
  { {0x28, 0xFF, 0x62, 0xCD, 0x43, 0x16, 0x03, 0x0F}, "dining_living", 98.670, -0.250, 0.000, 0.000 },
  { {0x28, 0xFF, 0x92, 0x36, 0x44, 0x16, 0x03, 0x36}, "living",        98.550, -0.458, 0.000, 0.000 },
  { {0x28, 0xFF, 0x39, 0xDC, 0x43, 0x16, 0x03, 0x5D}, "corridor",      98.875, -0.275, 0.000, 0.000 },
  { {0x28, 0xFF, 0x7B, 0x3B, 0x44, 0x16, 0x03, 0x82}, "toilets",       98.838, -0.462, 0.000, 0.000 }
};


void setup(void) {
  Serial.begin(115200);
  
  WiFi.hostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  mqttClient.setClient(wifiClient);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  
  sensors.begin();

  for(const auto &sensor : Sensors) {
    sensors.setResolution(sensor.address, TEMPERATURE_PRECISION);
  }
}


void loop(void) { 
  mqttConnect();
  mqttClient.loop();
  ArduinoOTA.handle();
  
  sensors.requestTemperatures(); 

  for(auto &sensor : Sensors) {
    float rawTemperature = sensors.getTempC(sensor.address);
    if (rawTemperature == DEVICE_DISCONNECTED_C) {
      Serial.print("Skipping temperature sensor: ");
      Serial.println(sensor.name);
      continue;
    }

    float temperature = mapf(rawTemperature, sensor.calibLow, sensor.calibHigh, REFERENCE_LOW, REFERENCE_HIGH);
    if (sensor.temperatureAvg == 0.00) {
      // Prime average value for starting
      sensor.temperatureAvg = temperature;
    }
    
    float temperatureAvg = TEMPERATURE_EXP_SMOOTH_ALPHA * (float) temperature + (1.0 - TEMPERATURE_EXP_SMOOTH_ALPHA) * sensor.temperatureAvg;
    if (abs(sensor.temperatureAvg - temperatureAvg) > TEMPERATURE_EPSILON_K) {
      sensor.temperature = temperature;
      sensor.temperatureAvg = temperatureAvg;
      
      mqttPushlishSensor(sensor);
    }
    
    sprintf(buffer, "%-15s\t%.3f\t%.3f\n", sensor.name, temperature, rawTemperature);    
    Serial.print(buffer);
  }
  
  Serial.println("------------------------");
  delay(2500);
}


void mqttConnect() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(WIFI_HOSTNAME, MQTT_TOPIC_LAST_WILL, 1, true, "disconnected")) {
      mqttClient.publish(MQTT_TOPIC_LAST_WILL, "connected", true);      
    } else {
      delay(2000);
    }
  }
}

void mqttPushlishSensor(t_TemperatureSensor &sensor) {
  sprintf(topic, "%s/%s", MQTT_TOPIC_BASE, sensor.name);
  sprintf(value, "%.3f", sensor.temperature);
  
  mqttClient.publish(topic, value, true);  
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
