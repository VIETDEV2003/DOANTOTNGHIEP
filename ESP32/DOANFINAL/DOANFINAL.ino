#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <BH1750.h>
#include "RTClib.h"
#include <WiFiManager.h>

// ==== MQTT Configuration ====
const char* mqtt_server = "103.146.22.13";
const int mqtt_port = 1883;
const char* mqtt_user = "user1";
const char* mqtt_pass = "12345678";
const char* topic_control = "doan/contrung/control";
const char* topic_sensor = "doan/contrung/sensor";
const char* topic_schedule = "doan/contrung/schedule";

// ==== Motor Control ====
#define IN1 16
#define IN2 17
#define PWM_CHANNEL 0
int motorSpeed = -1;
int motorTime = -1;
bool motorRunning = false;
unsigned long motorStartTime = 0;

// ==== System Logic ====
bool logic_motor = true;
bool logic_uva = true;
bool logic_led = true;
bool logic_sensor = true;

// ==== Sensor Definitions ====
#define DHTPIN 14
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;
RTC_DS3231 rtc;

// ==== Relay Control ====
#define UVA 2
#define LED 4

// ==== Network Clients ====
WiFiClient espClient;
PubSubClient client(espClient);

// ==== Timing Variables ====
unsigned long lastsensorSend = 0;
const unsigned long SENSOR_INTERVAL = 5000;

// ==== Schedule System ====
#define MAX_SCHEDULES 10
struct Schedule {
  String time;
  String status;
};
Schedule schedules[MAX_SCHEDULES];
int scheduleCount = 0;
String lastExecuted = "";

void setup_wifi() {
  WiFiManager wifiManager;
  
  // Set timeout for configuration portal (60 seconds)
  wifiManager.setTimeout(60);
  
  // Customize AP name
  if (!wifiManager.autoConnect("ESP32-Config")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }

  Serial.println("\n✅ WiFi connected!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  
  // Visual connection confirmation
  digitalWrite(UVA, HIGH); delay(500);
  digitalWrite(UVA, LOW); delay(500);
  digitalWrite(UVA, HIGH); delay(500);
  digitalWrite(UVA, LOW); delay(500);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📩 Message arrived [");
  Serial.print(topic);
  Serial.println("]");

  // Print raw payload
  Serial.print("🔧 Raw payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("❌ Failed to parse JSON");
    return;
  }

  serializeJsonPretty(doc, Serial);
  Serial.println();

  // Process control commands
  if (String(topic) == topic_control) {
    // Motor control
    if (doc.containsKey("speed") && doc.containsKey("time")) {
      ledcWrite(PWM_CHANNEL, 0);
      motorRunning = false;

      motorSpeed = constrain(doc["speed"].as<int>(), 0, 255);
      motorTime = doc["time"].as<int>();
      Serial.printf("➡️ Updated motor: speed=%d, time=%d\n", motorSpeed, motorTime);

      if (motorSpeed == 0 && motorTime == 0) {
        Serial.println("🛑 Stopping motor immediately");
        ledcWrite(PWM_CHANNEL, 0);
        motorRunning = false;
      } 
      else if (motorSpeed > 0 && motorTime == -1 && logic_motor) {
        Serial.println("🚀 Running motor continuously");
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, motorSpeed);
        motorRunning = true;
      } 
      else if (motorSpeed > 0 && motorTime > 0 && logic_motor) {
        Serial.printf("🚀 Running motor: speed=%d, time=%dms\n", motorSpeed, motorTime);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, motorSpeed);
        motorStartTime = millis();
        motorRunning = true;
      }
    }

    // UVA Light control
    if (doc.containsKey("led1") && logic_uva) {
      String ledCmd = doc["led1"].as<String>();
      digitalWrite(UVA, ledCmd == "on" ? HIGH : LOW);
      Serial.println("💡 UVA: " + ledCmd);
    }

    // LED control
    if (doc.containsKey("led2") && logic_led) {
      String ledCmd = doc["led2"].as<String>();
      digitalWrite(LED, ledCmd == "on" ? HIGH : LOW);
      Serial.println("💡 LED: " + ledCmd);
    }

    // Schedule management
    if (doc.containsKey("action")) {
      String action = doc["action"];
      if (action == "add_schedule" && scheduleCount < MAX_SCHEDULES) {
        schedules[scheduleCount++] = {
          doc["data"]["time"].as<String>(),
          doc["data"]["status"].as<String>()
        };
        Serial.println("📌 Schedule added successfully");
        client.publish(topic_control, "{\"action\":\"get_schedule\"}");
      } 
      else if (action == "get_schedule") {
        StaticJsonDocument<512> resp;
        JsonArray data = resp.to<JsonArray>();
        for (int i = 0; i < scheduleCount; i++) {
          JsonObject obj = data.createNestedObject();
          obj["time"] = schedules[i].time;
          obj["status"] = schedules[i].status;
        }
        char buffer[512];
        serializeJson(resp, buffer);
        client.publish(topic_schedule, buffer);
      } 
      else if (action == "delete_schedule") {
        int index = doc["index"];
        if (index >= 0 && index < scheduleCount) {
          for (int i = index; i < scheduleCount - 1; i++) {
            schedules[i] = schedules[i + 1];
          }
          scheduleCount--;
          Serial.print("🗑️ Deleted schedule at index: ");
          Serial.println(index);
          client.publish(topic_control, "{\"action\":\"get_schedule\"}");
        }
      }
    }
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("✅ MQTT connected");
      client.subscribe(topic_control);
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds");
      delay(2000);
    }
  }
}

void sendSensorDataToMQTT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float lux = lightMeter.readLightLevel();

  if (isnan(temp) || isnan(hum) || lux < 0 || lux == 65535) {
    Serial.println("❌ Failed to read sensors");
    return;
  }

  StaticJsonDocument<256> doc;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["light"] = lux;
  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(topic_sensor, buffer);
  Serial.println("📤 Sent sensor data: " + String(buffer));
}

void checkSchedule() {
  DateTime now = rtc.now();
  char currentTime[6];
  sprintf(currentTime, "%02d:%02d", now.hour(), now.minute());

  for (int i = 0; i < scheduleCount; i++) {
    if (schedules[i].time == currentTime) {
      String key = schedules[i].time + schedules[i].status;
      if (lastExecuted != key) {
        if (schedules[i].status == "on") {
          logic_motor = true;
          logic_uva = true;
          logic_led = true;
          logic_sensor = true;
          Serial.println("✅ System activated");
        } else {
          logic_motor = false;
          logic_uva = false;
          logic_led = false;
          logic_sensor = false;
          Serial.println("🛑 System deactivated");
        }
        lastExecuted = key;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize pins for visual feedback
  pinMode(UVA, OUTPUT);
  digitalWrite(UVA, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Connect to WiFi using WiFiManager
  setup_wifi();

  // Initialize MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // Motor control setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL, 5000, 8);
  ledcAttachPin(IN1, PWM_CHANNEL);

  // Sensor initialization
  dht.begin();
  Wire.begin(21, 22);

  if (!lightMeter.begin()) {
    Serial.println("❌ BH1750 initialization failed");
  }

  if (!rtc.begin()) {
    Serial.println("❌ RTC initialization failed");
  }

  if (rtc.lostPower()) {
    Serial.println("⚠️ RTC lost power, needs time reset");
  }
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Check and execute schedules
  checkSchedule();

  // Automatic LED control based on light level
  float lux = lightMeter.readLightLevel();
  if (lux < 300 && logic_led) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }

  // Motor timeout control
  if (motorRunning && motorTime > 0 && millis() - motorStartTime >= motorTime) {
    Serial.println("✅ Motor timeout reached, stopping");
    ledcWrite(PWM_CHANNEL, 0);
    motorRunning = false;
    motorSpeed = -1;
    motorTime = -1;
  }

  // Regular sensor data transmission
  if (millis() - lastsensorSend > SENSOR_INTERVAL && logic_sensor) {
    sendSensorDataToMQTT();
    DateTime now = rtc.now();
    Serial.printf("🕒 Current time: %02d:%02d:%02d\n", 
                 now.hour(), now.minute(), now.second());
    lastsensorSend = millis();
  }

  // Debug output
  Serial.printf("Motor status: speed=%d, time=%d, running=%s\n",
               motorSpeed, motorTime, motorRunning ? "true" : "false");
}