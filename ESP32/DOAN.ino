#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <BH1750.h>
#include "RTClib.h"

// ==== WI-FI ====
const char* ssid = "M Coffee"; 
const char* password = "1234567890";

// ==== MQTT ====
const char* mqtt_server = "103.146.22.13";
const int mqtt_port = 1883;
const char* mqtt_user = "user1";
const char* mqtt_pass = "12345678";
const char* topic_control = "doan/contrung/control";
const char* topic_sensor = "doan/contrung/sensor";
const char* topic_schedule = "doan/contrung/schedule";

// ==== MOTOR ====
#define IN1 16
#define IN2 17
#define PWM_CHANNEL 0
int motorSpeed = -1;
int motorTime = -1;
bool motorRunning = false;
unsigned long motorStartTime = 0;

// Các biến điều khiển logic
bool logic_motor = true;
bool logic_uva = true;
bool logic_led = true;
bool logic_sensor = true;

// ==== DHT11 ====
#define DHTPIN 14
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ==== BH1750 ====
BH1750 lightMeter;
#define SDA_PIN 32
#define SCL_PIN 33

// ==== RELAY ====
#define UVA 2     // đèn UVA
#define LED 4     // đèn led 24V

// ==== RTC ====
RTC_DS3231 rtc;

// ==== WiFi & MQTT ====
WiFiClient espClient;
PubSubClient client(espClient);

// ==== Sensor Timer ====
unsigned long lastsensorSend = 0;
const unsigned long SENSOR_INTERVAL = 5000;

// ==== Lịch trình ====
#define MAX_SCHEDULES 10
struct Schedule {
  String time;
  String status;
};
Schedule schedules[MAX_SCHEDULES];
int scheduleCount = 0;
String lastExecuted = "";

void setup_wifi() {
  delay(500);
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); delay(500);
  }
  Serial.println("\n✅ WiFi đã kết nối, IP: " + WiFi.localIP().toString());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);
  Serial.print("📩 Nhận từ topic ["); Serial.print(topic); Serial.println("]");

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("❌ Lỗi parse JSON");
    return;
  }

  // Xử lý lệnh motor/LED
  if (String(topic) == topic_control) {
    if (doc.containsKey("speed") && doc.containsKey("time")) {
      motorSpeed = constrain(doc["speed"].as<int>(), 0, 255);
      motorTime = max(doc["time"].as<int>(), 1);
    }
    ///////////////////////////////////////////////////////////////////////////// cần sửa
    if (doc.containsKey("led1")) {
      String ledCmd = doc["led1"].as<String>();
      digitalWrite(UVA, ledCmd == "on" ? HIGH : LOW);
      Serial.println("💡 UVA: " + ledCmd);
    }

    if (doc.containsKey("led2")) {
      String ledCmd = doc["led2"].as<String>();
      digitalWrite(LED, ledCmd == "on" ? HIGH : LOW);
      Serial.println("💡 LED: " + ledCmd);
    }

    if (doc.containsKey("action")) {
      String action = doc["action"];
      if (action == "add_schedule" && scheduleCount < MAX_SCHEDULES) {
        schedules[scheduleCount++] = {
          doc["data"]["time"].as<String>(),
          doc["data"]["status"].as<String>()
        };
        Serial.println("📌 Thêm lịch thành công");
      } else if (action == "get_schedule") {
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
      } else if (action == "delete_schedule") {
        int index = doc["index"];
        if (index >= 0 && index < scheduleCount) {
          for (int i = index; i < scheduleCount - 1; i++) {
            schedules[i] = schedules[i + 1];
          }
          scheduleCount--;
          Serial.print("🗑️ Xoá lịch tại index: "); Serial.println(index);
        }
      }
    }
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Kết nối MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("✅ MQTT kết nối");
      client.subscribe(topic_control);
    } else {
      Serial.print("❌ MQTT lỗi: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void sendSensorDataToMQTT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float lux = lightMeter.readLightLevel();

  if(lux < 2000 && logic_led == true)  digitalWrite(LED, HIGH);    // cường độ ánh sáng yếu và trong thời gian hoạt động cần bật led 24VDC
  else digitalWrite(LED, LOW);

  if (isnan(temp) || isnan(hum) || lux < 0 || lux == 65535) {
    Serial.println("❌ Lỗi đọc sensor");
    return;
  }

  StaticJsonDocument<256> doc;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["light"] = lux;
  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(topic_sensor, buffer);
  Serial.println("📤 Sensor gửi: " + String(buffer));
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
          Serial.println("Hệ thống đang hoạt động!");
        } else {
          logic_motor = false;
          logic_uva = false;
          logic_led = false;
          logic_sensor = false;
          Serial.println("Hệ thống dừng hoạt động!");
        }
        lastExecuted = key;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL, 5000, 8);
  ledcAttachPin(IN1, PWM_CHANNEL);

  pinMode(UVA, OUTPUT);
  digitalWrite(UVA, LOW);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (!lightMeter.begin()) {
    Serial.println("❌ BH1750 không hoạt động");
  }

  if (!rtc.begin()) {
    Serial.println("❌ RTC không kết nối");
  } else if (rtc.lostPower()) {
    Serial.println("⚠️ RTC mất nguồn, đặt lại thời gian.");
  }
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // Kiểm tra lịch hẹn giờ
  checkSchedule();

  // Điều khiển motor
  if (!motorRunning && motorSpeed >= 0 && motorTime > 0 && logic_motor == true) {
    Serial.printf("🚀 Motor chạy speed=%d time=%dms\n", motorSpeed, motorTime);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, motorSpeed);
    motorStartTime = millis();
    motorRunning = true;
  }

  if (motorRunning && millis() - motorStartTime >= motorTime && logic_motor == true) {
    ledcWrite(PWM_CHANNEL, 0);
    motorSpeed = -1;
    motorTime = -1;
    motorRunning = false;
  }

  // Gửi dữ liệu sensor mỗi 5s
  if (millis() - lastsensorSend > SENSOR_INTERVAL && logic_sensor == true) {
    sendSensorDataToMQTT();
    lastsensorSend = millis();
  }
}
