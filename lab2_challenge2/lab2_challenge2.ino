//Lab_2_parte2 embebidos


#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>


// ====== DHT22 ======
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


// ====== WiFi ======
const char* ssid = "Raspberry";
const char* password = "dgsa7891";


// ====== MQTT (HiveMQ Cloud) ======
const char* mqtt_server = "5928f04afc44498788c80a05fd55ecb5.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "esp32";
const char* mqtt_pass   = "Esp32_lab2";


// ====== Topics ======
const char* topic_pub = "esp32/sensores/dht22";
const char* topic_pub_temp = "esp32/sensores/dht22/temperatura";
const char* topic_pub_hum = "esp32/sensores/dht22/humedad
";
const char* topic_sub = "esp32/control/rpm";


WiFiClientSecure espClient;
PubSubClient client(espClient);


// ====== Variables ======
unsigned long lastMsg = 0;
const long interval = 5000;  // 5 segundos
int rpmSetpoint = 0;


// ====== Callback MQTT ======
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload, length);


  if (String(topic) == topic_sub) {
    rpmSetpoint = doc["rpm"];
    Serial.print("RPM recibido por MQTT: ");
    Serial.println(rpmSetpoint);
  }
}


// ====== Conexión MQTT ======
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println(" conectado");
      client.subscribe(topic_sub, 1);
    } else {
      Serial.print(" fallo, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}


// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  dht.begin();


  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");


  espClient.setInsecure(); // Para TLS sin certificado
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}


// ====== LOOP ======
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;


    float temp = dht.readTemperature();


    float hum  = dht.readHumidity();


    if (isnan(temp) || isnan(hum)) {
      Serial.println("Error leyendo DHT22");
      return;
    }


    StaticJsonDocument<200> doc;
    doc["temperature"] = temp;
    doc["humidity"] = hum;
    doc["timestamp"] = now / 1000;


    char buffer[256];
    serializeJson(doc, buffer);


    client.publish(topic_pub, buffer, true);
    Serial.println("Telemetría publicada:");
    Serial.println(buffer);
  }
}



