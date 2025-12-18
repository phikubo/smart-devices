//lab_2_part1  embebidos

#include <WiFi.h>
#include <WebServer.h>
#include "DHT.h"


#define DHTPIN 4          // Pin del DHT22
#define DHTTYPE DHT22


DHT dht(DHTPIN, DHTTYPE);


// ===== CONFIGURACIÓN WiFi (modo AP para servidor standalone) =====
const char* ssid = "ESP32_Server";
const char* password = "12345678";


WebServer server(80);


// Variable para guardar el setpoint enviado
String rpmSetpoint = "0";


// ======== HTML PRINCIPAL (UI RESPONSIVE) ==========
String htmlPage() {
  String page = R"=====(
    <!DOCTYPE html>
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <style>
        body {
          font-family: Arial;
          text-align: center;
          background: #f2f2f2;
          padding: 20px;
        }
        .card {
          background: white;
          padding: 20px;
          margin: auto;
          width: 90%;
          max-width: 400px;
          border-radius: 12px;
          box-shadow: 0px 0px 12px #aaa;
        }
        input {
          width: 80%;
          padding: 10px;
          font-size: 18px;
          margin-top: 10px;
        }
        button {
          padding: 10px 20px;
          font-size: 18px;
          margin-top: 10px;
        }
      </style>
      <script>
        // Funcion para actualizar valores cada 2 segundos
        function updateData() {
          fetch('/readings')
            .then(response => response.json())
            .then(data => {
              document.getElementById("temp").innerHTML = data.temperature;
              document.getElementById("hum").innerHTML = data.humidity;
              document.getElementById("time").innerHTML = data.time;
            });
        }
        setInterval(updateData, 2000);


        function sendRPM() {
          let rpm = document.getElementById("rpmInput").value;
          fetch('/setrpm?value=' + rpm);
          alert("RPM enviada: " + rpm);
        }
      </script>
    </head>
    <body>
      <h2>ESP32 Web Server - Monitoreo Ambiental</h2>


      <div class="card">
        <h3>Lecturas</h3>
        <p><b>Temperatura:</b> <span id="temp">--</span> °C</p>
        <p><b>Humedad:</b> <span id="hum">--</span> %</p>
        <p><b>Actualizado:</b> <span id="time">--</span></p>
      </div>


      <div class="card">
        <h3>Setpoint de Velocidad (RPM)</h3>
        <input id="rpmInput" type="number" placeholder="Ingrese RPM">
        <button onclick="sendRPM()">Enviar</button>
      </div>


    </body>
    </html>
  )=====";


  return page;
}


// ======== Enviar lecturas como JSON para AJAX ==========
void handleReadings() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();


  String json = "{";
  json += "\"temperature\": \"" + String(t) + "\",";
  json += "\"humidity\": \"" + String(h) + "\",";
  json += "\"time\": \"" + String(millis() / 1000) + " s\"";
  json += "}";


  server.send(200, "application/json", json);
}


// ======== Recibir RPM del usuario ===========
void handleSetRPM() {
  if (server.hasArg("value")) {
    rpmSetpoint = server.arg("value");
    Serial.println("RPM recibida: " + rpmSetpoint);
  }
  server.send(200, "text/plain", "OK");
}


// ============== SETUP ====================
void setup() {
  Serial.begin(115200);
  dht.begin();


  WiFi.softAP(ssid, password);
  Serial.println("Servidor WiFi iniciado");
  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.softAPIP());


  server.on("/", []() { server.send(200, "text/html", htmlPage()); });


  server.on("/readings", handleReadings);
  server.on("/setrpm", handleSetRPM);


  server.begin();
}


// ============== LOOP =====================
void loop() {
  server.handleClient();
}





