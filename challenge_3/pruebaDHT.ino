#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== DHT22 =====
#define DHT_PIN   4
#define DHT_TYPE  DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// ===== LM35 =====
#define LM35_PIN 15   // <-- ADC1, recomendado y estable

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== Variables compartidas =====
float tempDHT = NAN, humDHT = NAN, tempLM35 = NAN;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t hTaskSensores = NULL, hTaskLCD = NULL, hTaskSerial = NULL;

// ==================== LECTURA LM35 ====================
float leerLM35() {
  int lecturaADC = analogRead(LM35_PIN);   // ADC 0–4095
  float voltaje = lecturaADC * (3.3 / 4095.0);   // Conversión a voltios

  // LM35 → 10 mV por °C (0.01 V por °C)
  float temperatura = voltaje / 0.01;

  Serial.printf("[LM35] ADC=%d Volt=%.3fV Temp=%.2f°C\n",
                lecturaADC, voltaje, temperatura);

  return temperatura;
}

// ==================== TAREA DE SENSORES ====================
void taskLeerSensores(void *pvParameters) {
  Serial.println("[Tarea Sensores] Iniciada");

  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float t_lm35 = leerLM35();

    if (isnan(t) || isnan(h)) {
      Serial.println("[DHT22] Error de lectura");
    } else {
      Serial.printf("[DHT22] OK Temp=%.1f°C Hum=%.1f%%\n", t, h);
    }

    portENTER_CRITICAL(&mux);
    tempDHT  = t;
    humDHT   = h;
    tempLM35 = t_lm35;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ==================== TAREA LCD ====================
void taskActualizarLCD(void *pvParameters) {
  Serial.println("[Tarea LCD] Iniciada");

  for (;;) {
    float t_local, h_local, lm35_local;

    portENTER_CRITICAL(&mux);
    t_local = tempDHT;
    h_local = humDHT;
    lm35_local = tempLM35;
    portEXIT_CRITICAL(&mux);

    lcd.clear();
    lcd.setCursor(0, 0);

    if (isnan(t_local) || isnan(h_local)) {
      lcd.print("Esperando DHT...");
    } else {
      lcd.print("T:");
      lcd.print(t_local, 1);
      lcd.print((char)223);
      lcd.print("C H:");
      lcd.print(h_local, 1);
      lcd.print("%");
    }

    lcd.setCursor(0, 1);
    lcd.print("LM35:");
    lcd.print(lm35_local, 1);
    lcd.print((char)223);
    lcd.print("C");

    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}

// ==================== TAREA SERIAL ====================
void taskEnviarSerial(void *pvParameters) {
  Serial.println("[Tarea Serial] Iniciada");

  for (;;) {
    portENTER_CRITICAL(&mux);
    float t = tempDHT, h = humDHT, lm = tempLM35;
    portEXIT_CRITICAL(&mux);

    Serial.println("\n=== ESTADO ===");
    Serial.printf("DHT  -> %.1f°C  %.1f%%\n", t, h);
    Serial.printf("LM35 -> %.1f°C\n", lm);
    Serial.println("================\n");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // I2C correcto en ESP32
  Wire.begin(21, 22);

  // DHT22
  dht.begin();

  // LM35 → configurar ADC
  analogReadResolution(12);   // 0-4095
  analogSetAttenuation(ADC_11db); // rango ~3.3V

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  // Crear tareas
  xTaskCreatePinnedToCore(taskLeerSensores, "Sensores", 4096, NULL, 3, &hTaskSensores, 1);
  xTaskCreatePinnedToCore(taskActualizarLCD, "LCD",      4096, NULL, 1, &hTaskLCD,     1);
  xTaskCreatePinnedToCore(taskEnviarSerial,  "Serial",   4096, NULL, 1, &hTaskSerial,  1);

  Serial.println("Sistema listo.\n");
}

// ==================== LOOP ====================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));
}
