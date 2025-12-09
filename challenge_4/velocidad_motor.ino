// ===========================================================
// PRUEBA DE RAMPA DE VELOCIDAD 0 → 100% (de 10 en 10 cada 10s)
// Mide RPM reales con encoder
// ESP32 + Motor 12V + Driver L298N (o similar)
// ===========================================================

// ================== CONFIGURACIÓN DE PINES ==================
const int ENA_PIN         = 18;   // PWM velocidad
const int IN1_PIN         = 17;   // Dirección
const int IN2_PIN         = 19;   // Dirección
const int ENCODER_CHA_PIN = 2;    // Canal A (interrupción)
const int ENCODER_CHB_PIN = 4;    // Canal B

// ================== CONFIGURACIÓN ENCODER ==================
const int PPR = 480;              // ¡¡CAMBIAR SEGÚN TU MOTOR!!
                                  // Ejemplos comunes: 400, 880, 1024, 1600, 2000...
volatile long pulsos = 0;
unsigned long ultimoTiempo = 0;

// ===========================================================
void IRAM_ATTR contadorEncoder() {
  // Lectura rápida de ambos canales para saber dirección
  if (digitalRead(ENCODER_CHA_PIN) == digitalRead(ENCODER_CHB_PIN)) {
    pulsos++;
  } else {
    pulsos--;
  }
}

void setup() {
  // Pines del driver
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Encoder
  pinMode(ENCODER_CHA_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CHB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA_PIN), contadorEncoder, RISING);

  Serial.begin(115200);
  Serial.println(F("========================================"));
  Serial.println(F("  RAMPA DE VELOCIDAD 0 → 100% (10 en 10)"));
  Serial.println(F("  Cada paso dura 10 segundos"));
  Serial.print(F("  PPR del encoder: "));
  Serial.println(PPR);
  Serial.println(F("========================================"));
  delay(2000);

  // Dirección fija: adelante
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

void loop() {
  // Bucle principal: sube de 10 en 10 cada 10 segundos
  for (int velocidad = 0; velocidad <= 255; velocidad += 10) {
    analogWrite(ENA_PIN, velocidad);

    // Reiniciar contador de pulsos
    noInterrupts();
    pulsos = 0;
    interrupts();
    ultimoTiempo = millis();

    Serial.printf("\n--- VELOCIDAD PWM = %3d / 255 (%d%%) ---\n", velocidad, (velocidad * 100) / 255);

    // Durante 10 segundos medimos y mostramos RPM cada segundo
    for (int i = 0; i < 10; i++) {
      delay(1000);

      noInterrupts();
      long pulsosCopia = pulsos;
      interrupts();

      float rpm = (float)abs(pulsosCopia) * 60.0 / PPR;

      Serial.printf("  Tiempo: %2ds  →  Pulsos: %5ld  →  RPM: %6.1f\n", i + 1, pulsosCopia, rpm);
    }

    // Pequeña pausa antes del siguiente paso
    Serial.println("  → Siguiente velocidad...");
    delay(500);
  }

  // Al llegar al máximo, se queda ahí para siempre
  analogWrite(ENA_PIN, 255);
  Serial.println(F("\n¡PRUEBA FINALIZADA! Motor a máxima velocidad (255) permanentemente."));
  Serial.println(F("Puedes desconectar o subir otro código."));

  while (true) {
    delay(1000);  // Queda encendido al 100% para siempre
  }
}