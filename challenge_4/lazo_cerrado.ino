#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================== CONFIGURACIÓN DE PINES ==================
const int ENA_PIN           = 18;   // PWM puente H (LEDC)
const int IN1_PIN           = 17;   // Dirección 1
const int IN2_PIN           = 19;   // Dirección 2

const int ENCODER_CHA_PIN   = 2;    // Canal A del encoder (interrupción)
const int ENCODER_CHB_PIN   = 4;    // Canal B

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección 0x27, LCD 16x2

// ================== PARÁMETROS DEL ENCODER ==================
// Datasheet: 480 pulses per output shaft revolution per channel
const int CPR = 480;                // cycles/pulses por canal
const int EDGES_PER_CYCLE = 2;      // usamos CHANGE en A => 2 flancos
const float COUNTS_PER_REV = CPR * EDGES_PER_CYCLE;  // 480 * 2 = 960

// ================== PWM (LEDC) PARA ESP32 ==================
const int PWM_CHANNEL       = 0;        // Canal LEDC
const int PWM_FREQ          = 20000;    // 20 kHz
const int PWM_RESOLUTION    = 10;       // 10 bits -> 0..1023
const int PWM_MAX           = (1 << PWM_RESOLUTION) - 1;

// ================== CONTROL DE VELOCIDAD ==================
const unsigned long CONTROL_PERIOD_MS = 100;   // Período de control (100 ms)
const unsigned long DISPLAY_PERIOD_MS = 500;   // Actualización de LCD/Serial

// Variables de medición
volatile long encoderTicks = 0;      // Pulsos acumulados durante el período (con signo)
float rpmPerTick = 0.0;              // RPM generadas por cada pulso en CONTROL_PERIOD_MS

volatile float setpointRPM = 0.0;    // Consigna (-150..150)
float measuredRPM = 0.0;             // Velocidad medida (con signo)
float errorRPM    = 0.0;             // Error (con signo, en RPM)

// ================== PARÁMETROS PI ==================
// Estos valores son un punto de partida. Se pueden ajustar en pruebas:
float Kp = 3.0f;                     // Ganancia proporcional
float Ki = 0.8f;                     // Ganancia integral
float integral = 0.0f;               // Estado integral
const float INTEGRAL_LIMIT = PWM_MAX; // Límite anti-windup sobre la salida equivalente
float baseDeadzone = PWM_MAX * 0.30f; // ~30% del PWM para vencer fricción

// Tiempos para tareas periódicas
unsigned long lastControlMillis = 0;
unsigned long lastDisplayMillis = 0;

// ================== MANEJO DE CONSOLA SERIE ==================
enum InputState {
  NORMAL,
  ENTERING_SP
};

InputState inputState = NORMAL;
String inputBuffer = "";

const char SETPOINT_KEY = 'S';   // Tecla para entrar al modo de consigna

// ================== PROTOTIPOS ==================
void IRAM_ATTR encoderISR();
void stopMotor();
void setMotorDirection(bool forward);
void updateSpeedAndControl();
void updateDisplay();
void handleSerial();
void enterSetpointMode();
void processSetpointBuffer();
void emergencyStop();

// ================== ISR ENCODER ==================
// Quadrature simple: ISR en canal A, flanco CHANGE, se lee canal B para dirección
void IRAM_ATTR encoderISR() {
  bool a = digitalRead(ENCODER_CHA_PIN);
  bool b = digitalRead(ENCODER_CHB_PIN);

  // Si el sentido sale al revés (M negativa cuando SP>0),
  // simplemente intercambia ++ y -- aquí.
  if (a == b) {
    encoderTicks++;   // Un sentido
  } else {
    encoderTicks--;   // Sentido contrario
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Configuración de pines puente H
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // PWM por LEDC en ESP32
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);   // Motor apagado

  // Configuración de pines encoder
  pinMode(ENCODER_CHA_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CHB_PIN, INPUT_PULLUP);

  // Interrupción del encoder en CHANGE para cuadratura (2x)
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA_PIN), encoderISR, CHANGE);

  // Motor detenido al iniciar (requisito del enunciado)
  stopMotor();
  setpointRPM = 0.0;

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed Control");
  lcd.setCursor(0, 1);
  lcd.print("SP=0 RPM");

  // Cálculo del factor de RPM por pulso
  // RPM = (ticks / countsPerRev) * (60 s / periodo)
  rpmPerTick = (60.0f * 1000.0f) / (COUNTS_PER_REV * CONTROL_PERIOD_MS);

  Serial.println();
  Serial.println("===== Control de velocidad MOTORENCODER (ESP32) =====");
  Serial.println("Motor INICIALMENTE DETENIDO.");
  Serial.println("Teclas:");
  Serial.println("  S  -> Ingresar consigna [-150..150] RPM");
  Serial.println("  X  -> PARO de emergencia (motor OFF)");
  Serial.println();
}

// ================== LOOP PRINCIPAL ==================
void loop() {
  handleSerial();

  unsigned long now = millis();

  // Bucle de control de velocidad (lazo cerrado PI)
  if (now - lastControlMillis >= CONTROL_PERIOD_MS) {
    lastControlMillis = now;
    updateSpeedAndControl();
  }

  // Actualización de LCD y Serial
  if (now - lastDisplayMillis >= DISPLAY_PERIOD_MS) {
    lastDisplayMillis = now;
    updateDisplay();
  }
}

// ================== FUNCIONES DE MOTOR ==================
void stopMotor() {
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
}

// ================== CONTROL DE VELOCIDAD (PI + Deadzone) ==================
void updateSpeedAndControl() {
  long ticks;

  // 1) Leer y resetear los pulsos del encoder de forma atómica
  noInterrupts();
  ticks = encoderTicks;
  encoderTicks = 0;
  interrupts();

  // 2) Calcular RPM medida (con signo)
  measuredRPM = ticks * rpmPerTick;

  // 3) Si SP ≈ 0 → resetear integral y apagar motor
  if (fabs(setpointRPM) < 0.5f) {
    setpointRPM = 0.0f;
    integral = 0.0f;
    errorRPM = -measuredRPM;
    stopMotor();
    return;
  }

  // 4) Error con signo (si SP>0, queremos M>0; si SP<0, M<0)
  errorRPM = setpointRPM - measuredRPM;

  // 5) Actualizar integral (anti-windup simple)
  float dt = CONTROL_PERIOD_MS / 1000.0f;  // 0.1 s
  integral += errorRPM * dt;

  // Limitar integral para que Ki * integral no exceda INTEGRAL_LIMIT
  float maxIntegral = INTEGRAL_LIMIT / max(Ki, 0.001f);
  if (integral > maxIntegral) integral = maxIntegral;
  if (integral < -maxIntegral) integral = -maxIntegral;

  // 6) Control PI
  float control = Kp * errorRPM + Ki * integral;

  // 7) Magnitud del PWM
  float duty = fabs(control);

  // Saturación superior
  if (duty > PWM_MAX) duty = PWM_MAX;

  // 8) Compensación de zona muerta (para vencer fricción de la reductora)
  float localDeadzone = baseDeadzone;
  if (fabs(setpointRPM) < 10.0f) {
    // Para consignas muy pequeñas, usamos una zona muerta algo menor
    localDeadzone = PWM_MAX * 0.20f;  // ~20%
  }

  if (duty > 0.5f && duty < localDeadzone) {
    duty = localDeadzone;
  } else if (duty <= 0.5f) {
    duty = 0.0f;  // Evita pequeños movimientos cuando el error es muy pequeño
  }

  // 9) Aplicar dirección y PWM
  bool forward = (control >= 0.0f);
  setMotorDirection(forward);
  ledcWrite(PWM_CHANNEL, (int)duty);
}

// ================== ACTUALIZACIÓN DE LCD Y SERIAL ==================
void updateDisplay() {
  // Serial
  Serial.print("SP=");
  Serial.print(setpointRPM, 1);
  Serial.print(" RPM | M=");
  Serial.print(measuredRPM, 1);
  Serial.print(" RPM | Err=");
  Serial.print(errorRPM, 1);
  Serial.println(" RPM");

  // LCD (2 líneas sencillas)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SP:");
  lcd.print(setpointRPM, 1);
  lcd.print(" M:");
  lcd.print(measuredRPM, 1);

  lcd.setCursor(0, 1);
  lcd.print("Err:");
  lcd.print(errorRPM, 1);
  lcd.print(" RPM   ");
}

// ================== MANEJO DE CONSOLA SERIE ==================
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Tecla de PARO de emergencia (X/x)
    if (c == 'X' || c == 'x') {
      emergencyStop();
      continue;
    }

    if (inputState == NORMAL) {
      // Solo la tecla S (o s) entra al modo de consigna
      if (toupper(c) == SETPOINT_KEY) {
        enterSetpointMode();
      }
      // cualquier otra tecla se ignora
    } else if (inputState == ENTERING_SP) {
      // Fin de línea -> procesar consigna
      if (c == '\r' || c == '\n') {
        if (inputBuffer.length() > 0) {
          processSetpointBuffer();
        } else {
          Serial.println();
          Serial.println("Consigna vacia. Intente de nuevo.");
          Serial.print("Ingrese consigna [-150..150] RPM: ");
        }
      }
      // Backspace (opcional)
      else if (c == '\b' || c == 127) {
        if (inputBuffer.length() > 0) {
          inputBuffer.remove(inputBuffer.length() - 1);
          Serial.print("\b \b");
        }
      }
      // Se aceptan solo '+', '-', '.', y dígitos (para floats)
      else {
        if (c == '+' || c == '-' || c == '.' || isDigit(c)) {
          inputBuffer += c;
          Serial.print(c); // eco
        } else {
          // Otros símbolos se ignoran
        }
      }
    }
  }
}

void enterSetpointMode() {
  inputState = ENTERING_SP;
  inputBuffer = "";
  Serial.println();
  Serial.println("=== MODO CONSIGNA ===");
  Serial.print("Ingrese consigna [-150..150] RPM y ENTER: ");
}

void processSetpointBuffer() {
  Serial.println();

  // Convertir a float
  float value = inputBuffer.toFloat();

  // Verificar rango
  if (value < -150.0f || value > 150.0f) {
    Serial.println("ERROR: Consigna fuera de rango (-150..150 RPM).");
    inputBuffer = "";
    Serial.print("Ingrese consigna [-150..150] RPM: ");
    return;
  }

  // Consigna válida
  setpointRPM = value;
  inputBuffer = "";
  inputState = NORMAL;

  Serial.print("Nueva consigna aceptada: ");
  Serial.print(setpointRPM, 1);
  Serial.println(" RPM");

  Serial.println("Presione 'S' para cambiar de nuevo la consigna.");
  Serial.println("Presione 'X' para PARO de emergencia.");
}

void emergencyStop() {
  setpointRPM = 0.0f;
  errorRPM = 0.0f;
  integral = 0.0f;
  stopMotor();
  Serial.println();
  Serial.println(">>> PARO DE EMERGENCIA (X). Motor DETENIDO <<<");
}
