#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================== CONFIGURACIÓN DE PINES ==================
const int ENA_PIN           = 18;   // PWM puente H (LEDC)
const int IN1_PIN           = 17;   // Dirección 1
const int IN2_PIN           = 19;   // Dirección 2

const int ENCODER_CHA_PIN   = 2;    // Canal A del encoder (interrupción)
const int ENCODER_CHB_PIN   = 4;    // Canal B (no usado en este ejemplo)

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección 0x27, LCD 16x2

// ================== PARÁMETROS DEL ENCODER ==================
#define PULSOS_POR_REV 480   // <-- AJUSTAR SEGÚN TU MOTORENCODER REAL

// ================== PWM (LEDC) PARA ESP32 ==================
const int PWM_CHANNEL       = 0;        // Canal LEDC
const int PWM_FREQ          = 20000;    // 20 kHz
const int PWM_RESOLUTION    = 10;       // 10 bits -> 0..1023
const int PWM_MAX           = (1 << PWM_RESOLUTION) - 1;

// ================== CONTROL DE VELOCIDAD ==================
const unsigned long CONTROL_PERIOD_MS = 100;   // Período de control (100 ms)
const unsigned long DISPLAY_PERIOD_MS = 500;   // Actualización de LCD/Serial

// Variables de control
volatile long encoderTicks = 0;     // Pulsos acumulados durante el período
float rpmPerTick = 0.0;            // RPM generadas por cada pulso en CONTROL_PERIOD_MS

volatile float setpointRPM = 0.0;  // Consigna (-150..150)
float measuredRPM = 0.0;           // Velocidad medida (magnitud)
float errorRPM    = 0.0;           // Error (con signo, en RPM)

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
void IRAM_ATTR encoderISR() {
  // Solo contamos pulsos de canal A
  encoderTicks++;
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
  pinMode(ENCODER_CHB_PIN, INPUT_PULLUP); // no se usa pero se deja definido

  // Interrupción del encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA_PIN), encoderISR, RISING);

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
  rpmPerTick = (60.0f * 1000.0f) / (PULSOS_POR_REV * CONTROL_PERIOD_MS); // 0.125

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

  // Bucle de control de velocidad (lazo abierto: PWM proporcional a consigna)
  if (now - lastControlMillis >= CONTROL_PERIOD_MS) {
    lastControlMillis += CONTROL_PERIOD_MS;
    updateSpeedAndControl();
  }

  // Actualización de LCD y Serial
  if (now - lastDisplayMillis >= DISPLAY_PERIOD_MS) {
    lastDisplayMillis += DISPLAY_PERIOD_MS;
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

// ================== CONTROL DE VELOCIDAD (LAZO ABIERTO) ==================
void updateSpeedAndControl() {
  // 1) Leer y resetear los pulsos del encoder de forma atómica
  long ticks;
  noInterrupts();
  ticks = encoderTicks;
  encoderTicks = 0;
  interrupts();

  // 2) Calcular RPM medida (magnitud, sin signo)
  measuredRPM = ticks * rpmPerTick;

  // 3) Si la consigna es cero, detenemos el motor
  if (setpointRPM == 0.0f) {
    errorRPM = -measuredRPM;  // error respecto a 0 (solo informativo)
    stopMotor();
    return;
  }

  // 4) Signo de giro en función de la consigna
  float sign = (setpointRPM >= 0.0f) ? 1.0f : -1.0f;
  setMotorDirection(sign > 0.0f);

  // 5) Magnitud de la consigna (limitada a 150 RPM)
  float spAbs = fabs(setpointRPM);
  if (spAbs > 150.0f) spAbs = 150.0f;

  // 6) Mapear 0..150 RPM -> 0..PWM_MAX (lazo abierto)
  int duty = (int)(spAbs * (float)PWM_MAX / 150.0f);

  // Opcional: duty mínimo para vencer fricción (si spAbs > 0)
  if (spAbs > 0.1f && duty < (PWM_MAX * 0.15f)) {
    duty = (int)(PWM_MAX * 0.15f);   // por ejemplo 15% mínimo
  }

  ledcWrite(PWM_CHANNEL, duty);

  // 7) Error solo para mostrar (supone que la velocidad es positiva)
  float effectiveMeasured = measuredRPM * sign;
  errorRPM = setpointRPM - effectiveMeasured;
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
  lcd.print(setpointRPM, 0);
  lcd.print(" M:");
  lcd.print(measuredRPM, 0);
  
  lcd.setCursor(0, 1);
  lcd.print("Err:");
  lcd.print(errorRPM, 0);
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
      // Se aceptan solo '+', '-', y dígitos
      else {
        if (c == '+' || c == '-' || isDigit(c)) {
          inputBuffer += c;
          Serial.print(c); // eco
        } else {
          // Otros símbolos se ignoran, pero harán que el formato sea inválido al presionar ENTER
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

  // Validar formato: [signo opcional][digitos]
  bool ok = true;
  int startIdx = 0;

  if (inputBuffer.length() == 0) {
    ok = false;
  } else {
    char first = inputBuffer.charAt(0);
    if (first == '+' || first == '-') {
      startIdx = 1;
    }
    if (startIdx >= inputBuffer.length()) {
      ok = false; // Solo signo, sin dígitos
    } else {
      for (int i = startIdx; i < inputBuffer.length(); i++) {
        if (!isDigit(inputBuffer.charAt(i))) {
          ok = false;
          break;
        }
      }
    }
  }

  if (!ok) {
    Serial.println("ERROR: Formato invalido. Use solo signo opcional (+/-) y digitos.");
    inputBuffer = "";
    Serial.print("Ingrese consigna [-150..150] RPM: ");
    return;
  }

  // Convertir a entero
  int value = inputBuffer.toInt();

  // Verificar rango
  if (value < -150 || value > 150) {
    Serial.println("ERROR: Consigna fuera de rango (-150..150 RPM).");
    inputBuffer = "";
    Serial.print("Ingrese consigna [-150..150] RPM: ");
    return;
  }

  // Consigna válida
  setpointRPM = (float)value;
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
  stopMotor();
  Serial.println();
  Serial.println(">>> PARO DE EMERGENCIA (X). Motor DETENIDO <<<");
}