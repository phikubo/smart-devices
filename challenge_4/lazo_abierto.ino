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
#define CPR_PER_CHANNEL 480
#define PULSOS_POR_REV (CPR_PER_CHANNEL * 4)  // 1920 pulsos/vuelta (cuadratura x4)

// ================== PWM (LEDC) PARA ESP32 ==================
const int PWM_CHANNEL       = 0;
const int PWM_FREQ          = 20000;    // 20 kHz
const int PWM_RESOLUTION    = 8;        // 8 bits → 0..255
const int PWM_MAX           = (1 << PWM_RESOLUTION) - 1; // 255

const int PWM_MIN_RUN       = 120;      // Valor experimental de arranque

// ================== CONTROL DE VELOCIDAD ==================
const unsigned long CONTROL_PERIOD_MS = 100;
const unsigned long DISPLAY_PERIOD_MS = 500;

// Variables globales
volatile long encoderTicks = 0;
float rpmPerTick = 0.0;

volatile float setpointRPM = 0.0;
float measuredRPM = 0.0;
float errorRPM    = 0.0;
int currentPWM    = 0;                  // ← VARIABLE NUEVA: PWM actual

unsigned long lastControlMillis = 0;
unsigned long lastDisplayMillis = 0;

// ================== MANEJO DE CONSOLA SERIE ==================
enum InputState { NORMAL, ENTERING_SP };
InputState inputState = NORMAL;
String inputBuffer = "";
const char SETPOINT_KEY = 'S';

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
  bool a = digitalRead(ENCODER_CHA_PIN);
  bool b = digitalRead(ENCODER_CHB_PIN);
  if (a == b) {
    encoderTicks--;
  } else {
    encoderTicks++;
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  pinMode(ENCODER_CHA_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CHB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA_PIN), encoderISR, CHANGE);

  stopMotor();
  setpointRPM = 0.0;

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed Control");
  lcd.setCursor(0, 1);
  lcd.print("SP=0 PWM=0");

  rpmPerTick = (60.0f * 1000.0f) / (PULSOS_POR_REV * CONTROL_PERIOD_MS);

  Serial.println();
  Serial.println("===== Control de velocidad (LAZO ABIERTO) =====");
  Serial.println("Teclas: S → nueva consigna | X → paro emergencia");
  Serial.println("Salida: SP | RPM medidos | Error | PWM actual");
  Serial.println();
}

// ================== LOOP ==================
void loop() {
  handleSerial();

  unsigned long now = millis();

  if (now - lastControlMillis >= CONTROL_PERIOD_MS) {
    lastControlMillis = now;
    updateSpeedAndControl();
  }

  if (now - lastDisplayMillis >= DISPLAY_PERIOD_MS) {
    lastDisplayMillis = now;
    updateDisplay();
  }
}

// ================== FUNCIONES DE MOTOR ==================
void stopMotor() {
  currentPWM = 0;
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

// ================== CONTROL LAZO ABIERTO ==================
void updateSpeedAndControl() {
  long ticks;
  noInterrupts();
  ticks = encoderTicks;
  encoderTicks = 0;
  interrupts();

  measuredRPM = ticks * rpmPerTick;

  if (setpointRPM == 0.0f) {
    stopMotor();
    errorRPM = -measuredRPM;
    return;
  }

  bool forward = (setpointRPM >= 0.0f);
  setMotorDirection(forward);

  float spAbs = fabs(setpointRPM);
  const float RPM_MAX_UTIL = 80.0f;
  if (spAbs > RPM_MAX_UTIL) spAbs = RPM_MAX_UTIL;

  int duty = 0;
  if (spAbs >= 1.0f) {
    duty = PWM_MIN_RUN + (int)((spAbs - 1.0f) * (float)(PWM_MAX - PWM_MIN_RUN) / (RPM_MAX_UTIL - 1.0f));
    if (duty > PWM_MAX) duty = PWM_MAX;
  }

  currentPWM = duty;                   // ← Guardamos el valor actual de PWM
  ledcWrite(PWM_CHANNEL, duty);

  errorRPM = setpointRPM - measuredRPM;
}

// ================== ACTUALIZACIÓN DE PANTALLA Y SERIAL (CON PWM) ==================
void updateDisplay() {
  // ---------- SALIDA SERIAL (ahora incluye PWM) ----------
  Serial.print("SP=");
  Serial.print(setpointRPM, 1);
  Serial.print(" RPM | M=");
  Serial.print(measuredRPM, 1);
  Serial.print(" RPM | Err=");
  Serial.print(errorRPM, 1);
  Serial.print(" RPM | PWM=");
  Serial.print(currentPWM);           // ← AQUÍ ESTÁ EL VALOR DE PWM
  Serial.print("/255 (");
  Serial.print((currentPWM * 100) / 255);
  Serial.println("%)");

  // ---------- LCD ----------
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SP:");
  lcd.print(setpointRPM, 1);
  lcd.print(" M:");
  lcd.print(measuredRPM, 1);

  lcd.setCursor(0, 1);
  lcd.print("PWM:");
  lcd.print(currentPWM);
  lcd.print(" Err:");
  lcd.print(errorRPM, 0);
}

// ================== MANEJO DE CONSOLA SERIE (sin cambios) ==================
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'X' || c == 'x') {
      emergencyStop();
      continue;
    }

    if (inputState == NORMAL) {
      if (toupper(c) == SETPOINT_KEY) {
        enterSetpointMode();
      }
    } else if (inputState == ENTERING_SP) {
      if (c == '\r' || c == '\n') {
        if (inputBuffer.length() > 0) processSetpointBuffer();
        else {
          Serial.println();
          Serial.print("Ingrese consigna [-80..80] RPM y ENTER: ");
        }
      }
      else if (c == '\b' || c == 127) {
        if (inputBuffer.length() > 0) {
          inputBuffer.remove(inputBuffer.length() - 1);
          Serial.print("\b \b");
        }
      }
      else if (c == '+' || c == '-' || c == '.' || isDigit(c)) {
        inputBuffer += c;
        Serial.print(c);
      }
    }
  }
}

void enterSetpointMode() {
  inputState = ENTERING_SP;
  inputBuffer = "";
  Serial.println();
  Serial.println("=== MODO CONSIGNA ===");
  Serial.print("Ingrese consigna [-80..80] RPM y ENTER: ");
}

void processSetpointBuffer() {
  Serial.println();
  float value = inputBuffer.toFloat();
  if (value < -80.0f) value = -80.0f;
  if (value >  80.0f) value =  80.0f;

  setpointRPM = value;
  inputBuffer = "";
  inputState = NORMAL;

  Serial.print("Nueva consigna: ");
  Serial.print(setpointRPM, 1);
  Serial.println(" RPM");
}

void emergencyStop() {
  setpointRPM = 0.0f;
  stopMotor();
  Serial.println("\n>>> PARO DE EMERGENCIA <<< Motor detenido");
}