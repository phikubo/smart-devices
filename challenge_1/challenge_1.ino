  /* Updated for ENA, IN1, IN2 wiring - IN2 moved to GPIO18 // cambiado porque 22 está ocupado */
  #include <Arduino.h>

  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>

  // -------------------- CONFIGURACIÓN DE PINES --------------------
  const int pinENA = 18; // PWM -> ENA
  const int pinIN1 = 17; // Dirección 1 -> IN1
  const int pinIN2 = 19; //  Dirección 2 -> IN2 

  const int encA = 2;
  const int encB = 4;

  // -------------------- PARÁMETROS DEL ENCODER --------------------
  const unsigned int encoderPPR = 600;

  // -------------------- PID --------------------
  float Kp = 0.9;
  float Ki = 0.4;
  float Kd = 0.02;

  const unsigned long sampleTimeMs = 100;

  // -------------------- LÍMITES DE SETPOINT --------------------
  const int RPM_MAX = 150;
  const int RPM_MIN = -150;

  // -------------------- VARIABLES GLOBALES --------------------
  volatile long encCount = 0;
  float setpointRPM = 0.0;
  float measuredRPM = 0.0;
  float errorRPM = 0.0;
  float integralTerm = 0.0;
  float lastError = 0.0;
  unsigned long lastSampleTime = 0;

  bool waitingForSetpoint = false;
  LiquidCrystal_I2C lcd(0x27, 16, 2);
  String inputBuffer = "";

  void IRAM_ATTR onEncoderA() {
    int b = digitalRead(encB);
    if (b == HIGH) encCount++;
    else encCount--;
  }

  void setupHardware() {
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);

    const int pwmChannel = 0;
      ledcSetup(pwmChannel, 20000, 8);
    ledcAttachPin(pinENA, pwmChannel);

    attachInterrupt(digitalPinToInterrupt(encA), onEncoderA, CHANGE);

    lcd.init();
    lcd.backlight();

    ledcWrite(pwmChannel, 0);
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }

  void emergencyStop() {
    const int pwmChannel = 0;
    ledcWrite(pwmChannel, 0);
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    setpointRPM = 0.0;
    integralTerm = 0.0;
  }

  float computeRPM(long deltaCounts, unsigned long dtMs) {
    if (dtMs == 0) return 0.0;
    float revs = ((float)deltaCounts) / (float)encoderPPR;
    float rpm = revs * (60000.0f / (float)dtMs);
    return rpm;
  }

  bool parseSignedInt(const String &s, int &outVal) {
    String t = s;
    t.trim();
    if (t.length() == 0) return false;
    int sign = 1;
    int start = 0;
    if (t.charAt(0) == '+') { sign = 1; start = 1; }
    else if (t.charAt(0) == '-') { sign = -1; start = 1; }
    if (start >= t.length()) return false;
    long val = 0;
    for (int i = start; i < t.length(); ++i) {
      char c = t.charAt(i);
      if (c < '0' || c > '9') return false;
      val = val * 10 + (c - '0');
    }
    outVal = (int)(sign * val);
    return true;
  }

  void applyControl(float effort) {
    const int pwmChannel = 0;

    if (effort >= 0) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);
    } else {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, HIGH);
    }

    float mag = fabs(effort);
    if (mag > 1.0) mag = 1.0;

    int pwmVal = (int)(mag * 255.0);
    ledcWrite(pwmChannel, pwmVal);
  }

  void controlLoop() {
    unsigned long now = millis();
    if (now - lastSampleTime < sampleTimeMs) return;
    unsigned long dt = now - lastSampleTime;
    lastSampleTime = now;

    long countSnapshot;
    noInterrupts();
    countSnapshot = encCount;
    encCount = 0;
    interrupts();

    measuredRPM = computeRPM(countSnapshot, dt);
    errorRPM = setpointRPM - measuredRPM;

    integralTerm += errorRPM * ((float)dt / 1000.0f);
    float integralLimit = 300.0f;
    if (integralTerm > integralLimit) integralTerm = integralLimit;
    if (integralTerm < -integralLimit) integralTerm = -integralLimit;

    float derivative = (errorRPM - lastError) / ((float)dt / 1000.0f);
    float u = Kp * errorRPM + Ki * integralTerm + Kd * derivative;

    float salidaMaxRPM = 200.0f;
    if (u > salidaMaxRPM) u = salidaMaxRPM;
    if (u < -salidaMaxRPM) u = -salidaMaxRPM;

    applyControl(u / salidaMaxRPM);
    lastError = errorRPM;
  }

  void updateLCD() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.printf("SP:%+4.0f RPM", setpointRPM);
    lcd.setCursor(0,1);
    lcd.printf("M:%+4.0f E:%+4.0f", measuredRPM, errorRPM);
  }

  void updateSerialStatus() {
    Serial.print("SETPOINT: "); Serial.print((int)setpointRPM);
    Serial.print(" RPM	MED: "); Serial.print((int)measuredRPM);
    Serial.print(" RPM	ERR: "); Serial.println((int)errorRPM);
  }

  void setup() {
    Serial.begin(115200);
    setupHardware();
    Serial.println("Control Motor ENA/IN1/IN2 listo! Presiona 's' o 'x'");
    lastSampleTime = millis();
  }

  void loop() {
    if (Serial.available()) {
      char c = Serial.read();
      if (!waitingForSetpoint) {
        if (c == 's' || c == 'S') {
          waitingForSetpoint = true;
          inputBuffer = "";
          Serial.println("Ingrese setpoint (ej: -120) y ENTER");
        } else if (c == 'x' || c == 'X') {
          Serial.println("PARADA INMEDIATA");
          emergencyStop();
        }
      } else {
        if (c == '\n') {
          int val;
          if (parseSignedInt(inputBuffer, val) && val >= RPM_MIN && val <= RPM_MAX) {
            setpointRPM = val;
            Serial.print("Setpoint OK: "); Serial.println(val);
          } else {
            Serial.println("ERROR: Setpoint inválido");
          }
          waitingForSetpoint = false;
        } else if (c != '\n') {
          inputBuffer += c;
          Serial.print(c);
        }
      }
    }

    controlLoop();

    static unsigned long lastUI = 0;
    unsigned long now = millis();
    if (now - lastUI > 500) {
      updateLCD();
      updateSerialStatus();
      lastUI = now;
    }
  }
