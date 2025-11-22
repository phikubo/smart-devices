
const int BUTTON_PIN = 5;
const int LED_PINS[] = {18, 19, 21, 22};
const int NUM_LEDS = 4;

//Se establecen los estados de la secuencia.
enum SequenceState {
  STOPPED,
  RUNNING
};

//Se inicializa el estado actual de la secuencia en STOPPED.
SequenceState currentState = STOPPED;

//Variables para la configuración de los LEDS
const long LED_INTERVAL = 1000;
unsigned long lastLedChangeTime = 0;
int currentLedIndex = 0;

//Variables para la configuración del botón.
const long DEBOUNCE_DELAY = 50;   //Se establece un tiempo para controlar el antirrebote
int buttonState = HIGH;          // Estado inicial del botón.
int lastButtonReading = HIGH;    // Última lectura del botón.
unsigned long lastDebounceTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("--- Se inicializa la secuencia de LEDs. ---");

  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Presiona el botón para iniciar la secuencia.");
}

void loop() {
  readAndDebounceButton();

  if (currentState == RUNNING) {
    runSequence();
  }
}

void readAndDebounceButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) { // Validar si se presiona el botón para iniciar y parar la secuencia
        currentState = (currentState == STOPPED) ? RUNNING : STOPPED;
        Serial.print("Estado de secuencia: ");
        Serial.println((currentState == RUNNING) ? "RUNNING" : "STOPPED");
      }
    }
  }

  lastButtonReading = reading;
}

void runSequence() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastLedChangeTime >= LED_INTERVAL) {
    lastLedChangeTime = currentMillis;

    digitalWrite(LED_PINS[currentLedIndex], LOW);
    currentLedIndex = (currentLedIndex + 1) % NUM_LEDS;
    digitalWrite(LED_PINS[currentLedIndex], HIGH);

    Serial.print("LED activo: ");
    Serial.println(currentLedIndex);
  }
}
