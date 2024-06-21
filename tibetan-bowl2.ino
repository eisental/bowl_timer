#include "LEDFader.h"
#include "secrets.h"
#include <ArduinoMqttClient.h>
#include <R4_Touch.h>
#include <TimeAlarms.h>
#include <WiFi101.h>

/*
  meditation-timer v2 - june, 2024
*/

enum state { idle, preparing, meditating };

// timing

const int maxTotalDuration = 60 * 60;
const int minTotalDuration = 1 * 60;
const int maxIntervalDuration = 60 * 60;
const int minIntervalDuration = 1 * 60;
const int maxPreparationDuration = 1 * 60;
const int minPreparationDuration = 1;

int totalDuration = 40 * 60;    // in seconds
int intervalDuration = 10 * 60; // in seconds
int preparationDuration = 20;   // in seconds

int longChimeStrike = 150;
int shortChimeStrike = 100;

// hardware

int solenoidPin = 2;             // digital output
int activityLedPin = 11;         // pwm output
int standbyLedPin = 10;          // pwm output
int touchPin = 0;                // digital in
int totalDurationPin = A1;       // analog in
int intervalDurationPin = A2;    // analog in
int preparationDurationPin = A4; // analog in

// capacitive bowl

int bowlTouchThreshold = 50000;    // switch point between on/off
int touchRef;                      // offset reference value for touch.
unsigned long touchStartTime = -1; // measure touch time for long/short touch.
int longTouchDuration = 2000;      // ms
int shortTouchDuration = 100;      // ms

TouchSensor bowlTouch;
ctsu_pin_settings_t touchSettings = {.div = CTSU_CLOCK_DIV_28,
                                     .gain = CTSU_ICO_GAIN_40,
                                     .ref_current = 0,
                                     .offset = 75,
                                     .count = 3};

// status LEDs

LEDFader activityLed = LEDFader(activityLedPin);
LEDFader standbyLed = LEDFader(standbyLedPin);

// alarm IDs

int currentActivityAlarm;
int currentIntervalAlarm;

// state

state currentState = idle;
bool isPaused = false;
bool isTouching = false;

int currentIntervalDuration;     // in seconds
int remainingTotalDuration;      // in seconds
int remainingIntervalDuration;   // in seconds
unsigned long intervalStartTime; // in milliseconds

void readDurations();
void meditate();
void fadeOutStandbyLed();
void fadeOutActivityLed();
void startInterval(int intervalDuration, int strikeDuration);
void chime(int duration);
void reset();
void disableAlarms();

// state machine

void prepare() {
  readDurations();

  currentState = preparing;
  currentIntervalAlarm = Alarm.timerOnce(preparationDuration, meditate);
  activityLed.fade(255, preparationDuration * 1000);
  standbyLed.fade(255, 1000);
  currentActivityAlarm = Alarm.timerOnce(1, fadeOutStandbyLed);
}

void meditate() {
  currentState = meditating;
  standbyLed.fade(255, 5000);
  fadeOutActivityLed();
  remainingTotalDuration = totalDuration;
  startInterval(intervalDuration, longChimeStrike);
}

void endMeditation() {
  chime(longChimeStrike);
  standbyLed.fade(0, 5000);
  reset();
}

void pause() {
  disableAlarms();

  int finishedIntervalDuration = (millis() - intervalStartTime) / 1000;
  remainingIntervalDuration =
      currentIntervalDuration - finishedIntervalDuration;
  remainingTotalDuration -=
      finishedIntervalDuration; // remove partial interval from remaining total.

  // turn off standby led while paused.
  standbyLed.fade(0, 200);

  // DEBUG
  Serial.print("PAUSED | remaining: ");
  Serial.print(remainingIntervalDuration);
  Serial.print(" currentIntervalDuration: ");
  Serial.println(currentIntervalDuration);
  // END DEBUG
}

void unpause() {
  activityLed.fade(0, 200);
  startInterval(remainingIntervalDuration, 0);
}

void cancel() {
  standbyLed.fade(0, 500);
  activityLed.fade(0, 500);
  reset();
}

void reset() {
  disableAlarms();
  remainingTotalDuration = totalDuration;
  currentState = idle;
  isPaused = false;
}

// handle bowl touch

void onShortTouch() {
  switch (currentState) {
  case idle:
    prepare();
    break;
  case preparing:
    cancel();
    break;
  case meditating:
    isPaused = !isPaused;
    if (isPaused) {
      pause();
    } else {
      unpause();
    }
    break;
  }
}

void onLongTouch() { cancel(); }

// intervals
void intervalEnded();

void startInterval(int intervalDuration, int strikeDuration) {
  currentIntervalDuration = min(intervalDuration, remainingTotalDuration);
  intervalStartTime = millis();
  currentIntervalAlarm =
      Alarm.timerOnce(currentIntervalDuration, intervalEnded);
  chime(strikeDuration);

  activityLed.fade(255, 1000);
  currentActivityAlarm = Alarm.timerOnce(1, fadeOutActivityLed);

  // DEBUG
  Serial.print("Starting interval of ");
  Serial.print(currentIntervalDuration);
  Serial.print(" seconds. Remaining total: ");
  Serial.println(remainingTotalDuration);
  // END DEBUG
}

void intervalEnded() {
  remainingTotalDuration -= currentIntervalDuration;

  if (remainingTotalDuration > 1) {
    startInterval(intervalDuration, shortChimeStrike);
  } else {
    Serial.println("Ending meditation");
    endMeditation();
  }
}

// lower level logic

void disableAlarms() {
  Alarm.disable(currentActivityAlarm);
  Alarm.disable(currentIntervalAlarm);
  Alarm.free(currentActivityAlarm);
  Alarm.free(currentIntervalAlarm);
}

void chime(int duration) {
  if (duration <= 0) {
    return;
  }

  digitalWrite(solenoidPin, HIGH);
  delay(duration);
  digitalWrite(solenoidPin, LOW);
}

// these are timer functions

void fadeOutActivityLed() { activityLed.fade(0, 1000); }

void fadeOutStandbyLed() { standbyLed.fade(0, 1000); }

// WiFi + MQTT

WiFiClient wifi;
MqttClient mqtt(wifi);

void setupMQTT() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("WiFi: ");
  display.println(wifiSSID);
  display.display();

  // TODO: try to connect in the background
  while (WiFi.begin(wifiSSID, wifiPass) != WL_CONNECTED) {
    display.print(".");
    display.display();
    readButton();
    if (buttonState) {
      break;
      buttonState = false;
    }
    delay(1000);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("Connected!");
    display.display();
  } else {
    display.println("No WiFi!");
    display.display();
    return;
  }

  delay(500);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to MQTT:");
  display.print(mqttHost);
  display.print(":");
  display.println(mqttPort);
  display.display();

  if (!mqtt.connect(mqttHost, mqttPort)) {
    char err[80];
    sprintf(err, "MQTT connect failed. Error code: %ld", mqtt.connectError());
    freezeOnError(err);
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connected to MQTT!\n");
    delay(500);
  }
}

void publishMQTT() {
  if (!mqtt.connected()) {
    return;
  }
  publishToggle = !publishToggle;
  mqtt.beginMessage(mqttTopic);
  mqtt.print("{\"probe_temp\": ");
  mqtt.print(thermoCoupleValue);
  mqtt.print(", \"ambient_temp\": ");
  mqtt.print(ambientTempValue);
  mqtt.print(", \"uv\": ");
  mqtt.print(uvValue);
  mqtt.print(", \"light\": ");
  mqtt.print(lightValue);
  mqtt.print(", \"timestamp\": ");
  mqtt.print(millis());
  mqtt.println("}");
  mqtt.endMessage();
}

// loop functions

void updateLeds() {
  activityLed.update();
  standbyLed.update();

  if (currentState == meditating && !isPaused &&
      standbyLed.is_fading() == false) {
    // while operating normally the green light pulses gently.
    if (standbyLed.get_value() == 255) {
      standbyLed.fade(50, 3000);
    } else {
      standbyLed.fade(255, 3000);
    }
  } else if (isPaused && activityLed.is_fading() == false) {
    // when timer is paused the orange activity lights are blinking.
    if (activityLed.get_value() == 255) {
      activityLed.fade(10, 500);
    } else {
      activityLed.fade(255, 500);
    }
  }
}

void readCapacitiveTouch() {
  bool value = bowlTouch.read();

  // DEBUG CAPACITANCE
  // int ref = bowlTouch.readReference();
  // int raw = bowlTouch.readRaw();
  // Serial.print("ref/raw: ");
  // Serial.print(ref);
  // Serial.print(" ");
  // Serial.print(raw);
  // Serial.print("cap: ");
  // Serial.println(value);

  int touchDuration = (isTouching ? millis() - touchStartTime : -1);
  if (value) {
    if (!isTouching) {
      // touch began
      touchStartTime = millis();
      isTouching = true;
    }

    if (touchDuration > longTouchDuration && currentState != idle) {
      Serial.println("Cancelling meditation");
      onLongTouch();
    }
  } else {
    if (isTouching) {
      // stopped touching
      isTouching = false;
      if (touchDuration < longTouchDuration &&
          touchDuration > shortTouchDuration) {
        Serial.println("Starting/Pausing meditation");
        onShortTouch();
      }
    }
  }
}

int roundDiv(int a, int b) { return round((float)a / (float)b); }

void readDurations() {
  // read analog pins and scale values.
  int total = map(analogRead(totalDurationPin), 0, 1023, minTotalDuration,
                  maxTotalDuration);
  int interval = map(analogRead(intervalDurationPin), 0, 1023,
                     minIntervalDuration, maxIntervalDuration);
  int prepReading = analogRead(preparationDurationPin);
  int preparation =
      map(prepReading, 0, 1023, minPreparationDuration, maxPreparationDuration);

  Serial.print(total);
  Serial.print(" ");
  Serial.print(interval);
  Serial.print(" ");
  Serial.println(preparation);
  Serial.println(prepReading);
  Serial.print("\n");
  // reduce durations resolution.

  if (total < 10 * 60) {
    // 1 minute jumps
    totalDuration = roundDiv(total, 60) * 60;
  } else {
    // 5 minute jumps
    totalDuration = roundDiv(total, 300) * 300;
  }

  if (interval < 10 * 60) {
    // 1 minute jumps
    intervalDuration = roundDiv(interval, 60) * 60;
  } else {
    // 5 minute jumps
    intervalDuration = roundDiv(interval, 300) * 300;
  }

  if (preparation < 10) {
    // 1 second jumps
    preparationDuration = preparation;
  } else {
    // 5 second jumps
    preparationDuration = roundDiv(preparation, 5) * 5;
  }

  Serial.print("Configuration: total - ");
  Serial.print(totalDuration / 60);
  Serial.print(" mins; interval - ");
  Serial.print(intervalDuration / 60);
  Serial.print(" mins; preparation - ");
  Serial.print(preparationDuration);
  Serial.println(" secs.");
}

// MAIN HOOKS

void setup() {
  // hardware setup
  pinMode(solenoidPin, OUTPUT);
  pinMode(activityLedPin, OUTPUT);
  pinMode(standbyLedPin, OUTPUT);

  // initialize capacitive sensor
  bowlTouch.begin(touchPin, bowlTouchThreshold);
  bowlTouch.applyPinSettings(touchSettings);
  TouchSensor::start(); // TODO: is this good or better to use startSingle?

  Serial.begin(115200);
  Serial.println("Meditation Timer");
}

void loop() {
  updateLeds();
  readCapacitiveTouch();

  Alarm.delay(10);
}
