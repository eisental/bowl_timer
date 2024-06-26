#include "LEDFader.h"
#include "secrets.h"
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <R4_Touch.h>
#include <TimeAlarms.h>
#include <WiFiS3.h>
/*
  meditation-timer v2 - june, 2024
  by Tal Eisenberg (https://www.github.com/eisental)
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

int curPreparationMeasurement = -1;
int curIntervalMeasurement = -1;
int curTotalMeasurement = -1;

void setDurationsFromKnobs();
void meditate();
void fadeOutStandbyLed();
void fadeOutActivityLed();
void startInterval(int intervalDuration, int strikeDuration);
void chime(int duration);
void reset();
void disableAlarms();

// state machine

void prepare() {

  Serial.print("Starting timer: total - ");
  Serial.print(totalDuration / 60);
  Serial.print(" mins; interval - ");
  Serial.print(intervalDuration / 60);
  Serial.print(" mins; preparation - ");
  Serial.print(preparationDuration);
  Serial.println(" secs.");

  currentState = preparing;
  currentIntervalAlarm = Alarm.timerOnce(preparationDuration, meditate);
  activityLed.fade(255, preparationDuration * 1000);
  standbyLed.fade(255, 1000);
  currentActivityAlarm = Alarm.timerOnce(1, fadeOutStandbyLed);

  char payload[100];
  sprintf(payload,
          "{\"state\":\"preparing\", \"totalDuration\":%d, "
          "\"intervalDuration\":%d, \"preparationDuration\":%d, "
          "\"reason\":\"start\"}",
          totalDuration, intervalDuration, preparationDuration);
  publishMQTT(payload);
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
  publishMQTT("{\"state\":\"idle\", \"reason\":\"end\"}");
}

void pause() {
  isPaused = true;
  disableAlarms();

  int finishedIntervalDuration = (millis() - intervalStartTime) / 1000;
  remainingIntervalDuration =
      currentIntervalDuration - finishedIntervalDuration;
  remainingTotalDuration -=
      finishedIntervalDuration; // remove partial interval from remaining total.

  // turn off standby led while paused.
  standbyLed.fade(0, 200);

  publishMQTT(
      "{\"state\":\"meditating\", \"paused\":true, \"reason\":\"pause\"}");

  // DEBUG
  Serial.print("PAUSED | remaining: ");
  Serial.print(remainingIntervalDuration);
  Serial.print(" currentIntervalDuration: ");
  Serial.println(currentIntervalDuration);
  // END DEBUG
}

void unpause() {
  isPaused = false;
  activityLed.fade(0, 200);
  startInterval(remainingIntervalDuration, 0);
  publishMQTT(
      "{\"state\":\"meditating\", \"paused\":false, \"reason\":\"unpause\"}");
}

void cancel() {
  Serial.println("Cancelling meditation");
  standbyLed.fade(0, 500);
  activityLed.fade(0, 500);
  reset();
  publishMQTT("{\"state\":\"idle\", \"reason\":\"cancel\"}");
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
    setDurationsFromKnobs();
    prepare();
    break;
  case preparing:
    cancel();
    break;
  case meditating:
    if (!isPaused) {
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

  char msg[256];
  sprintf(msg,
          "{\"state\":\"meditating\", \"remaining\":%d, "
          "\"reason\":\"startInterval\"}",
          remainingTotalDuration);
  publishMQTT(msg);

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

char publish_topic[] = "bowl_timer/status";
char subscribe_topic[] = "bowl_timer/command";

WiFiClient wifi;
MqttClient mqtt(wifi);

void onMQTTMessage(int messageSize) {
  if (mqtt.available() > 0) {
    StaticJsonDocument<2048> json;
    DeserializationError error = deserializeJson(json, mqtt);

    switch (error.code()) {
    case DeserializationError::Ok:
      break;
    case DeserializationError::EmptyInput:
      return;
    case DeserializationError::IncompleteInput:
    case DeserializationError::InvalidInput:
    case DeserializationError::NoMemory:
    default:
      publishError(error.c_str());
      return;
    }

    if (json.is<JsonArray>()) {
      runCommand(json.as<JsonArray>());
    } else {
      publishError("Expecting json root to be an array or an object.");
    }
  }
}

void publishError(const char *msg) {
  char err[256];
  sprintf(err, "{\"error\": \"%s\"}", msg);
  publishMQTT(err);
}

void runCommand(JsonArray c) {
  if (c.size() < 1) {
    publishError("Expecting at least one command.");
    return;
  }

  if (!c[0].is<const char *>()) {
    publishError("Invalid command name");
    return;
  }

  if (c[0] == "chime") {
    if (c.size() > 1) {
      if (!c[1].is<int>()) {
        publishError("Invalid chime duration");
        return;
      }

      char msg[256];
      sprintf(msg, "Chiming for %d ms.", (int)c[1]);
      Serial.println(msg);
      chime(c[1]);
    } else {
      Serial.println("Chiming default duration.");
      chime(longChimeStrike);
    }
  } else if (c[0] == "start") {
    if (currentState != idle) {
      publishError("Timer is already running.");
      return;
    }

    if (c.size() == 1) {
      // use knobs for durations
      setDurationsFromKnobs();
      prepare();
      return;
    }

    if (c.size() < 4) {
      publishError("Expecting start command to have three arguments");
      return;
    }

    if (!c[1].is<int>() || !c[2].is<int>() || !c[3].is<int>()) {
      publishError("Invalid start command arguments");
      return;
    }

    totalDuration = c[1];
    intervalDuration = c[2];
    preparationDuration = c[3];
    prepare();
  } else if (c[0] == "pause") {
    if (c.size() < 2) {
      publishError("Expecting pause command to have one argument");
      return;
    }

    if (currentState != meditating) {
      publishError("Timer is not currently running.");
      return;
    }

    if (!c[1].is<bool>()) {
      publishError("Invalid pause command arguments");
      return;
    }

    if (c[1]) {
      pause();
    } else {
      unpause();
    }
  } else if (c[0] == "cancel") {
    if (currentState == idle) {
      publishError("Timer is not currently running.");
      return;
    }

    cancel();
  } else {
    publishError("Unknown command");
  }
}

void setupMQTT() {
  Serial.print("Connecting to WiFi ");
  Serial.println(wifiSSID);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi module!");
    return;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the WiFi firmware");
  }

  while (WiFi.begin(wifiSSID, wifiPass) != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
  } else {
    Serial.println("No WiFi!");
    return;
  }

  delay(500);
  Serial.println("Connecting to MQTT:");
  Serial.print(mqttHost);
  Serial.print(":");
  Serial.println(mqttPort);

  if (!mqtt.connect(mqttHost, mqttPort)) {
    char err[80];
    sprintf(err, "MQTT connect failed. Error code: %ld", mqtt.connectError());
    Serial.println(err);
    return;
  }

  Serial.println("Connected to MQTT!\n");
  publishMQTT("{\"state\":\"online\"}");

  mqtt.onMessage(onMQTTMessage);
  mqtt.subscribe(subscribe_topic);
}

void publishMQTT(char message[]) {
  if (!mqtt.connected()) {
    return;
  }
  mqtt.beginMessage(publish_topic);
  mqtt.print(message);
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

void setDurationsFromKnobs() {
  // read analog pins and scale values.
  int total = constrain(
      map(curTotalMeasurement, 980, 0, minTotalDuration, maxTotalDuration),
      minTotalDuration, maxTotalDuration);

  int interval = constrain(map(curIntervalMeasurement, 920, 0,
                               minIntervalDuration, maxIntervalDuration),
                           minIntervalDuration, maxIntervalDuration);

  int preparation =
      constrain(map(curPreparationMeasurement, 1023, 0, minPreparationDuration,
                    maxPreparationDuration),
                minPreparationDuration, maxPreparationDuration);

  Serial.print(total);
  Serial.print(" ");
  Serial.print(interval);
  Serial.print(" ");
  Serial.println(preparation);

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
}

void readKnobs() {
  curTotalMeasurement = analogRead(totalDurationPin);
  curIntervalMeasurement = analogRead(intervalDurationPin);
  curPreparationMeasurement = analogRead(preparationDurationPin);
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

  // initialize serial
  Serial.begin(115200);
  Serial.println("Meditation Timer");

  // initialize wifi + mqtt
  setupMQTT();
}

void loop() {
  updateLeds();
  readKnobs();
  readCapacitiveTouch();
  mqtt.poll();
  Alarm.delay(10);
}
