#include <AccelStepper.h>
#include <Arduino.h>
#include <NeoPixelBus.h>
#include <debouncer.h>
#include <esp32_wifi/wifi.h>
#include <mqtt_client.h>
#include <rotary.h>
#include <seven_segment.h>

#include "constants.h"
#include "pins.h"

#define COUNTDOWN_INTERVAL 60 * 1000
#define ENCODER_START_VALUE 10
#define MINIMUM_TIME_PER_INTERVAL 5 * 1000

#define MQTT_SERVER "192.168.68.106"
#define MQTT_PORT 1883
#define MQTT_CLIENT "Mama Timer"
#define MQTT_TOPIC "mama-timer/remaining"

#define STATE_IDLE 1
#define STATE_RUNNING 2
#define STATE_ALARM 3

int state;
EventGroupHandle_t timerGroup;
ESP32Wifi network;
esp_mqtt_client_handle_t mqttHandle;
AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_BLUE_PIN, STEPPER_YELLOW_PIN, STEPPER_PINK_PIN, STEPPER_ORANGE_PIN);

Rotary encoder(ENCODER_CLK, ENCODER_DT);
SevenSegment display;
int remaining;
int encoderValue;
bool canStop;

NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(1, 18);

/*****************************************************************************/
#pragma region Motor

#define STEPS_PER_CIRCLE 2048

TaskHandle_t motorTaskHandle;

void motorTask(void *pparams) {
  while (1) {
    // Wait for us to be started.
    ulTaskNotifyTake(CLEAR_ON_EXIT, portMAX_DELAY);

    // Take the first step.
    stepper.run();

    // Run until we reach the desired position.
    while (1) {
      stepper.run();
      if (!stepper.isRunning()) {
        vTaskDelay(200);
        stepper.disableOutputs();
        xEventGroupSetBits(timerGroup, MAMA_MOVED);
        break;
      }
    }
  }
}

// 'postition' is 0 to 60 (full circle) representing minutes.
void setPosition(int position) {
  int target = -position * STEPS_PER_CIRCLE / 60;
  stepper.moveTo(target);
}

#pragma endregion

/*****************************************************************************/
#pragma region Alarm

esp_timer_handle_t alarmHandle;
bool alarmSounding;

void alarmCallback(void *pargs) {
  if (alarmSounding) {
    gpio_set_level((gpio_num_t)BUZZER_PIN, 0);
    gpio_set_level((gpio_num_t)RUNNING_PIN, 0);
    alarmSounding = false;
  } else {
    gpio_set_level((gpio_num_t)BUZZER_PIN, 1);
    gpio_set_level((gpio_num_t)RUNNING_PIN, 1);
    alarmSounding = true;
  }
}

void alarmOn() {
  esp_timer_create_args_t timerArgs;

  memset(&timerArgs, 0, sizeof timerArgs);
  timerArgs.callback = alarmCallback;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.skip_unhandled_events = 1;
  esp_timer_create(&timerArgs, &alarmHandle);
  esp_timer_start_periodic(alarmHandle, 500000);

  gpio_set_level((gpio_num_t)BUZZER_PIN, 1);
  gpio_set_level((gpio_num_t)RUNNING_PIN, 1);
  alarmSounding = true;
}

void alarmOff() {
  gpio_set_level((gpio_num_t)BUZZER_PIN, 0);
  gpio_set_level((gpio_num_t)RUNNING_PIN, 0);

  if (alarmHandle) {
    esp_timer_stop(alarmHandle);
    esp_timer_delete(alarmHandle);
    alarmHandle = 0;
  }
}

#pragma endregion

/*****************************************************************************/
#pragma region Countdown

esp_timer_handle_t countdownHandle;
TaskHandle_t countdownTaskHandle, minTimeHandle, mqttTaskHandle;

void mqttTask(void *pparams) {
  char s[16];

  while (1) {
    ulTaskNotifyTake(CLEAR_ON_EXIT, portMAX_DELAY);
    // Connect to WiFi and send an MQTT message with the remaining time.
    network.connect(WIFI_CONNECTED);
    xEventGroupWaitBits(timerGroup, WIFI_CONNECTED, NO_CLEAR, WAIT_ALL, pdMS_TO_TICKS(10000));
    if (network.isConnected) {
      itoa(remaining, s, 10);
      esp_mqtt_client_start(mqttHandle);  // Connects to broker
      esp_mqtt_client_publish(mqttHandle, MQTT_TOPIC, s, strlen(s), 1, 1);
      vTaskDelay(500);
    }

    xEventGroupSetBits(timerGroup, MQTT_SENT);
  }
}

void minTimeTask(void *pparams) {
  while (1) {
    ulTaskNotifyTake(CLEAR_ON_EXIT, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(MINIMUM_TIME_PER_INTERVAL));
    xEventGroupSetBits(timerGroup, MIN_TIME_ELAPSED);
  }
}

void countdownTask(void *pparam) {
  unsigned long started;

  while (1) {
    // Wait to start a new countdown.
    ulTaskNotifyTake(CLEAR_ON_EXIT, portMAX_DELAY);

    // Each time round the loop is one interval.
    remaining = encoderValue;

    while (1) {
      canStop = false;
      started = millis();

      // Set up to show the current values.
      display.setValue(remaining);
      setPosition(remaining);

      // Start the process of showing the values, and also start the minimum time task.
      xEventGroupClearBits(timerGroup, MAMA_MOVED | MIN_TIME_ELAPSED | MQTT_SENT);
      xTaskNotifyGive(motorTaskHandle);
      xTaskNotifyGive(mqttTaskHandle);

      if (remaining == 0) {
        gpio_set_level((gpio_num_t)RUNNING_PIN, 0);
        alarmOn();
        state = STATE_ALARM;
        break;
      }

      xTaskNotifyGive(minTimeHandle);

      // If we stop the countdown during any of the task waits then we should stop waiting and break immediately.
      // Tricky, maybe just disable it during this phase.
      // Or maybe pressing encoder during run cancels it and returns to startup conditions (actual restart?).

      // Wait for everything related to the interval processing to complete.
      xEventGroupWaitBits(timerGroup, MAMA_MOVED | MIN_TIME_ELAPSED | MQTT_SENT, CLEAR_ON_EXIT, WAIT_ALL, pdMS_TO_TICKS(20000));
      canStop = true;

      // Sleep until the next interval. Blank the display as otherwise random stuff appears when the multiplexer stops.
      display.blank();
      esp_sleep_enable_timer_wakeup((COUNTDOWN_INTERVAL - (millis() - started)) * 1000);
      gpio_wakeup_enable(gpio_num_t(ENCODER_BUTTON), GPIO_INTR_LOW_LEVEL);
      esp_sleep_enable_gpio_wakeup();
      esp_light_sleep_start();

      if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
        break;
      }

      remaining--;
    }
  }
}

void startCountdown() {
  gpio_set_level((gpio_num_t)RUNNING_PIN, 1);
  xTaskNotifyGive(countdownTaskHandle);
}

void stopCountdown() {
  gpio_set_level((gpio_num_t)RUNNING_PIN, 0);
}

#pragma endregion

int getEncoder() {
  return gpio_get_level((gpio_num_t)ENCODER_BUTTON) == 0;
}

void encoderChanged(int value) {
  if (!value) {
    return;
  }

  switch (state) {
    case STATE_IDLE:
      if (encoderValue > 0) {
        startCountdown();
        state = STATE_RUNNING;
      }
      break;

    case STATE_RUNNING:
      if (canStop) {
        stopCountdown();
        encoderValue = remaining;
        display.setValue(encoderValue);
        state = STATE_IDLE;
      }
      break;

    case STATE_ALARM:
      alarmOff();
      encoderValue = ENCODER_START_VALUE;
      display.setValue(encoderValue);
      state = STATE_IDLE;
  }
}

Debouncer encoderDebouncer(getEncoder, encoderChanged, 100);

void encoderTask(void *pparams) {
  int direction;

  while (1) {
    encoderDebouncer.loop();

    if (state != STATE_IDLE) {
      continue;
    }

    switch ((direction = encoder.getDirection())) {
      case 1:
        if (encoderValue < 60) {
          display.setValue(++encoderValue);
        }
        break;

      case -1:
        if (encoderValue > 1) {
          display.setValue(--encoderValue);
        }
        break;
    }
  }
}

void setup() {
  esp_mqtt_client_config_t mqtt_config;

  strip.Begin();
  strip.Show();

  Serial.begin(115200);
  Serial.println("Starting");

  timerGroup = xEventGroupCreate();
  esp_event_loop_create_default();

  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("Buzzer initialised");

  digitalWrite(RUNNING_PIN, LOW);
  pinMode(RUNNING_PIN, OUTPUT);
  Serial.println("LED initialised");

  xTaskCreatePinnedToCore(minTimeTask, "min-time", 400, NULL, 1, &minTimeHandle, 1);
  Serial.println("Minimum time initialised");

  xTaskCreatePinnedToCore(countdownTask, "countdown", 1024, NULL, 1, &countdownTaskHandle, 1);
  Serial.println("Countdown initialised");

  network.init(timerGroup, WIFI_STARTED);
  xEventGroupWaitBits(timerGroup, WIFI_STARTED, NO_CLEAR, WAIT_ALL, pdMS_TO_TICKS(10000));
  Serial.println("WiFi initialised");

  memset(&mqtt_config, 0, sizeof mqtt_config);
  mqtt_config.host = MQTT_SERVER;
  mqtt_config.port = MQTT_PORT;
  mqtt_config.client_id = MQTT_CLIENT;
  mqttHandle = esp_mqtt_client_init(&mqtt_config);
  xTaskCreatePinnedToCore(mqttTask, "mqtt", 2048, NULL, 1, &mqttTaskHandle, 1);
  Serial.println("MQTT initialised");

  display.setDigitPins(2, DIGIT_1_PIN, DIGIT_2_PIN);
  display.setSegmentPins(SEGMENT_A_PIN, SEGMENT_B_PIN, SEGMENT_C_PIN, SEGMENT_D_PIN, SEGMENT_E_PIN, SEGMENT_F_PIN, SEGMENT_G_PIN);
  display.begin();
  Serial.println("Display initialised");

  encoderValue = ENCODER_START_VALUE;
  display.setValue(encoderValue);
  state = STATE_IDLE;
  Serial.println("State initialised");

  stepper.setMaxSpeed(500.0);
  stepper.setAcceleration(1000.0);
  xTaskCreatePinnedToCore(motorTask, "motor", 2048, NULL, 1, &motorTaskHandle, 1);
  Serial.println("Motor initialised");

  pinMode(ENCODER_BUTTON, INPUT);
  encoder.begin();
  xTaskCreatePinnedToCore(encoderTask, "encoder", 2048, NULL, 1, NULL, 1);
  Serial.println("Encoder initialised");
}

void loop() {
  taskYIELD();
  return;
}
