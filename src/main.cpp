#include <Arduino.h>
#include <esp_now.h>
#include "WiFi.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- ESP-NOW broadcast address ---
const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Message struct for receiving: packed, single byte 'dance'
typedef struct __attribute__((packed)) {
  uint8_t dance; // 1 = dancing, 0 = stopped
} dance_msg_t;

// Message struct for sending: packed with discriminator byte so other ESP32s ignore it
// The discriminator (0xFF) is not valid for dance_msg_t, which only expects 1 byte (the dance value)
typedef struct __attribute__((packed)) {
  uint8_t discriminator;   // 0xFF: identifies this as a button-triggered message to ignore
  uint8_t dance_trigger;   // 1 = trigger dance, 0 = stop dance
} dance_trigger_msg_t;

// Global dance state updated by ESP-NOW callback
volatile bool isDancing = true;
// When a stop message is received, keep dancing until this deadline (ms since boot).
volatile unsigned long danceKeepUntil = 0;
// When a start message is received, schedule dancing to begin at this time (ms since boot).
volatile unsigned long danceStartAt = 0;

// Send callback for debug
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW receive callback
void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(dance_msg_t)) {
    Serial.printf("Received unexpected len %d (expected %d)\n", len, (int)sizeof(dance_msg_t));
    return;
  }

  dance_msg_t msg;
  memcpy(&msg, incomingData, sizeof(msg));
  bool dancing = (msg.dance != 0);
  // update main loop state
  unsigned long now = millis();
  // determine whether we are currently active (dancing or in keep-alive or started)
  bool currentlyActive = (isDancing || (danceKeepUntil && now < danceKeepUntil) || (danceStartAt && now >= danceStartAt));
  isDancing = dancing;
  if (dancing) {
    // cancel any pending keep-alive when start dancing
    danceKeepUntil = 0;
    // schedule start only if we're not already active
    if (!currentlyActive) {
      danceStartAt = now + 5000UL; // start after 5 seconds
    }
  } else {
    // cancel any scheduled start and keep dancing for 7000 ms after receiving stop
    danceStartAt = 0;
    danceKeepUntil = now + 7000UL;
  }

  Serial.printf("Received dance=%u from %02X:%02X:%02X:%02X:%02X:%02X\n",
                msg.dance,
                mac_addr[0], mac_addr[1], mac_addr[2],
                mac_addr[3], mac_addr[4], mac_addr[5]);
}

// Use Adafruit PCA9685 driver for servo control
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Toggle between random phase offsets and a small fixed offset pattern
bool useRandomPhase = true; // set to true for random, false for small offset pattern

// Button pin
const int BUTTON_PIN = 34;  // GPIO 34 for button input

// Built-in LED pin (on ESP32 boards, typically GPIO 2)
const int LED_PIN = LED_BUILTIN;

// Support servos with per-servo phase offsets
const int NUM_SERVOS = 2;
float gPhaseOffsets[NUM_SERVOS];
int gRestAngles[NUM_SERVOS] = {0, 180};
int gCurAngles[NUM_SERVOS];
int gChannels[NUM_SERVOS] = {0, 1};

// Convert angle (0..180) to PCA9685 ticks for given frequency (default 50Hz)
uint16_t angleToTick(uint8_t angle, float freq = 50.0f) {
  if (angle > 180) angle = 180;
  const float minPulse = 500.0f;   // microseconds
  const float maxPulse = 2500.0f;  // microseconds
  float pulse = minPulse + (maxPulse - minPulse) * (angle / 180.0f);
  float ticks = pulse * (freq * 4096.0f) / 1e6f;
  if (ticks > 4095.0f) ticks = 4095.0f;
  return (uint16_t)ticks;
}

// Track last sent angle per channel to avoid unnecessary updates
static uint8_t lastSentAngles[16] = {0xFF}; // initialize to invalid value

void servoWriteAdafruit(uint8_t channel, uint8_t angle) {
  // Only send update if angle has changed
  if (lastSentAngles[channel] != angle) {
    uint16_t tick = angleToTick(angle);
    pwm.setPWM(channel, 0, tick);
    lastSentAngles[channel] = angle;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Configure button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Configure built-in LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Put WiFi in station mode and disconnect from any AP to start ESP-NOW cleanly
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // register a broadcast peer so we can send to all
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add broadcast peer");
    }
  }

  // Initialize I2C and Adafruit PCA9685 for servos
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // servos expect ~50Hz

  // Initialize phase offsets for NUM_SERVOS (random or small fixed offset pattern)
  if (useRandomPhase) {
#if defined(ARDUINO_ARCH_ESP32)
    randomSeed((uint32_t)esp_random());
#endif
    for (int i = 0; i < NUM_SERVOS; ++i) {
      int deg = random(0, 360); // random phase in degrees
      gPhaseOffsets[i] = deg * 3.14159265f / 180.0f;
    }
  } else {
    // Use small fixed offset phase pattern (e.g., 0, 30, 60, 90, 120 degrees)
    int smallOffsets[5] = {0, 30, 60, 90, 120};
    for (int i = 0; i < NUM_SERVOS; ++i) {
      gPhaseOffsets[i] = smallOffsets[i] * 3.14159265f / 180.0f;
    }
  }
  for (int i = 0; i < NUM_SERVOS; ++i) {
    gCurAngles[i] = gRestAngles[i];
    // write rest positions using mapped channel
    servoWriteAdafruit(gChannels[i], gRestAngles[i]);
  }
}

void loop() {
  // Oscillate servos smoothly between 70 and 100 while `isDancing` is true.
  static unsigned long lastUpdate = 0;
  const unsigned long intervalMs = 50; // ms between small steps (controls speed)
  const int minAngle = 80;
  const int maxAngle = 110;

  static float phase = 0.0f;         // time-like phase for sine oscillator
  const float phaseStep = 0.06f;     // controls speed (radians per update)
  const float twoPi = 6.28318530718f;

  // sine-based oscillator parameters
  const float center = (minAngle + maxAngle) / 2.0f; // midpoint
  const float amplitude = (maxAngle - minAngle) / 2.0f; // half-range

  static bool wasDancing = false;
  static bool inTransition = false;
  unsigned long now = millis();

  // Active dancing state: true when a start message was received and any
  // start delay has elapsed, or for `danceKeepUntil` milliseconds after a stop message.
  // Clear expired keep-alive and clear scheduled start when reached.
  if (danceStartAt && now >= danceStartAt) {
    // start time reached; clear schedule so we treat isDancing as active
    danceStartAt = 0;
  }
  bool active = (isDancing && (danceStartAt == 0)) || (danceKeepUntil && now < danceKeepUntil);
  // clear expired keep-alive
  if (!isDancing && danceKeepUntil && now >= danceKeepUntil) danceKeepUntil = 0;

  if (active) {
    if (!wasDancing) {
      wasDancing = true;
      inTransition = true;
      phase = 0.0f;
      // initialize current angles to rest positions so we ramp from there
      for (int i = 0; i < NUM_SERVOS; ++i) gCurAngles[i] = gRestAngles[i];
      Serial.println("Dance started: transitioning to oscillation (smooth)");
    }

    if (now - lastUpdate >= intervalMs) {
      lastUpdate = now;
      phase += phaseStep;
      if (phase >= twoPi) phase -= twoPi;

      // compute targets for all servos using their per-servo phase offsets
      int targets[NUM_SERVOS];
      for (int i = 0; i < NUM_SERVOS; ++i) {
        float s = sinf(phase + gPhaseOffsets[i]);
        targets[i] = (int)(center + amplitude * s + 0.5f);
      }

      if (inTransition) {
        int maxDelta = (int)(amplitude * phaseStep + 0.5f);
        if (maxDelta < 1) maxDelta = 1;
        bool allClose = true;
        for (int i = 0; i < NUM_SERVOS; ++i) {
          int diff = targets[i] - gCurAngles[i];
          int step = 0;
          if (diff > 0) step = min(diff, maxDelta);
          else if (diff < 0) step = max(diff, -maxDelta);
          gCurAngles[i] += step;
          servoWriteAdafruit(gChannels[i], (uint8_t)gCurAngles[i]);
          if (abs(targets[i] - gCurAngles[i]) > 1) allClose = false;
        }
        if (allClose) inTransition = false;
      } else {
        for (int i = 0; i < NUM_SERVOS; ++i) servoWriteAdafruit(gChannels[i], (uint8_t)targets[i]);
      }
    }
  } else {
    if (wasDancing) {
      wasDancing = false;
      inTransition = true; // use transition to smoothly ramp down to rest
      Serial.println("Dance stopped: transitioning to rest (smooth)");
    }

    if (inTransition && now - lastUpdate >= intervalMs) {
      lastUpdate = now;
      int maxDelta = (int)(amplitude * phaseStep + 0.5f);
      if (maxDelta < 1) maxDelta = 1;
      bool allAtRest = true;
      for (int i = 0; i < NUM_SERVOS; ++i) {
        int rest = gRestAngles[i];
        int diff = rest - gCurAngles[i];
        int step = 0;
        if (diff > 0) step = min(diff, maxDelta);
        else if (diff < 0) step = max(diff, -maxDelta);
        gCurAngles[i] += step;
        servoWriteAdafruit(gChannels[i], (uint8_t)gCurAngles[i]);
        if (gCurAngles[i] != rest) allAtRest = false;
      }
      if (allAtRest) inTransition = false;
    }
  }

  // Handle button press to send ESP-NOW dance message when not currently dancing
  static unsigned long lastButtonCheck = 0;
  const unsigned long debounceMs = 50;
  if (now - lastButtonCheck >= debounceMs) {
    lastButtonCheck = now;
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW); // button active low
    static bool wasPressed = false;

    if (buttonPressed && !wasPressed && !active) {
      // Button just pressed and not currently dancing: send trigger message
      wasPressed = true;
      digitalWrite(LED_PIN, HIGH);  // turn on LED
      dance_trigger_msg_t msg;
      msg.discriminator = 0xFF;     // identifies as button-triggered, not a dance command
      msg.dance_trigger = 1;        // trigger dance
      esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
      if (res != ESP_OK) {
        Serial.print("esp_now_send failed: ");
        Serial.println(res);
      } else {
        Serial.print("Sent dance state: ");
        Serial.println(msg.dance_trigger ? "true" : "false");
      }
    } else if (!buttonPressed) {
      wasPressed = false;
      digitalWrite(LED_PIN, LOW);   // turn off LED
    }
  }

  // Let callbacks and other background tasks run
  delay(10);
}