// ============================================================
// UNO LASER CONTROLLER
//
// Mega TX2 pin 16 -> UNO D10
// UNO D11 -> Mega RX2 pin 17
//
// Encoder A -> D2
// Encoder B -> D3
//
// Laser TTL/PWM -> D5
//
// Fire switch -> D6 to GND
// Arm switch  -> D7 to GND
//
// Pot wiper -> A0
//
// Uses:
// - Manual fire
// - 360 fire
// - Aim mode
// - Pot power control
// ============================================================

#include <SoftwareSerial.h>

SoftwareSerial linkSerial(10, 11); // RX, TX

// ---------------- Encoder ----------------
#define ENC_A 2
#define ENC_B 3

volatile long encoderCount = 0;

// ---------------- Pins ----------------
const int laserPwmPin = 5;
const int firePin = 6;
const int armPin = 7;
const int potPin = A0;


// ---------------- Laser ----------------
bool laserEnabled = false;
bool laserOn = false;
bool firing360 = false;
bool aimMode = false;

int powerLevel = 0;
int aimPower = 8;

// ---------------- Encoder ----------------
const long COUNTS_PER_REV = 1007;

// ---------------- Modes ----------------
enum FireMode {
  MODE_360,
  MODE_MANUAL
};

FireMode fireMode = MODE_MANUAL;

// ---------------- Fire tracking ----------------
long startCount = 0;
long lastEncoderCount = 0;

unsigned long fireStartMillis = 0;
unsigned long lastMoveMillis = 0;

const unsigned long noMovementTimeout = 8000;
const unsigned long maxFireTime = 100000;

// ---------------- Serial ----------------
String cmd = "";

// ============================================================
// Encoder ISR
// ============================================================
void readEncoder() {

  if (digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

// ============================================================
// Setup
// ============================================================
void setup() {

  Serial.begin(115200);
  linkSerial.begin(9600);

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    readEncoder,
    RISING
  );

  // Laser output
  pinMode(laserPwmPin, OUTPUT);
  analogWrite(laserPwmPin, 0);

  // Switches
  pinMode(firePin, INPUT_PULLUP);
  pinMode(armPin, INPUT_PULLUP);

  Serial.println("UNO LASER CONTROLLER STARTED");

  reportToMega("UNO READY");
}

// ============================================================
// Main loop
// ============================================================
void loop() {

  readCommands();

  readPot();

  handleFireButton();

  service360();
}

// ============================================================
// Send status to Mega
// ============================================================
void reportToMega(const String &msg) {

  linkSerial.println(msg);

  Serial.println(msg);
}

// ============================================================
// Read commands from Mega
// ============================================================
void readCommands() {

  while (linkSerial.available()) {

    char c = linkSerial.read();

    if (c == '\n' || c == '\r') {

      cmd.trim();

      if (cmd.length() > 0) {

        Serial.print("CMD:");
        Serial.println(cmd);

        handleCommand(cmd);

        cmd = "";
      }
    }
    else {
      cmd += c;
    }
  }
}

// ============================================================
// Handle commands
// ============================================================
void handleCommand(String c) {

  c.trim();

  if (c == "LON") {
  if (laserEnabled && digitalRead(armPin) == LOW) {
    aimMode = false;
    laserOnFunc();
    reportToMega("LASER ON");
  } else {
    reportToMega("NOT ARMED");
  }
  return;
}

  // ---------------- Aim low ----------------

  if (c == "LLOW") {

    if (!laserEnabled) {

      aimMode = true;

      analogWrite(laserPwmPin, aimPower);

      laserOn = true;

      reportToMega("AIM LOW");
    }

    return;
  }

  // ---------------- Ready ----------------

  if (c == "LREADY") {

    stopLaserAll();

    reportToMega("READY");

    return;
  }

  // ---------------- Arm toggle ----------------

  if (c == "LSTATE") {

    laserEnabled = !laserEnabled;

    stopLaserAll();

    if (laserEnabled)
      reportToMega("ARMED");
    else
      reportToMega("SAFE");

    return;
  }

  // ---------------- Mode select ----------------

  if (c == "L360") {

    fireMode = MODE_360;

    reportToMega("MODE 360");

    return;
  }

  if (c == "LMAN") {

    fireMode = MODE_MANUAL;

    reportToMega("MODE MANUAL");

    return;
  }

  // ---------------- Off ----------------

  if (c == "LOFF" || c == "STOP") {

    stopLaserAll();

    reportToMega("LASER OFF");

    return;
  }

  // ---------------- Manual fire ----------------

  if (c == "FIRE") {

    handleFirePress();

    return;
  }

  // ---------------- Start 360 ----------------

  if (c == "BURN1") {

    start360Fire();

    return;
  }
}

// ============================================================
// Potentiometer
// ============================================================
void readPot() {

  int raw = analogRead(potPin);

  powerLevel = map(raw, 0, 1023, 0, 255);

  powerLevel = constrain(powerLevel, 0, 255);

  if (laserOn && !aimMode && !firing360) {

    analogWrite(laserPwmPin, powerLevel);
  }

  static int lastPercent = -1;

  int percent = map(powerLevel, 0, 255, 0, 100);

  if (abs(percent - lastPercent) >= 10) {

    reportToMega("POT:" + String(percent));

    lastPercent = percent;
  }
}

// ============================================================
// Fire button
// ============================================================
void handleFireButton() {

  static bool lastStableState = HIGH;
  static bool lastReading = HIGH;
  static unsigned long lastChangeTime = 0;

  const unsigned long debounceMs = 40;

  bool reading = digitalRead(firePin);

  if (reading != lastReading) {

    lastChangeTime = millis();

    lastReading = reading;
  }

  if ((millis() - lastChangeTime) > debounceMs) {

    if (reading != lastStableState) {

      lastStableState = reading;

      if (lastStableState == LOW) {

        handleFirePress();
      }
    }
  }
}

// ============================================================
// Fire action
// ============================================================
void handleFirePress() {

  if (!laserEnabled) {

    reportToMega("NOT ENABLED");

    return;
  }

  if (digitalRead(armPin) != LOW) {

    reportToMega("NOT ARMED");

    return;
  }

  if (fireMode == MODE_360) {

    start360Fire();
  }
  else {

    if (laserOn) {

      laserOffFunc();

      reportToMega("MAN OFF");
    }
    else {

      aimMode = false;

      laserOnFunc();

      reportToMega("MAN ON");
    }
  }
}

// ============================================================
// Start 360 fire
// ============================================================
void start360Fire() {

  if (firing360)
    return;

  if (!laserEnabled) {

    reportToMega("NOT ENABLED");

    return;
  }

  if (digitalRead(armPin) != LOW) {

    reportToMega("NOT ARMED");

    return;
  }

  noInterrupts();

  startCount = encoderCount;

  lastEncoderCount = encoderCount;

  interrupts();

  fireStartMillis = millis();

  lastMoveMillis = millis();

  firing360 = true;

  aimMode = false;

  laserOnFunc();

  reportToMega("360 START");
}

// ============================================================
// 360 service
// ============================================================
void service360() {

  if (!firing360)
    return;

  long currentCount;

  noInterrupts();

  currentCount = encoderCount;

  interrupts();

  long travelled = abs(currentCount - startCount);

  if (currentCount != lastEncoderCount) {

    lastMoveMillis = millis();

    lastEncoderCount = currentCount;
  }

  if (travelled >= COUNTS_PER_REV) {

    laserOffFunc();

    firing360 = false;

    reportToMega("360 DONE");

    return;
  }

  if (millis() - lastMoveMillis > noMovementTimeout) {

    laserOffFunc();

    firing360 = false;

    reportToMega("ENC STOP");

    return;
  }

  if (millis() - fireStartMillis > maxFireTime) {

    laserOffFunc();

    firing360 = false;

    reportToMega("TIMEOUT");

    return;
  }
}

// ============================================================
// Laser ON
// ============================================================
void laserOnFunc() {

  analogWrite(laserPwmPin, powerLevel);

  laserOn = true;
}

// ============================================================
// Laser OFF
// ============================================================
void laserOffFunc() {

  analogWrite(laserPwmPin, 0);

  digitalWrite(laserPwmPin, LOW);

  laserOn = false;
}

// ============================================================
// Stop everything
// ============================================================
void stopLaserAll() {

  analogWrite(laserPwmPin, 0);

  digitalWrite(laserPwmPin, LOW);

  laserOn = false;

  firing360 = false;

  aimMode = false;
}