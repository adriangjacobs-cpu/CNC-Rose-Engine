#include <AccelStepper.h>
#include <math.h>

// ===========================================================
// MEGA ROSETTE + NEXTION + DM542 Y + UNO LASER LINK
//
// Nextion TX -> Mega RX1 pin 19
// Nextion RX -> Mega TX1 pin 18
//
// Mega TX2 pin 16 -> UNO D10
// UNO D11 -> Mega RX2 pin 17
//
// Encoder A -> Mega D2
// Encoder B -> Mega D3
//
// Y axis:
// Mega D60/A6 -> DM542 PUL-
// Mega D61/A7 -> DM542 DIR-
// DM542 PUL+ and DIR+ -> +5V
// ===========================================================

// ---------- RAMPS / MEGA pins ----------
#define X_STEP_PIN    60
#define X_DIR_PIN     61
#define X_ENABLE_PIN  56

#define Y_STEP_PIN    22
#define Y_DIR_PIN     23
#define Y_ENABLE_PIN  24

#define Z_STEP_PIN    46
#define Z_DIR_PIN     48
#define Z_ENABLE_PIN  62



// ---------- Encoder ----------
#define ENC_A 2
#define ENC_B 3

volatile long spindleCount = 0;

// ---------- Motion ----------
float X_STEPS_PER_MM = 320.0;
float Y_STEPS_PER_MM = 213.0;   // DM542 at 3200 pulses/rev
float Z_STEPS_PER_MM = 320.0;

float jogMm = 1.0;

// ---------- Rosette ----------
int pattern = 0;
bool rosetteMode = false;
const long COUNTS_PER_REV = 1008;   // 1020 PPR encoder, A rising only

long roseStartCount = 0;

int lobes = 6;
float amplitude = 10.0;       // total travel in mm
int phaseTenths = 0;          // 25 = 2.5 degrees

float ROSETTE_GAIN = 3.0;
float laserRosetteMaxSpeed = 12000;
float routerRosetteMaxSpeed = 3000;
bool routerMode = false;

long rosetteCentreY = 0;

// ---------- Hold jog ----------
bool holdMode = false;
bool holdActive = false;
char holdAxis = ' ';
int holdDir = 0;

// ---------- Laser mode ----------
enum FireMode {
  MODE_360,
  MODE_MANUAL
};

FireMode fireMode = MODE_MANUAL;

// ---------- Serial ----------
String rxCmd = "";

// ---------- Steppers ----------
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// ===========================================================
// Encoder ISR
// ===========================================================
void updateSpindle() {
  if (digitalRead(ENC_B)) spindleCount++;
  else spindleCount--;
}

// ===========================================================
// Setup
// ===========================================================
void setup() {
  Serial.begin(115200);

  Serial1.begin(9600);   // Nextion
  Serial2.begin(9600);   // UNO laser controller

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite(Y_ENABLE_PIN, HIGH);  // disabled at startup
  disableAllMotors();



  // DM542 common-positive wiring
  stepperY.setPinsInverted(true, true, false);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateSpindle, RISING);

  stepperX.setMaxSpeed(1200);
  stepperX.setAcceleration(5000);

  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(100000);
  stepperY.setMinPulseWidth(5);

stepperZ.setMaxSpeed(1200);
stepperZ.setAcceleration(5000);
  

  updateJogDisplay();
  updateRosetteDisplay();
  setLaserModeText();
  setLaserState("READY");

  Serial.println("MEGA ROSETTE CONTROLLER START");
}

// ===========================================================
// Main loop
// ===========================================================
void loop() {
  readNextion();

  if (rosetteMode) {
    serviceRosette();
  }

  readLaserStatus();
  serviceHoldJog();
  runMotors();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    noInterrupts();
    long c = spindleCount;
    interrupts();

    Serial.print("COUNT=");
    Serial.println(c);

    lastPrint = millis();
  }
}

// ===========================================================
// Read Nextion commands
// ===========================================================
void readNextion() {
  static uint8_t ffCount = 0;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    if (c == 0xFF) {
      ffCount++;

      if (ffCount >= 3) {
        if (rxCmd.length() > 0) {
          rxCmd.trim();

          Serial.print("CMD:[");
          Serial.print(rxCmd);
          Serial.println("]");

          handleCommand(rxCmd);
          rxCmd = "";
        }

        ffCount = 0;
      }
    } else {
      ffCount = 0;
      rxCmd += char(c);
    }
  }
}

// ===========================================================
// Read UNO laser status
// ===========================================================
void readLaserStatus() {
  static String laserMsg = "";

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n' || c == '\r') {
      laserMsg.trim();

      if (laserMsg.length() > 0) {
        Serial.print("UNO:");
        Serial.println(laserMsg);

        if (laserMsg == "ARMED") setLaserState("ARMED");
        else if (laserMsg == "SAFE") setLaserState("SAFE");
        else if (laserMsg == "AIM LOW") setLaserState("AIM LOW");
        else if (laserMsg == "READY") setLaserState("READY");
        else if (laserMsg == "MAN ON") setLaserState("MAN FIRE");
        else if (laserMsg == "MAN OFF") setLaserState("MAN OFF");
        else if (laserMsg == "LASER OFF") setLaserState("LASER OFF");
        else if (laserMsg == "NOT ARMED") setLaserState("NOT ARMED");
        else if (laserMsg == "NOT ENABLED") setLaserState("NOT ENABLED");
        else if (laserMsg == "MODE 360") {
          fireMode = MODE_360;
          setLaserModeText();
        }
        else if (laserMsg == "MODE MANUAL") {
          fireMode = MODE_MANUAL;
          setLaserModeText();
        }
        else if (laserMsg.startsWith("POT:")) {
          String p = laserMsg.substring(4);
          sendText("tLaserpower", p + "%");
        }

        laserMsg = "";
      }
    } else {
      laserMsg += c;
    }
  }
}
void setRoseState(const String &msg) {
  sendText("tRoseState", msg);
}


// ===========================================================
// Send command to UNO
// ===========================================================
void sendLaserCommand(const String &cmd) {
  Serial2.println(cmd);
  Serial.print("Laser -> ");
  Serial.println(cmd);
}

// ===========================================================
// Command handler
// ===========================================================
void handleCommand(String cmd) {
  cmd.trim();

  // ---------- Laser commands ----------
  if (cmd == "LLOW") {
    sendLaserCommand("LLOW");
    return;
  }

  if (cmd == "LREADY") {
    sendLaserCommand("LREADY");
    return;
  }

  if (cmd == "LSTATE") {
    sendLaserCommand("LSTATE");
    return;
  }

  if (cmd == "L360") {
    fireMode = MODE_360;
    setLaserModeText();
    sendLaserCommand("L360");
    return;
  }

  if (cmd == "LMAN") {
    fireMode = MODE_MANUAL;
    setLaserModeText();
    sendLaserCommand("LMAN");
    return;
  }

  if (cmd == "LOFF") {
    sendLaserCommand("LOFF");
    return;
  }

  if (cmd == "FIRE") {
    sendLaserCommand("FIRE");
    return;
  }

  if (cmd == "PAT+") {
    rosetteMode = false;
    stepperY.setSpeed(0);
    stepperY.moveTo(stepperY.currentPosition());

    pattern++;
    if (pattern > 3) pattern = 0;

    stepperY.setCurrentPosition(0);
    rosetteCentreY = 0;

    updateRosetteDisplay();
    return;
  }

  if (cmd == "PAT-") {
    rosetteMode = false;
    stepperY.setSpeed(0);
    stepperY.moveTo(stepperY.currentPosition());

    pattern--;
    if (pattern < 0) pattern = 3;

    stepperY.setCurrentPosition(0);
    rosetteCentreY = 0;

    updateRosetteDisplay();
    return;
  }

  // ---------- Rosette ----------
  if (cmd == "RON") {
    digitalWrite(Y_ENABLE_PIN, LOW);    // enable Y
    noInterrupts();
    roseStartCount = spindleCount;
    interrupts();

    rosetteMode = true;
    holdActive = false;

    enableAllMotors();

    stepperY.setCurrentPosition(0);
    rosetteCentreY = 0;

    setRoseState("ROSETTE RUN");
    updateRosetteDisplay();

    return;
  }

  if (cmd == "ROFF") {
    digitalWrite(Y_ENABLE_PIN, HIGH);    // enable Y
    rosetteMode = false;

    stepperY.setSpeed(0);
    stepperY.moveTo(stepperY.currentPosition());

    sendLaserCommand("LOFF");

    setRoseState("ROSETTE STOP");
    updateRosetteDisplay();

    return;
  }



  if (cmd == "L+") {
    lobes++;
    if (lobes > 24) lobes = 24;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "L-") {
    if (lobes > 1) lobes--;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "A+") {
    amplitude += 0.5;
    if (amplitude > 40.0) amplitude = 40.0;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "A-") {
    amplitude -= 0.5;
    if (amplitude < 0.0) amplitude = 0.0;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "P+") {
    phaseTenths += 25;
    if (phaseTenths >= 3600) phaseTenths -= 3600;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "P-") {
    phaseTenths -= 25;
    if (phaseTenths < 0) phaseTenths += 3600;
    updateRosetteDisplay();
    return;
  }

  if (cmd == "ROUTER") {
    routerMode = true;
    setLaserState("ROUTER MODE");
    return;
  }

  if (cmd == "LASER") {
    routerMode = false;
    setLaserState("LASER MODE");
    return;
  }

  // ---------- Hold mode ----------
  if (cmd == "HOLDON") {
    holdMode = true;
    stopHold();
    setLaserState("HOLD ON");
    return;
  }

  if (cmd == "HOLDOFF") {
    holdMode = false;
    stopHold();
    setLaserState("HOLD OFF");
    return;
  }

  if (cmd == "HSTOP") {
    if (holdMode) stopHold();
    return;
  }

  // ---------- Jog size ----------
  if (cmd == "J0.1") {
    jogMm = 0.1;
    updateJogDisplay();
    return;
  }

  if (cmd == "J1") {
    jogMm = 1.0;
    updateJogDisplay();
    return;
  }

  if (cmd == "J10") {
    jogMm = 10.0;
    updateJogDisplay();
    return;
  }

  // ---------- Movement ----------
  if (cmd == "X+") {
    if (holdMode) startHold('X', +1);
    else moveXByMm(+jogMm);
    return;
  }

  if (cmd == "X-") {
    if (holdMode) startHold('X', -1);
    else moveXByMm(-jogMm);
    return;
  }

  if (cmd == "Y+") {
    if (holdMode) startHold('Y', +1);
    else moveYByMm(+jogMm);
    return;
  }

  if (cmd == "Y-") {
    if (holdMode) startHold('Y', -1);
    else moveYByMm(-jogMm);
    return;
  }

 if (cmd == "Z+") {
  if (holdMode) startHold('Z', +1);
  else moveZByMm(+jogMm);
  return;
}

if (cmd == "Z-") {
  if (holdMode) startHold('Z', -1);
  else moveZByMm(-jogMm);
  return;
} 

  if (cmd == "STOP") {
    rosetteMode = false;
    stopHold();
    emergencyStop();
    sendLaserCommand("STOP");
    setRoseState("STOP");
    updateRosetteDisplay();
    return;
  }

  setLaserState("UNKNOWN");
}

// ===========================================================
// Rosette motion only — laser is now independent/manual
// ===========================================================
void serviceRosette() {
  long count;

  noInterrupts();
  count = spindleCount;
  interrupts();

  long wrapped = (count - roseStartCount) % COUNTS_PER_REV;
  if (wrapped < 0) wrapped += COUNTS_PER_REV;

  float angle = (float)wrapped * TWO_PI / COUNTS_PER_REV;
  angle += (phaseTenths / 10.0) * DEG_TO_RAD;

  float profile = 0;

if (pattern == 0) {
  // Smooth sine rosette
  profile = sin(angle * lobes);
}
else if (pattern == 1) {
  // Spiro
  
//star harmonic pattern, stable and repeatable
  profile =
    0.65 * sin(angle * lobes) +
    0.35 * sin(angle * lobes * 3);
}
else if (pattern == 2) {
  // Hex/polygon rosette with bearing follower effect
  float a = fmod(angle * lobes, TWO_PI);
  if (a < 0) a += TWO_PI;

  float c = cos(a);

  

  // rounded flats with sharper valleys
  if (c >= 0) {
    profile = pow(c, 0.35);   // broad rounded high area
  } else {
    profile = -pow(-c, 2.5);  // sharper drop
  }
}
else if (pattern == 3) {
  // Ratchet rosette: sharp peaks, smooth concave hollows
  // lobes = number of peaks, e.g. 36

  float a = fmod(angle * lobes, TWO_PI);
  if (a < 0) a += TWO_PI;

  float t = a / TWO_PI;   // 0 to 1 within each tooth

  if (t < 0.12) {
    // fast rise to sharp peak
    profile = t / 0.12;
  } else {
    // long smooth concave fall into hollow
    float u = (t - 0.12) / 0.88;
    profile = 1.0 - (u * u);
  }

  profile = (profile * 2.0) - 1.0;
}

  long targetSteps =
    rosetteCentreY +
    (long)(profile * (amplitude / 2.0) * Y_STEPS_PER_MM);

  long error = targetSteps - stepperY.currentPosition();

  long speedCmd = error * ROSETTE_GAIN;

  float maxRoseSpeed = routerMode ? routerRosetteMaxSpeed : laserRosetteMaxSpeed;

  if (speedCmd > maxRoseSpeed) speedCmd = maxRoseSpeed;
  if (speedCmd < -maxRoseSpeed) speedCmd = -maxRoseSpeed;

  stepperY.setSpeed(speedCmd);
  stepperY.runSpeed();
}

// ===========================================================
// Hold jog
// ===========================================================
void startHold(char axis, int dir) {
  holdActive = true;
  holdAxis = axis;
  holdDir = dir;
}

void serviceHoldJog() {
  if (!holdActive) return;

  enableAllMotors();

  float speedSteps;

  if (jogMm == 0.1) speedSteps = 150;
  else if (jogMm == 1.0) speedSteps = 600;
  else speedSteps = 1200;

  speedSteps *= holdDir;

  if (holdAxis == 'X') {
    stepperX.setSpeed(speedSteps);
    stepperX.runSpeed();
  }
  else if (holdAxis == 'Y') {
    stepperY.setSpeed(speedSteps);
    stepperY.runSpeed();

    
  }

  else if (holdAxis == 'Z') {
  stepperZ.setSpeed(speedSteps);
  stepperZ.runSpeed();
}
}

void stopHold() {
  holdActive = false;
  holdAxis = ' ';
  holdDir = 0;

  stepperX.setSpeed(0);
  stepperY.setSpeed(0);
  stepperZ.setSpeed(0);

  disableAllMotors();
}

// ===========================================================
// Fixed jog movement
// ===========================================================
void moveXByMm(float mm) {
  long steps = lround(mm * X_STEPS_PER_MM);
  enableAllMotors();
  stepperX.moveTo(stepperX.currentPosition() + steps);
}

void moveYByMm(float mm) {
  long steps = lround(mm * Y_STEPS_PER_MM);
  enableAllMotors();
  stepperY.moveTo(stepperY.currentPosition() + steps);
}

void moveZByMm(float mm) {
  long steps = lround(mm * Z_STEPS_PER_MM);
  enableAllMotors();
  stepperZ.moveTo(stepperZ.currentPosition() + steps);
}

void runMotors() {
  if (holdActive) return;

  bool anyRunning = false;

  if (stepperX.distanceToGo() != 0) {
    stepperX.run();
    anyRunning = true;
  }

  if (stepperY.distanceToGo() != 0) {
    stepperY.run();
    anyRunning = true;
  }
if (stepperZ.distanceToGo() != 0) {
  stepperZ.run();
  anyRunning = true;
}


  if (!anyRunning && !holdActive && !rosetteMode) {
    disableAllMotors();
  }
}

void emergencyStop() {
  stepperX.moveTo(stepperX.currentPosition());
  stepperY.moveTo(stepperY.currentPosition());

stepperZ.moveTo(stepperZ.currentPosition());
  disableAllMotors();
}

// ===========================================================
// Enable / disable
// ===========================================================
void enableAllMotors() {
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
}

void disableAllMotors() {
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
}
// ===========================================================
// Nextion helpers
// ===========================================================
void updateJogDisplay() {
  if (jogMm == 0.1) sendText("tJog", "JOG 0.1");
  else if (jogMm == 1.0) sendText("tJog", "JOG 1");
  else sendText("tJog", "JOG 10");
}

void updateRosetteDisplay() {
  sendText("tMode", rosetteMode ? "RUN" : "STOP");
  sendText("tLobes", String(lobes));
  sendText("tAmp", String(amplitude, 1) + " mm");
  sendText("tPhase", String(phaseTenths / 10.0, 1) + " deg");
  if (pattern == 0) sendText("tPattern", "SINE");
else if (pattern == 1) sendText("tPattern", "LACE");
else if (pattern == 2) sendText("tPattern", "POLY");
}

void setLaserState(const String &msg) {
  sendText("tLaserState", msg);
}

void setLaserModeText() {
  sendText("tMserMOde", fireMode == MODE_360 ? "360" : "MAN");
}

void sendText(const char *obj, const String &value) {
  Serial1.print(obj);
  Serial1.print(".txt=\"");
  Serial1.print(value);
  Serial1.print("\"");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}