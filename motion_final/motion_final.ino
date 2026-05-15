// =============================
// BOAT MOTION - v9
// ESP32 WROOM - Arduino Core v3.x
// + Robust quadrature trim encoder
// + Autopilot input from Nav ESP via UART2
// + Joystick takeover DROPS autopilot to STANDBY (user must re-engage)
// + STATE notifications back to Nav ESP -> BLE -> HTML
// =============================
#include <Preferences.h>

Preferences prefs;

// ===== PIN DEFINITIONS =====
#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14

#define JOY_PIN 34
#define POT_PIN 32

#define ENC_A   18
#define ENC_B   19
#define ENC_SW  21

// ===== AUTOPILOT UART (Nav ESP <-> Motion ESP) =====
#define NAV_RX_PIN  16   // Motion side: Serial2 RX  (Nav GPIO 26 -> here)
#define NAV_TX_PIN  17   // Motion side: Serial2 TX  (here -> Nav GPIO 25)
#define NAV_BAUD    115200
#define AUTO_TIMEOUT_MS 1500

// ===== LEDC =====
#define LEDC_FREQ  5000
#define LEDC_RES   8

// ===== CONTROL VARIABLES =====
int joyRaw  = 0;
int potRaw  = 0;
float joyPct = 0;
int target  = 0;
int error   = 0;
int pwmOut  = 0;

// ===== TUNING DEFAULTS =====
float KP     = 0.22;
int deadband = 8;
int errorDB  = 15;
int joyDB    = 80;
int maxPWM   = 180;
int minPWM   = 45;

// ===== TRIM =====
int nudgeFine   = 10;
int nudgeCoarse = 50;

// ===== STEERING LIMITS =====
int leftLimit  = 1222;
int rightLimit = 2417;
int center     = 1765;

// ===== JOYSTICK CALIBRATION =====
int joyMin    = 0;
int joyMax    = 4095;
int joyCenter = 2048;

// ===== SMOOTHING =====
#define ADC_SAMPLES 12

// ===== QUADRATURE ENCODER =====
#define ENC_STEPS_PER_DETENT 4
static const int8_t QDEC_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
volatile uint8_t qState = 0;
volatile int32_t qSubCount = 0;
volatile int     encoderDelta = 0;

bool     swPressed     = false;
unsigned long swLastMs = 0;
#define SW_DEBOUNCE_MS 40

unsigned long lastStatus     = 0;
unsigned long lastStateBeat  = 0;
String inputBuffer = "";

// ===== AUTOPILOT STATE =====
bool          autoMode    = false;     // engaged or not
int           autoErr     = 0;         // -100..+100 from Nav ESP
unsigned long lastAutoMs  = 0;
String        navBuffer   = "";
const char*   controlSrc  = "STANDBY"; // JOY / AUTO / STANDBY (for STATUS)

// =============================
// ENCODER ISR
// =============================
void IRAM_ATTR encoderISR() {
  uint8_t bits = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  qState = ((qState << 2) | bits) & 0x0F;
  int8_t step = QDEC_TABLE[qState];
  if (step == 0) return;
  qSubCount += step;
  while (qSubCount >=  ENC_STEPS_PER_DETENT) { encoderDelta = encoderDelta + 1; qSubCount -= ENC_STEPS_PER_DETENT; }
  while (qSubCount <= -ENC_STEPS_PER_DETENT) { encoderDelta = encoderDelta - 1; qSubCount += ENC_STEPS_PER_DETENT; }
}

// =============================
// SAVE / LOAD
// =============================
void saveAll() {
  prefs.begin("bmcal", false);
  prefs.putInt("leftLimit",   leftLimit);
  prefs.putInt("rightLimit",  rightLimit);
  prefs.putInt("center",      center);
  prefs.putInt("joyMin",      joyMin);
  prefs.putInt("joyMax",      joyMax);
  prefs.putInt("joyCenter",   joyCenter);
  prefs.putFloat("KP",        KP);
  prefs.putInt("deadband",    deadband);
  prefs.putInt("errorDB",     errorDB);
  prefs.putInt("joyDB",       joyDB);
  prefs.putInt("maxPWM",      maxPWM);
  prefs.putInt("minPWM",      minPWM);
  prefs.putInt("nudgeFine",   nudgeFine);
  prefs.putInt("nudgeCoarse", nudgeCoarse);
  prefs.end();
  Serial.println("SAVED");
}

void loadAll() {
  prefs.begin("bmcal", true);
  leftLimit   = prefs.getInt("leftLimit",   1222);
  rightLimit  = prefs.getInt("rightLimit",  2417);
  center      = prefs.getInt("center",      1765);
  joyMin      = prefs.getInt("joyMin",      0);
  joyMax      = prefs.getInt("joyMax",      4095);
  joyCenter   = prefs.getInt("joyCenter",   2048);
  KP          = prefs.getFloat("KP",        0.22);
  deadband    = prefs.getInt("deadband",    8);
  errorDB     = prefs.getInt("errorDB",     15);
  joyDB       = prefs.getInt("joyDB",       80);
  maxPWM      = prefs.getInt("maxPWM",      180);
  minPWM      = prefs.getInt("minPWM",      45);
  nudgeFine   = prefs.getInt("nudgeFine",   10);
  nudgeCoarse = prefs.getInt("nudgeCoarse", 50);
  prefs.end();
}

// =============================
// STATE NOTIFICATIONS (-> Nav ESP -> HTML)
// =============================
//   STATE:AUTO\n
//   STATE:STANDBY:BOOT\n
//   STATE:STANDBY:USER\n
//   STATE:STANDBY:JOY\n
//   STATE:STANDBY:TIMEOUT\n
// =============================
void emitState(const char* state, const char* reason) {
  String msg = "STATE:";
  msg += state;
  if (reason && reason[0]) { msg += ":"; msg += reason; }
  msg += "\n";
  Serial2.print(msg);
  Serial.print("STATE_OUT=");
  Serial.print(msg);   // also visible on USB serial for debugging
}

void setAutoMode(bool engaged, const char* reason) {
  if (engaged == autoMode) return;       // no change, don't spam notifications
  autoMode = engaged;
  if (!engaged) autoErr = 0;
  emitState(engaged ? "AUTO" : "STANDBY", reason);
}

// =============================
// AUTOPILOT INPUT PARSER (Serial2 from Nav ESP, also USB for testing)
// =============================
void parseAutopilotLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("ERR:") || cmd.startsWith("ERR ")) {
    int v = cmd.substring(4).toInt();
    autoErr    = constrain(v, -100, 100);
    lastAutoMs = millis();
    return;
  }

  if (cmd.startsWith("MODE:") || cmd.startsWith("MODE ")) {
    String m = cmd.substring(5);
    m.trim();
    if (m.equalsIgnoreCase("AUTO"))    { setAutoMode(true,  "USER"); lastAutoMs = millis(); }
    if (m.equalsIgnoreCase("STANDBY")) { setAutoMode(false, "USER"); }
    return;
  }
}

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  Serial2.begin(NAV_BAUD, SERIAL_8N1, NAV_RX_PIN, NAV_TX_PIN);

  ledcAttach(RPWM, LEDC_FREQ, LEDC_RES);
  ledcAttach(LPWM, LEDC_FREQ, LEDC_RES);

  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  pinMode(ENC_A,  INPUT_PULLUP);
  pinMode(ENC_B,  INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  qState    = ((digitalRead(ENC_A) << 1) | digitalRead(ENC_B)) & 0x03;
  qSubCount = 0;
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  loadAll();
  Serial.println("BOOT OK");

  // Announce boot state to Nav ESP / HTML
  emitState("STANDBY", "BOOT");

  sendStatus();
}

// =============================
// SMOOTH ANALOG READ
// =============================
int readSmooth(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return sum / ADC_SAMPLES;
}

// =============================
// MOTOR CONTROL
// =============================
void setMotor(int pwm) {
  pwm = constrain(pwm, -maxPWM, maxPWM);
  if (abs(pwm) < deadband) {
    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, 0);
    return;
  }
  if (pwm > 0) { ledcWrite(RPWM, pwm); ledcWrite(LPWM, 0); }
  else         { ledcWrite(RPWM, 0);   ledcWrite(LPWM, -pwm); }
}

// =============================
// PROCESS ENCODER (trim)
// =============================
void processEncoder() {
  noInterrupts();
  int delta = encoderDelta;
  encoderDelta = 0;
  interrupts();
  if (delta == 0) return;

  unsigned long now = millis();
  if (now - swLastMs > SW_DEBOUNCE_MS) {
    swPressed = (digitalRead(ENC_SW) == LOW);
    swLastMs  = now;
  }

  int step = swPressed ? nudgeCoarse : nudgeFine;
  center += delta * step;
  center = constrain(center, leftLimit + 20, rightLimit - 20);
  saveAll();

  Serial.print("TRIM=");
  Serial.println(center);
}

// =============================
// COMMAND HANDLER (USB serial)
// =============================
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("KP "))           { KP          = cmd.substring(3).toFloat();   saveAll(); }
  if (cmd.startsWith("DEADBAND "))     { deadband    = cmd.substring(9).toInt();     saveAll(); }
  if (cmd.startsWith("ERRORDB "))      { errorDB     = cmd.substring(8).toInt();     saveAll(); }
  if (cmd.startsWith("JOYDB "))        { joyDB       = cmd.substring(6).toInt();     saveAll(); }
  if (cmd.startsWith("MAXPWM "))       { maxPWM      = cmd.substring(7).toInt();     saveAll(); }
  if (cmd.startsWith("MINPWM "))       { minPWM      = cmd.substring(7).toInt();     saveAll(); }
  if (cmd.startsWith("NUDGE_FINE "))   { nudgeFine   = cmd.substring(11).toInt();    saveAll(); }
  if (cmd.startsWith("NUDGE_COARSE ")) { nudgeCoarse = cmd.substring(13).toInt();    saveAll(); }
  if (cmd.startsWith("JOYMIN "))       { joyMin      = cmd.substring(7).toInt();     saveAll(); }
  if (cmd.startsWith("JOYMAX "))       { joyMax      = cmd.substring(7).toInt();     saveAll(); }
  if (cmd.startsWith("JOYCENTER "))    { joyCenter   = cmd.substring(10).toInt();    saveAll(); }

  if (cmd.startsWith("CENTER"))        { center      = potRaw;   saveAll(); }
  if (cmd.startsWith("CAPLEFT"))       { leftLimit   = potRaw;   saveAll(); }
  if (cmd.startsWith("CAPRIGHT"))      { rightLimit  = potRaw;   saveAll(); }
  if (cmd.startsWith("ENC LIMIT LEFT "))  { leftLimit  = cmd.substring(16).toInt();  saveAll(); }
  if (cmd.startsWith("ENC LIMIT RIGHT ")) { rightLimit = cmd.substring(17).toInt();  saveAll(); }

  // Autopilot test commands over USB
  if (cmd.startsWith("ERR:") || cmd.startsWith("ERR "))   parseAutopilotLine(cmd);
  if (cmd.startsWith("MODE:") || cmd.startsWith("MODE ")) parseAutopilotLine(cmd);
  if (cmd.equalsIgnoreCase("AUTO"))    { setAutoMode(true,  "USER"); lastAutoMs = millis(); }
  if (cmd.equalsIgnoreCase("STANDBY")) { setAutoMode(false, "USER"); }

  if (cmd.startsWith("SAVE"))   saveAll();
  if (cmd.startsWith("STATUS")) sendStatus();
}

// =============================
// STATUS OUTPUT (USB)
// =============================
void sendStatus() {
  Serial.println("STATUS_BEGIN");
  Serial.print("POT_RAW=");        Serial.println(potRaw);
  Serial.print("JOY_RAW=");        Serial.println(joyRaw);
  Serial.print("JOY_PCT=");        Serial.println(joyPct, 1);
  Serial.print("TARGET_REL=");     Serial.println(target);
  Serial.print("ERROR=");          Serial.println(error);
  Serial.print("PWM=");            Serial.println(pwmOut);

  Serial.print("CENTER_RAW=");     Serial.println(center);
  Serial.print("LEFT_LIMIT=");     Serial.println(leftLimit);
  Serial.print("RIGHT_LIMIT=");    Serial.println(rightLimit);
  Serial.print("JOY_MIN=");        Serial.println(joyMin);
  Serial.print("JOY_MAX=");        Serial.println(joyMax);
  Serial.print("JOY_CENTER=");     Serial.println(joyCenter);
  Serial.print("KP_VAL=");         Serial.println(KP, 3);
  Serial.print("DEADBAND_VAL=");   Serial.println(deadband);
  Serial.print("ERRORDB_VAL=");    Serial.println(errorDB);
  Serial.print("JOYDB_VAL=");      Serial.println(joyDB);
  Serial.print("MAXPWM_VAL=");     Serial.println(maxPWM);
  Serial.print("MINPWM_VAL=");     Serial.println(minPWM);
  Serial.print("NUDGE_FINE_VAL="); Serial.println(nudgeFine);
  Serial.print("NUDGE_CRS_VAL=");  Serial.println(nudgeCoarse);

  Serial.print("AUTO_MODE=");      Serial.println(autoMode ? "AUTO" : "STANDBY");
  Serial.print("AUTO_ERR=");       Serial.println(autoErr);
  Serial.print("CONTROL_SRC=");    Serial.println(controlSrc);
  Serial.println("STATUS_END");
}

// =============================
// LOOP
// =============================
void loop() {

  // ===== SERIAL INPUT (USB) =====
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { inputBuffer.trim(); handleCommand(inputBuffer); inputBuffer = ""; }
    else           { inputBuffer += c; }
  }

  // ===== AUTOPILOT INPUT (Serial2 from Nav ESP) =====
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n')      { parseAutopilotLine(navBuffer); navBuffer = ""; }
    else if (c != '\r') { navBuffer += c; if (navBuffer.length() > 64) navBuffer = ""; }
  }

  // Auto timeout safety -> STANDBY
  if (autoMode && (millis() - lastAutoMs > AUTO_TIMEOUT_MS)) {
    setAutoMode(false, "TIMEOUT");
  }

  // ===== ENCODER TRIM =====
  processEncoder();

  // ===== READ SENSORS =====
  joyRaw = readSmooth(JOY_PIN);
  potRaw = readSmooth(POT_PIN);

  // ===== CONTROL SOURCE SELECT =====
  // Joystick out of centre deadband always wins.
  // If we were in AUTO when this happens, drop to STANDBY (joystick override).
  // Once in STANDBY, the user must explicitly re-engage (no auto-resume).
  int joyOffset = joyRaw - joyCenter;
  bool joyActive = (abs(joyOffset) > joyDB);

  if (joyActive) {
    if (autoMode) {
      setAutoMode(false, "JOY");   // <-- key safety behaviour
    }
    controlSrc = "JOY";
    if (joyOffset > 0)
      joyPct = (float)(joyOffset - joyDB) / (joyMax - joyCenter - joyDB) * 100.0;
    else
      joyPct = (float)(joyOffset + joyDB) / (joyCenter - joyMin - joyDB) * 100.0;
    joyPct = constrain(joyPct, -100, 100);
    target = map((long)joyPct, -100, 100, leftLimit, rightLimit);
  }
  else if (autoMode) {
    controlSrc = "AUTO";
    int e = constrain(autoErr, -100, 100);
    joyPct = (float)e;
    target = (e == 0) ? center : map((long)e, -100, 100, leftLimit, rightLimit);
  }
  else {
    controlSrc = "STANDBY";
    joyPct = 0;
    target = center;
  }

  // ===== ERROR =====
  error = target - potRaw;

  // ===== CONTROL LOOP =====
  if (abs(error) <= errorDB) {
    pwmOut = 0;
    setMotor(0);
  } else {
    pwmOut = (int)(error * KP);
    if (abs(pwmOut) > 0 && abs(pwmOut) < minPWM) {
      pwmOut = (pwmOut > 0) ? minPWM : -minPWM;
    }
    setMotor(pwmOut);
  }

  // ===== STATE HEARTBEAT (1 Hz) so HTML always knows current state =====
  if (millis() - lastStateBeat > 1000) {
    String hb = "STATE:";
    hb += (autoMode ? "AUTO" : "STANDBY");
    hb += "\n";
    Serial2.print(hb);
    lastStateBeat = millis();
  }

  // ===== STATUS BROADCAST (USB) =====
  if (millis() - lastStatus > 500) {
    sendStatus();
    lastStatus = millis();
  }

  delay(1);
}
