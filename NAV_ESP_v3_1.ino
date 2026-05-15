// =====================================================================
// NAV ESP v3.3 — BNO055 yaw-rate damped autopilot, selectable features
// =====================================================================
// What's new vs v2.6
// ------------------
// Each BNO055-derived control benefit is now an independent on/off toggle,
// saved to NVS and settable over BLE. Lets you A/B compare features
// during sea trials without recompiling.
//
//   USE_RATE_D     — yaw-rate D term on/off       (default ON)
//   USE_CAL_GATE   — require gyro cal >= 1        (default ON)
//   USE_BNO_HDG    — use BNO heading vs QMC       (default ON)
//
// The data-freshness gate (refuse yaw-rate D if data is stale > 500 ms)
// is NOT toggleable — control loops must never run on stale sensor data.
//
// BLE additions:
//   incoming:  URD:<0|1>    set USE_RATE_D
//              UCG:<0|1>    set USE_CAL_GATE
//              UBH:<0|1>    set USE_BNO_HDG
//   outgoing:  FEAT:<urd>,<ucg>,<ubh>             (on change / on connect)
//
// All v2.6 behaviour preserved when all flags are at their defaults.
// =====================================================================
// What's in v2.6 (kept)
// ------------------
// The PID's D term is now computed from the BNO055 GYRO Z axis
// (the boat's actual yaw rate) instead of from the time-derivative of
// the heading error. This is the standard "derivative on measurement"
// trick used by every commercial marine autopilot — it kills the
// zig-zag failure mode that comes from differentiating a smoothed,
// laggy heading signal.
//
// Why:
//   * The BNO055 gyro is high-bandwidth (>50 Hz internally) and clean.
//   * Heading goes through HDG_SMOOTH samples of circular averaging,
//     adding 400–500 ms of phase lag. Differentiating that lagged
//     signal gives D-action that arrives AFTER the overshoot — useless.
//   * Reading gyro Z directly gives D-action that arrives BEFORE
//     heading even moves much, so the controller can ease off in time.
//
// New tunables (saved to NVS, exchanged over BLE):
//   KD_RATE        — gain applied to yaw rate (deg/s).  default 1.0
//   RATE_DEADBAND  — yaw-rate magnitude below which D is treated as 0.
//                    Stops integral chatter from sensor noise. default 0.3 deg/s
//   RATE_SIGN      — +1.0 or -1.0. Flips the polarity of the gyro
//                    relative to the heading convention. Default -1.0
//                    (correct for the BNO055 default axis remap with a
//                    standard upright mounting). Flip during sea trial
//                    if the boat starts steering FASTER away from target.
//
// The legacy KD (derivative-of-error) is kept but DEFAULTS TO 0 in v2.6.
// You can leave it at 0 forever; it's only there as a fallback if the
// BNO055 is unavailable and the autopilot has to run on the QMC compass.
//
// Gating: yaw-rate D is only used when:
//   - bnoOk == true, AND
//   - bnoCalGyro >= 1 (gyro has calibrated at least once), AND
//   - millis() - bnoLastReadMs < 500   (data is fresh)
// Otherwise the controller falls back to legacy error-derivative D.
//
// New BLE traffic:
//   incoming:  KDR:<float>   set KD_RATE
//              RDB:<float>   set RATE_DEADBAND
//              RSIGN:<+1|-1> set RATE_SIGN
//   outgoing:  PID2:<KD_RATE>,<RATE_DEADBAND>,<RATE_SIGN>     (on change / on connect)
//              AUTOQ:<rateOk>,<calGyro>,<calSys>,<rateUsed>   (1 Hz while connected)
//                rateOk=1 means yaw-rate D is in use this tick.
//              LIVE:<err>,<yawDps>,<pidOut>,<rateUsed>         (5 Hz while connected)
//                One line per PID tick — used by the HTML "Tune" page to
//                plot live heading error / yaw rate / PID output for
//                interactive gain tuning.
//
// All v2.5 BLE traffic (HDG/GPS/MAG/SYS/STATE/TGT/PID/ADV/REV/
// BNO/BNOA/BNOM/BNOG/BNOQ) is unchanged.
// =====================================================================

#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>
#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial GPSserial(1);

#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define MOTION_RX_PIN 25
#define MOTION_TX_PIN 26
#define MOTION_BAUD   115200

#define GPS_RX_PIN    4
#define GPS_TX_PIN    13
#define GPS_BAUD      115200

#define PID_INTERVAL_MS    200
#define HDG_INTERVAL_MS    200
#define GPS_NOTIFY_PERIOD  1000
#define GPSI_NOTIFY_PERIOD 1000
#define MAG_NOTIFY_PERIOD  1000
#define SAT_NOTIFY_PERIOD  2000
#define SYS_NOTIFY_PERIOD  5000
#define GPS_STALE_MS       4000

// BNO data send rates (ms)
#define BNO_PERIOD_MS      200    // orientation+cal at 5 Hz
#define BNOA_PERIOD_MS     500    // accel/gravity at 2 Hz
#define BNOM_PERIOD_MS     1000   // mag+temp at 1 Hz
#define BNOG_PERIOD_MS     500    // gyro at 2 Hz
#define BNOQ_PERIOD_MS     1000   // quaternion at 1 Hz

#define HDG_SMOOTH_MAX     15

QMC5883LCompass compass;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Preferences     prefs;

BLECharacteristic *bleTx;
BLECharacteristic *bleRx;
bool deviceConnected = false;

// ---- PID state ----
float KP=2.0f, KI=0.05f, KD=0.0f, DEADBAND=1.0f;     // KD default 0 in v2.6
                                                     // (use KD_RATE instead)
// ---- v2.6: yaw-rate damping ----
float KD_RATE       = 1.0f;     // deg of output per (deg/s) of yaw rate
float RATE_DEADBAND = 0.3f;     // deg/s — below this we treat yaw rate as 0
float RATE_SIGN     = -1.0f;    // +1 or -1; sign-correct gyro Z so that
                                // RATE_SIGN*gyroZ_degPerSec == heading_rate
bool  rateUsedLastTick = false; // for AUTOQ telemetry

// ---- v2.7: BNO feature toggles (each independently switchable) ----
// USE_RATE_D : master switch for yaw-rate-derived D term. When false, the
//              PID uses the legacy error-derivative D (using KD), even if
//              the BNO is healthy. Lets you A/B compare in the field.
// USE_CAL_GATE: when true, yaw-rate D is suppressed until bnoCalGyro >= 1.
//              When false, yaw-rate D is used as soon as the BNO is present
//              and reporting fresh data, regardless of calibration status.
// USE_BNO_HDG : when true (default), heading comes from the BNO055 fused
//              AHRS if present. When false, the QMC5883L compass is used
//              even if the BNO is present.
// (Data-freshness gate is NOT user-toggleable — running control on stale
//  sensor data is unsafe.)
bool USE_RATE_D    = true;
bool USE_CAL_GATE  = true;
bool USE_BNO_HDG   = true;

#define AUTOQ_PERIOD_MS  1000   // 1 Hz autopilot quality telemetry

float currentTarget=0.0f;
bool  haveTarget=false, autoEngaged=false;
float integralTerm=0, prevError=0;
unsigned long lastPidMs=0, lastHdgMs=0;
float currentHeading=0.0f;
String motionBuffer = "";

bool steeringReversed = false;

// ---- Helm-calming params ----
float SLEW       = 60.0f;
int   HDG_SMOOTH = 5;
float DALPHA     = 0.7f;

float prevOutput = 0.0f;
float dFiltered  = 0.0f;

float hdgCosBuf[HDG_SMOOTH_MAX];
float hdgSinBuf[HDG_SMOOTH_MAX];
int   hdgBufIdx   = 0;
int   hdgBufCount = 0;

// ---- GPS state ----
String gpsLineBuf = "";
double gpsLat=0, gpsLon=0;
float  gpsSpdMps=0, gpsCog=0, gpsHdop=0, gpsAltM=0;
int    gpsFixQuality=0, gpsSatsUsed=0, gpsSatsInView=0;
bool   gpsHaveFix=false;
unsigned long gpsLastFixMs=0, gpsLastNotifyMs=0;
unsigned long gpsiLastNotifyMs=0, satLastNotifyMs=0;

// ---- QMC compass diag (still read as fallback) ----
int16_t magX=0, magY=0, magZ=0;
unsigned long magLastNotifyMs=0;

// ---- BNO055 FULL state ----
bool    bnoOk = false;
float   bnoHeading=0, bnoPitch=0, bnoRoll=0;
float   bnoYawRate=0;                              // gyro Z (heading rate)
float   bnoGyroX=0, bnoGyroY=0, bnoGyroZ=0;        // full gyro vector rad/s
float   bnoLinAx=0, bnoLinAy=0, bnoLinAz=0;        // gravity-removed accel m/s²
float   bnoGravX=0, bnoGravY=0, bnoGravZ=0;        // gravity vector m/s²
float   bnoMagX=0, bnoMagY=0, bnoMagZ=0;           // BNO mag μT
float   bnoQuatW=1, bnoQuatX=0, bnoQuatY=0, bnoQuatZ=0;
int8_t  bnoTempC = 0;
uint8_t bnoCalSys=0, bnoCalGyro=0, bnoCalAccel=0, bnoCalMag=0;
unsigned long bnoLastReadMs=0;

// BLE schedule timers (per message type)
unsigned long bnoLastNotifyMs=0, bnoaLastNotifyMs=0,
              bnomLastNotifyMs=0, bnogLastNotifyMs=0, bnoqLastNotifyMs=0;
unsigned long autoqLastNotifyMs=0;

// ---- Sys ----
unsigned long sysLastNotifyMs=0;

// =====================================================================
// UBX permanent NMEA config
// =====================================================================
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len){
  uint8_t hdr[6] = { 0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t cka = 0, ckb = 0;
  for (int i = 2; i < 6; i++){ cka += hdr[i]; ckb += cka; }
  for (uint16_t i = 0; i < len; i++){ cka += payload[i]; ckb += cka; }
  GPSserial.write(hdr, 6);
  if (len) GPSserial.write(payload, len);
  uint8_t cks[2] = { cka, ckb };
  GPSserial.write(cks, 2);
  GPSserial.flush();
}

void configureUbloxM10(){
  delay(200);
  while (GPSserial.available()) GPSserial.read();
  static const uint8_t payload[] = {
    0x00, 0x07, 0x00, 0x00,
    0x01, 0x00, 0x74, 0x10, 0x00,
    0x02, 0x00, 0x74, 0x10, 0x01,
    0xBB, 0x00, 0x91, 0x20, 0x01,
    0xAC, 0x00, 0x91, 0x20, 0x01,
    0xC5, 0x00, 0x91, 0x20, 0x05,
    0xC0, 0x00, 0x91, 0x20, 0x00,
    0xB1, 0x00, 0x91, 0x20, 0x00,
    0xCA, 0x00, 0x91, 0x20, 0x00,
  };
  sendUbx(0x06, 0x8A, payload, sizeof(payload));
  delay(150);
  Serial.println("u-blox M10: NMEA-only config sent (saved to FLASH)");
}

// =====================================================================
// HELPERS
// =====================================================================
float wrap360(float a){ while(a>=360)a-=360; while(a<0)a+=360; return a; }
float angleErr(float t,float c){ float d=t-c; while(d>180)d-=360; while(d<-180)d+=360; return d; }
float clampf(float v, float lo, float hi){ if (v<lo) return lo; if (v>hi) return hi; return v; }

float pushHdg(float deg){
  float r = deg * (PI / 180.0f);
  hdgCosBuf[hdgBufIdx] = cosf(r);
  hdgSinBuf[hdgBufIdx] = sinf(r);
  hdgBufIdx = (hdgBufIdx + 1) % HDG_SMOOTH;
  if (hdgBufCount < HDG_SMOOTH) hdgBufCount++;
  if (hdgBufCount <= 1) return wrap360(deg);
  float sumC = 0, sumS = 0;
  for (int i=0; i<hdgBufCount; i++){ sumC += hdgCosBuf[i]; sumS += hdgSinBuf[i]; }
  float avg = atan2f(sumS, sumC) * (180.0f / PI);
  return wrap360(avg);
}

void resetHelmFilters(){
  hdgBufIdx = 0; hdgBufCount = 0;
  for (int i=0;i<HDG_SMOOTH_MAX;i++){ hdgCosBuf[i]=0; hdgSinBuf[i]=0; }
  prevOutput = 0.0f; dFiltered = 0.0f;
  integralTerm = 0.0f; prevError = 0.0f;
}

// =====================================================================
// BNO055 — read everything available into globals.
// Called from loop() at ~5 Hz alongside heading reads.
// =====================================================================
void readBnoFull(){
  if (!bnoOk) return;

  imu::Vector<3> euler   = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linAcc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> mag     = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  bnoHeading = wrap360(euler.x());
  bnoRoll    = euler.y();
  bnoPitch   = euler.z();

  bnoGyroX = gyro.x();  bnoGyroY = gyro.y();  bnoGyroZ = gyro.z();
  bnoYawRate = bnoGyroZ;

  bnoLinAx = linAcc.x();  bnoLinAy = linAcc.y();  bnoLinAz = linAcc.z();
  bnoGravX = gravity.x(); bnoGravY = gravity.y(); bnoGravZ = gravity.z();
  bnoMagX  = mag.x();     bnoMagY  = mag.y();     bnoMagZ  = mag.z();

  imu::Quaternion q = bno.getQuat();
  bnoQuatW = q.w();  bnoQuatX = q.x();  bnoQuatY = q.y();  bnoQuatZ = q.z();

  bnoTempC = bno.getTemp();
  bno.getCalibration(&bnoCalSys, &bnoCalGyro, &bnoCalAccel, &bnoCalMag);

  bnoLastReadMs = millis();
}

float readCompassRaw(){
  // Always pull the BNO if it's present — that keeps gyro/yaw-rate, accel,
  // mag, and quaternion telemetry flowing for the diag/tune pages, and
  // keeps yaw-rate D available even if the user has chosen QMC for heading.
  if (bnoOk){
    readBnoFull();
    if (USE_BNO_HDG){
      return bnoHeading;
    }
  }
  // Either BNO not present, or user has opted to use QMC heading.
  compass.read();
  magX = compass.getX();
  magY = compass.getY();
  magZ = compass.getZ();
  int az = compass.getAzimuth();
  float h=(float)az; if(h<0) h+=360;
  return h;
}

// =====================================================================
// NMEA parsing
// =====================================================================
double nmeaToDecimal(const String& v, char hemi){
  if (v.length() < 4) return NAN;
  int dot = v.indexOf('.');
  int degChars = (dot >= 2) ? (dot - 2) : 0;
  if (degChars <= 0) return NAN;
  double deg = v.substring(0, degChars).toDouble();
  double min = v.substring(degChars).toDouble();
  double dec = deg + (min / 60.0);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

bool nmeaChecksumOk(const String& s){
  int star = s.indexOf('*');
  if (star < 1 || star+2 >= (int)s.length()) return false;
  uint8_t cs = 0;
  for (int i = 1; i < star; i++) cs ^= (uint8_t)s.charAt(i);
  char buf[3] = { s.charAt(star+1), s.charAt(star+2), 0 };
  uint8_t given = (uint8_t)strtoul(buf, nullptr, 16);
  return cs == given;
}

int nmeaSplit(const String& s, String* out, int maxOut){
  int n=0, start=0;
  for (int i=0; i <= (int)s.length(); i++){
    char c = (i == (int)s.length()) ? ',' : s.charAt(i);
    if (c == '*'){ if (n<maxOut) out[n++] = s.substring(start, i); break; }
    if (c == ','){ if (n<maxOut) out[n++] = s.substring(start, i); start = i+1; }
  }
  return n;
}

void parseNmeaLine(const String& line){
  if (line.length() < 7 || line.charAt(0) != '$') return;
  if (!nmeaChecksumOk(line)) return;
  String f[20];
  int n = nmeaSplit(line, f, 20);
  if (n < 1 || f[0].length() < 6) return;
  String type = f[0].substring(3);

  if (type == "RMC" && n >= 10){
    bool valid = (f[2] == "A");
    if (!valid){ return; }
    double la = nmeaToDecimal(f[3], f[4].length() ? f[4].charAt(0) : 'N');
    double lo = nmeaToDecimal(f[5], f[6].length() ? f[6].charAt(0) : 'E');
    if (isnan(la) || isnan(lo)) return;
    gpsLat = la; gpsLon = lo;
    if (f[7].length()) gpsSpdMps = f[7].toFloat() * 0.514444f;
    if (f[8].length()) gpsCog    = f[8].toFloat();
    gpsHaveFix = true; gpsLastFixMs = millis();
    return;
  }
  if (type == "GGA" && n >= 7){
    int q = f[6].toInt();
    gpsFixQuality = q;
    if (n >= 8 && f[7].length()) gpsSatsUsed = f[7].toInt();
    if (n >= 9 && f[8].length()) gpsHdop     = f[8].toFloat();
    if (n >= 10 && f[9].length()) gpsAltM    = f[9].toFloat();
    if (q <= 0) return;
    double la = nmeaToDecimal(f[2], f[3].length() ? f[3].charAt(0) : 'N');
    double lo = nmeaToDecimal(f[4], f[5].length() ? f[5].charAt(0) : 'E');
    if (isnan(la) || isnan(lo)) return;
    gpsLat = la; gpsLon = lo;
    gpsHaveFix = true; gpsLastFixMs = millis();
    return;
  }
  if (type == "GSV" && n >= 4){
    if (f[3].length()) gpsSatsInView = f[3].toInt();
    return;
  }
}

// =====================================================================
// Persistence
// =====================================================================
void loadPrefs(){
  prefs.begin("navpid", true);
  KP=prefs.getFloat("kp",2.0f); KI=prefs.getFloat("ki",0.05f);
  KD=prefs.getFloat("kd",0.0f); DEADBAND=prefs.getFloat("db",1.0f);
  currentTarget = prefs.getFloat("tgt", 0.0f);
  haveTarget    = prefs.getBool ("haveTgt", false);
  steeringReversed = prefs.getBool ("rev", false);
  SLEW       = prefs.getFloat("slew",   60.0f);
  HDG_SMOOTH = prefs.getInt  ("smooth",  5);
  DALPHA     = prefs.getFloat("dalpha", 0.7f);
  // v2.6 new gains
  KD_RATE       = prefs.getFloat("kdr",   1.0f);
  RATE_DEADBAND = prefs.getFloat("rdb",   0.3f);
  RATE_SIGN     = prefs.getFloat("rsign",-1.0f);
  // v2.7 feature toggles
  USE_RATE_D    = prefs.getBool ("urd", true);
  USE_CAL_GATE  = prefs.getBool ("ucg", true);
  USE_BNO_HDG   = prefs.getBool ("ubh", true);
  prefs.end();
  if (HDG_SMOOTH < 1) HDG_SMOOTH = 1;
  if (HDG_SMOOTH > HDG_SMOOTH_MAX) HDG_SMOOTH = HDG_SMOOTH_MAX;
  DALPHA = clampf(DALPHA, 0.0f, 0.95f);
  SLEW   = clampf(SLEW,   1.0f, 1000.0f);
  KD_RATE       = clampf(KD_RATE,       0.0f, 50.0f);
  RATE_DEADBAND = clampf(RATE_DEADBAND, 0.0f,  5.0f);
  if (RATE_SIGN >= 0.0f) RATE_SIGN = 1.0f; else RATE_SIGN = -1.0f;
}
void savePid(){
  prefs.begin("navpid", false);
  prefs.putFloat("kp",KP); prefs.putFloat("ki",KI);
  prefs.putFloat("kd",KD); prefs.putFloat("db",DEADBAND);
  prefs.end();
}
void savePid2(){     // v2.6 yaw-rate damping gains
  prefs.begin("navpid", false);
  prefs.putFloat("kdr",   KD_RATE);
  prefs.putFloat("rdb",   RATE_DEADBAND);
  prefs.putFloat("rsign", RATE_SIGN);
  prefs.end();
}
void saveFeat(){     // v2.7 feature toggles
  prefs.begin("navpid", false);
  prefs.putBool("urd", USE_RATE_D);
  prefs.putBool("ucg", USE_CAL_GATE);
  prefs.putBool("ubh", USE_BNO_HDG);
  prefs.end();
}
void saveTarget(){
  prefs.begin("navpid", false);
  prefs.putFloat("tgt", currentTarget);
  prefs.putBool ("haveTgt", haveTarget);
  prefs.end();
}
void saveReverse(){
  prefs.begin("navpid", false);
  prefs.putBool("rev", steeringReversed);
  prefs.end();
}
void saveAdv(){
  prefs.begin("navpid", false);
  prefs.putFloat("slew",   SLEW);
  prefs.putInt  ("smooth", HDG_SMOOTH);
  prefs.putFloat("dalpha", DALPHA);
  prefs.end();
}

// =====================================================================
// BLE notify helpers
// =====================================================================
void notify(const String& s){
  if (!deviceConnected) return;
  bleTx->setValue((uint8_t*)s.c_str(), s.length());
  bleTx->notify();
}
void notifyTarget(){ String s="TGT:"; s+=(int)round(currentTarget); notify(s); }
void notifyGains(){
  char b[64];
  snprintf(b,sizeof(b),"PID:%.3f,%.4f,%.3f,%.2f", KP,KI,KD,DEADBAND);
  notify(String(b));
}
void notifyAdv(){
  char b[48];
  snprintf(b,sizeof(b),"ADV:%.1f,%d,%.2f", SLEW, HDG_SMOOTH, DALPHA);
  notify(String(b));
}
void notifyPid2(){    // v2.6 yaw-rate gains
  char b[64];
  snprintf(b,sizeof(b),"PID2:%.3f,%.3f,%.0f", KD_RATE, RATE_DEADBAND, RATE_SIGN);
  notify(String(b));
}
void notifyFeat(){    // v2.7 feature toggles
  char b[32];
  snprintf(b,sizeof(b),"FEAT:%d,%d,%d",
           USE_RATE_D?1:0, USE_CAL_GATE?1:0, USE_BNO_HDG?1:0);
  notify(String(b));
}
void notifyAutoQ(){   // v2.6 autopilot quality / what D source is active
  char b[48];
  snprintf(b,sizeof(b),"AUTOQ:%d,%u,%u,%d",
           (bnoOk ? 1 : 0), bnoCalGyro, bnoCalSys,
           (rateUsedLastTick ? 1 : 0));
  notify(String(b));
}
// v2.6 — high-rate tuning telemetry (sent every PID tick from inside the
// PID function). One line per tick so the HTML tune page can plot:
//   err     — heading error (target - heading) in degrees, -180..+180
//   yawDps  — heading-frame yaw rate in deg/s (already sign-corrected)
//   out     — PID output sent to Motion ESP (-100..+100, after slew & reverse)
//   rateUsed — 1 if yaw-rate D was used this tick, 0 if legacy/fallback
void notifyLive(float err, float yawDps, int out, bool rateUsed){
  if (!deviceConnected) return;
  char b[64];
  snprintf(b,sizeof(b),"LIVE:%.2f,%.2f,%d,%d", err, yawDps, out, rateUsed?1:0);
  notify(String(b));
}
void notifyGps(){
  char b[80];
  snprintf(b,sizeof(b),"GPS:%.6f,%.6f,%.2f,%.1f", gpsLat,gpsLon,gpsSpdMps,gpsCog);
  notify(String(b));
}
void notifyGpsi(){
  char b[64];
  snprintf(b,sizeof(b),"GPSI:%d,%d,%.1f,%.1f", gpsFixQuality, gpsSatsUsed, gpsHdop, gpsAltM);
  notify(String(b));
}
void notifySat(){
  char b[32];
  snprintf(b,sizeof(b),"SAT:%d,%d", gpsSatsUsed, gpsSatsInView);
  notify(String(b));
}
void notifyMag(){
  char b[32];
  snprintf(b,sizeof(b),"MAG:%d,%d,%d", magX,magY,magZ);
  notify(String(b));
}

// ---- Full BNO notifications ----
void notifyBno(){       // orientation + cal — 9 fields
  char b[96];
  snprintf(b,sizeof(b),"BNO:%d,%.1f,%.1f,%.1f,%.3f,%u,%u,%u,%u",
           bnoOk ? 1 : 0,
           bnoHeading, bnoPitch, bnoRoll, bnoYawRate,
           bnoCalSys, bnoCalGyro, bnoCalAccel, bnoCalMag);
  notify(String(b));
}
void notifyBnoA(){      // linear accel + gravity — 6 fields
  char b[96];
  snprintf(b,sizeof(b),"BNOA:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           bnoLinAx, bnoLinAy, bnoLinAz,
           bnoGravX, bnoGravY, bnoGravZ);
  notify(String(b));
}
void notifyBnoM(){      // BNO mag + temperature — 4 fields
  char b[64];
  snprintf(b,sizeof(b),"BNOM:%.2f,%.2f,%.2f,%d",
           bnoMagX, bnoMagY, bnoMagZ, (int)bnoTempC);
  notify(String(b));
}
void notifyBnoG(){      // gyro vector rad/s — 3 fields
  char b[64];
  snprintf(b,sizeof(b),"BNOG:%.3f,%.3f,%.3f", bnoGyroX, bnoGyroY, bnoGyroZ);
  notify(String(b));
}
void notifyBnoQ(){      // quaternion — 4 fields
  char b[80];
  snprintf(b,sizeof(b),"BNOQ:%.4f,%.4f,%.4f,%.4f",
           bnoQuatW, bnoQuatX, bnoQuatY, bnoQuatZ);
  notify(String(b));
}

void notifySys(){
  char b[48];
  snprintf(b,sizeof(b),"SYS:%lu,%u",
           (unsigned long)(millis()/1000),
           (unsigned)ESP.getFreeHeap());
  notify(String(b));
}
void notifyReverse(){
  String s = "REV:"; s += (steeringReversed ? "1" : "0");
  notify(s);
}

// =====================================================================
// PID  (v2.6 — derivative on yaw rate, not on error)
// =====================================================================
//
// Standard form:   u = Kp*err + Ki*∫err dt - Kd_rate * headingRate
//
// headingRate is the boat's actual yaw rate, in deg/s, with the SAME sign
// convention as heading (positive = heading is INCREASING, i.e. boat is
// yawing CW from above).
//
// The BNO055 gyro reports body-frame angular velocity. For the default
// Adafruit axis remap with the chip mounted upright, gyro Z's sign is
// OPPOSITE to the heading-rate convention — so we apply RATE_SIGN
// (default -1.0) to flip it.
//
// If the boat steers FASTER away from target when KD_RATE > 0, the sign
// is wrong: send "RSIGN:1" (or -1) to flip it without recompiling.
// =====================================================================
void runPidIfDue(){
  unsigned long now=millis();
  if (now-lastPidMs < PID_INTERVAL_MS) return;
  float dt=(now-lastPidMs)/1000.0f; if (dt>1.0f) dt=PID_INTERVAL_MS/1000.0f;
  lastPidMs=now;
  if (!autoEngaged || !haveTarget){
    integralTerm=0; prevError=0; dFiltered=0; prevOutput=0;
    rateUsedLastTick = false;
    return;
  }
  float err = angleErr(currentTarget, currentHeading);

  // --- Decide which D source we can trust this tick ---------------------
  // Yaw-rate D requires: feature enabled, BNO present, data fresh, and
  // (optionally) calibrated gyro. Falls back to legacy error-D otherwise.
  bool useRateD = USE_RATE_D
                  && bnoOk
                  && (now - bnoLastReadMs) < 500
                  && (!USE_CAL_GATE || bnoCalGyro >= 1);
  rateUsedLastTick = useRateD;

  // Yaw rate in heading-convention deg/s.
  // bnoYawRate is gyro Z in rad/s as reported by the sensor.
  float headingRate_dps = RATE_SIGN * bnoYawRate * 57.29577951f;

  // Apply a small deadband to the rate signal so sensor noise doesn't
  // produce constant low-level D activity while sitting still.
  float rateForD = headingRate_dps;
  if (fabsf(rateForD) < RATE_DEADBAND) rateForD = 0.0f;

  // --- Inside heading-error deadband: ramp output down to zero ---------
  if (fabsf(err) < DEADBAND){
    integralTerm *= 0.9f;
    float maxStep = SLEW * dt;
    if (prevOutput > maxStep)       prevOutput -= maxStep;
    else if (prevOutput < -maxStep) prevOutput += maxStep;
    else                            prevOutput = 0;
    int sendVal = (int)round(steeringReversed ? -prevOutput : prevOutput);
    Serial2.print("ERR:"); Serial2.println(sendVal);
    notifyLive(err, headingRate_dps, sendVal, useRateD);
    prevError = err;
    dFiltered = 0.0f;       // reset legacy D filter while parked
    return;
  }

  // --- Build the D term ------------------------------------------------
  float D;
  if (useRateD){
    // Derivative on measurement. Note the MINUS: D opposes motion.
    D = -KD_RATE * rateForD;
    // Keep the legacy filter state coherent in case we fall back later.
    dFiltered = 0.0f;
  } else {
    // Legacy fallback: derivative of error, EMA-filtered.
    float dRaw = (err - prevError) / dt;
    dFiltered = DALPHA * dFiltered + (1.0f - DALPHA) * dRaw;
    D = KD * dFiltered;
  }

  float P = KP * err;
  float prelim = P + KI * integralTerm + D;

  // Anti-windup: only integrate when not saturated.
  if (prelim > -100.0f && prelim < 100.0f){
    integralTerm += err * dt;
    float lim = (KI > 0.0001f) ? (100.0f / KI) : 100.0f;
    integralTerm = clampf(integralTerm, -lim, lim);
  }

  float target = clampf(P + KI * integralTerm + D, -100.0f, 100.0f);

  // Slew-rate limit on output (helm-calming).
  float maxStep = SLEW * dt;
  float delta = target - prevOutput;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  prevOutput += delta;

  prevError = err;
  float out = steeringReversed ? -prevOutput : prevOutput;
  int outI = (int)round(out);
  Serial2.print("ERR:"); Serial2.println(outI);
  notifyLive(err, headingRate_dps, outI, useRateD);
}

// =====================================================================
// BLE callbacks
// =====================================================================
class RxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *p) override {
    String v = p->getValue(); v.trim();
    if (v.length()==0) return;

    if (v.startsWith("TARGET:")){
      currentTarget = wrap360(v.substring(7).toFloat());
      if (!haveTarget){ resetHelmFilters(); }
      haveTarget=true; saveTarget(); notifyTarget(); return;
    }
    if (v.startsWith("MODE:")){
      String m=v.substring(5); m.trim();
      if (m.equalsIgnoreCase("AUTO")){
        autoEngaged=true; resetHelmFilters(); lastPidMs=millis();
      } else if (m.equalsIgnoreCase("STANDBY")){
        autoEngaged=false; Serial2.print("ERR:0\n");
        prevOutput=0; dFiltered=0; integralTerm=0;
      }
      Serial2.print(v); Serial2.print('\n'); return;
    }
    if (v.startsWith("KP:"))      { KP      = v.substring(3).toFloat(); savePid(); notifyGains(); return; }
    if (v.startsWith("KI:"))      { KI      = v.substring(3).toFloat(); savePid(); notifyGains(); return; }
    if (v.startsWith("KD:"))      { KD      = v.substring(3).toFloat(); savePid(); notifyGains(); return; }
    if (v.startsWith("DEADBAND:")){ DEADBAND= v.substring(9).toFloat(); savePid(); notifyGains(); return; }
    // v2.6 yaw-rate damping
    if (v.startsWith("KDR:")){
      KD_RATE = clampf(v.substring(4).toFloat(), 0.0f, 50.0f);
      savePid2(); notifyPid2(); return;
    }
    if (v.startsWith("RDB:")){
      RATE_DEADBAND = clampf(v.substring(4).toFloat(), 0.0f, 5.0f);
      savePid2(); notifyPid2(); return;
    }
    if (v.startsWith("RSIGN:")){
      float s = v.substring(6).toFloat();
      RATE_SIGN = (s >= 0.0f) ? 1.0f : -1.0f;
      savePid2(); notifyPid2(); return;
    }
    // v2.7 feature toggles. Accept "1"/"0", "ON"/"OFF", "TRUE"/"FALSE".
    auto parseBool = [](String s)->bool{
      s.trim();
      if (s.equalsIgnoreCase("ON") || s.equalsIgnoreCase("TRUE")) return true;
      if (s.equalsIgnoreCase("OFF")|| s.equalsIgnoreCase("FALSE"))return false;
      return s.toInt() != 0;
    };
    if (v.startsWith("URD:")){
      USE_RATE_D = parseBool(v.substring(4));
      saveFeat(); notifyFeat(); return;
    }
    if (v.startsWith("UCG:")){
      USE_CAL_GATE = parseBool(v.substring(4));
      saveFeat(); notifyFeat(); return;
    }
    if (v.startsWith("UBH:")){
      USE_BNO_HDG = parseBool(v.substring(4));
      // reset heading buffer on source change to avoid a step transient
      hdgBufIdx = 0; hdgBufCount = 0;
      saveFeat(); notifyFeat(); return;
    }
    if (v.startsWith("SLEW:")){
      SLEW = clampf(v.substring(5).toFloat(), 1.0f, 1000.0f);
      saveAdv(); notifyAdv(); return;
    }
    if (v.startsWith("SMOOTH:")){
      int s = v.substring(7).toInt();
      if (s < 1) s = 1; if (s > HDG_SMOOTH_MAX) s = HDG_SMOOTH_MAX;
      HDG_SMOOTH = s;
      hdgBufIdx = 0; hdgBufCount = 0;
      saveAdv(); notifyAdv(); return;
    }
    if (v.startsWith("DALPHA:")){
      DALPHA = clampf(v.substring(7).toFloat(), 0.0f, 0.95f);
      saveAdv(); notifyAdv(); return;
    }
    if (v.startsWith("REVERSE:")){
      String r = v.substring(8); r.trim();
      bool newVal = (r.toInt() != 0) || r.equalsIgnoreCase("ON") || r.equalsIgnoreCase("TRUE");
      if (newVal != steeringReversed){
        steeringReversed = newVal; saveReverse();
      }
      notifyReverse(); return;
    }
    if (v.startsWith("ERR:") || v.startsWith("ERR ")){
      if (!haveTarget){
        Serial2.print(v);
        if (!v.endsWith("\n")) Serial2.print('\n');
      }
      return;
    }
    Serial2.print(v);
    if (!v.endsWith("\n")) Serial2.print('\n');
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected=true;
    notifyTarget(); notifyGains(); notifyAdv(); notifyPid2(); notifyFeat();
    if (gpsHaveFix) notifyGps();
    notifyGpsi(); notifySat(); notifySys();
    notifyBno(); notifyBnoA(); notifyBnoM(); notifyBnoG(); notifyBnoQ();
    notifyReverse(); notifyAutoQ();
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected=false;
    autoEngaged=false;
    Serial2.print("MODE:STANDBY\n");
    Serial2.print("ERR:0\n");
    prevOutput=0; dFiltered=0; integralTerm=0;
    BLEDevice::startAdvertising();
  }
};

// =====================================================================
// Forwarders / pumps
// =====================================================================
void pumpMotionUart(){
  while (Serial2.available()){
    char c=Serial2.read();
    if (c=='\n'){
      motionBuffer.trim();
      if (motionBuffer.startsWith("STATE:")){
        notify(motionBuffer);
        Serial.print("FROM_MOTION: "); Serial.println(motionBuffer);
      }
      motionBuffer="";
    } else if (c!='\r'){
      motionBuffer += c;
      if (motionBuffer.length()>64) motionBuffer="";
    }
  }
}

void pumpGpsUart(){
  static unsigned long lastRawForwardMs = 0;
  const unsigned long RAW_FORWARD_INTERVAL_MS = 500;
  while (GPSserial.available()){
    char c=(char)GPSserial.read();
    if (c=='\n'||c=='\r'){
      if (gpsLineBuf.length() > 0){
        Serial.print("GPS> "); Serial.println(gpsLineBuf);
        if (deviceConnected && (millis() - lastRawForwardMs) > RAW_FORWARD_INTERVAL_MS){
          lastRawForwardMs = millis();
          String s = "RAW:" + gpsLineBuf;
          if (s.length() > 100) s = s.substring(0, 100);
          notify(s);
        }
        if (gpsLineBuf.length()>=7 && gpsLineBuf.charAt(0)=='$'){
          parseNmeaLine(gpsLineBuf);
        }
      }
      gpsLineBuf="";
    } else {
      gpsLineBuf += c;
      if (gpsLineBuf.length()>120) gpsLineBuf="";
    }
  }
  if (gpsHaveFix && (millis()-gpsLastFixMs) > GPS_STALE_MS){
    gpsHaveFix=false; gpsFixQuality=0;
  }
  unsigned long now=millis();
  if (deviceConnected){
    if (gpsHaveFix && (now-gpsLastNotifyMs > GPS_NOTIFY_PERIOD)){
      gpsLastNotifyMs=now; notifyGps();
    }
    if (now-gpsiLastNotifyMs > GPSI_NOTIFY_PERIOD){
      gpsiLastNotifyMs=now; notifyGpsi();
    }
    if (now-satLastNotifyMs > SAT_NOTIFY_PERIOD){
      satLastNotifyMs=now; notifySat();
    }
  }
}

// =====================================================================
// SETUP / LOOP
// =====================================================================
void setup(){
  Serial.begin(115200);
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial2.begin(MOTION_BAUD, SERIAL_8N1, MOTION_RX_PIN, MOTION_TX_PIN);

  configureUbloxM10();

  Wire.begin(21,22);
  compass.init();

  bnoOk = bno.begin();
  if (bnoOk){
    delay(1000);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 found at 0x29 — using BNO fused heading as primary");
  } else {
    Serial.println("BNO055 not found at 0x29 — falling back to QMC5883L heading");
  }

  loadPrefs();
  resetHelmFilters();

  BLEDevice::init("NAV_ESP");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  bleTx = pService->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  bleRx = pService->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
  bleRx->setCallbacks(new RxCallbacks());
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial2.print("MODE:STANDBY\n");
  Serial2.print("ERR:0\n");

  Serial.println("NAV v2.7 ready (selectable BNO055 features + yaw-rate damped autopilot)");
}

void loop(){
  pumpMotionUart();
  pumpGpsUart();

  unsigned long now = millis();

  // Heading + smoothing at 5 Hz, also pulls full BNO data into globals.
  if (now - lastHdgMs > HDG_INTERVAL_MS){
    lastHdgMs = now;
    float raw = readCompassRaw();   // also fills bno* globals if BNO present
    currentHeading = pushHdg(raw);
    if (deviceConnected){
      char b[24];
      snprintf(b,sizeof(b),"HDG:%.1f", currentHeading);
      notify(String(b));
    }
  }

  // BNO message schedule
  if (deviceConnected){
    if (now - bnoLastNotifyMs  > BNO_PERIOD_MS)  { bnoLastNotifyMs  = now; notifyBno();  }
    if (now - bnoaLastNotifyMs > BNOA_PERIOD_MS) { bnoaLastNotifyMs = now; notifyBnoA(); }
    if (now - bnomLastNotifyMs > BNOM_PERIOD_MS) { bnomLastNotifyMs = now; notifyBnoM(); }
    if (now - bnogLastNotifyMs > BNOG_PERIOD_MS) { bnogLastNotifyMs = now; notifyBnoG(); }
    if (now - bnoqLastNotifyMs > BNOQ_PERIOD_MS) { bnoqLastNotifyMs = now; notifyBnoQ(); }

    // Legacy QMC mag — sent when BNO isn't present, OR when the user has
    // forced QMC as the heading source (USE_BNO_HDG off). magX/Y/Z are
    // only refreshed when readCompassRaw() reads the QMC.
    if ((!bnoOk || !USE_BNO_HDG) && (now - magLastNotifyMs > MAG_NOTIFY_PERIOD)){
      magLastNotifyMs = now; notifyMag();
    }
    if (now - sysLastNotifyMs > SYS_NOTIFY_PERIOD){
      sysLastNotifyMs = now; notifySys();
    }
    if (now - autoqLastNotifyMs > AUTOQ_PERIOD_MS){
      autoqLastNotifyMs = now; notifyAutoQ();
    }
  }

  runPidIfDue();
}
