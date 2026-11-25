// Universal Curve Tracer - polar plotter firmware (Arduino Nano/Uno + 2x A4988 + servo)
// Platen (stepper A) = angle, pen slide (stepper B) = radius. Every curve is converted (x,y)->(r,theta).
// Serial 115200:  H = home | U/D = pen up/down | C id p0 p1 p2 p3 sec asp sq = draw curve
// sec (cross-section for ids 1,2,10): 0 circle 1 ellipse 2 squarish 3 rectangularish 4 triangle 5 hexagon
// asp = height/width (ellipse, rect), sq = squareness n (squarish, rect: 2 round .. 20 sharp). The web demo prints this command.
// ids/params match the website: 1 spiral(a b turns) 2 helix(R pitch turns) 3 hyperbola(a b range)
// 4 parabola(f halfwidth) 5 ellipse(a b tilt) 6 circle(R) 7 line(L angle) 8 cycloid(R d arches)
// 9 cardioid(r k) 10 toroid(R r q) 11 flower(a p q) 12 pattern(R r d layers) 13 polygon(n R rotation) 14 spiralling helix(a b pitch turns)
#include <Servo.h>
#define A_STEP 2
#define A_DIR 3
#define B_STEP 4
#define B_DIR 5
#define EN 8
#define SERVO_PIN 9
#define HOME_SW 10            // normally-open switch to GND at r = 0
const float SPR_A = 3200.0;   // platen steps/rev (200 * 1/16 microstep, 1:1 belt) - CALIBRATE
const float SPM_B = 400.0;    // radial steps/mm (T8x8 screw, 1/16)             - CALIBRATE
const float STEP_MM = 0.5;    // max segment length
const unsigned PULSE_US = 500;
const float TAU = 6.2831853;
Servo pen; long aPos = 0, bPos = 0; float thU = 0, px = 0, py = 0, P[5], SEC[3] = {0, 0.6, 4}; int cid = 1;

void penUp()  { pen.write(60); delay(150); }
void penDown(){ pen.write(95); delay(150); }
void stepTo(long aT, long bT) {
  long da = aT - aPos, db = bT - bPos, n = max(labs(da), labs(db)), ea = 0, eb = 0;
  digitalWrite(A_DIR, da >= 0); digitalWrite(B_DIR, db >= 0); da = labs(da); db = labs(db);
  for (long k = 0; k < n; k++) {
    ea += da; eb += db;
    if (ea >= n) { ea -= n; digitalWrite(A_STEP, 1); }
    if (eb >= n) { eb -= n; digitalWrite(B_STEP, 1); }
    delayMicroseconds(PULSE_US); digitalWrite(A_STEP, 0); digitalWrite(B_STEP, 0); delayMicroseconds(PULSE_US);
  }
  aPos = aT; bPos = bT;
}
void goXY(float x, float y) {                       // straight line in paper space
  float d = hypot(x - px, y - py); int n = max(1, (int)ceil(d / STEP_MM));
  for (int k = 1; k <= n; k++) {
    float xx = px + (x - px) * k / n, yy = py + (y - py) * k / n, r = hypot(xx, yy);
    if (r > 0.3) {                                  // angle undefined at the centre: hold it
      float dth = atan2(yy, xx) - fmod(thU, TAU);
      while (dth >  TAU / 2) dth -= TAU;
      while (dth < -TAU / 2) dth += TAU;
      thU += dth;
    }
    stepTo(lround(thU / TAU * SPR_A), lround(r * SPM_B));
  }
  px = x; py = y;
}
long g(long a, long b) { return b ? g(b, a % b) : a; }
float polyR(int n, float f) { float w = TAU / n; f = fmod(f, w); if (f < 0) f += w; return cos(PI / n) / cos(f - PI / n); }
float rho(float f) {                                // cross-section outline radius at polar angle f
  float A = SEC[1], N = SEC[2];
  switch ((int)SEC[0]) {
    case 1: return 1 / hypot(cos(f), sin(f) / A);
    case 2: return pow(pow(fabs(cos(f)), N) + pow(fabs(sin(f)), N), -1 / N);
    case 3: return pow(pow(fabs(cos(f)), N) + pow(fabs(sin(f) / A), N), -1 / N);
    case 4: return polyR(3, f - PI / 2);
    case 5: return polyR(6, f);
  }
  return 1;
}
int npts() {
  if (cid == 14) return (int)max(1500.0f, min(8000.0f, 400 * P[3]));
  if (cid == 1 || cid == 2) return (int)max(1500.0f, min(8000.0f, 400 * P[2]));
  if (cid == 10) return (int)max(1500.0f, min(8000.0f, 250 * P[2]));
  if (cid == 13) return 40 * (int)P[0];
  return 1200;
}
int segs() { return cid == 3 ? 2 : cid == 12 ? max(1, (int)P[3]) : 1; }
void curve(float u, int i, float &x, float &y) {
  float z = 0, t, r;
  switch (cid) {
    case 1: t = u * P[2] * TAU; r = (P[0] + P[1] * t) * rho(t); x = r * cos(t); y = r * sin(t); break;
    case 2: t = u * P[2] * TAU; r = P[0] * rho(t); x = r * cos(t); y = r * sin(t); z = P[1] * (t / TAU - P[2] / 2); break;
    case 3: t = (2 * u - 1) * P[2]; x = (i ? -1 : 1) * P[0] * cosh(t); y = P[1] * sinh(t); break;
    case 4: x = (2 * u - 1) * P[1]; y = x * x / (4 * P[0]); break;
    case 5: { t = u * TAU; float a = P[0] * cos(t), b = P[1] * sin(t), c = P[2] * DEG_TO_RAD;
              x = a * cos(c) - b * sin(c); y = a * sin(c) + b * cos(c); } break;
    case 6: x = P[0] * cos(u * TAU); y = P[0] * sin(u * TAU); break;
    case 7: { t = (u - .5) * P[0]; float c = P[1] * DEG_TO_RAD; x = t * cos(c); y = t * sin(c); } break;
    case 8: t = (u - .5) * P[2] * TAU; x = P[0] * t - P[1] * sin(t); y = P[0] - P[1] * cos(t); break;
    case 9: { t = u * TAU; float q = P[1] + 1; x = P[0] * (q * cos(t) - cos(q * t)); y = P[0] * (q * sin(t) - sin(q * t)); } break;
    case 10: { t = u * TAU; float m = P[2] * t, sz = P[1] * rho(m), w = P[0] + sz * cos(m); x = w * cos(t); y = w * sin(t); z = sz * sin(m); } break;
    case 11: t = u * TAU * P[2]; r = P[0] * cos(P[1] / P[2] * t); x = r * cos(t); y = r * sin(t); break;
    case 12: { float R = P[0], rr = min(P[1], R - 1); long gg = g((long)R, (long)rr);
               t = u * TAU * rr / gg; float k = (R - rr) / rr, X = (R - rr) * cos(t) + P[2] * cos(k * t), Y = (R - rr) * sin(t) - P[2] * sin(k * t);
               float a = i * TAU / P[3]; x = X * cos(a) - Y * sin(a); y = X * sin(a) + Y * cos(a); } break;
    case 14: t = u * P[3] * TAU; r = (P[0] + P[1] * t) * rho(t); x = r * cos(t); y = r * sin(t); z = P[2] * (t / TAU - P[3] / 2); break;
    case 13: { float s = u * P[0]; int j = min((int)P[0] - 1, (int)s); float v = s - j, a0 = P[2] * DEG_TO_RAD + j * TAU / P[0], a1 = a0 + TAU / P[0];
               x = P[1] * ((1 - v) * cos(a0) + v * cos(a1)); y = P[1] * ((1 - v) * sin(a0) + v * sin(a1)); } break;
  }
  if (cid == 2 || cid == 10 || cid == 14) {                      // 3-D curves: draw projection (yaw 30, pitch 35 as in the demo)
    float yw = 30 * DEG_TO_RAD, pt = 35 * DEG_TO_RAD, X = x * cos(yw) - y * sin(yw), Y0 = x * sin(yw) + y * cos(yw);
    x = X; y = Y0 * sin(pt) + z * cos(pt);
  }
}
void drawCurve() {
  int S = segs(), N = npts();
  for (int i = 0; i < S; i++) {
    float x, y; curve(0, i, x, y); penUp(); goXY(x, y); penDown();
    for (int k = 1; k <= N; k++) { curve((float)k / N, i, x, y); goXY(x, y); }
  }
  penUp(); goXY(0, 0);
}
void homeAxis() {
  digitalWrite(B_DIR, 0);
  while (digitalRead(HOME_SW)) { digitalWrite(B_STEP, 1); delayMicroseconds(800); digitalWrite(B_STEP, 0); delayMicroseconds(800); }
  aPos = bPos = 0; thU = px = py = 0;
}
void setup() {
  pinMode(A_STEP, OUTPUT); pinMode(A_DIR, OUTPUT); pinMode(B_STEP, OUTPUT); pinMode(B_DIR, OUTPUT);
  pinMode(EN, OUTPUT); digitalWrite(EN, LOW); pinMode(HOME_SW, INPUT_PULLUP);
  pen.attach(SERVO_PIN); penUp(); Serial.begin(115200); Serial.setTimeout(100); Serial.println(F("Ready. H=home, C id p0..p3"));
}
void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'H') homeAxis();
  else if (c == 'U') penUp();
  else if (c == 'D') penDown();
  else if (c == 'C') { cid = Serial.parseInt(); for (int k = 0; k < 4; k++) P[k] = Serial.parseFloat();
    SEC[0] = Serial.parseInt(); SEC[1] = Serial.parseFloat(); SEC[2] = Serial.parseFloat();
    if (SEC[1] <= 0) SEC[1] = 0.6; if (SEC[2] < 2) SEC[2] = 4; drawCurve(); Serial.println(F("Done")); }
}
