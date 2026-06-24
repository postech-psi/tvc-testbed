#include <Servo.h>

Servo yaw;
Servo pitch;

// 링키지 비율 반영
// hinge 10deg = yaw servo 24deg, pitch servo 15deg
const int YAW_SERVO_10DEG   = 24;
const int PITCH_SERVO_10DEG = 15;

void sweepAxis(Servo &s, int center, int amplitude, int stepDelay) {
  // center → +amplitude → -amplitude → center
  for (int a = 0; a <= amplitude; a++)      { s.write(center + a); delay(stepDelay); }
  for (int a = amplitude; a >= -amplitude; a--) { s.write(center + a); delay(stepDelay); }
  for (int a = -amplitude; a <= 0; a++)     { s.write(center + a); delay(stepDelay); }
}

void hold(Servo &s, int pos, int duration) {
  s.write(pos);
  delay(duration);
}

void setup() {
  pitch.attach(8);
  yaw.attach(9);
  pitch.write(90);
  yaw.write(90);
  delay(10000);

  // --- 1단계: pitch 단독 (hinge ±10°) ---
  // 천천히 3회 반복
  for (int i = 0; i < 1; i++) {
    sweepAxis(pitch, 90, PITCH_SERVO_10DEG, 30);
    delay(100);
  }
  hold(pitch, 90, 1000);

  // --- 2단계: yaw 단독 (hinge ±10°) ---
  for (int i = 0; i < 1; i++) {
    sweepAxis(yaw, 90, YAW_SERVO_10DEG, 30);
    delay(100);
  }
  hold(yaw, 90, 1000);

  // --- 3단계: TVC 원형 sweep (hinge ±10°) ---
  // 3바퀴
  for (int lap = 0; lap < 15; lap++) {
    for (int deg = 0; deg <= 360; deg += 2) {
      float rad = deg * PI / 180.0;
      pitch.write(90 + (int)(PITCH_SERVO_10DEG * sin(rad)));
      yaw.write(90   + (int)(YAW_SERVO_10DEG   * cos(rad)));
      delay(4);
    }
  }

  // 중립 복귀
  pitch.write(90);
  yaw.write(90);
}

void loop() {}