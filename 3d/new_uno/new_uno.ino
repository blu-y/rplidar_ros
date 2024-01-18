#include <Stepper.h>
#define MP1  8 // IN1 on the ULN2003
#define MP2  9 // IN2 on the ULN2003
#define MP3  10 // IN3 on the ULN2003
#define MP4  11 // IN4 on the ULN2003
// 28BYJ-48 stepper motor
const int spr = 2048;  // Steps required for one full rotation
// const int rpm = 6;     // rpm = 6; 5-second half rotation
const int rpm = 12;     // rpm = 12; 2.5-second half rotation
Stepper cw(spr, MP4, MP2, MP3, MP1);
Stepper ccw(spr, MP1, MP3, MP2, MP4);
void setup() {
//  cw.moveTo(0)
  cw.setSpeed(rpm);
  ccw.setSpeed(rpm);
  Serial.begin(9600);
}
void loop() {
  cw.step(spr/2);
  ccw.step(spr/2);
}
