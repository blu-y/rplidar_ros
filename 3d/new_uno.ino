#include <Stepper.h>
const int stepsPerRevolution = 2048;  // Steps required for one full rotation
Stepper myStepper(stepsPerRevolution, 11, 9, 10, 8);
long totalSteps = 0;  // Total number of steps taken
void setup() {
  myStepper.setSpeed(6);  // Speed set for 10-second rotation
  Serial.begin(9600);
}
void loop() {
  myStepper.step(stepsPerRevolution);
  totalSteps += stepsPerRevolution;
  reportRotation();
}
void reportRotation() {
  float rotations = totalSteps / (float)stepsPerRevolution;
  Serial.print("Total Rotations: ");
  Serial.println(rotations, 3);
}