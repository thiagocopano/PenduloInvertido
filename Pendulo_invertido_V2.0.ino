/*
   INVERTED PENDULUM CODE

   For more information: http://www.moreinformation.com

   Writed by:
    - Alan Nascimento
    - Hartson
    - Thiago Copano
    - Daniela Oliveira
    - Bruna Kapfenberger
    - Larissa Bassanes

*/
// includes PID library
#include <PID_v1.h>
#include <Encoder.h>

Encoder pendulo(3,2);
Encoder carro(18,19);
long oldPosition = -999;

// Define output pins
#define leftGate_0 6 //in1
#define rightGate_0 7 //in2
#define rightGate_1 8 //in3
#define leftGate_1 9 //in4

// Define vars for PID controlling
int key = 0;
double angle = 0;
double pos = 0, Setpoint, Output;
double Kp = 8, Ki =0.2, Kd = 0.3;
// PID settings
PID myPID(&angle, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define vars for printing info
int moveLeft, moveLeft1, moveRight, moveRight1, counter, temp;

void setup() {

  // PID  settings
  Setpoint = 0.00;
  myPID.SetMode(AUTOMATIC);

  // Sets the output pins
  pinMode (leftGate_0, OUTPUT);
  pinMode (leftGate_1, OUTPUT);
  pinMode (rightGate_0, OUTPUT);
  pinMode (rightGate_1, OUTPUT);

  // Begins serial COM
  Serial.begin(1000000);
}
void loop() {

long pos = pendulo.read();
if (pos != oldPosition){
  oldPosition = pos;
}
// define angular position
angle = (pos*0.45);

// Sets the limit for beginning the pendulum
// control (after it reaches 180 degrees)
if (angle == 180) {
  key = 1;
}
// Starts de controlling when the position equals 180 degrees
if (key == 1) {

  // Turn down the car's movement when
  // the pendulum's position is at 180 degrees
  if (angle == 180) {
    //Outputs
    digitalWrite(leftGate_0, LOW);
    digitalWrite(leftGate_1, LOW);
    digitalWrite(rightGate_0, LOW);
    digitalWrite(rightGate_1, LOW);
  } else {
    // Turn the car's movement left when
    // the pendulum's position > 180 degrees
    if (angle > 180) {

      // PID settings
      myPID.SetControllerDirection(REVERSE);
      myPID.Compute();
      // Outputs
      analogWrite(leftGate_0, Output);
      analogWrite(leftGate_1, Output);
      digitalWrite(rightGate_0, LOW);
      digitalWrite(rightGate_1, LOW);
    }
    // Turn the car's movement right when
    // the pendulum's position < 180 degrees
    if (angle < 180) {

      // PID settings
      myPID.SetControllerDirection(DIRECT);
      myPID.Compute();
      // Outputs
      digitalWrite(leftGate_0, LOW);
      digitalWrite(leftGate_1, LOW);
      analogWrite(rightGate_0, Output);
      analogWrite(rightGate_1, Output);
    }

  }
}

// Read and print pins information and pendulum angular position
moveLeft = analogRead(leftGate_0);
moveLeft1 = analogRead(leftGate_1);
moveRight = analogRead(rightGate_0);
moveRight1 = analogRead(rightGate_1);
Serial.print(Output);
Serial.print(" || ");
Serial.println(angle);

  }

