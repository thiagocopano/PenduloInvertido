#include <PID_v1.h>
#include <Encoder.h>

#define esquerda 6
#define direita  9

#define potSetpoint A0

float Pendulo, lastPendulo = 0;

Encoder linear(19, 18);
double lastLinear;
double linearSetpoint, linearInput, linearOutput;
double linearKp = 3;
double linearKi = 0;
double linearKd = 0;

int linearRead;
PID linPID(&linearInput, &linearOutput, &linearSetpoint, linearKp, linearKi, linearKd, DIRECT);

Encoder angular(3, 2);
double lastAngular;
double angularSetpoint, angularInput, angularOutput;
double angularKp = 1;
double angularKi = 0;
double angularKd = 0;

double Output;

int fimEsquerdo, lastfimEsquerdo;
int fimDireito, lastfimDireito;

int lowSpeed = 255*2/9;
int maxSpeed = 255*1/2;
int posMin;
int posMax;

void setup() {
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.print("Pêndulo Invertido V2");
  delay(500);
  Serial.println();
  Serial.println("Leitura dos Encoders");
  delay(500);




  fimDireito = digitalRead(12);
  fimEsquerdo = digitalRead(11);

  while (fimEsquerdo != 0) {
    fimEsquerdo = digitalRead(11);
    fimDireito = digitalRead(12);

    Serial.println("Loop (1)");
    
    analogWrite(esquerda, lowSpeed);
    digitalWrite(direita, LOW);
  }
  linear.write(0);
  posMin = linear.read();
  delay(2000);
  while (fimEsquerdo != 1 || fimDireito != 0) {
    fimEsquerdo = digitalRead(11);
    fimDireito = digitalRead(12);

  Serial.println("Loop (2)");
    
    analogWrite(direita, lowSpeed);
    digitalWrite(esquerda, LOW);

  }
  posMax = linear.read();

  Serial.print(linear.read());
  delay(2000);

  linearSetpoint = posMax/200;
  linPID.SetMode(AUTOMATIC);
}

void loop() {
  //Car Setpoint control
//  linearRead = analogRead(potSetpoint);
//  linearSetpoint = map(linearRead, 0, 1023, 0, posMax);
//  

  
  linearInput = abs(linear.read()) / 100;
  angularInput = angular.read();


  fimDireito = digitalRead(12);
  fimEsquerdo = digitalRead(11);

  if (linearInput != lastLinear || angularInput != lastPendulo) {

    if (linearInput > linearSetpoint ) {
      linPID.SetControllerDirection(REVERSE);
      linPID.Compute();

      Output = map(linearOutput, 0, 255, 0, maxSpeed);
      analogWrite(esquerda, Output);
      digitalWrite(direita, LOW);


    }
    if (linearInput == linearSetpoint) {
      linPID.Compute();
      digitalWrite(direita, LOW);
      digitalWrite(esquerda, LOW);

    }
    if (linearInput < linearSetpoint) {
      linPID.SetControllerDirection(DIRECT);
      linPID.Compute();

      Output = map(linearOutput, 0, 255, 0, maxSpeed);
      analogWrite(direita, Output);
      digitalWrite(esquerda, LOW);

    }


//    Serial.print("Posição Linear: ");
    Serial.print("[");
    Serial.print(linearInput);
    Serial.print("]");
    Serial.print("  ");
//    Serial.print("Saída PID Linear: ");
    Serial.print("[");
    Serial.print(Output);
//    Serial.print("]");
//    Serial.print("  ");
//    Serial.print("Posição Angular: ");
//    Serial.print("[");
//    Serial.print(angularInput);
    Serial.println("]");
    lastLinear = linearInput;
    lastPendulo = angularInput;
  }
}
