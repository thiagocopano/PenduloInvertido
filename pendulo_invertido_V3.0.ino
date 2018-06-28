#include <PID_v1.h>
#include <Encoder.h>

#define esquerda 8
#define direita  9


float Pendulo, lastPendulo = 0;

Encoder linear(3, 4);
double lastLinear;
double linearSetpoint, linearInput, linearOutput;
double linearKp = 450;
double linearKi = 10;
double linearKd = 0.2;

int linearRead;
PID linPID(&linearInput, &linearOutput, &linearSetpoint, linearKp, linearKi, linearKd, DIRECT);

Encoder angular(18, 19);
double lastAngular;
double angularSetpoint, angularInput, angularOutput;
double angularKp = 10;
double angularKi = 0;
double angularKd = 0;

int fimEsquerdo, lastfimEsquerdo;
int fimDireito, lastfimDireito;

int posMin;
int posMax;

void setup() {
  pinMode(11, INPUT);
  pinMode(12, INPUT);

  Serial.begin(115200);
  Serial.print("Pêndulo Invertido V3");
  delay(500);
  Serial.println();
  Serial.println("Leitura dos Encoders");
  delay(500);




  fimDireito = digitalRead(11);
  fimEsquerdo = digitalRead(12);

  while (fimEsquerdo != 0) {
    fimEsquerdo = digitalRead(12);
    fimDireito = digitalRead(11);

    Serial.println(fimDireito);
    
    digitalWrite(esquerda, HIGH);
    digitalWrite(direita, LOW);
    
  }
  linear.write(0);
  posMin = linear.read();
  delay(2000);
  while (fimEsquerdo != 1 || fimDireito != 0) {
    fimEsquerdo = digitalRead(12);
    fimDireito = digitalRead(11);

  Serial.println("Loop (2)");
    
    digitalWrite(direita, HIGH);
    digitalWrite(esquerda, LOW);

  }
  posMax = linear.read();

  Serial.print(linear.read());
  delay(2000);

  linearSetpoint = 122.00;
  linPID.SetMode(AUTOMATIC);
}

void loop() {
  //Car Setpoint control
//  linearRead = analogRead(potSetpoint);
//  linearSetpoint = map(linearRead, 0, 1023, 0, posMax);
//  

  
  linearInput = abs(linear.read()) / 100;
  angularInput = angular.read();


  fimDireito = digitalRead(11);
  fimEsquerdo = digitalRead(12);

  if (linearInput != lastLinear || angularInput != lastPendulo) {

    if (linearInput > linearSetpoint ) {
      linPID.SetControllerDirection(REVERSE);
      linPID.Compute();
      
      analogWrite(esquerda, linearOutput);
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

      analogWrite(direita, linearOutput);
      digitalWrite(esquerda, LOW);

    }


//    Serial.print("Posição Linear: ");
    Serial.print("[");
    Serial.print(linearInput);
    Serial.print("]");
    Serial.print("  ");
//    Serial.print("Saída PID Linear: ");
    Serial.print("[");
//    Serial.print(Output);
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
