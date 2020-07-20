#include <digitalWriteFast.h>

#include<Arduino.h>
volatile long int pulseCount;
int mA=2;
int mB=3;
#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 
volatile int flag=0;
int dir;
int dT;
int currentTime;
int pastTime; 
float rpm;
float x, x_dot,x_1;
const float r=0.0325;
const float pi=3.14159;
volatile const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
volatile int Old,New,OUT;

void motor_init(){
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void setup() {
  Serial.begin(57600);
  motor_init(); 
  
  attachInterrupt(digitalPinToInterrupt(mA),Counter,FALLING);
  dT=0;
  currentTime=0;
  pastTime=0;
  Old=0;
  New=0;
}


void motorForwardL(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}
void motorReverseL(int PWM_val)  {
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}

void motorForwardR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
void motorReverseR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

void Counter()
{ if(flag==0)
{
  flag=1;
  return;
}
  else
  {
  if(digitalRead(mA)==digitalRead(mB))
  pulseCount++;
  else
  pulseCount--;
  flag=0;
  }
//  else
//  {
//    
//  }  
}


void loop() 
{
  pastTime=currentTime;
  currentTime=millis();
  dT=currentTime-pastTime;
  x_1=x;
  x=(pulseCount*2.0/270.0)*2.0*pi*r;
  x_dot = (x-x_1)*1000.0/dT;
  Serial.print("pulses = ");Serial.print(pulseCount);
  //Serial.print("\tOld= ");Serial.print(Old);
 // Serial.print("\tNew= ");Serial.print(New);
//  Serial.print(" dT2 = ");Serial.print(currentTime);
//  Serial.print(" dT1 = ");Serial.print(pastTime);
//  Serial.print(" dT = ");Serial.print(dT);
  Serial.print("\tx = ");Serial.print(x);
  Serial.print("\tx_dot = ");Serial.println(x_dot);
//  motorReverseL(255);
//  motorReverseR(255);
  delay(5);
}
