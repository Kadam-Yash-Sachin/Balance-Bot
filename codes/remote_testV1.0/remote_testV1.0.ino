#include <Arduino.h>


#define MagF            51                     // electromagnet pin

#define buzz_pin        31                     // Buzzer pin

#define RED_pin         43                     //LED Pin
#define Common_pin      45                     //LED Pin
#define GREEN_pin       47                     //LED Pin
#define BLUE_pin        49                     //LED Pin


#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

    int sw;
    int X_MSB;
    int X_LSB;
    int Y_MSB;
    int Y_LSB;
    int X;
    int Y;

int buzz_count=0;
void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);
    digitalWrite(Common_pin,HIGH);
}

void BUZZ_init(){
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

void MAG_init(){
    pinMode(MagF, OUTPUT);
    
    digitalWrite(MagF, LOW);
}

void motor_direction(int X,int Y)
{
    /* Forward */
    if(X>700){         
        motorForwardR(255);
        motorForwardL(255); 
        Serial.println("forward"); 
//        delay(250);    
        return;
    }   
    //Stop
    else if((X>300&&X<700)&&(Y>300&&Y<700))
    {
        Serial.println("Stop");
        motorForwardR(0);
        motorForwardL(0); 
//      delay(250);
        return;
    }
    
    /* Reverse */
    else if(X<300){         
        motorReverseR(128);
        motorReverseL(128);
        Serial.println("Reverse");  
//        delay(250);
        return;    
    } 

    /*Left*/
    else if(Y<300)
    {
      motorReverseR(128);
      motorForwardL(128);
      Serial.println("left");
      return;
    }
    /*Right*/
     else if(Y>700)
    {
      motorReverseL(128);
      motorForwardR(128);
      digitalWrite(BLUE_pin, HIGH);
      Serial.println("Right");
      return;
    }
}
void motor_init(){
    pinMode(MagF, OUTPUT);
    
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void motorForwardL(int PWM_val)  {
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

void MagPick(void)  {
  digitalWrite(MagF, HIGH);
  Serial.println("Mag pick");
}

void MagDrop(void)  {
  digitalWrite(MagF, LOW);
  Serial.println("Mag drop");
}

void setup() {
    Serial.begin(9600);// opens serial port, sets data rate to 9600 bps
    Serial1.begin(9600);
    
    motor_init();
    Serial.println("motor_init_finish");
    
    LED_init();
    Serial.println("LED_init_finish");

    BUZZ_init();
   Serial.println("BUZZ_init_finish");
    
    MAG_init();
    Serial.println("MAG_init_finish");
}

void loop() 
{
  int k=1;
  while(k==1)
{ 
   if(Serial1.available()>19)
  {
    if(Serial1.read()==0x7E){
    for(int i=0;i<11;i++)
    {
      byte discard_byte= Serial1.read();
    }
    // Check Frame Type
    int sw = Serial1.read();
    int X_MSB = Serial1.read();
    int X_LSB = Serial1.read();
    int Y_MSB = Serial1.read();
    int Y_LSB = Serial1.read();
    int X = constrain(X_MSB*256+X_LSB,0,720);
    int Y = constrain(Y_MSB*256+Y_LSB,0,720);
    Serial.print("Switch= ");
    Serial.println(sw,BIN);
    Serial.print("X = ");
    Serial.println(X);
    Serial.print("Y= ");
    Serial.println(Y);
    motor_direction(X,Y);
    if(sw==8)
    {
      MagDrop();
      digitalWrite(BLUE_pin, HIGH);
      digitalWrite(RED_pin, LOW);
      digitalWrite(buzz_pin, HIGH);
    }
    else if(sw==24)
    {
      MagPick();
      digitalWrite(BLUE_pin, LOW);
      digitalWrite(RED_pin, HIGH);
      tone( buzz_pin, 1000, 500);
    }
    }
  }

    
