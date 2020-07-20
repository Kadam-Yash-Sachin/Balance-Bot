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
   
    int X_MSB;
    int X_LSB;
    int Y_MSB;
    int Y_LSB;
    int X;
    int Y;//

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
//COUNTER
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
void setup() {
    //Serial.begin(9600);// opens serial port, sets data rate to 9600 bps
    Serial1.begin(9600);

  Serial.begin(57600);
  attachInterrupt(digitalPinToInterrupt(mA),Counter,FALLING);
  dT=0;
  currentTime=0;
  pastTime=0;
  Old=0;
  New=0;
    
    motor_init();
    Serial.println("motor_init_finish");
    
    LED_init();
    Serial.println("LED_init_finish");

    BUZZ_init();
   Serial.println("BUZZ_init_finish");
    
    MAG_init();
    Serial.println("MAG_init_finish");
    
//    timer1_init();                           //for periodic buzzer beep
//    Serial.println("timer1_init_finish");
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
//    if(Serial.read() != 0x83) goto label;
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
    if(sw==0x08)
    {
      MagDrop();
      digitalWrite(BLUE_pin, HIGH);
      digitalWrite(RED_pin, LOW);
      digitalWrite(buzz_pin, HIGH);
    }
    else if(sw==0x18)
    {
      MagPick();
      digitalWrite(BLUE_pin, LOW);
      digitalWrite(RED_pin, HIGH);
      tone( buzz_pin, 1000, 500);
    }
    }
  }

    

//    /* blinking LED in RGB*/
//    for(int i=1; i<=3; i++){
//        if(i%3==0){
//            digitalWrite(RED_pin, HIGH);
//            digitalWrite(GREEN_pin, HIGH); 
//            digitalWrite(BLUE_pin, LOW); 
//            delay(1000);
//        }
//        else if(i%2==0){
//            digitalWrite(RED_pin, HIGH);
//            digitalWrite(GREEN_pin, LOW); 
//            digitalWrite(BLUE_pin, HIGH); 
//            delay(1000);
//        }
//        else{
//            digitalWrite(RED_pin, LOW);
//            digitalWrite(GREEN_pin, HIGH); 
//            digitalWrite(BLUE_pin, HIGH);        
//            delay(1000);
//        }
//    }
//
//    
//    digitalWrite(RED_pin, LOW);
//    digitalWrite(GREEN_pin, LOW); 
//    digitalWrite(BLUE_pin, LOW); 
//    /* blinking LED in RGB*/
    
    
    
}  
    
}


//void timer1_init()
//{
//    cli(); //Clears the global interrupts
//    TIMSK1 = 0x01; //timer1 overflow interrupt enable
//    TCCR1B = 0x00; //stop
//    TCNT1H = 0x0B; //Counter higher 8 bit value
//    TCNT1L = 0xDB; //Counter lower 8 bit value
//    TCCR1A = 0x00;
//    TCCR1C = 0x00;
//    TCCR1B = 0x04; //start Timer, prescaler 256
//    sei();   //Enables the global interrupts
//}
//ISR (TIMER1_OVF_vect)
//{
//    buzz_count=buzz_count+1;
//    TCNT1H = 0x0B; //Counter higher 8 bit value
//    TCNT1L = 0xDB; //Counter lower 8 bit value
//    if (buzz_count>=30 & buzz_count%2==0){
//        digitalWrite(buzz_pin, HIGH);
//    }
//    if (buzz_count>=30 & buzz_count%2!=0){
//        digitalWrite(buzz_pin, LOW);
//    }
//    
//} 
