#include<Wire.h>
#include<digitalWriteFast.h>
//Version 4.0 YARK
#define MagF            51                      // electromagnet pin

#define buzz_pin        31                      // Buzzer pin

#define RED_pin         43                      //LED Pin
#define Common_pin      45                      //LED Pin
#define GREEN_pin       47                      //LED Pin
#define BLUE_pin        49                      //LED Pin

#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

#define alphaHL         0.864                   //alpha for HPF and LPF
#define alpha           0.0305                    //alpha for Complimentary filter
#define dT              0.005                   //Sampling frequency
#define f_cut           5                       //Cutoff frequency
#define pi              3.14159                 //pi 
#define r               0.025                  //radius of wheel
#define mA              2                       //Encoder A channel 
#define mB              3                       //Encoder B channel
#define km              0.4304
#define ke              0.382
#define R               6.6

double dt;
unsigned long int currentTime;
unsigned long int pastTime;

//-0.93811  -23.322 121.04  10.342 
//-2.1275  -3.3422 34.507  6.1832
//-2.4755  -23.638 73.674  10.405
//1  0.18005 -19.723 -1.1793
//1.0484  -5.4601 -16.709 -1.5866
//-2.1921  -22.121 51.67 5.3723

#define K1              -5.4755
#define K2              -23.638
#define K3               100.674
#define K4               15.405

const int MPU6050_addr=0x68;
volatile int i;
volatile int flag=0;
volatile float pulseCount;
volatile float ax,ay,az,gx,gy,gz;
volatile float ayx=0;
volatile float ayy=0;
volatile float ayz=0;
volatile float gyx=0;
volatile float gyy=0;
volatile float gyz=0;
volatile float gx_1=0;
volatile float gy_1=0;
volatile float gz_1=0;
volatile float U;
volatile float T;
volatile float V;
volatile int PWM;
volatile double x=0;
volatile double x_1=0;
volatile double x_dot=0;
volatile double theta=0;
volatile double theta_1=0;
volatile double theta_dot=0;
volatile float acc;
volatile int sw;
volatile int X_MSB;
volatile int X_LSB;
volatile int Y_MSB;
volatile int Y_LSB;
volatile int X;
volatile int Y;

const int min_value=0;
const float U_offset=0; 


void motor_init()
{
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void LED_init()
{
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);
    digitalWrite(Common_pin,HIGH);
}
void BUZZ_init()
{
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

void MAG_init()
{
    pinMode(MagF, OUTPUT);
    
    digitalWrite(MagF, LOW);
}

void Motor_INT()
{
  if(flag==0)
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
}
void motor_direction(int X,int Y)
{
    /* Forward */
    if(X>700){         
        motorForwardR(255);
        motorForwardL(255);  
//      delay(250);    
        return;
    }   
    //Stop
    else if((X>300&&X<700)&&(Y>300&&Y<700))
    {
        motorForwardR(0);
        motorForwardL(0); 
//      delay(250);
        return;
    }
    
    /* Reverse */
    else if(X<300){         
        motorReverseR(128);
        motorReverseL(128);  
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

void motorForwardL(int PWM_val)  
{
//  Serial.print("\tPWM=");Serial.println(PWM_val);
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}
void motorReverseL(int PWM_val) 
{
//    Serial.print("\tPWMr=");Serial.println(PWM_val);

    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}

void motorForwardR(int PWM_val)
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
void motorReverseR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}



void readAngle()
{
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,6,true);
  ax=Wire.read()<<8|Wire.read();
  ay=Wire.read()<<8|Wire.read();
  az=Wire.read()<<8|Wire.read();

  ax=ax/16384;
  ay=ay/16384;
  az=az/16384;

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,6,true);
  gx=Wire.read()<<8|Wire.read();
  gy=Wire.read()<<8|Wire.read();
  gz=Wire.read()<<8|Wire.read();

  gx=gx/131.0;
  gy=gy/131.0;
  gz=gz/131.0;
  lowpassfilter();
  highpassfilter();
  comp_filter_roll();
}


void lowpassfilter()
{
//  float alpha;
//  float Tau= 1/(2*pi*f_cut);
//  alpha = Tau/(Tau+dT);
  ayx=(1-alphaHL)*ax+alphaHL*ayx;
  ayy=(1-alphaHL)*ay+alphaHL*ayy;
  ayz=(1-alphaHL)*az+alphaHL*ayz;
  
}
ISR(TIMER1_COMPA_vect){//timer1 interrupt 200Hz samples gyro values and applies LQR(ts=5ms)
  sei();
  readAngle();
}

ISR(TIMER3_COMPA_vect){//timer3 interrupt 200Hz remote i/p(ts=5ms)
  x_1=x;
  x=(pulseCount*2.0/270.0)*2.0*pi*r;
  x_dot=(x-x_1)/dT;
  theta_dot=(theta-theta_1)/dT;
  theta_1=theta;
  U=constrain((K1*x + K2*x_dot + K3*(theta-2.5)/57.3 + K4*(theta_dot)/57.3)*255/12,-255,255);
  
  if(U>0)
  {
    //PWM=constrain(PWM,100,255);
    motorReverseR(U);
    motorReverseL(U);
  }
  else
  {
    //PWM=constrain(PWM,-255,-100);
    motorForwardR(abs(U));
    motorForwardL(abs(U));
  }
}

void highpassfilter()
{
//  float alpha;
//  float Tau= 1/(2*pi*f_cut);
//  alpha = Tau/(Tau+dT);
  gyx=(1-alphaHL)*gyx+(1-alphaHL)*(gx-gx_1);
  gyy=(1-alphaHL)*gyy+(1-alphaHL)*(gy-gy_1);
  gyz=(1-alphaHL)*gyz+(1-alphaHL)*(gz-gz_1);
  gx_1=gx;
  gy_1=gy;
  gz_1=gz;
}


void comp_filter_roll()
  {
//  float alpha=0.03,acc;
  acc=atan(ayx/abs(ayz))*57.3;
  theta=(1-alpha)*(theta+gyy*dT)+alpha*acc;
  }

  
void setup()
{
  Serial.begin(115200);
  motor_init();
  LED_init();
  attachInterrupt(digitalPinToInterrupt(mA),Motor_INT,FALLING);
  Wire.begin();
 // Wire.setClock(400000UL);
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  timer1_init();
  timer3_init();
}


void timer1_init()
{
  cli();//stop interrupts
  //noInterrupts();
  //set timer1 interrupt at 200Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 200hz increments
  OCR1A = 1249;// = (16*10^6) / ((1249+1)*64) (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);//for timer1
  TCCR1B |= (1 << CS11)|(1 << CS10);  // Set CS#1 bit for 64 prescaler for timer 1 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
 // interrupts();
  sei();//allow interrupts
}

void timer3_init()
{
   cli();//stop interrupts
  //noInterrupts();

  //set timer3 interrupt at 200Hz
  TCCR3A = 0;// set entire TCCR1A register to 0
  TCCR3B = 0;// same for TCCR1B
  TCNT3  = 0;//initialize counter value to 0
  // set compare match register for 200hz increments
  OCR3A = 1249;// = (16*10^6) / ((1249+1)*64) (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM32);//for timer3
  TCCR3B |= (1 << CS31)|(1 << CS30); // Set CS#30 and CS#31 bit for 64 prescaler for timer 3 
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);
 // interrupts();
  sei();//allow interrupts
}

void loop()
{
//  pastTime=currentTime;
//  currentTime=millis();
//  dt=currentTime-pastTime;
//   Serial.print("dt=");Serial.print(dt);Serial.print("\t\t");
//  Serial.print("theta=");Serial.print(theta);Serial.print("\t\t");
//  Serial.print("theta_dot=");Serial.print(theta_dot);Serial.print("\t\t");
//  Serial.print("x=");Serial.print(x);Serial.print("\t\t");
//  Serial.print("x_dot=");Serial.print(x_dot);Serial.println("\t\t"); 
}
