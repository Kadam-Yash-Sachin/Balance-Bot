#include<Wire.h>
#include<digitalWriteFast.h>

//Version1 Gyro
#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

#define alpha_HL        0.864                   //alpha for HPF and LPF
#define alpha           0.07                  //alpha for Complimentary filter
#define dT              0.005                   //Sampling frequency
#define f_cut           5                       //Cutoff frequency
#define pi              3.14159                 //pi 
#define r               0.0325                  //radius of wheel
#define mA              2                       //Encoder A channel 
#define mB              3                       //Encoder B channel
double dt;
unsigned long int currentTime;
unsigned long int pastTime; 
#define K1             -2.904
#define K2            -23.393
#define K3              87.65
#define K4              10.261
const int MPU6050_addr=0x68;
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
volatile double x=0;
volatile double x_1=0;
volatile double x_dot=0;
volatile double theta=0;
volatile double theta_1=0;
volatile double theta_dot=0;
volatile float acc;
const int min_value=0;
const float U_offset=0; 


void motor_init(){
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
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
  ayx=(1-alpha)*ax+alpha*ayx;
  ayy=(1-alpha)*ay+alpha*ayy;
  ayz=(1-alpha)*az+alpha*ayz;
  
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt 200Hz samples gyro values(ts=5ms)
  sei();
  readAngle();
}



void highpassfilter()
{
//  float alpha;
//  float Tau= 1/(2*pi*f_cut);
//  alpha = Tau/(Tau+dT);
  gyx=(1-alpha)*gyx+(1-alpha)*(gx-gx_1);
  gyy=(1-alpha)*gyy+(1-alpha)*(gy-gy_1);
  gyz=(1-alpha)*gyz+(1-alpha)*(gz-gz_1);
  gx_1=gx;
  gy_1=gy;
  gz_1=gz;
}


void comp_filter_roll()
  {
//  float alpha=0.03,acc;
  acc=atan(ayx/abs(ayz))*180/pi;
  theta=(1-alpha)*(theta+gyy*dT)+alpha*acc;
  }

  
void setup()
{
  Serial.begin(115200);
  motor_init();
  attachInterrupt(digitalPinToInterrupt(mA),Motor_INT,FALLING);
  Wire.begin();
 // Wire.setClock(400000UL);
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  timer1_init();
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
void loop()
{
  pastTime=currentTime;
  currentTime=millis();
  dt=currentTime-pastTime;
  x=(pulseCount*2.0/270.0)*2.0*pi*r;
  x_dot=(x-x_1)/dT;
  x_1=x;
  theta_dot=(theta-theta_1)/dT;
  theta_1=theta;
  Serial.print("dt=");Serial.print(dt);Serial.print("\t\t");
  Serial.print("theta=");Serial.print(theta);Serial.print("\t\t");
//  U = -(-2.904*x -23.393*x_dot + 1.53*(theta) + 0.1791*theta_dot);
  Serial.print("x=");Serial.print(x);Serial.print("\t\t");
  Serial.print("x_dot=");Serial.print(x_dot);Serial.print("\t\t"); 
  U=constrain(-(K1*x + K2*x_dot + K3*theta/57.3 + K4*theta_dot/57.3)*255/12,-255,255);
  Serial.print("U=");Serial.println(U);
  
  if(U>0)
  {
    motorReverseR(U);
    motorReverseL(U);
  }
  else
  {
    motorForwardR(abs(U));
    motorForwardL(abs(U));
  }
}

 
