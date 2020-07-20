#include<Wire.h>
#include<digitalWriteFast.h>

//Version1 Gyro
#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

#define alpha_HL        0.761                   //alpha for HPF and LPF
#define alpha           0.03                    //alpha for Complimentary filter
#define dT              0.01                    //Sampling frequency
#define f_cut           5                       //Cutoff frequency
#define pi              3.14159                 //pi 
#define r               0.0325                  //radius of wheel
#define mA              2                       //Encoder A channel 
#define mB              3                       //Encoder B channel
double dt;
unsigned long int currentTime;
unsigned long int pastTime; 
#define K1             -1.00
#define K2             -23.798
#define K3             91.924
#define K4             10.47
 
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
volatile unsigned long int time_ms = 0;
volatile unsigned long int time_sec = 0;

void motor_init(){
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void setup()
{
  Serial.begin(115200);
  motor_init();
  attachInterrupt(digitalPinToInterrupt(mA),Motor_INT,FALLING);
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  currentTime=0;
  pastTime=0;
  
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

void lowpassfilter()
{
//  float alpha;
//  float Tau= 1/(2*pi*f_cut);
//  alpha = Tau/(Tau+dT);
  ayx=(1-alpha_HL)*ax+alpha_HL*ayx;
  ayy=(1-alpha_HL)*ay+alpha_HL*ayy;
  ayz=(1-alpha_HL)*az+alpha_HL*ayz;
}

void highpassfilter()
{

//  float Tau= 1/(2*pi*f_cut);
//  alpha = Tau/(Tau+dT);

  gyx=(1-alpha_HL)*gyx+(1-alpha_HL)*(gx-gx_1);
  gyy=(1-alpha_HL)*gyy+(1-alpha_HL)*(gy-gy_1);
  gyz=(1-alpha_HL)*gyz+(1-alpha_HL)*(gz-gz_1);
  gx_1=gx;
  gy_1=gy;
  gz_1=gz;
}
//void comp_filter_pitch()
//{
//  float  alpha=0.05,acc;
//  acc=atan(ayy/abs(ayz))*180/pi;
//  pitch=(1-alpha)*(pitch+gyx*dT)+alpha*acc;
//}

void comp_filter_roll()
  {
  acc=atan(ayx/abs(ayz))*180/pi;
  theta=(1-alpha)*(theta+gyy*dT)+alpha*acc;
  }
  
void loop()
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
//comp_filter_pitch();
//theta =0.0872;  
  comp_filter_roll();
//Serial.print("pitch=");Serial.print(pitch);
  Serial.print("theta=");Serial.print(theta);Serial.print("\t\t");
  pastTime=currentTime;
  currentTime= millis();
  dt=currentTime-pastTime;
  dt=dt/1000.0;
  Serial.print("dt=");Serial.print(dt);Serial.print("\t\t");
  x_1=x;
  x=(pulseCount*2.0/270.0)*2.0*pi*r;
  x_dot=(x-x_1)/dt;
  theta_dot=(theta-theta_1)/dt;
  theta_1=theta;
  Serial.print("theta_dot=");Serial.print(theta_dot);Serial.print("\t\t");
  Serial.print("gx=");Serial.print(gx);Serial.print("\t\t");
// U = (-2.7501*x -22.742*x_dot + 1.52*(theta) + 0.178*theta_dot);
  Serial.print("x=");Serial.print(x);Serial.print("\t\t");
  Serial.print("x_dot=");Serial.print(x_dot);Serial.print("\t\t"); 
  U=-1*constrain(-(K1*x + K2*x_dot + K3*theta/57.3 + K4*theta_dot/57.3)*255/12,-255,255);
  Serial.print("U=");Serial.println(U);
  
  if(U<0)
  {
    motorReverseR(abs(U-50));
    motorReverseL(abs(U-50));
  }
  else
  {
    motorForwardR(abs(U+50));
    motorForwardL(abs(U+50));
  }
  //delay(5);
}
