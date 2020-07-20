#include<Wire.h> 

volatile unsigned long int time_ms = 0;
volatile unsigned long int time_sec = 0;
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
int dt;
int currentTime;
int pastTime; 

const int MPU6050_addr=0x68;
volatile int flag=0;
volatile float pulseCount;
volatile float acc;
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
volatile float x=0;
volatile float x_1=0;
volatile float x_dot=0;
volatile float theta=0;
volatile float theta_1=0;
volatile float theta_dot=0;
const int min_value=0;
const float U_offset=0; 


void setup() {
  Serial.begin(57600);
  timer1_init();
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  currentTime=0;
  pastTime=0;
  timer4_init();
  start_timer4();
  
}
void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer1 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0xF6; //Counter higher 8 bit value
    TCNT1L = 0x3C; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x03; //start Timer, prescaler 64
    sei();   //Enables the global interrupts
}
void timer4_init()
{
  TCCR4B = 0x00;    // Stop Timer
  TCNT4  = 0xC667;  // 0.0009999593097s (~0.001s)
  OCR4A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR4B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR4C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR4   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR4A = 0x00;
  TCCR4C = 0x00;
}
void start_timer4()
{
  TCCR4B = 0x01;    // Prescaler None 0-0-1
  TIMSK4 = 0x01;    // Enable Timer Overflow Interrupt
}
ISR(TIMER4_OVF_vect)
{
  TCNT4 = 0xC667;   // Reload counter value
  time_ms++;      // Increment ms value
  
  if (time_ms>=1000)
  {
    time_ms = 0;  // Reset ms value
    time_sec++;   // Increments seconds value
  }
}
ISR (TIMER1_OVF_vect)
{
    TIMSK1=0x00;
    angle_read();
    TCNT1H = 0xF6; //Counter higher 8 bit value
    TCNT1L = 0x3C; //Counter lower 8 bit value
    TIMSK1=0x01;
} 
unsigned long epoch()
{
  unsigned long elapsed_time;
  elapsed_time = time_sec*1000 + time_ms;
  return elapsed_time;
}
void angle_read()
{
  gyro_read();
  accel_read();
  lowpassfilter();
  highpassfilter();
  comp_filter_roll();
  
}
void gyro_read()
{  
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
  
}

void accel_read()
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
//  float alpha=0.71;
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
  
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("theta=");Serial.print(theta);Serial.print("\t\t");
  pastTime=currentTime;
  currentTime=epoch();
  dt=currentTime-pastTime;
  Serial.print("dt=");Serial.println(dt);
}
