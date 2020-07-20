#include<Wire.h>
//Version1 Gyro
const int MPU6050_addr=0x68;
float ax,ay,az,gx,gy,gz;
const float dT=0.005;
const float pi=3.14159265;
float pitch=0;
float roll=0;
float ayx=0;
float ayy=0;
float ayz=0;
float gyx=0;
float gyy=0;
float gyz=0;
float gx_1=0;
float gy_1=0;
float gz_1=0;
const int f_cut=5;
float dt;
float dt1;
float dt2;


void setup() {

  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(57600);
  
  //INTERRUPTS
  cli();//stop interrupts

  //set timer1 interrupt at 200Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 100hz increments
  OCR1A = 9999;// = (16*10^6) / (100*8) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);//for timer1
  TCCR1B |= (1 << CS11);  // Set CS#1 bit for 8 prescaler for timer 1 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
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
}
void lowpassfilter()
{
  float alpha;
  float Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);
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
  float alpha;
  float Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);
  gyx=(1-alpha)*gyx+(1-alpha)*(gx-gx_1);
  gyy=(1-alpha)*gyy+(1-alpha)*(gy-gy_1);
  gyz=(1-alpha)*gyz+(1-alpha)*(gz-gz_1);
  gx_1=gx;
  gy_1=gy;
  gz_1=gz;
}
void comp_filter_roll()
  {
  float alpha=0.03,acc;
  acc=atan(ayx/abs(ayz))*180/pi;
  roll=(1-alpha)*(roll+gyy*dT)+alpha*acc;
  }

  
void loop() {
  // put your main code here, to run repeatedly:
  dt1=dt2;
  dt2=micros();
  dt=dt2-dt1;
  Serial.println(dt);
  lowpassfilter();
  highpassfilter();
  comp_filter_roll();
  Serial.print("\troll=");Serial.println(roll);  
}
