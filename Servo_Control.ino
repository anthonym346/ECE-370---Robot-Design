#include <Wire.h>

int Apin = 5;
int Bpin = 9;
int Rpin = 13;
int R2pin = 12;
int Cpin = 10;
bool tick = 0;
int tickCount = 0;
float prop = 0.0;
int times[] = {0,0,0,0,0,0,0,0,0,0,0,0};
int timesIndex = 0;

//char[100] BUF;
//int ind = 0;
  
void setup() {
  pinMode(Apin, OUTPUT);
  pinMode(Bpin, OUTPUT);
  pinMode(Rpin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int motorDegrees = tickCount * 360 / 4;
  int theta = tickCount * 4 / 75;
  int theta_d = 0;
  bool go = true;
  float input = 0.0;
  int speedCount = 0;
  int firstTime = 0;
  int lastTime = 0; 
  int tickTime = 0;
  int motorSpeed = 0;
  bool start = false;

  //while(Serial.available() == false);

  while(1) {

    if(Serial.available()) { //change to while(serial.available()) and read chars to speed up
      input = readFloat();

      Serial.println(input);
      
      theta_d = input;
      go = true;
      
    }

    if (go) {
      prop = input;
      setVelos(input);
      go = false;
    }
    

    bool A = digitalRead(Rpin);
  
    if(tick != temp)
    {
      tick = temp;
      //Serial.println(tickCount);
    }
    else
    {
      continue;
    }
    
    bool B = digitalRead(R2pin);

    if(B != A) //clockwise
    {
      tickCount = tickCount + 1;
    }
    else
    {
      tickCount = tickCount - 1;
    }
    
    motorDegrees = tickCount * 360 / 4; //Get degrees for the motor
    theta = motorRevolutions / 75; //Get degrees for the output shaft
    
    proportionalControl(theta,theta_d);
    
  }
}

//To read more than one character
//char BUFF[100];
//int ind = 0;
float readFloat() { 
  String c = Serial.readString();

  //Serial.println(c);

  return c.toFloat();

  //convert from ascii ?
  //if('\n' == a) {
  //BUFF[ind] = '/0';
  //ind = 0; 
  //break;    

  //BUFF[ind] = c;
  //ind += 1;
}

int setSpeed(float S) {
  if (S > 230.0) {
    S = 255.0;
  }
  else if (S < 0.3*255.0) {
    S = 0.3*255.0;    
  }

  int out = (int)(S);

  return out;
}

void setVelos(float V) {
  float S = V;

  if (S < 0) {
    S = -S;
  }
  
  unsigned Si = setSpeed(S);

  bool A = LOW;
  bool B = HIGH;

  //Get Direction
  if (V > 0) {
    A = HIGH;
    B = LOW;
  }
  else {
    A = LOW;
    B = HIGH;
  }

  //Serial.println(A);

  //Set Direction
  if (HIGH == A) {
    analogWrite(Bpin, 0);
    analogWrite(Apin, Si);
    //Serial.println(Si);
    //analogWrite(Cpin, S);
  }
  else {
    //Serial.println("reverse");
    analogWrite(Apin, 0);
    analogWrite(Bpin, Si);
  }
  return;
}

//Control motor shaft position by controlling pwm
void proportionalControl(float theta, float theta_d) {
  int Tc = 5;
  //int tic = micros();
  float kp = 0.5;
  float e = theta_d - theta;
  //motorSpeed = 1000*1000*1.2/(tickTime); degrees/second

  if(tickCount % 4 == 0)
  {
    Serial.print("Current Position : ");
    Serial.println(theta);
    Serial.print("Desired Position : ");
    Serial.println(theta_d);
  }
  
  float out = kp*e;
  prop = prop + 0.5*(theta_d-theta+2);
  
  setVelos(prop);
  //Serial.println(theta);
  //int tock = micros();
  //int Tc_1 = Tc - (tock - tic);
  //delayMicroseconds(1000);
  
}
