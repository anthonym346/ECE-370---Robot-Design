#include <Wire.h>

int Apin = 5;
int Bpin = 9;
int Rpin = 13;
int R2pin = 12;
int tickCount = 0;

//char[100] BUF;
//int ind = 0;
  
void setup() {
  pinMode(Apin, OUTPUT);
  pinMode(Bpin, OUTPUT);
  pinMode(Rpin, INPUT);
  pinMode(R2pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int motorDegrees = tickCount * 360 / 4;
  int theta = motorDegrees / 75;
  int theta_d = 0;
  bool go = false;
  float input = 0.0;
  bool start = false;
  bool tick = digitalRead(Rpin);

  //while(Serial.available() == false);

  while(1) {

    if(Serial.available()) { //change to while(serial.available()) and read chars to speed up
      input = readFloat();

      Serial.println(input);
      
      theta_d = input;

      if(theta_d > 720.0) 
      {
        theta_d = 720.0;
      }
      else if(theta_d < -720.0)
      {
        theta_d = -720.0;
      }
      
      go = true;
      
    }

    if (go) {
      proportionalControl(theta,theta_d);
      go = false;
    }    

    bool A = digitalRead(Rpin);
  
    if(tick != A)
    {
      tick = A;
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
    theta = motorDegrees / 75; //Get degrees for the output shaft
    
    proportionalControl(theta,theta_d);
    
  }
}

//To read more than one character
//char BUFF[100];
//int ind = 0;
float readFloat() { 
  String c = Serial.readString();

  //Serial.println(c);

  if(c.indexOf("R") > -1) //.strcmp if char array
  {
    theta = 0.0;
    return 0.0;
  }

  return c.toFloat();
  //while(Serial.available()){
  //convert from ascii ?
  //if('\n' == a) {
  //BUFF[ind] = '/0';
  //ind = 0; 
  //break;    

  //BUFF[ind] = c;
  //ind += 1;
  //}
}

int setSpeed(float S) {
  if (S >= 1.0) {
    S = 1.0;
  }
  else if (S <= 0.3 & S > 0) {
    S = 0.3;    
  }
  else { S = 0.0; }

  int out = (int)((S)*255.0);

  return out;
}

void setVelos(float V) {
  float S = V;

  if (S < 0) {
    S = -S;
  }
  
  unsigned Si = setSpeed(S);

  bool X = LOW;
  bool Y = HIGH;

  //Get Direction
  if (V > 0) {
    X = HIGH;
    Y = LOW;
  }
  else {
    X = LOW;
    Y = HIGH;
  }

  //Serial.println(A);

  //Set Direction
  if (HIGH == X) {
    analogWrite(Bpin, 0);
    analogWrite(Apin, Si);
    //Serial.println(Si);
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

  if(tickCount % 4 == 0)
  {
    Serial.print("Current Position : ");
    Serial.println(theta);
    Serial.print("Desired Position : ");
    Serial.println(theta_d);
  }
  
  float out = kp*e;

  //Potentially give it buffer if it has trouble settling
  
  if(out > 360.0)
  {
    out = 360.0;
  }
  else if(out < -360.0)
  {
    out = -360.0;
  }

  out = out/360.0;
  
  //prop = prop + 0.5*(theta_d-theta+2);
  
  setVelos(out);
  //Serial.println(theta);
  //int tock = micros();
  //int Tc_1 = Tc - (tock - tic);
  //delayMicroseconds(1000);
  
}
