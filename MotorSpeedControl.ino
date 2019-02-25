#include <Wire.h>

int Apin = 5;
int Bpin = 9;
int Rpin = 13;
int Cpin = 10;
bool tick = 0;
int tickCount = 0;

//char[100] BUF;
//int ind = 0;
  
void setup() {
  pinMode(Apin, OUTPUT);
  pinMode(Bpin, OUTPUT);
  pinMode(Rpin, INPUT);
  Serial.begin(9600);
}

void loop() {
  bool go = true;
  float input = 0.0;
  int speedCount = 0;
  int firstTime = 0;
  int lastTime = 0; 
  int motorSpeed = 0;
  bool start = false;

  //while(Serial.available() == false);

  while(1) {

    
    if(Serial.available()) { //maybe while(serial.available())
      input = readFloat();

      Serial.println(input);
      

      speedCount = 1;
      go = true;
    }

    if (go) {
      setVelos(input);
      go = false;
    }

    if(speedCount > 0 & speedCount <= 305) {
      start = readSpeed();
      
      if(start == false) {
        continue;
      }
      
      if(speedCount > 4){
        tickCount = tickCount + 1;
        if(speedCount == 5) {
          firstTime = micros();
        }
        else if(speedCount == 305) {
          lastTime = micros();
        }
      }

      speedCount = speedCount + 1;
    }

    if(speedCount == 306) {
      motorSpeed = 1000*1000*360/(lastTime - firstTime);
      Serial.print("Speed of the motor (degrees/second): ");
      Serial.println(motorSpeed);
      speedCount = speedCount+1;
    }

  }
}

//To read more than one character
//char BUFF[100];
//int ind = 0;
float readFloat() { 
  String c = Serial.readString();

  //Serial.println(c);

  return c.toFloat();

  //might be in ascii
  //if('\n' == a) {
  //BUFF[ind] = '/0';
  //ind = 0; 
  //break;    

  //BUFF[ind] = c;
  //ind += 1;
}

bool readSpeed() {
  bool temp = digitalRead(Rpin);
  
  if(tick != temp)
  {
    tick = temp;
    //Serial.println(tickCount);
    return true;
  }

  return false;
}

int setSpeed(float S) {
  if (S > 1.0) {
    S = 1.0;
  }
  else if (S < 0.0) {
    S = 0.0;    
  }

  int out = (int)(S*255.0);
  //Serial.println(out);
  return out;
}

void setVelos(float V) {
  float S = V;

  if (S < 0) {
    S = -S;
  }

  Serial.println(V);

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
    Serial.println("reverse");
    analogWrite(Apin, 0);
    analogWrite(Bpin, Si);
  }
  return;
}
