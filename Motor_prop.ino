#include <Wire.h>

int Apin = 5;
int Bpin = 9;
int Rpin = 13;
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

    
    if(Serial.available()) { //maybe while(serial.available())
      input = readFloat();

      Serial.println(input);
      
      motorSpeed = input;
      speedCount = 1;
      go = true;
      
    }

    if (go) {
      prop = input;
      setVelos(input);
      go = false;
    }
    

    if(speedCount > 0) {
      start = readSpeed();
      
      if(start == false) {
        continue;
      }
      
      if(speedCount > 4){
        tickCount = tickCount + 1;
        if(speedCount == 5) {
          firstTime = micros();
        }
        //else if(speedCount == 5+12) {
        else if(speedCount > 5) {
          lastTime = micros();
          times[timesIndex] = lastTime - firstTime;
          firstTime = lastTime;
          //tickTime = (lastTime - firstTime)/12;
          int i = 0;
          int counter = 0;
          while(i < 12) {
            if(times[i] > 0) {
              tickTime = tickTime + times[i];
              counter++;
            }
            i++;
          }
          
          if(counter > 0) 
          {
            tickTime = tickTime/counter;
          }
          else
          {
            tickTime = 0;
          }
      
          if(timesIndex < 12) 
          {
            timesIndex += 1;
          }
          else
          {
            timesIndex = 0;
          }
          //speedCount = 0;
          motorSpeed = 1000*1000*1.2/(tickTime);
//          Serial.print("Speed of the motor : ");
//          Serial.println(motorSpeed);
//          Serial.print("Expected speed : ");
//          Serial.println(input);

          proportionalControl(motorSpeed, input);
        }
      }

      speedCount = speedCount + 1;
    }

    

//    if(speedCount%18 == 0) {
//      motorSpeed = 1000*1000*1.2/(tickTime);
//      Serial.print("Speed of the motor (degrees/second): ");
//      Serial.println(motorSpeed);
//      speedCount = speedCount+1;
//    }

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

void proportionalControl(float theta, float theta_d) {
  int Tc = 5;
  //int tic = micros();
  float kp = 0.5;
  float e = theta_d - theta;

  if(tickCount % 50 == 0)
  {
    Serial.print("Speed of the motor : ");
    Serial.println(theta);
    Serial.print("Desired speed : ");
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
