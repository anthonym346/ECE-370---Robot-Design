#include <Wire.h>

// Set wheel radius 'r' and base length 'L'
#define r 2
#define L 3

double x, y, th, dx, dy, dth, Pt, dphi, phi;

//Input pins for right wheel sensor (11) and left wheel sensor (13)
int Rpin = 11;
int Lpin = 13;

void setup() {

  pinMode(Rpin, INPUT);
  pinMode(Lpin, INPUT);

  //Pins 11 and 13 start their isr when their value changes
  attachInterrupt(digitalPinToInterrupt(Rpin),Right,CHANGE);
  attachInterrupt(digitalPinToInterrupt(Lpin),Left,CHANGE);

  //initial position and angle of 0
  x = 0.0;
  y = 0.0;
  phi = 0.0;

  //2 tick encoder, 75:1 encoder:wheel
  th = 2*PI/(2*75); //radians per tick
  
  Pt = r*th; //distance per tick
  
  dphi = atan2(Pt,L); //angle turned per tick

}

void loop() {
  // print position and angle

  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.print("Angle: ");
  Serial.println(phi);

}

// isr when right wheel sensor detects encoder tick
// estimates change in position and add changes to global variables
void Right() {
  dx = L/2 * sin(phi);
  dy = (L*cos(phi))/2;
  
  x = x + dx;
  y = y + dy;
  phi = phi + dphi;
}

// isr when left wheel sensor detects encoder tick
// estimates change in position and add changes to global variables
void Left() {
  dx = L/2 * sin(phi);
  dy = (L*cos(phi))/2;
  
  x = x + dx;
  y = y + dy;
  phi = phi - dphi;
}
