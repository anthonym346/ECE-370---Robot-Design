#include <Wire.h>

// Set wheel radius 'r' and base length 'L'
#define r 2
#define L 3

double x, y, th, dx, dy, dth, Pt, dphi, phi;

//Transition matrix at start
double TP[][4] = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if right wheel
double TR[][4] = {{cos(dphi), -sin(dphi), 0, 0}, {sin(dphi), cos(dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if left wheel (other direction so -dphi)
double TL[][4] = {{cos(-dphi), -sin(-dphi), 0, 0}, {sin(-dphi), cos(-dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Input pins for right wheel sensor (11) and left wheel sensor (13)
int Rpin = 11;
int Lpin = 13;

void setup() {

  pinMode(Rpin, INPUT);
  pinMode(Lpin, INPUT);
  
  //Pins 11 and 13 start their isr when their value changes
  attachInterrupt(digitalPinToInterrupt(Rpin),Right,CHANGE);
  attachInterrupt(digitalPinToInterrupt(Lpin),Left,CHANGE);
  
  //2 tick encoder, 75:1 encoder:wheel
  th = 2*PI/(2*75); //radians per tick
  
  Pt = r*th; //distance per tick
  
  dphi = atan2(Pt,L); //angle turned per tick

  // To multiply distance Pt into right and left arrays
  double tempR[][4] = {{1, 0, 0, Pt}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  double tempL[][4] = {{1, 0, 0, Pt}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  double tempTR[4][4];
  
  memcpy(tempTR,TR,sizeof(TR)); //copy of TR, since multiply value is stored in TR
  
  multiply(tempTR,tempR,TR);

  double tempTL[4][4];
  
  memcpy(tempTL,TL,sizeof(TL)); //copy of TL, since multiply value is stored in TL

  multiply(tempTL,tempL,TL);
}

void loop() {
  // print position and angle

  Serial.print("X: ");
  Serial.println(TP[0][3]);
  Serial.print("Y: ");
  Serial.println(TP[1][3]);
  Serial.print("Angle: ");
  Serial.println(atan2(TP[2][1],TP[1][1]));

}

// isr when right wheel sensor detects encoder tick
void Right() {

  double tempTP[4][4];
  
  memcpy(tempTP,TP,sizeof(TP)); //copy of TP, since multiply value is stored in TP
  
  multiply(tempTP,TR,TP); //multiply matrices to get new position/angle
}

// isr when left wheel sensor detects encoder tick
void Left() {
  double tempTP[4][4];
  
  memcpy(tempTP,TP,sizeof(TP)); //copy of TP, since multiply value is stored in TP
  
  multiply(tempTP,TL,TP); //multiply matrices to get new position/angle
}

// matrix product
void multiply(double mat1[][4], double mat2[][4], double res[][4]) 
{ 
    int i, j, k; 
    
    for (i = 0; i < 4; i++) 
    { 
        for (j = 0; j < 4; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < 4; k++) 
                res[i][j] += mat1[i][k]*mat2[k][j]; 
        } 
    } 
} 
