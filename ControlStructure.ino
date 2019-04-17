#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 
#include <Wire.h>

// Set wheel radius 'r' and base length 'L'
#define r 2
#define L 3

double x, y, dx, dy, dth, phi;

//2 tick encoder, 80:1 encoder:wheel  
double th = 2*PI/(2*80); //radians per tick

double Pt = r*th; //distance per tick

double dphi = atan2(Pt,L); //angle turned per tick

//Transition matrix at start
double TP[][4] = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if right wheel
double TR[][4] = {{cos(dphi), -sin(dphi), 0, 0}, {sin(dphi), cos(dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if left wheel (other direction so -dphi)
double TL[][4] = {{cos(-dphi), -sin(-dphi), 0, 0}, {sin(-dphi), cos(-dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

int Ml, Mr;

int IR1 = 13;
int IR2 = 15;
int Motor1 = 5;
int Motor2 = 9;

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

struct instructions 
{
  int velos;
  int theta;
  int mode;
};

typedef struct instructions inst;

inst a;

void setup() {
  AP();

  OpenPort();

  InitStruct();

  setIR(In);

  setOdomMatrices();

  MotorSetup();

  IMUSetup(2);
}

void loop() {
  int V, theta;
  
  // put your main code here, to run repeatedly:
  checkIMU();

  a = checkUDP(); --no blocking

   theta_d = a.theta;
   vel_d = a.vel
  (V,theta) = parseInput(a);

  setSpeed(V);

  setDir(theta);

  if(picked_up) 
  {
    Ml = 0;
    Mr = 0;
  }

  setMot(Ml,Mr);
}

void SetIR(In)
{
  pinmode(IR1,INPUT_PULLUP);
  pinmode(IR2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IR1),Right,RISING);
  attachInterrupt(digitalPinToInterrupt(IR2),Left,RISING);
}

void setOdomMatrices()
{
  // To multiply distance Pt into right and left arrays
  double tempR[][4] = {{1, 0, 0, Pt/2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  double tempL[][4] = {{1, 0, 0, Pt/2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  double tempTR[4][4];
  
  memcpy(tempTR,TR,sizeof(TR)); //copy of TR, since multiply value is stored in TR
  
  multiply(tempTR,tempR,TR);

  double tempTL[4][4];
  
  memcpy(tempTL,TL,sizeof(TL)); //copy of TL, since multiply value is stored in TL

  multiply(tempTL,tempL,TL);
}

void MotorSetup()
{
  pinmode(motorPin1,OUTPUT);
  pinmode(motorPin2,OUTPUT);

  Vel = 0;

  set_P_Dir();
}

void InitStruct() 
{
  
}

struct checkUDP()
{
  a = getPort();

  if(a.mode == m_reset) { pick_up = false;}

  //a.velos;
  //a.theta;
}

void checkIMU()
{
  int ax2 = ax;
  int ay2 = ay;
  int az2 = az;
  
  ax, ay, az = getIMU();

  Jx = abs(ax - ax2);

  Jy = abs(ay - ay2);

  Jz = abs(az -az2);

  if(Jx > ThreshX) { pick_up = true;}
  if(Jy > ThreshY) { pick_up = true;}
  if(Jz > ThreshZ) { pick_up = true;}

}

void AP()
{
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid);

  
}

void OpenPort()
{
  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


//Odometry
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
void multiply(double m[][4], double n[][4], double res[][4]) 
{ 
    int i, j, k; 
    
    for (i = 0; i < 4; i++) 
    { 
        for (j = 0; j < 4; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < 4; k++) 
                res[i][j] += m[i][k]*n[k][j]; 
        } 
    } 
} 
