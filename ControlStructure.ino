#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 
#include <Wire.h>

// Set wheel radius 'r' and base length 'L'
#define r 2
#define L 3

//Placeholder thresholds
#define ThreshX 10
#define ThreshY 10
#define ThreshZ 10

#define m_reset 0

double x, y, dx, dy, dth, phi;

double ax, ay, az, theta_x, theta_y, theta_z;

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
bool picked_up = false;

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

  MotorSetup();

  IMUSetup(2);
}

void loop() {
  int vel_d, theta_d, S, proportional;
  proportional = 0;
  
  checkIMU();

  a = checkUDP(); --no blocking

   theta_d = a.theta;
   vel_d = a.velos;

  //proportional = proportionalControl(vel, vel_d); //add proportional control later
  
  S = setSpeed(proportional);

  setDir(theta);

  if(picked_up) 
  {
    Ml = 0;
    Mr = 0;
  }

  setMot(Ml,Mr);
}

void setMot(int Ml, int Mr)
{
  analogWrite(motorPin1, Ml);
  analogWrite(motorPin2, Mr);
}

void SetIR(In)
{
  pinmode(IR1,INPUT_PULLUP);
  pinmode(IR2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IR1),Right,RISING);
  attachInterrupt(digitalPinToInterrupt(IR2),Left,RISING);
}


void MotorSetup()
{
  pinmode(motorPin1,OUTPUT);
  pinmode(motorPin2,OUTPUT);

  Vel = 0;

  //set_P_Dir(); //add later
}

void InitStruct() 
{


  //Motor //add later

  //UDPInput(Buffer); //add later
  
  //Odom
  
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

void checkUDP()
{
  a = getPort(); //get message from port and format into struct later

  if(a.mode == m_reset) { pick_up = false;}

  //a.velos;
  //a.theta;
}

void checkIMU()
{
  int ax2 = ax;
  int ay2 = ay;
  int az2 = az;
  
  ax, ay, az, theta_x, theta_y, theta_z = getIMU();

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
  //Initialize serial:
  Serial.begin(9600);

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid);

  while (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
  }
 
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

void setDir(float theta, float S)
{
  if (theta_d > theta)
  {
    Ml = 0;
    Mr = S;
    
  }
  else {
    Ml = S;
    Mr = 0;
  }
}
