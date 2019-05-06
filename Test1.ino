#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

float heading;
LSM303 compass;


// Set wheel radius 'r' and base length 'L'
#define r 20
#define L 90

//Robot Modes
#define STOP      0
#define MOVE      1
#define TURN      2
#define SERVO     3
#define PICKED_UP 4

//Motor values
int Ml = 0;
int Mr = 0;

int count = 0;

int mode = 0;
int dir = 0;

int velSet = 0;

IPAddress remoteIp;

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

IPAddress ip(192, 168, 43, 153);
unsigned int localPort = 5005;      // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";
char  BadReply[] = "Incorrect Message";
char  Pose[] = "";

WiFiUDP Udp;

//2 tick encoder, 75:1 encoder:wheel
double th = 2 * PI / (2 * 75); //radians per tick

double Pt = r * th; //distance per tick

double dphi = atan2(Pt, L); //angle turned per tick

//Transition matrix at start
double TP[][4] = {{cos(0), -sin(0), 0, 0}, {sin(0), cos(0), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if right wheel
double TR[][4] = {{cos(dphi), -sin(dphi), 0, 0}, {sin(dphi), cos(dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

//Transition Matrix if left wheel (other direction so -dphi)
double TL[][4] = {{cos(-dphi), -sin(-dphi), 0, 0}, {sin(-dphi), cos(-dphi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

int IR1 = 11;
int IR2 = 12;
int MotorL1 = 6;
int MotorL2 = 10;
int MotorR1 = 9;
int MotorR2 = 5;

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
int baseZ = 0;

float FramePhi = 0.0;

float baseHeading;

void setup() {

  setupUDP();

  Wire.begin();
  //  Serial.println("compass init");
  compass.init();
  Serial.println("enable");
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>) {
    -4274,  -3279,  -1901
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +1717,  +2125,  +4233
  };

  MotorSetup();
  Serial.println("Motor Set Up");

  OdomInit();
  Serial.println("Odometry Set Up");

  SetIR();
  Serial.println("Sensors Set Up");

  //  Setup the IMU:

  
  Serial.println("setup done");
  compass.read();
  baseZ = compass.a.z;
  Serial.println("Loop");

  baseHeading = compass.heading((LSM303::vector<int>) {
    -1, 0, 0 }); //Use for turn 15 degrees if Odometry problem
}

int counter = 1;
int num = 0;
void loop() {
  float phi;
  int heading_d;
  int z = compass.a.z - baseZ;

  compass.read();

  //  if(abs(z) > 7000)
  //  {
  //    mode = PICKED_UP;
  //  }
  //  Serial.print("Ax: ");
  //  Serial.println(compass.a.x);
  //  Serial.print("Ay: ");
  //  Serial.println(compass.a.y);
  //  Serial.print("Az: ");
  //  Serial.println(compass.a.z);

  heading = compass.heading((LSM303::vector<int>) {
    -1, 0, 0 });

  phi = (heading-baseHeading);

  if(phi > 0)
  { phi = 360 - phi;}
  else
  { phi = 360 + phi;}



  //  Serial.println(heading);
  char result[20];
  sprintf(result, "%2.2f", heading);

  counter = counter + 1;

//  if (counter % 100 == 0) {
//    //    Serial.println(heading);
//    //    Udp.beginPacket(remoteIp, 4242);
//    //    Udp.write(result);
//    //    Udp.endPacket();
//  }

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    if (sscanf(packetBuffer, "%d,%d", &mode, &dir) == 2)
    {
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(remoteIp, 4242);
      Udp.write(ReplyBuffer);
      Udp.endPacket();

      switch (dir) {
        case 0:
          heading_d = 360;
          break;
        case 1:
          heading_d = 90;
          break;
        case 2:
          heading_d = 180;
          break;
        case 3:
          heading_d = 270;
          break;
      }
    }
    else
    {
      mode = 0;
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(remoteIp, 4242);
      Udp.write(BadReply);
      Udp.endPacket();
    }

    num = 0;


  }

  if (counter % 1000 == 0)
  {
    int x = TP[0][3];

    int y = TP[1][3];

//    phi = atan2(TP[1][0], TP[0][0]);
//    phi = int(phi);

    char result[20];
    sprintf(result, "Position: (%d,%d,%d)", x, y, phi);
    Udp.beginPacket(remoteIp, 4242);
    Udp.write(result);
    Udp.endPacket();
  }

  //
  switch (mode) {
    case STOP:
      if (num == 0)
      {
        Serial.println("Mode = Stop");
        num = 1;
      }
      Ml = 0;
      Mr = 0;
      setMot();
      break;

    case MOVE:
      if (num == 0)
      {
        Serial.println("Mode = Move");
        num = 1;
      }

      Ml = 255;
      Mr = 255;
      setMot();

      break;

    case TURN:
      if (num == 0)
      {
        Serial.println("Mode = Turn");
        num = 1;
        phi = atan2(TP[1][0], TP[0][0]) * 180 / PI;

        FramePhi = phi;
      }

      phi = atan2(TP[1][0], TP[0][0]) * 180 / PI;

      if (dir == 1)
      {
        MovementControl(90.0, phi - FramePhi);
      }

      if (dir == 2)
      {
        MovementControl(180.0, phi - FramePhi);
      }

      if (dir == 3)
      {
        MovementControl(270.0, phi - FramePhi);
      }


      setMot();
      break;

    case SERVO:
      if (num == 0)
      {
        Serial.println("Mode = Servo");
        num = 1;
      }

      setMot();

      if(dir == 0)
      {
        proportionalControl(heading, 0);
      }
      else if(dir == 1)
      {
        proportionalControl(heading, 90);
      }
      else if(dir == 2)
      {
        proportionalControl(heading, 180);
      }
      else
      {
        proportionalControl(heading, 270);
      }
      
//      proportionalControl(heading, heading_d);
      //        if(Mr < 20){ mode = STOP;}
      break;

    case PICKED_UP:
      if (num < 2)
      {
        Serial.println("Mode = RESET");
        char  Res[] = "Reset";
        Udp.beginPacket(remoteIp, 4242);
        Udp.write(Res);
        Udp.endPacket();
        num = 2;
      }
      Mr = 0;
      Ml = 0;
      setMot();
      break;

    default:

      break;
  }

  delay(0.05);
}

void setMot()
{
  int left;
  int right;
  if (Ml > 50) {
    left = 50;
  }
  else {
    left = Ml;
  }
  if (Mr > 50) {
    right = 50;
  }
  else {
    right = Mr;
  }
  //  Serial.println(left);
  //  Serial.println(right);
  analogWrite(MotorL1, left);
  analogWrite(MotorR1, right);
  
}

void setupUDP()
{
  WiFi.setPins(8, 7, 4, 2);

  Serial.begin(9600);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
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
        res[i][j] += m[i][k] * n[k][j];
    }
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void proportionalControl(float theta, float theta_d) {

  count = count + 1;

  float kp = 1;
  float e = abs(abs(theta_d) - abs(theta));

  if (e > 360) {
    e = 100;
  }

  float out = kp * e;

  if (out > 50.0)
  {
    out = 50.0;
  }
  else if (out < 0.0)
  {
    out = 1.0;
  }

  out = out / 50.0;

  //  int x = TP[0][3];
  //
  //  int y = TP[1][3];

  //  float phi = atan2(TP[1][0],TP[0][0]);

  

  if (count % 250 == 0)
  {
    Serial.print("Current Position : ");
    Serial.println(theta);
    Serial.print("Desired Position : ");
    Serial.println(theta_d);

    char result[20];
    sprintf(result, "Heading: %2.2f", heading);
    Udp.beginPacket(remoteIp, 4242);
    Udp.write(result);
    Udp.endPacket();
  }

  out = out * 200;

  if (theta < theta_d)
  {
    Ml = out;
    Mr = 0;
  }
  else
  {
    if(abs(theta-theta_d) > 180)
    {
      if(theta_d == 0)
      {
        out = 50;
      }
      Ml = out;
      Mr = 0;
    }
    else
    {
      Mr = out;
      Ml = 0;
    }
  }

}

void MovementControl(float theta, float theta_d)
{
  count = count + 1;

  float kp = 1.5;
  float e = abs(theta_d - theta);

  if (e > 100) {
    e = 100;
  }

  float out = kp * e;

  if (out > 50.0)
  {
    out = 50.0;
  }
  else if (out < 0.0)
  {
    out = 1.0;
  }

  out = out / 50.0;

  if (count % 100 == 0)
  {
    int x = TP[0][3];

    int y = TP[1][3];

    float p = atan2(TP[1][0], TP[0][0]);
    p = int(p) % 360;

    char result[20];
    sprintf(result, "Position: (%d,%d,%d)", x, y, p);
    Udp.beginPacket(remoteIp, 4242);
    Udp.write(result);
    Udp.endPacket();
  }

  out = out * 200;
  if (theta < theta_d)
  {
    Ml = out;
    Mr = 0;
  }
  else
  {
    Mr = out;
    Ml = 0;
  }

}

void SetIR()
{
  pinMode(IR1, INPUT_PULLUP);
  pinMode(IR2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IR1), Right, RISING);
  attachInterrupt(digitalPinToInterrupt(IR2), Left, RISING);
}

void MotorSetup()
{
  pinMode(MotorL1, OUTPUT);
  //  pinMode(MotorL2,OUTPUT);
  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  //
  digitalWrite(MotorL1, LOW);
  //  digitalWrite(MotorL2,LOW);
  digitalWrite(MotorR1, LOW);
  digitalWrite(MotorR2, LOW);
}


void OdomInit()
{
  // To multiply distance Pt into right and left arrays
  double tempR[][4] = {{1, 0, 0, Pt / 2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  double tempL[][4] = {{1, 0, 0, Pt / 2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  double tempTR[4][4];

  memcpy(tempTR, TR, sizeof(TR)); //copy of TR, since multiply value is stored in TR

  multiply(tempTR, tempR, TR);

  double tempTL[4][4];

  memcpy(tempTL, TL, sizeof(TL)); //copy of TL, since multiply value is stored in TL

  multiply(tempTL, tempL, TL);
}

//Odometry
// isr when right wheel sensor detects encoder tick
void Right() {

  double tempTP[4][4];

  memcpy(tempTP, TP, sizeof(TP)); //copy of TP, since multiply value is stored in TP

  multiply(tempTP, TR, TP); //multiply matrices to get new position/angle
}

// isr when left wheel sensor detects encoder tick
void Left() {
  double tempTP[4][4];

  memcpy(tempTP, TP, sizeof(TP)); //copy of TP, since multiply value is stored in TP

  multiply(tempTP, TL, TP); //multiply matrices to get new position/angle
}
