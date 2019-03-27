#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 

char ssid[] = SECRET_SSID;       
char pass[] = SECRET_PASS;   
int keyIndex = 0;  

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  WiFi.setPins(8,7,4,2);

  Serial.begin(9600);
  while (!Serial) {

  }

  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);  


  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");

    while (true);
  }



  WiFi.config(IPAddress(192, 168, 1, 24));

  Serial.print("SSID: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");

    while (true);
  }

  delay(5000);

  server.begin();

  printWiFiStatus();
}


void loop() {

  if (status != WiFi.status()) {

    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      byte remoteMac[6];

      Serial.print("Device connected to AP, MAC address: ");
      WiFi.APClientMacAddress(remoteMac);
      printMacAddress(remoteMac);
    } else {

      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available(); 

  if (client) {                     
    Serial.println("new client");  
    String currentLine = "";    
    while (client.connected()) {           
      if (client.available()) {            
        char c = client.read();           
        Serial.write(c);                 
        if (c == '\n') {                  

          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();


            client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");


            client.println();

            break;
          }
          else {     
            currentLine = "";
          }
        }
        else if (c != '\r') {    
          currentLine += c;     
        }


        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led, HIGH);              
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);             
        }
      }
    }

    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());


  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);


  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("Page: ");
  Serial.println(ip);

}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
