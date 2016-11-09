/*
   Wearable device that reads from 4 x flex sensors
   and sends a string of values to another Espresso
   on the same network

   /dev/cu.usbserial-AH03IYRS
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP Udp;

//IPAddress ipAP(192, 168, 4, 1); //Access Point IP address
//IPAddress ipMulti(239, 0, 0, 57); //multicast IP address
IPAddress ipLocal(192, 168, 1, 23); //local IP address
IPAddress ipTarget(192, 168, 1, 22); //target node IP address
IPAddress gateway(192, 168, 1, 9);
IPAddress subnet(255, 255, 255, 0);
//unsigned int portMulti = 12345;
unsigned int portLocal = 7778;
unsigned int portTarget = 4210;

char incomingPacket[255];
char outgoingPacket[255];
char replyPacket[] = "OK";

#define FLEX_MASTER_READ_PIN A0
#define SEL_0_PIN 12
#define SEL_1_PIN 14
#define NUM_FLEX_PINS 4

uint8_t brightLevels[NUM_FLEX_PINS];

int sendInterval = 50;//data writeout interval in ms
long lastWriteOutTime;

void setup() {

  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.config(ipLocal, gateway, subnet);
  WiFi.begin("Roving Spectres", "spanglei");

  Serial.print("Connecting ");
  while (WiFi.status() != WL_CONNECTED) { //connecting is 6, connected is 3
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("connected");

  Udp.begin(portLocal);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), portLocal);

  pinMode(FLEX_MASTER_READ_PIN, INPUT_PULLUP);
  pinMode(SEL_0_PIN, OUTPUT);
  pinMode(SEL_1_PIN, OUTPUT);
}

void loop() {

  read_flexpins();

  udp_listen();

  udp_send();
}

void read_flexpins() {

  int myFlexReading;

  for (int i = 0; i < NUM_FLEX_PINS; i++) {

    switch (i) {
      case (0):
        digitalWrite(SEL_0_PIN, LOW);
        digitalWrite(SEL_1_PIN, LOW);
        break;
      case (1):
        digitalWrite(SEL_0_PIN, HIGH);
        digitalWrite(SEL_1_PIN, LOW);
        break;
      case (2):
        digitalWrite(SEL_0_PIN, LOW);
        digitalWrite(SEL_1_PIN, HIGH);
        break;
      case (3):
        digitalWrite(SEL_0_PIN, HIGH);
        digitalWrite(SEL_1_PIN, HIGH);
        break;
    }
    
    myFlexReading = analogRead(FLEX_MASTER_READ_PIN);

    int minVal = 450, maxVal = 550;

    if (myFlexReading < minVal) myFlexReading = minVal;
    if (myFlexReading > maxVal) myFlexReading = maxVal;

    myFlexReading = map(myFlexReading, minVal, maxVal, 0, 255);

//    Serial.printf("%d: %d\t",i, myFlexReading);

    brightLevels[i] = myFlexReading;

    delay(5);
  }

  Serial.println();
}

void udp_listen() {

  int packetSize = Udp.parsePacket();

  if (packetSize) {
    // receive incoming UDP packets
    //    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    //    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    Serial.print("udp in: "); Serial.println(incomingPacket);
  }
}

void udp_send() {

  String outgoingString = String(brightLevels[0]) + "," + String(brightLevels[1]) + "," + String(brightLevels[2])  + "," + String(brightLevels[3])  + "\n";

  if (millis() - lastWriteOutTime > sendInterval) {

    outgoingString.toCharArray(outgoingPacket, 255);

    Udp.beginPacket(ipTarget, portTarget);
    Udp.write(outgoingPacket);
    Udp.endPacket();

    Serial.print("udp out: "); Serial.println(outgoingString);
    lastWriteOutTime = millis();
  }
}



