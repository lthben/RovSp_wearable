//------------------------WIFI-------------------------
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

//------------------------GESTURE-------------------------

#include <Wire.h>
#include <SparkFun_APDS9960.h>

// Pins on wemos D1 mini
#define APDS9960_INT    13  //AKA GPIO12 -- Interupt pin
#define APDS9960_SDA    0  //AKA GPIO0
#define APDS9960_SCL    5  //AKA GPIO5
// Constants

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
volatile bool isr_flag = 0;

//------------------------OLED-------------------------

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 3 //use 4 if using default SDA SCL pins
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

const int IDLETIME = 250;//time in ms when idle to trigger READY screen
long gestureTriggeredTime;
bool isGestureReady;

//coord for drawing the shapes on the oled screen to show gesture detected. Reduces computation overhead in draw loop.
int upCoord[] = { display.width() / 2, int(0.25 * display.height()) ,
                  display.width() / 2 - int(0.2887 * display.height()), int(0.75 * display.height()),
                  display.width() / 2 + int(0.2887 * display.height()), int(0.75 * display.height())
                };
int downCoord[] = { display.width() / 2, int(0.75 * display.height()) ,
                    display.width() / 2 - int(0.2887 * display.height()), int(0.25 * display.height()),
                    display.width() / 2 + int(0.2887 * display.height()), int(0.25 * display.height())
                  };
int leftCoord[] = { display.width() / 2 + int(0.25 * display.height()), int(0.25 * display.height()) ,
                    display.width() / 2 + int(0.25 * display.height()), int(0.75 * display.height()),
                    display.width() / 2 - int(0.25 * display.height()), int(0.5 * display.height())
                  };
int rightCoord[] = { display.width() / 2 - int(0.25 * display.height()), int(0.25 * display.height()) ,
                     display.width() / 2 - int(0.25 * display.height()), int(0.75 * display.height()),
                     display.width() / 2 + int(0.25 * display.height()), int(0.5 * display.height())
                   };
int nearCoord[] = { display.width() / 2, display.height() / 2, int(0.4 * display.height()) };
int farCoord[] = { display.width() / 2, display.height() / 2, int(0.1 * display.height()) };
int noneCoord[] = { 0, 0, display.width(), display.height(), 0, display.height(), display.width(), 0 };

/****************************
    SETUP
 ****************************/

void setup() {

  Serial.begin(115200);
  //Start I2C with pins defined above
  Wire.begin(APDS9960_SDA, APDS9960_SCL);

  //OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  oled_show("SPLASH");

  //WIFI
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

  //GESTURE
  // Set interrupt pin as input
  pinMode(APDS9960_INT, INPUT);

  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));

  // Initialize interrupt service routine
  attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  if (!apds.disableLightSensor()) {
    Serial.println("failed to disable light sensor");
  } else {
    Serial.println("disabled light sensor");
  }

  if (!apds.disableProximitySensor()) {
    Serial.println("failed to disable proximity sensor");
  } else {
    Serial.println("disabled proximity sensor");
  }

  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }

  pinMode(FLEX_MASTER_READ_PIN, INPUT_PULLUP);
  pinMode(SEL_0_PIN, OUTPUT);
  pinMode(SEL_1_PIN, OUTPUT);

  //ready to begin
  oled_show("READY");
  isGestureReady = true;
}

//uint8_t state = 0, prevState = 1;

void loop() {

  udp_listen();

  udp_send();

  if (isGestureReady) {
    if ( isr_flag == 1) {
      detachInterrupt(APDS9960_INT);

      isGestureReady = false;
      
      handleGesture();
      isr_flag = 0;
      attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
  }

  read_flexpins();

  refresh_ready_screen();

  /*  //testing sensor auto reset
    state = apds.getMode();
    if (state != prevState) {
      Serial.print("mode: ");
      Serial.println(state);
      prevState = state;
    }
    if (state != 77) {
      Serial.println("HANGED");
      oled_show("ERROR");
    }
  */
}

//------------------------GESTURE-------------------------

void interruptRoutine() {
  isr_flag = 1;
}
void handleGesture() {
  if ( apds.isGestureAvailable() ) {

    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("UP");
        oled_show("UP");
        break;
      case DIR_DOWN:
        Serial.println("DOWN");
        oled_show("DOWN");
        break;
      case DIR_LEFT:
        Serial.println("LEFT");
        oled_show("LEFT");
        break;
      case DIR_RIGHT:
        Serial.println("RIGHT");
        oled_show("RIGHT");
        break;
      case DIR_NEAR:
        Serial.println("NEAR");
        oled_show("NEAR");
        break;
      case DIR_FAR:
        Serial.println("FAR");
        oled_show("FAR");
        break;
      default:
        Serial.println("NONE");
        oled_show("NONE");
    }
    gestureTriggeredTime = millis();
  }
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

    delay(2);
  }

  //  Serial.println();
}

//------------------------WIFI-------------------------

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
    //    Serial.print("udp in: "); Serial.println(incomingPacket);
  }
}

void udp_send() {

  String outgoingString = String(brightLevels[0]) + "," + String(brightLevels[1]) + "," + String(brightLevels[2])  + "," + String(brightLevels[3])  + "\n";

  if (millis() - lastWriteOutTime > sendInterval) {

    outgoingString.toCharArray(outgoingPacket, 255);

    Udp.beginPacket(ipTarget, portTarget);
    Udp.write(outgoingPacket);
    Udp.endPacket();

    //    Serial.print("udp out: "); Serial.println(outgoingString);
    lastWriteOutTime = millis();
  }
}

//------------------------OLED-------------------------

void oled_show(String msg) {

  display.clearDisplay();

  if (msg.equals("UP")) {
    display.fillTriangle(upCoord[0], upCoord[1], upCoord[2], upCoord[3], upCoord[4], upCoord[5], WHITE);
  } else if (msg.equals("DOWN")) {
    display.fillTriangle(downCoord[0], downCoord[1], downCoord[2], downCoord[3], downCoord[4], downCoord[5], WHITE);
  } else if (msg.equals("LEFT")) {
    display.fillTriangle(leftCoord[0], leftCoord[1], leftCoord[2], leftCoord[3], leftCoord[4], leftCoord[5], WHITE);
  } else if (msg.equals("RIGHT")) {
    display.fillTriangle(rightCoord[0], rightCoord[1], rightCoord[2], rightCoord[3], rightCoord[4], rightCoord[5], WHITE);
  } else if (msg.equals("NEAR")) {
    display.fillCircle(nearCoord[0], nearCoord[1], nearCoord[2], WHITE);
  } else if (msg.equals("FAR")) {
    display.fillCircle(farCoord[0], farCoord[1], farCoord[2], WHITE);
  } else if (msg.equals("NONE")) {
    display.drawLine(noneCoord[0], noneCoord[1], noneCoord[2], noneCoord[3], WHITE);
    display.drawLine(noneCoord[4], noneCoord[5], noneCoord[6], noneCoord[7], WHITE);
  } else if (msg.equals("READY")) {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(35, 25);
    display.println("READY");
  } else if (msg.equals("SPLASH")) {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30, 10);
    display.println("ROVING");
    display.setCursor(20, 40);
    display.println("SPECTRES");
  } else if (msg.equals("ERROR")) {
    display.setTextSize(4);
    display.setTextColor(WHITE);
    display.setCursor(10, 10);
    display.println("ERROR");
  }
  display.display();
}

void refresh_ready_screen() {
  if (millis() - gestureTriggeredTime > IDLETIME) {
    oled_show("READY");
    isGestureReady = true;
  }
}


