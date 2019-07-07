// **********************************************************************************
// ninja rfm69 iot sensor to mqtt (d1) gateway
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

// RFM libs
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

// ESPwifi / mDNS / MQTT libs
#include <ESP8266WiFi.h>    / included with ESP8266 board def install
#include <ESP8266mDNS.h>    // included with ESP8266 board def install
#include <PubSubClient.h>   // get it here : https://github.com/knolleary/pubsubclient
#include "RFM96_toMqtt.h"   // local settings : please create to store wifi credentials

// RFM69 pauliexpress WS2812 LEDs
#include <Adafruit_NeoPixel.h>  // get it here https://github.com/adafruit/Adafruit_NeoPixel

//const char* ssid = ""; //your WiFi Name
//const char* password = "";  //Your Wifi Password

//*********************************************************************************************
//*********** IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************

#define SERIAL_BAUD 115200

//*********************************************************************************************
// wifi settings - in RFM96_toMqtt.h
//*********************************************************************************************
// MQTT settings
String mqtt_server = "";
int mqtt_port = 1883;
String mqtt_clientId = "";
String mqtt_topic = "";
String mqtt_base_topic = "sensors/433reciever";
String msg = "";

//*********************************************************************************************
// RFM69 stuff
//*********************************************************************************************

#define NODEID      1
#define NETWORKID   100
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ESP8266

//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#if defined (MOTEINO_M0) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL // Required for Serial on Zero based boards
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network


//*********************************************************************************************
// WS2812 neopixel stuff
//*********************************************************************************************

// Which pin on the Arduino is connected to the NeoPixels?
#define PIXEL_PIN D3 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// led pin
const byte led_pin = 2; // internal LED pin on esp12g

// how often whilst waiting for input to do HB indicator
int HB = 10000;
int micro_sleep = 50;

// number of chars on output for HB indicator line wrap
int max_width = 40;

// count acknoledgements
byte ackCount=0;

// polling loop ms register
long unsigned int last_check_millis = 0;

// pretty serial output
int cur_width = 0;

// rfm96 receiver struct
typedef struct __attribute__((__packed__))  { // turn off padding to allow struct casting on incoming data
  short int      nodeId; //store this nodeId needs to be short int to support atmega328p
  unsigned long uptime; //uptime in ms
  float         temp;   //temperature maybe?
} Payload;
Payload theData;

// complex type globals
// esp wifi object
WiFiClient espClient;
// MQTT client object
PubSubClient client(espClient);

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void initRadio()
{
  // hard reset radio board
 /* pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  delay(100);
  digitalWrite(5, LOW);
  delay(100); */

  Serial.println("[RFM96] initialising");
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  Serial.println("[RFM96] setting how power (HCW defined)");
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  Serial.print("[RFM96] setting enc key: [");
  Serial.print(ENCRYPTKEY);
  Serial.println("]");
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode); 

  Serial.print("[RFM96] initialised, listening @");
  char buff[50];
  sprintf(buff, "%d Mhz", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

}

//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
void init_pins(){
    Serial.print("[PINS] setting up internal pins:  ");
    
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);

    Serial.println("done");
}

void init_neopixels(){
    Serial.print("[NEO] setting up NEO PIXELS: ");
    
    pixels.begin();
    pixels.clear(); 

    Serial.println("done");
}


//----------------------------------------------------------------------------------
void init_wifi() {
  //##################################
  // connect to wifi
  Serial.println("[WIFI] initialising");
  Serial.print("[WIFI] MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("[WIFI] SSID:");
  Serial.println(ssid);

  Serial.print("[WIFI] Attaching: ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("[WIFI] connected IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("[WIFI] Setup complete");
}

//----------------------------------------------------------------------------------
void init_mqtt() {
  // mDNS to discover MQTT server
  if (!MDNS.begin("ESP")) {
    Serial.println("[mDNS] Error setting up mDNS");
  } else {
    Serial.println("[mDNS] Setup - Sending Query");
    int n = MDNS.queryService("mqtt", "tcp");
    if (n == 0) {
      Serial.println("[mDNS] No service found");
    } else {
      // at least one MQTT service is found
      // ip no and port of the first one is MDNS.IP(0) and MDNS.port(0)
      mqtt_server = MDNS.IP(0).toString();
      mqtt_port = MDNS.port(0);
      Serial.print("[mDNS] Service discovered: ");
      Serial.print(mqtt_server);
      Serial.print(":");
      Serial.println(mqtt_port);
      
    }
  }

  // can only setup clientID and topic once WiFi is up
  mqtt_clientId = WiFi.macAddress();
  mqtt_topic = mqtt_base_topic + "/" + mqtt_clientId + "/";
  
  // mqtt setup (setup unqiue client id from mac
  Serial.println("[MQTT] initiliasing");
  Serial.print("(MQTT] clientId: ");
  Serial.println(mqtt_clientId);
  Serial.print("[MQTT] topic: ");
  Serial.println(mqtt_topic);
    client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setCallback(mqtt_callback);
  //nb mqtt connection is handled later..
}

//----------------------------------------------------------------------------------
void mqtt_callback(char* topic, byte * payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// mqtt stuff
//----------------------------------------------------------------------------------
void mqtt_reconnect() {
  int exit = 1;
  Serial.println();
  Serial.print("[MQTT] Not connected! Attempting new connection: ");
  while (!client.connected() && exit == 1 ) {
    Serial.print("#");
    if (client.connect(mqtt_clientId.c_str())) {
      Serial.println("> OK");
      // Once connected, publish an announcement...
      String connected_msg = mqtt_clientId + " joined";
      client.publish("clientJoin", connected_msg.c_str());
      client.loop();
    } else {
      Serial.print("> FAIL, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// led blink function
//----------------------------------------------------------------------------------
void ledBlink(int pin, int duration_ms) {
  digitalWrite(pin, LOW);
  delay(duration_ms);
  digitalWrite(pin, HIGH);
} // endfunc

// small blinky function
//----------------------------------------------------------------------------------
void doBlinky(int pin, int duration_ms, int blinks) {
  for ( int i = 0; i < blinks; i++ ) {
    ledBlink(pin,duration_ms);
    delay(duration_ms);
  } // end for
} // endfunc



void handleSerial(){
    char input = Serial.read();

    Serial.println();
    Serial.print("# Serial input:[");
    Serial.print(input);
    Serial.println("]");

    // handle input command
    if (input == 'r') //d=dump all register values
      radio.readAllRegs();
    if (input == 'E') //E=enable encryption
      radio.encrypt(ENCRYPTKEY);
    if (input == 'e') //e=disable encryption
      radio.encrypt(null);
    if (input == 'p')
    {
      promiscuousMode = !promiscuousMode;
      radio.promiscuous(promiscuousMode);
      Serial.print("Promiscuous mode ");Serial.println(promiscuousMode ? "on" : "off");
    }
    
//    if (input == 'd') //d=dump flash area
//    {
//      Serial.println("Flash content:");
//      int counter = 0;
//
//      while(counter<=256){
//        Serial.print(flash.readByte(counter++), HEX);
//        Serial.print('.');
//      }
//      while(flash.busy());
//      Serial.println();
//    }
//    if (input == 'D')
//    {
//      Serial.print("Deleting Flash chip content... ");
//      flash.chipErase();
//      while(flash.busy());
//      Serial.println("DONE");
//    }
//    if (input == 'i')
//    {
//      Serial.print("DeviceID: ");
//      word jedecid = flash.readDeviceId();
//      Serial.println(jedecid, HEX);
//    }
}

void handleRadioReceive(){

    // output info on received radio data
    Serial.println();
    Serial.print("[RFM96] RCVD [Node:");Serial.print(radio.SENDERID, DEC);
    Serial.print("  RSSI:");Serial.print(radio.readRSSI());Serial.print("] ");    
    if (promiscuousMode) {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }
    Serial.println();


    // output raw data payload
    Serial.print(" > Data received [");
    Serial.print(radio.DATALEN);
    Serial.print("b]: [");
    for (byte i = 0; i < radio.DATALEN; i++){
      Serial.print((char)radio.DATA[i],HEX);
      Serial.print(" ");
    }
    Serial.println("]");

    // check node id (understand expected incoming data structure)
    
    //if (radio.DATALEN != sizeof(Payload))
    if (1 == 0 )
      Serial.print("Err: Invalid payload received, not matching Payload struct!");
    else
    {
      theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      Serial.print(" > vars: nodeId=");
      Serial.print(theData.nodeId);
      Serial.print(" uptime=");
      Serial.print(theData.uptime);
      Serial.print(" temp=");
      Serial.print(theData.temp);
      Serial.println();

      // push data to mqtt
      Serial.print(" > Pushing data to MQTT: ");
      client.publish(String(mqtt_topic + theData.nodeId + "/trigger").c_str(), "");
      client.publish(String(mqtt_topic + theData.nodeId + "/uptime" ).c_str(), String(theData.uptime).c_str() );
      client.publish(String(mqtt_topic + theData.nodeId + "/temp" ).c_str(), String(theData.temp).c_str()  );
      Serial.println("done");
    }


    // send back ack if requested (do this ASAP - before other processing)
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;

      // send back ack 
      Serial.print(" > Ack requested, sending: ");
      radio.sendACK();
      Serial.println("sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" > Periodic ping node check to [");
        Serial.print(theNodeID);
        Serial.print("] sending:");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.println(" OK");
        else 
          Serial.println("NO RESPONSE");
      }
    }

    Serial.println();
    Blink(LED_BUILTIN,3);
}

//*********************************************************************************************
//*********** SETUP ***************************************************************************
//*********************************************************************************************
void setup() {
  Serial.begin(SERIAL_BAUD);

  //##################################
  Serial.println();
  Serial.println("================================");
  Serial.println("[SETUP] Starting");
  Serial.println("--------------------------------");

  // setup GPIO/LEDS etc
  init_pins();
  
  // setup neopixel
  init_neopixels();
  
  // setup WIFI
  init_wifi();

  // mDNS MQTT setup
  init_mqtt();



  // initialise RFM96
  initRadio();

}


//*********************************************************************************************
//*********** LOOP ***************************************************************************
//*********************************************************************************************

void loop() {

  // call MQTT loop to handle active connection
  if (!client.connected()) {
    mqtt_reconnect();
  }
  client.loop();

  // gateway "HeartBeat" blink LED/debug output regularly to show we're still ticking
  if ( millis() - last_check_millis > HB ) {
    Serial.print (".");
    ledBlink(LED_BUILTIN,50);
    last_check_millis = millis();

    // line wrap handler for debug output
    cur_width++;
    if (cur_width > max_width) {
      Serial.println();
      cur_width = 0;
    }
  } // endif
  
  
  // handle any serial input
  if (Serial.available() > 0)
      handleSerial();
 
  // handle any incoming radio frames
  if (radio.receiveDone())
      handleRadioReceive();
      
}
 
