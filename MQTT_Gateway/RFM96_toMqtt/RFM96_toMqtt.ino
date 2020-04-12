// **********************************************************************************
// OwlTronics RFM69 IoT sensor to MQTT (D1) gateway
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
#include <ESP8266WiFi.h>    // included with ESP8266 board def install
#include <ESP8266mDNS.h>    // included with ESP8266 board def install
#include <PubSubClient.h>   // get it here : https://github.com/knolleary/pubsubclient
#include <WiFiManager.h>

// WS2812 LEDs
#include <Adafruit_NeoPixel.h>  // get it here https://github.com/adafruit/Adafruit_NeoPixel

// included here temporarily
const char* ssid = ""; //your WiFi Name
const char* password = "";  //Your Wifi Password

#define SERIAL_BAUD 115200

// MQTT settings
String mqtt_server = "192.168.0.254";
int mqtt_port = 1883;
String mqtt_clientId = "";
String mqtt_topic = "";
String mqtt_base_topic = "sensors/433reciever";
String msg = "";

// add WiFi MAC address to the publish topic
#define ADD_MAC_TO_MQTT_TOPIC

//*********************************************************************************************
// RFM69 stuff
//*********************************************************************************************

#define NODEID      1
#define NETWORKID   100
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY    RF69_433MHZ // other options are RF69_868MHZ and RF69_915MHZ
#define ENCRYPTKEY   "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

//Auto Transmission Control - dials down transmit power to save battery
#define ENABLE_ATC    // comment out this line to disable AUTO TRANSMISSION CONTROL

#ifdef ENABLE_ATC
  RFM69_ATC radio(D0, D8);;
#else
  RFM69 radio(D0, D8);;
#endif

//set to 'true' to sniff all packets on the same network
bool promiscuousMode = false;

//*********************************************************************************************
// WS2812 neopixel stuff
//*********************************************************************************************
#define PIXEL_PIN D3 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 2
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// how often whilst waiting for input to do HB indicator
unsigned int HB = 10000;
int micro_sleep = 50;

// number of chars on output for HB indicator line wrap
int max_width = 40;

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


//*********************************************************************************************
// Global objects
//*********************************************************************************************

// esp wifi object
WiFiClient espClient;
// MQTT client object
PubSubClient client(espClient);

/*
 *  Helper LED blink function
 */
void ledBlink(int pin, int duration_ms) {
  digitalWrite(pin, LOW);
  delay(duration_ms);
  digitalWrite(pin, HIGH);
}

/*
 *  Initialise RFM69 module
 */
void initRadio()
{
  Serial.println("[RFM96] initialising");
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  Serial.println("[RFM96] setting high power (HCW defined)");
  radio.setHighPower();
#endif
  Serial.print("[RFM96] setting enc key: [");
  Serial.print(ENCRYPTKEY);
  Serial.println("]");
  radio.encrypt(ENCRYPTKEY);
  radio.spyMode(promiscuousMode); 

  Serial.print("[RFM96] initialised, listening @");
  char buff[10];
  sprintf(buff, "%d Mhz", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

/*
 *  Initialise the onboard NeoPixels
 */
void init_neopixels(){
    Serial.println("[NEOPX] Setting up NeoPixels");
    
    pixels.begin();
    pixels.clear(); 
    pixels.show();

    Serial.println("[NEOPX] NeoPixel init complete");
}

/*
 *  Initialise WiFi (via WifiManager)
 */
void init_wifi() {
  Serial.println("[WIFI ] Setup begin");

  // start WiFi auto configuration
  WiFiManager wifiManager;
  wifiManager.autoConnect("RFM69-Gw_AutoConfig");

  // dump some info to serial once we're connected
  Serial.print("[WIFI ] Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("[WIFI ] IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("[WIFI] Setup complete");
}

/*
 *  Connect to MQTT server
 */
void init_mqtt() {
  // mDNS to discover MQTT server
  if (!MDNS.begin("ESP")) {
    Serial.println("[mDNS ] Error setting up mDNS");
  } else {
    Serial.println("[mDNS ] Setup - Sending Query");
    int n = MDNS.queryService("mqtt", "tcp");
    if (n == 0) {
      Serial.println("[mDNS ] No service found");
    } else {
      // at least one MQTT service is found
      // ip no and port of the first one is MDNS.IP(0) and MDNS.port(0)
      mqtt_server = MDNS.IP(0).toString();
      mqtt_port = MDNS.port(0);
      Serial.print("[mDNS ] Service discovered: ");
      Serial.print(mqtt_server);
      Serial.print(":");
      Serial.println(mqtt_port);
    }
  }

  // can only setup clientID and topic once WiFi is up
  mqtt_clientId = WiFi.macAddress();
  #ifdef ADD_MAC_TO_MQTT_TOPIC
    mqtt_topic = mqtt_base_topic + "/" + mqtt_clientId + "/";
  #else
    mqtt_topic = mqtt_base_topic + "/";
  #endif
  
  // mqtt setup (setup unqiue client id from mac
  Serial.println("[MQTT ] initiliasing");
  Serial.print("[MQTT ] clientId: ");
  Serial.println(mqtt_clientId);
  Serial.print("[MQTT ] topic: ");
  Serial.println(mqtt_topic);
  client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setCallback(mqtt_callback);
}

/*
 *  Handle data received over MQTT
 */
void mqtt_callback(char* topic, byte * payload, unsigned int length) {
  Serial.print("[MQTT ] Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/*
 *  Reconnect to MQTT server
 *  (will need to rewrite it to be non-blocking)
 */
void mqtt_reconnect() {
  int exit = 1;
  Serial.print("[MQTT ] Not connected! Attempting new connection: ");
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

/*
 *  Handle serial input
 */
void handleSerial() {
  // this will be removed at a later stage
  // as this functionality will be served
  // over a web page
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
    radio.spyMode(promiscuousMode);
    Serial.print("Promiscuous mode ");Serial.println(promiscuousMode ? "on" : "off");
  }
}

/*
 *  Process data received over radio
 */
void handleRadioReceive() {
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
  
  //theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
  // temporary char* to work around strict aliasing
  char *tPtr = (char*)radio.DATA;
  theData = *(Payload*)tPtr;
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

  // send back ack if requested (do this ASAP - before other processing)
  if (radio.ACKRequested())
  {
    // send back ack 
    Serial.print(" > Ack requested, sending: ");
    radio.sendACK();
    Serial.println("sent.");
  }

  Serial.println();
}

/*
 *  Setup function - called once
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("[SETUP] Starting");

  // setup neopixel
  init_neopixels();
  
  // setup WIFI
  init_wifi();

  // mDNS MQTT setup
  init_mqtt();

  // initialise RFM96
  initRadio();

  Serial.println("[SETUP] Complete");
  Serial.println("--------------------------------");
}

/*
 *  Main loop
 */
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
  }
  
  // handle any serial input
  if (Serial.available() > 0) {
    handleSerial();
  }
 
  // handle any incoming radio frames
  if (radio.receiveDone()) {
    handleRadioReceive();
  }
}
