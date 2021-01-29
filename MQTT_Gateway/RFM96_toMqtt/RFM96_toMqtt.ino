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

// work around SPIFFS deprecation
// redefine it to be LitteFS
#include <FS.h>
#define SPIFFS LittleFS
#include <LittleFS.h> 

// ESPwifi / mDNS / MQTT libs
#include <ESP8266WiFi.h>    // included with ESP8266 board def install
#include <ESP8266mDNS.h>    // included with ESP8266 board def install
#include <PubSubClient.h>   // get it here : https://github.com/knolleary/pubsubclient
//#include <WiFiManager.h>
// web server
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>

// OTA update
#include <ArduinoOTA.h>
String hostName = "rfm69gw";

// WS2812 LEDs
#include <Adafruit_NeoPixel.h>  // get it here https://github.com/adafruit/Adafruit_NeoPixel

// Uptime library
#include <uptime.h>
#include <uptime_formatter.h>

#define SERIAL_BAUD 115200

// MQTT settings
String mqtt_server = "192.168.0.254";
int mqtt_port = 1883;
String mqtt_clientId = "";
String mqtt_topic = "";
String mqtt_base_topic = "RFM69Gw";
String msg = "";

// store the last 5 radio packets
typedef struct {
  bool valid;
  uint16_t senderId;
  uint8_t dataLen;
  uint8_t data[RF69_MAX_DATA_LEN+1];
  uint8_t ackReq;
  int16_t rssi;
} __attribute__((packed)) RadioPacket;

// use a ring-buffer for radio packet storage
RadioPacket recvPackets[5];
uint8_t lastPacket = -1;

// add WiFi MAC address to the publish topic
#define ADD_MAC_TO_MQTT_TOPIC
// push RSSI to MQTT (will move to configurable option later)
#define PUSH_RSSI_TO_MQTT

//*********************************************************************************************
// RFM69 stuff
//*********************************************************************************************

#define NODEID      1
#define NETWORKID   89
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

// breathing light config
// which LED we're using for the breathing light
#define STATUS_NEOPX_POSITION     0

// breathing light state machine
#define STATUS_NEOPX_OFF          0
#define STATUS_NEOPX_BRIGHTENING  1
#define STATUS_NEOPX_DIMMING      2

// timestamp of the last state machine change and initial state
unsigned long lastStatusChange = 0;
byte breathingLedStatus = STATUS_NEOPX_OFF;

// which LED we're using for radio status
#define RADIO_STATUS_NEOPX_POSITION 1
#define RADIO_STATUS_NEOPX_ONTIME   50

#define STATUS_RADIO_NEOPX_OFF  0
#define STATUS_RADIO_NEOPX_ON   1

unsigned long radioStatusOnTime = 0;
byte radioLedStatus = STATUS_RADIO_NEOPX_OFF;

// how often whilst waiting for input to do HB indicator
unsigned int HB = 10000;

// number of chars on output for HB indicator line wrap
int max_width = 40;

// polling loop ms register
long unsigned int last_check_millis = 0;

// pretty serial output
int cur_width = 0;

// RFM96 receiver struct
// this does not contain the actual payload; only the node ID
typedef struct {
  uint16_t      nodeId;
} __attribute__((__packed__)) Payload;
Payload theData;

// esp wifi object
WiFiClient espClient;
// MQTT client object
PubSubClient client(espClient);
// HTTP server
//ESP8266WebServer httpServer(80);

AsyncWebServer server(80);
DNSServer dns;


/*
 *  Helper LED blink function
 */
void ledBlink(int pin, int duration_ms) {
  digitalWrite(pin, LOW);
  delay(duration_ms);
  digitalWrite(pin, HIGH);
}


/*
 *  Helper to zero pad numbers
 */
String padDigits(int digits) {
  if(digits < 10) {
    return "0" + String(digits);
  }
  return String(digits);
}


/*
 *  Helper function for creating hex strings
 */
char hexDigit(byte v)
{
  v &= 0x0F; // just the lower 4 bits

  return v < 10 ? '0' + v : 'A' + (v - 10);
}

void init_OTA() {
  Serial.println("[OTA  ] Setting up OTA");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "program";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    SPIFFS.end();
    Serial.println("[OTA  ] Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA  ] Update finished");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char buf[32];
    sprintf(buf, "[OTA  ] Progress: %u%%\r", (progress / (total / 100)));
    Serial.println(buf);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    char buf[16];
    sprintf(buf, "[OTA  ] Error[%u]: ", error);
    Serial.println(buf);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  // start OTA receiver with mDNS
  ArduinoOTA.setHostname(hostName.c_str());
  ArduinoOTA.begin(true);

  Serial.println("[OTA  ] OTA setup complete");
}

/*
 *  Initialise RFM69 module
 */
void init_radio() {
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

  Serial.print("[RFM96] initialised, listening @ ");
  char buff[10];
  sprintf(buff, "%d Mhz", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}


/*
 *  Initialise the onboard NeoPixels
 */
void init_neopixels() {
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
  AsyncWiFiManager wifiManager(&server, &dns);;
  wifiManager.autoConnect("RFM69-Gw_AutoConfig");

  // dump some info to serial once we're connected
  Serial.print("[WIFI ] Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("[WIFI ] IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("[WIFI ] Setup complete");
}


/*
 *  Connect to MQTT server
 */
void init_mqtt() {
/*  // mDNS to discover MQTT server
  if (!MDNS.begin("RFM69Gw")) {
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
*/
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
  if (input == 'r') {
    // dump all registers
    radio.readAllRegs();
  } else if (input == 'E') {
    //E=enable encryption
    radio.encrypt(ENCRYPTKEY);
  } else if (input == 'e') {
    //e=disable encryption
    radio.encrypt(null);
  } else if (input == 'p') {
    promiscuousMode = !promiscuousMode;
    radio.spyMode(promiscuousMode);
    Serial.print("Promiscuous mode ");
    Serial.println(promiscuousMode ? "on" : "off");
  }
}


/*
 *  Process data received over radio
 */
void handleRadioReceive() {
  // light up the radio status LED
  pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(16, 8, 0));
  pixels.show();

  radioStatusOnTime = millis();
  radioLedStatus = STATUS_RADIO_NEOPX_ON;

  // output info on received radio data
  Serial.println();
  Serial.print("[RFM96] RCVD [Node:");Serial.print(radio.SENDERID, DEC);
  Serial.print("  RSSI:");Serial.print(radio.readRSSI());Serial.print("] ");    
  if (promiscuousMode) {
    Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
  }
  Serial.println();

  // max data length is 61, so we allocate 2x61 + 1 for string termination
  char  hexData[123];
  byte  ptr = 0;
  for (byte i = 0; i < radio.DATALEN; i++) {
    hexData[ptr++] = hexDigit(radio.DATA[i] >> 4);
    hexData[ptr++] = hexDigit(radio.DATA[i]);
  }
  hexData[ptr] = '\0';
  String hexPayload = String(hexData);

  // output raw data payload
  Serial.print("[RFM96]  > Data received [");
  Serial.print(radio.DATALEN);
  Serial.print("b]: [");

  for(byte i = 0; i < hexPayload.length(); i += 2) {
    Serial.print(hexPayload.substring(i, i + 2));
    Serial.print(" ");
  }
  Serial.println("]");

  // temporary char* to work around strict aliasing
  char *tPtr = (char*)radio.DATA;
  theData = *(Payload*)tPtr;

  // push data to mqtt
  Serial.print(" > Pushing data to MQTT: ");
  client.publish(String(mqtt_topic + theData.nodeId + "/payload" ).c_str(), hexPayload.c_str());
  #ifdef PUSH_RSSI_TO_MQTT
    client.publish(String(mqtt_topic + theData.nodeId + "/rssi" ).c_str(), String(radio.readRSSI()).c_str());
  #endif
  Serial.println("done");

  // save the received radio packet into our buffer
  lastPacket++;
  if (lastPacket > 4) {
    lastPacket = 0;
  }

  recvPackets[lastPacket].valid = true;
  recvPackets[lastPacket].senderId = radio.SENDERID;
  recvPackets[lastPacket].dataLen = radio.DATALEN;
  memcpy(recvPackets[lastPacket].data, radio.DATA, sizeof radio.DATA);
  recvPackets[lastPacket].rssi = radio.RSSI;
  recvPackets[lastPacket].ackReq = radio.ACK_REQUESTED;

  // send back ack if requested (do this ASAP - before other processing)
  if (radio.ACKRequested()) {
    // send back ack 
    Serial.print(" > Ack requested, sending: ");
    radio.sendACK();
    Serial.println("sent.");
  }
}


/*
 *  Register on mDNS
 */
void init_mDNS() {
  // mDNS already set up by ArduinoOTA
  // all we need to do is advertise the HTTP server
  MDNS.addService("http", "tcp", 80);
}


/*
 *  Web page processor
 */
String processor(const String& var) {
  if(var == "HOSTNAME") {
    return hostName + ".local";
  }
  else if (var == "UPTIME") {
    return uptime_formatter::getUptime();
  }
  else if (var == "RECVPACKETS") {
    String s = "";
    // max data length is 61, so we allocate 2x61 + 1 for string termination
    char  hexData[123];

    int8_t c = lastPacket;
    for(uint8_t i = 0; i < 5; i++) {
      if (recvPackets[c].valid) {
        s += "<tr>";

        s += "<td>";
        s += recvPackets[c].senderId;
        s += "</td>";

        s += "<td>";
        s += recvPackets[c].dataLen;
        s += "</td>";

        s += "<td onmouseover=\"tts(";
        s += i;
        s += ")\" onmouseout=\"tth(";
        s += i;
        s += ")\" class=\"cwc\" id=\"rc";
        s += i;
        s += "\">";
        uint8_t  ptr = 0;
        for (uint8_t i = 0; i < recvPackets[c].dataLen; i++) {
          hexData[ptr++] = hexDigit(recvPackets[c].data[i] >> 4);
          hexData[ptr++] = hexDigit(recvPackets[c].data[i]);
        }
        hexData[ptr] = '\0';
        s += String(hexData);
        s += "<span class=\"cc\" id=\"rtt";
        s += i;
        s += "\">x</span>";
        s += "</td>";

        s += "<td>";
        s += recvPackets[c].ackReq ? "yes" : "no";
        s += "</td>";

        s += "<td>";
        s += recvPackets[c].rssi;
        s += "</td>";

        s += "</tr>";
      }

      // move on to the previous packet
      c--;
      if (c < 0) {
        c = 4;
      }
    }

    return s;
  }

  // catch all
  // this normally means a '%%' token
  return "%";
}


/*
 *  Register on mDNS
 */
void init_webServer() {
  Serial.println("[HTTP ] Setting up web server");

  // route for root page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // route for the stylesheet
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/style.css", String(), false, processor);
  });

  // Start HTTP server
  server.begin();

  Serial.println("[HTTP ] Web server started");
}


/*
 *  Setup function - called once
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("[SETUP] Starting");

  // init radio packet buffer
  for(uint8_t i = 0; i < 5; i++) {
    recvPackets[i].valid = false;
  }

  // open the filesystem
  SPIFFS.begin();

  // setup neopixel
  init_neopixels();
  
  // setup WIFI
  init_wifi();

  // setup OTA
  init_OTA();

  // MQTT setup
  init_mqtt();

  // initialise RFM96
  init_radio();

  // register on mDNS
  init_mDNS();

  // start the webserver
  init_webServer();

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

  // handle OTA stuff
  // this updates mDNS as well
  ArduinoOTA.handle();

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

  // breathing light
  // work around the 50 day rollover
  if (millis() < lastStatusChange) {
    lastStatusChange = 0;
  }
  switch (breathingLedStatus) {
    case STATUS_NEOPX_OFF:
      // led is off for 5 seconds
      if (millis() - lastStatusChange > 5000) {
        // move to the next status
        breathingLedStatus = STATUS_NEOPX_BRIGHTENING;
        lastStatusChange = millis();
      }
      break;
    case STATUS_NEOPX_BRIGHTENING:
      // we go from 0 to 16 brightness over 1 second
      if (millis() - lastStatusChange > 1000) {
        // move to next status
        breathingLedStatus = STATUS_NEOPX_DIMMING;
        lastStatusChange = millis();
      } else {
        // see if we need to update the colour
        uint32_t c = pixels.getPixelColor(STATUS_NEOPX_POSITION);
        byte green = (byte)(c >> 8);
        byte newGreen = (millis() - lastStatusChange) / 62;
        
        if (green != newGreen) {
          pixels.setPixelColor(STATUS_NEOPX_POSITION, pixels.Color(0, newGreen, 0));
          pixels.show();
        }
      }
      break;
    case STATUS_NEOPX_DIMMING:
      // we go from 16 to 0 brightness over 1 second
      if (millis() - lastStatusChange > 1000) {
        // move to next status
        breathingLedStatus = STATUS_NEOPX_OFF;
        lastStatusChange = millis();

        // just for good measure
        pixels.setPixelColor(STATUS_NEOPX_POSITION, 0);
        pixels.show();
      } else {
        // see if we need to update the colour
        uint32_t c = pixels.getPixelColor(STATUS_NEOPX_POSITION);
        byte green = (byte)(c >> 8);
        byte newGreen = 16 - ((millis() - lastStatusChange) / 62);

        if (green != newGreen) {
          pixels.setPixelColor(STATUS_NEOPX_POSITION, pixels.Color(0, newGreen, 0));
          pixels.show();
        }
      }
      break;
  }

  // radio status light
  // work around the 50 day rollover
  if (millis() < radioStatusOnTime) {
    radioStatusOnTime = 0;
  }
  if (radioLedStatus == STATUS_RADIO_NEOPX_ON) {
    if (millis() - radioStatusOnTime > RADIO_STATUS_NEOPX_ONTIME) {
      // the status LED is only on for 50msec
      radioLedStatus = STATUS_RADIO_NEOPX_OFF;
      pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(0, 0, 0));
      pixels.show();
    }
  }
}
