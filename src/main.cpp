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

// Uncomment this if you're building the gateway for dev purposes
// this changes the hostname, MQTT base topic and network id
// #define DEV_BUILD

// Uncomment this if you want to log loop times
// this is only required for troubleshooting
// #define LOG_LOOP_TIMES

#include <Arduino.h>

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
#ifndef DEV_BUILD
    String hostName = "rfm69gw";
#else
    String hostName = "rfm69gw-dev";
#endif

// WS2812 LEDs
#include <Adafruit_NeoPixel.h>  // get it here https://github.com/adafruit/Adafruit_NeoPixel

// Uptime library
#include <uptime.h>
#include <uptime_formatter.h>

#define SERIAL_BAUD 115200

// MQTT settings
#ifndef DEV_BUILD
    String mqtt_server = "192.168.0.201";
#else
    String mqtt_server = "192.168.0.200";
#endif
int mqtt_port = 1883;
String mqtt_clientId = "";
String mqtt_topic = "";
#ifndef DEV_BUILD
    String mqtt_base_topic = "RFM69Gw";
#else
    String mqtt_base_topic = "RFM69Gw-dev";
#endif
String msg = "";
unsigned long mqttMessagesIn = 0;
unsigned long mqttMessagesOut = 0;
unsigned long mqttReconnects = 0;
unsigned long mqttLastReconnectAttempt = 0;
bool mqttLoggedBackoffEvent = false;

#define MQTT_BACKOFF_TIMER  5000

#define NUM_PACKETS_TO_STORE    10

// store the last NUM_PACKETS_TO_STORE radio packets
typedef struct {
    bool valid;
    unsigned long ts;
    uint16_t senderId;
    uint8_t dataLen;
    uint8_t data[RF69_MAX_DATA_LEN+1];
    uint8_t ackReq;
    int16_t rssi;
} __attribute__((packed)) RadioPacket;

// use a ring-buffer for radio packet storage
RadioPacket recvPackets[NUM_PACKETS_TO_STORE];
uint8_t lastPacket = -1;

// add WiFi MAC address to the publish topic
#define ADD_MAC_TO_MQTT_TOPIC
// push RSSI to MQTT (will move to configurable option later)
// #define PUSH_RSSI_TO_MQTT

//*********************************************************************************************
// RFM69 stuff
//*********************************************************************************************

#define NODEID      1
#ifndef DEV_BUILD
    #define NETWORKID   89
#else
    #define NETWORKID   90
#endif

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY    RF69_433MHZ // other options are RF69_868MHZ and RF69_915MHZ
#define ENCRYPTKEY   "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

//Auto Transmission Control - dials down transmit power to save battery
#define ENABLE_ATC    // comment out this line to disable AUTO TRANSMISSION CONTROL

#ifdef ENABLE_ATC
    RFM69_ATC radio(D0, D8);
#else
    RFM69 radio(D0, D8);
#endif

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

// colour of the radio activity LED
uint32_t radioActivityLEDColour = pixels.Color(16, 0, 8);

// colour definitions for the breathing light
// these are simply bit-shift counters to achieve the correct colour ;)
#define NEOPIXEL_COLOUR_RED     16
#define NEOPIXEL_COLOUR_GREEN   8
#define NEOPIXEL_COLOUR_BLUE    0

// colour of the breathing light LED
#define STATUS_NEOPX_COLOUR NEOPIXEL_COLOUR_RED

// how often whilst waiting for input to do HB indicator
#define SYSTEM_HEARTBEAT_INTERVAL   10000

// polling loop ms register
unsigned long last_check_millis = 0;

#ifdef LOG_LOOP_TIMES
    unsigned long loopTimeStart = 0;
    unsigned long loopTime = 0;
    unsigned long loopTimeAverage = 0;
    unsigned long loopTimeMax = 0;
    unsigned long loopTimeMin = 0;
    unsigned long loopCounter = 0;
    bool loopStatReset = false;
#endif

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
AsyncWebServer server(80);
DNSServer dns;

// buffer to dump radio data into
// max data length is 61, so we allocate 2x61 + 1 for string termination
char  hexData[123];

// store the free heap after startup, so we can see a percentage utilisation
uint32_t    startupFreeHeap;

/*
 *  Helper to zero pad numbers
 */
String padDigits(int digits) {
    if(digits < 10)
        return "0" + String(digits);

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


/*
 *  Initialise OTA
 */
void init_OTA() {
    Serial.println("[OTA  ] Setting up OTA");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "program";
        else // U_FS
            type = "filesystem";

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        SPIFFS.end();
        Serial.println("[OTA  ] Start updating " + type);

        // update status LEDs to show OTA progress
        pixels.setPixelColor(STATUS_NEOPX_POSITION, pixels.Color(0, 16, 0));
        pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(0, 0, 0));
        pixels.show();
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA  ] Update finished");

        // update complete; turn off LEDs
        pixels.setPixelColor(STATUS_NEOPX_POSITION, pixels.Color(0, 0, 0));
        pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(0, 0, 0));
        pixels.show();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        char buf[32];
        sprintf(buf, "[OTA  ] Progress: %u%%\r", (progress / (total / 100)));
        Serial.println(buf);

        // blink alternatively between red and blue every 5%
        if (((uint8_t)(progress / (total /100))) % 10 < 5) {
            pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(16, 0, 0));
            breathingLedStatus = 0;
        } else {
            pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(0, 0, 16));
            breathingLedStatus = 1;
        }
        
        // if the status has changed, then update the LED
        if (radioLedStatus != breathingLedStatus) {
            pixels.show();
            radioLedStatus = breathingLedStatus;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        char buf[16];
        sprintf(buf, "[OTA  ] Error[%u]: ", error);
        Serial.println(buf);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
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
    radio.spyMode(false);

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
 *  Handle data received over MQTT
 */
void mqtt_callback(char* topic, byte * payload, unsigned int length) {
    Serial.print("[MQTT ] Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    mqttMessagesIn++;
    for (unsigned int i = 0; i < length; i++)
        Serial.print((char)payload[i]);

    Serial.println();
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
    mqtt_clientId = hostName + "-" + WiFi.macAddress() + "-" + String(random(0xffff), HEX);
#ifdef ADD_MAC_TO_MQTT_TOPIC
    mqtt_topic = mqtt_base_topic + "/" + mqtt_clientId + "/";
#else
    mqtt_topic = mqtt_base_topic + "/";
#endif
  
    // mqtt setup (setup unqiue client id from mac)
    Serial.println("[MQTT ] initiliasing");
    Serial.print("[MQTT ] clientId: ");
    Serial.println(mqtt_clientId);
    Serial.print("[MQTT ] topic: ");
    Serial.println(mqtt_topic);
    client.setServer(mqtt_server.c_str(), mqtt_port);
    client.setCallback(mqtt_callback);
}


/*
 *  Reconnect to MQTT server
 */
void mqtt_reconnect() {
    // obey the MQTT reconnect timer
    if (millis() - MQTT_BACKOFF_TIMER > mqttLastReconnectAttempt || millis() < mqttLastReconnectAttempt) {
        // regenerate client id
        mqtt_clientId = hostName + "-" + WiFi.macAddress() + "-" + String(random(0xffff), HEX);

        Serial.println("[MQTT ] Not connected! Attempting new connection");

        // increase the reconnect counter
        mqttReconnects++;

        if (client.connect(mqtt_clientId.c_str())) {
            client.loop();
            Serial.println("[MQTT ] Reconnected successfully");
        } else
            Serial.println("[MQTT ] Reconnect failed - will try again in the next loop");
        
        // save the last reconnect attempt
        mqttLastReconnectAttempt = millis();
        mqttLoggedBackoffEvent = false;
    } else
        if (!mqttLoggedBackoffEvent) {
            Serial.println("[MQTT ] Not connected! Backoff timer not expired yet, so delaying reconnect.");
            mqttLoggedBackoffEvent = true;
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
    if (input == 'r')
        // dump all registers
        radio.readAllRegs();
    else if (input == 'E')
        //E=enable encryption
        radio.encrypt(ENCRYPTKEY);
    else if (input == 'e')
        //e=disable encryption
        radio.encrypt(null);
}


/*
 *  Process data received over radio
 */
void handleRadioReceive() {
    // TODO: first save the packet, then ACK it ASAP, then do all the logging!
    // save the received radio packet into our buffer
    lastPacket++;
    if (lastPacket > NUM_PACKETS_TO_STORE - 1)
        lastPacket = 0;

    recvPackets[lastPacket].valid = true;
    recvPackets[lastPacket].ts = millis();
    recvPackets[lastPacket].senderId = radio.SENDERID;
    recvPackets[lastPacket].dataLen = radio.DATALEN;
    memcpy(recvPackets[lastPacket].data, radio.DATA, sizeof radio.DATA);
    recvPackets[lastPacket].rssi = radio.RSSI;
    recvPackets[lastPacket].ackReq = radio.ACK_REQUESTED;

    // send back ack if requested (do this ASAP - before other processing)
    if (radio.ACKRequested()) {
        // send back ack 
        Serial.print("[RFM69] Ack requested, sending: ");
        radio.sendACK();
        Serial.println("sent");
    }

    // light up the radio status LED
    pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, radioActivityLEDColour);
    pixels.show();

    radioStatusOnTime = millis();
    radioLedStatus = STATUS_RADIO_NEOPX_ON;

    // output info on received radio data
    Serial.print("[RFM96] RCVD [Node:");
    Serial.print(recvPackets[lastPacket].senderId, DEC);
    Serial.print("  RSSI:");
    Serial.print(recvPackets[lastPacket].rssi);
    Serial.println("] ");

    byte  ptr = 0;
    for (byte i = 0; i < recvPackets[lastPacket].dataLen; i++) {
        hexData[ptr++] = hexDigit(recvPackets[lastPacket].data[i] >> 4);
        hexData[ptr++] = hexDigit(recvPackets[lastPacket].data[i]);
    }
    hexData[ptr] = '\0';
    String hexPayload = String(hexData);

    // output raw data payload
    Serial.print("[RFM96] Data received [");
    Serial.print(recvPackets[lastPacket].dataLen);
    Serial.print("b]: [");

    for(byte i = 0; i < hexPayload.length(); i += 2) {
        Serial.print(hexPayload.substring(i, i + 2));
        if (i < hexPayload.length() - 2)
            Serial.print(" ");
    }
    Serial.println("]");

    // temporary char* to work around strict aliasing
    char *tPtr = (char*)recvPackets[lastPacket].data;
    theData = *(Payload*)tPtr;

    // push data to mqtt
    Serial.print("[MQTT ] Pushing data to MQTT: ");
    if (client.connected()) {
        client.publish(String(mqtt_topic + theData.nodeId + "/payload" ).c_str(), hexPayload.c_str());
        client.loop();
        mqttMessagesOut++;
    }
#ifdef PUSH_RSSI_TO_MQTT
    if (client.connected()) {
        client.publish(String(mqtt_topic + theData.nodeId + "/rssi" ).c_str(), String(radio.readRSSI()).c_str());
        client.loop();
        mqttMessagesOut++;
    }
#endif
    if (client.connected())
        Serial.println("done");
    else
        Serial.println("failed; not connected!");
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
    if (var == "HOSTNAME")
        return hostName + ".local";
    else if (var == "D1STATS") {
        String s = "";
        s += "<li>Uptime: " + uptime_formatter::getUptime() + "</li>";
        s += "<li>Free memory: ";
        s += ESP.getFreeHeap();
        s += " bytes (";
        s += ESP.getFreeHeap() / (double)startupFreeHeap * 100;
        s += "&#37; of startup)</li>";
        s += "<li>MAC address: " + WiFi.macAddress() + "</li>";

        return s;
    } else if (var == "RFM69STATS") {
        String s = "";
        s += "<li>Frequency: ";
        s += radio.getFrequency() / 1000000;
        s += "MHz</li>";
        s += "<li>Network ID: ";
        s += NETWORKID;
        s += "</li>";
        s += "<li>Node ID: ";
        s += NODEID;
        s += "</li>";
        s += "<li>Power level: ";
        s += -18 + radio.getPowerLevel();
        s += "dBm</li>";
        s += "<li>Temperature: ";
        s += radio.readTemperature();
        s += "C</li>";

        return s;
    } else if (var == "MQTTSTATS") {
        String s = "";
        s += "<li>Connected: ";
        s += client.connected() ? "yes" : "no";
        s += "</li>";
        s += "<li>Server: ";
        s += mqtt_server;
        s += "</li>";
        s += "<li>Inbound messages: ";
        s += mqttMessagesIn;
        s += "</li>";
        s += "<li>Outbound messages: ";
        s += mqttMessagesOut;
        s += "</li>";
        s += "<li>Reconnects: ";
        s += mqttReconnects;
        s += "</li>";
#ifdef PUSH_RSSI_TO_MQTT
        s += "<li>Send RSSI to MQTT: yes</li>";
#else
        s += "<li>Send RSSI to MQTT: no</li>";
#endif

        return s;
    } else if (var == "NUMSTOREDPACKETS") {
        return String(NUM_PACKETS_TO_STORE);
    } else if (var == "RECVPACKETS") {
        String s = "";
        // max data length is 61, so we allocate 2x61 + 1 for string termination
        char  hexData[123];

        // get current timestamp
        unsigned long curTime = millis();

        int8_t c = lastPacket;
        for (uint8_t i = 0; i < NUM_PACKETS_TO_STORE; i++) {
            if (recvPackets[c].valid) {
                s += "<tr>";

                s += "<td>";
                s += (((float)curTime - recvPackets[c].ts) / 1000);
                s += " s</td>";

                s += "<td>";
                // check if we can add an icon to the sensor
                if (recvPackets[c].dataLen > 3) {
                    uint8_t sensorType = recvPackets[c].data[2];

                    if (sensorType == 1 || sensorType == 2 || sensorType == 3)
                        s += "<img src=\"power.png\">";
                    else if (sensorType == 4)
                        s += "<img src=\"temperature.png\">";
                    else if (sensorType == 5)
                        s += "<img src=\"pressure.png\">";
                    else if (sensorType == 6)
                        s += "<img src=\"trigger.png\">";
                    else
                        s += "<img src=\"question.png\">";
                }
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
                uint8_t ptr = 0;
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
            if (c < 0)
                c = NUM_PACKETS_TO_STORE - 1;
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
    // no need for the 'processor' as we have no tokens to replace
    // (plus it interferes with the % signs!)
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", String(), false);
    });

    // favicon entries
    server.on("/favicon-16x16.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/favicon-16x16.png", String(), false);
    });

    server.on("/favicon-32x32.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/favicon-32x32.png", String(), false);
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/favicon.ico", String(), false);
    });

    // sensor icons
    server.on("/power.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/power.png", String(), false);
    });

    server.on("/temperature.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/temperature.png", String(), false);
    });

    server.on("/pressure.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/pressure.png", String(), false);
    });

    server.on("/trigger.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/trigger.png", String(), false);
    });

    server.on("/question.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/question.png", String(), false);
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
    for(uint8_t i = 0; i < 5; i++)
        recvPackets[i].valid = false;

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

    // save free heap
    startupFreeHeap = ESP.getFreeHeap();
}


/*
 *  Main loop
 */
void loop() {
#ifdef LOG_LOOP_TIMES
    loopTimeStart = millis();
#endif
    // call MQTT loop to handle active connection
    if (!client.connected())
        mqtt_reconnect();
    else
        client.loop();

    // handle OTA stuff
    // this updates mDNS as well
    ArduinoOTA.handle();

    // gateway "HeartBeat" blink LED/debug output regularly to show we're still ticking
    if ( millis() - last_check_millis > SYSTEM_HEARTBEAT_INTERVAL ) {
        Serial.println("[SYS  ] Heartbeat");
        last_check_millis = millis();
#ifdef LOG_LOOP_TIMES
        Serial.print("[SYS  ] Avg loop time: ");
        Serial.print(loopTimeAverage);
        Serial.print("ms, min: ");
        Serial.print(loopTimeMin);
        Serial.print("ms, max: ");
        Serial.print(loopTimeMax);
        Serial.print("ms, loops: ");
        Serial.println(loopCounter);

        loopCounter = 0;
        loopTimeAverage = 0;
        loopTimeMax = 0;
        loopTimeMin = 0;
        loopStatReset = true;
#endif
    }
 
    // handle any serial input
    if (Serial.available() > 0)
        handleSerial();
 
    // handle any incoming radio frames
    if (radio.receiveDone())
        handleRadioReceive();

    // breathing light
    // work around the 50 day rollover
    if (millis() < lastStatusChange)
        lastStatusChange = 0;

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
                byte curColour = (byte)(c >> STATUS_NEOPX_COLOUR);
                byte newColour = (millis() - lastStatusChange) / 62;
                uint32_t colValue = ((uint32_t)newColour << STATUS_NEOPX_COLOUR);
        
                if (curColour != newColour) {
                    pixels.setPixelColor(STATUS_NEOPX_POSITION, colValue);
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
                byte curColour = (byte)(c >> STATUS_NEOPX_COLOUR);
                byte newColour = 16 - ((millis() - lastStatusChange) / 62);
                uint32_t colValue = ((uint32_t)newColour << STATUS_NEOPX_COLOUR);

                if (curColour != newColour) {
                    pixels.setPixelColor(STATUS_NEOPX_POSITION, colValue);
                    pixels.show();
                }
            }
            break;
    }

    // radio status light
    // work around the 50 day rollover
    if (millis() < radioStatusOnTime)
        radioStatusOnTime = 0;

    if (radioLedStatus == STATUS_RADIO_NEOPX_ON)
        if (millis() - radioStatusOnTime > RADIO_STATUS_NEOPX_ONTIME) {
            // the status LED is only on for 50msec
            radioLedStatus = STATUS_RADIO_NEOPX_OFF;
            pixels.setPixelColor(RADIO_STATUS_NEOPX_POSITION, pixels.Color(0, 0, 0));
            pixels.show();
        }

#ifdef LOG_LOOP_TIMES
    if (!loopStatReset) {
        loopTime = millis() - loopTimeStart;

        //loopTimeAverage = (loopTimeAverage * loopCounter + loopTime) / 2;
        loopTimeAverage = (loopTimeAverage * loopCounter + loopTime) / (loopCounter + 1);

        if (loopTime < loopTimeMin || loopTimeMin == 0)
            loopTimeMin = loopTime;

        if (loopTime > loopTimeMax || loopTimeMax == 0)
            loopTimeMax = loopTime;

        loopCounter++;
    } else
        loopStatReset = false;
#endif
}
