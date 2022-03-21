/*********
  Ruche connectée
*********/

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <string.h>

#define WIFI_SSID "ASC-CD"
#define WIFI_PASSWORD "Asczonesud"

// Raspberri Pi Mosquitto MQTT Broker. IP fixe
#define MQTT_HOST IPAddress(88, 168, 198, 119)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

char topic[21];     //Format du topic MQTT: ruche/COUL/CAPT/
char Valeur[6]; // Format de la température et du poids: 5 caractères soit 6
String LoRaData;
String Couleur; //Code Couleur de la ruche
String Capteur; // Reference du capteur

byte MasterNode = 0xFF;     //Adresse Master
byte Node1 = 0xBB;  //Adresse Ruche 1
byte Node2 = 0xCC; //Adresse Ruche 2

String SenderNode = "";
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages

// Tracks the time since last event fired
unsigned long previousMillis=0;
unsigned long int previoussecs = 0; 
unsigned long int currentsecs = 0; 
unsigned long currentMillis = 0;
int interval= 1 ; // updated every 1 second
int Secs = 0; 


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
    //Ne semble pas fonctionner...
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void setup() { 
  //initialize Serial Monitor
  Serial.begin(115200);
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA RECEIVER ");
  display.display();

  Serial.println("LoRa Receiver Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(10);           // ranges from 6-12,default 7 see API docs
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();  
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  // mqttClient.setCredentials("Olivier", "olivier");
  connectToWifi();
}

void loop() {
currentMillis = millis();
   currentsecs = currentMillis / 1000; 
    if ((unsigned long)(currentsecs - previoussecs) >= interval) {
     Secs = Secs + interval;
     //Serial.println(Secs);
     if ( Secs >= 21 )
       {
        Secs = 0; 
      }
    if ( (Secs >= 1) && (Secs <= 10) )
      {    
      String message = "34"; 
      sendMessage(message,MasterNode, Node1);
      }
    if ( (Secs >= 11 ) && (Secs <= 20))
      { 
      String message = "55"; 
      sendMessage(message,MasterNode, Node2);
      } 
    previoussecs = currentsecs;
    }
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket()); 
  }

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(otherNode);              // add destination address
  LoRa.write(MasterNode);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  Couleur="";
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  if( sender == 0XBB )
  Couleur = "bleu:";
  if( sender == 0XCC )
  Couleur = "roug:";
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  // if the recipient isn't this device or broadcast,
  if (Couleur!="" && recipient != MasterNode) {
    Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  } else {
  // if message is for this device, or broadcast, print details:
  Serial.print("De ruche:" + Couleur);
  Serial.println("-> à 0x" + String(recipient, HEX));
  Serial.print("Message ID: " + String(incomingMsgId));
  Serial.println("  Message length: " + String(incomingLength));
  Serial.print("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("  Snr: " + String(LoRa.packetSnr()));
    //clear display
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(Couleur);
  // String incoming = "";

  while (LoRa.available()) {
        Capteur = LoRa.readStringUntil('/');  // on lit les 4 caractères: ref capteur
        if (Capteur.length()==4){ //Vérification de la bonne longueur
          //Ajouter test sur string: isAlphaNumeric(Capteur)
          Serial.print (Capteur); //Identifiant du capteur
          LoRaData = LoRa.readStringUntil('/'); // On lit la valeur
          // Ajouter test sur string: isDigit(LoRaData)
          LoRaData.toCharArray(Valeur,6); // On la convertit en char
          Serial.print (LoRaData);
          //Publication sur MQTT    
          LoRaData="ruche/test/"+Couleur+"/"+Capteur+"/";
          LoRaData.toCharArray(topic,21); 
          uint16_t packetIdPub1 = mqttClient.publish(topic, 1, true, Valeur);          
        } else {
          Serial.print ("Erreur");
        } 
        display.setCursor(0,28);
        display.print(Capteur + " " + Valeur);
        display.display();     
        //incoming += (char)LoRa.read();
  }
// test sur la longueur: enlevé
 // if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");
  //  ;
  //  return;                             // skip rest of function
  }


}
