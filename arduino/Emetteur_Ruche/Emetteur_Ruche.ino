/*********
  Emetteur ruche
  Envoi d'un seul message par appel
*********/

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Librairie pour le HX711
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 21;
const int LOADCELL_SCK_PIN = 22;
long poids=0;
HX711 scale;

// Librairies pour les capteurs OneWire
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>
// GPIO where the DS18B20 is connected to GPIO 13
const int oneWireBus = 13;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
// Number of temperature devices found
int numberOfDevices;
// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 
// Temperature value
float tempSensor;
char DistString[6]; // Format de la température

//Couleur de la ruche
String COUL="bleu";
String outgoing;    //message texte d'envoi LORA
byte msgCount = 0;            // Nb de messages envoyés
byte MasterNode = 0xFF;     //Adresse du récepteur ASC
byte Node1 = 0xBB;          //Adresse de l'émetteur. Du coup, on peut enlever la couleur de ruche (paramétrable au niveau du récepteur)



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

//packet counter
int counter = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);

  //initialisation HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

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
  display.setRotation(2);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Test");

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
  display.print("Ruche "+COUL+" OK!");
  display.display();
  delay(2000);

  // Start up the library (Capt. temp)  
  sensors.begin();
  Serial.println();
    // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  
  // locate devices on the bus
  Serial.print("Détection des capteurs de température...");
  Serial.print("Trouvé ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" capteurs.");
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    delay(200);
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Capteur ");
      Serial.print(i, DEC);
      Serial.print(", addresse: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  // il faudrait ajouter un contacteur sous le toit de la ruche pour soit tourner en boucle sur les capteurs avec affichage, soit éteindre l'affichage
  
}

void loop() {

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(MasterNode);              // add destination address
  Serial.print ("Noeud maitre:" + String(MasterNode));
  LoRa.write(otherNode);             // add sender address
  Serial.print (" Noeud envoyeur:" + String(otherNode) );
  LoRa.write(msgCount);                 // add message ID
  Serial.print ("msg n°:" + String(msgCount));
  LoRa.write(outgoing.length());        // add payload length
  Serial.println ("Longueur:" + String(outgoing.length()));
  LoRa.print(outgoing);                 // add payload
  Serial.println (outgoing);
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
   ;
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
    Serial.println(incoming);
    int Val = incoming.toInt();
    if(Val == 34)
// Message reçu du master OK
    { 
      String message ="";
      if (scale.is_ready()) {
          poids = scale.read()/100+400;
          Serial.print("Poid/");
          Serial.println(poids);
          String message = "Poid/"+String(poids)+"/"; 
          // sendMessage(message,MasterNode,Node1); //Remplacé par adresse Node1
          // delay(100);
        } else {
          Serial.println("HX711 non trouvé");
          poids=0;
        }
      
        sensors.requestTemperatures(); // Send the command to get temperatures
       // Loop through each device, print out temperature data
        for(int i=0;i<numberOfDevices; i++){
          // Search the wire for address
          if(sensors.getAddress(tempDeviceAddress, i)){
            // Valeur température
            float tempC = sensors.getTempC(tempDeviceAddress);
           //Vérifier le format de tempC: a faire
            // adresse capteur
            String refcapteur = "";
            if (tempDeviceAddress[1] < 16) refcapteur= refcapteur+"0";
            refcapteur= refcapteur+String(tempDeviceAddress[1],HEX);
            if (tempDeviceAddress[7] < 16) refcapteur= refcapteur+"0";
            refcapteur= refcapteur+String(tempDeviceAddress[7],HEX)+"/";
            //Vérifier que refcapteur est une valeur hexa: à faire
            refcapteur= refcapteur+"/";    
            message = message + refcapteur+"/"+String(tempC,1)+"/";
            delay(200);                
            }
         }
        sendMessage(message,MasterNode,Node1); //Remplacé par adresse Node1

         // Enlever le display à terme
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Ruche "+ COUL);
        display.setCursor(0,10);
        display.setTextSize(1);
        display.print("Poids: ");
        display.setCursor(50,10);
        display.print(poids);
        display.setCursor(80,10);
        display.print(msgCount);      
        display.display();
    }
  
}

     

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}
