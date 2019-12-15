#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> //Library to use BLE as server
#include <BLE2902.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "Adafruit_APDS9960.h"
#define REPORTING_PERIOD_MS     1000 
#include "Adafruit_FONA.h"

// standard pins for the shield
#define FONA_RX 2
#define FONA_TX 5
#define FONA_RST 4

HardwareSerial *fonaSerial = new HardwareSerial(1);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Variables globales
char pincode[5]="1234"; // Array con el PIN.
char *PIN = pincode;    // Puntero al array con el PIN.
char lat[18]="";        // Latitud
char lon[18]="";        // Longitud
char gpsloc[40]="";     //Cadena para almacenar "lat,lon"
bool flagLoc = true;    // Bandera para que el GPS solo se active 1 vez.
bool flagSIM = true;    // Bandera para que el desbloqueo de SIM solo se ejecute 1 vez.

//variables para el botón de cancelar y desbloqueo
const int buttonPin = 15;    // the number of the pushbutton pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int freq = 250;
int channel = 0;
int resolution = 8;

Adafruit_APDS9960 apds;
uint32_t tsLastReport = 0;
uint8_t pulsaciones = 0;
uint8_t oxygen = 0;
int veces_por_debajo = 0;
PulseOximeter pox;
bool bloqueada = false;
int veces_zumbador = 0;
bool inicializada = false;
bool desconectada = false;
bool _BLEClientConnected = false;

#define HeartService BLEUUID((uint16_t)0x180D) 
BLECharacteristic HeartLevelCharacteristic(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor HeartLevelDescriptor(BLEUUID((uint16_t)0x2901));

#define GeoService BLEUUID((uint16_t)0x1819) 
BLECharacteristic PositionDosDCharacteristic(BLEUUID((uint16_t)0x2A2F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor PositionDosDDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("GeoCardioBand");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service to heart
  BLEService *pHeart = pServer->createService(HeartService);

  pHeart->addCharacteristic(&HeartLevelCharacteristic);
  HeartLevelDescriptor.setValue("Heart BPMs");
  HeartLevelCharacteristic.addDescriptor(&HeartLevelDescriptor);
  HeartLevelCharacteristic.addDescriptor(new BLE2902());

  pServer->getAdvertising()->addServiceUUID(HeartService);

  //Create the BLE Service to geolocation
  BLEService *pGeo = pServer->createService(GeoService);

  pGeo->addCharacteristic(&PositionDosDCharacteristic);
  PositionDosDDescriptor.setValue("Position in 2D");
  PositionDosDCharacteristic.addDescriptor(&PositionDosDDescriptor);
  PositionDosDCharacteristic.addDescriptor(new BLE2902());

  pServer->getAdvertising()->addServiceUUID(HeartService);

  pGeo->start();
  pHeart->start();
  // Start advertising
  pServer->getAdvertising()->start();
}


void notifyHeartRate(int heartRate){
  uint8_t heartRateCharArray[2] = { 0, (uint8_t)heartRate };
  HeartLevelCharacteristic.setValue(heartRateCharArray, 2); //nofificamos al dispositivo móvil las pulsaciones
  HeartLevelCharacteristic.notify();
}

void notifyGPS(){
  char* localizacion = gpsloc;
  PositionDosDCharacteristic.setValue(localizacion);
  PositionDosDCharacteristic.notify();
}

bool bandConnected(){
  return apds.readProximity() > 200;
}

uint8_t getHeartRate(){
  return pox.getHeartRate();
}

uint8_t getOxygenRate(){
  return pox.getSpO2();
}


// Procedimiento para enviar un SMS a emergencias añadiendo la localización.
void enviarSMS(){
    char sendto[21] = "677148940";
    char message[141];
    Serial.print(F("Send to #"));
    Serial.println(sendto);
    Serial.print(F("Type out one-line message (140 char): "));
    sprintf(message,"Emergencia! Se ha provocado un infarto en la localizacion: %s",gpsloc);
    Serial.println(message);
    if (!fona.sendSMS(sendto, message)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("Sent!"));
    }
}

void notifyEmergency(){
  enviarSMS();
  Serial.println("Mandando mensajes a emergencias!");
}

void sonarZumbador(){
  ledcWrite(channel, freq);
  delay(1000);
  ledcWriteTone(channel, 0);
}

void encenderSensorCardiaco(){
    if (!pox.begin()) {
        //Serial.println("FAILED");
        for(;;);
    } else {
        //Serial.println("SUCCESS");
    }  
}


void encenderGPS(){
  if (!fona.enableGPS(true))
          Serial.println(F("Failed to turn off"));
}

void apagarGPS(){
  if (!fona.enableGPS(false))
          Serial.println(F("Failed to turn off"));
}

void obtenerLocalizacion() {
  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  
  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (gps_success) {

    // Una vez encontrada la posición, esto se actualizará constantemente
    //Convertir a un charArray
    dtostrf(latitude,8,6,lat); //Llamada a la función: nº a convertir, tam del nº, nº decimales, buffer
    dtostrf(longitude,8,6,lon);
    Serial.print("Localizacion lat: ");
    Serial.println(lat);
    Serial.print("Localizacion lon: ");
    Serial.println(lon);
    // Unir en una sola cadena (variable gpsloc) y mostrar
    sprintf(gpsloc,"%s,%s",lat,lon);
    Serial.print("Cadena completa: ");
    Serial.println(gpsloc);

  } else {
    Serial.println("Waiting for FONA GPS 3D fix...");
  }
}

//Procedimiento para desbloquear la SIM con el PIN. Introducir el pin más de una vez, dará error.
void desbloquearSIM(){
    Serial.print(F("Unlocking SIM card: "));
    if (! fona.unlockSIM(PIN)) {
      Serial.println(F("Failed. Wrong PIN or SIM already unlocked!"));
    } else {
      Serial.println(F("OK!"));
    }
}



void setup() {

  while(! Serial);
  
  Serial.begin(115200);
  ledcSetup(channel, freq, resolution);
    ledcAttachPin(12, channel);
    
  //SerialBT.begin("GeoCardioBand"); //Bluetooth device name
  Serial.println("Heart rate Indicator - BLE");
  Serial.println("Bluetooth started, now you can pair it with bluetooth!");
  InitBLE();
  pinMode(buttonPin, INPUT);
  //pox.setOnBeatDetectedCallback(onBeatDetected);

  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Proximity initialized!"); 
  
  apds.enableProximity(true);
  apds.setProximityInterruptThreshold(0, 175);
  apds.enableProximityInterrupt();

  fonaSerial->begin(9600, SERIAL_8N1, FONA_RX, FONA_TX);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  Serial.print("Initializing pulse oximeter..");
  encenderSensorCardiaco();
  //cambiar intesidad de la luz del cardiaco: default = 50mA
  //pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
}


void loop() {
  pox.update();
  if(!inicializada){
      Serial.println("Inicializando..."); // print it
  }
  while(!inicializada){ //espera para que cargue el sensor cardiaco
    uint8_t value = getHeartRate();
    Serial.print("Heart Rate is now: "); // print it
    Serial.println((int)value);
    if(value>50){
      inicializada=true;
        if(flagSIM==true){ //desbloqueamos la SIM
          desbloquearSIM();
          flagSIM=false;
        }
      Serial.println("Inicializada..."); // print it
    }
    else{
      pox.update();
    }
  }

  if(bloqueada == false){
    if (bandConnected() == true){
      if(desconectada==true){
        encenderSensorCardiaco();
        desconectada=false;
      }
      veces_zumbador = 0;
      if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        uint8_t heartRate = getHeartRate();
        uint8_t oxygen = getOxygenRate();
        Serial.print("Heart Rate is now: "); // print it
        Serial.println((int)heartRate);
        Serial.print("Oxygen is now: "); // print it
        Serial.println((int)oxygen);
        
        notifyHeartRate(heartRate); //notificar las pulsaciones
      
        if (heartRate < 50){
          veces_por_debajo ++;
          if(veces_por_debajo >=10){
            Serial.println("10 veces por debajo"); // print it
            //TODO activar actuador
            boolean cancelar = false;
            encenderGPS();
            //TODO esperar por la respuesta del botón
            for (int i = 0; i<10; i++){
              //TODO sonar zumbador 1 segundo, sustituyendo delay
              //delay(1000);
              sonarZumbador();
              Serial.println("Zumbador sonando!");
              //leer botón
              int respuesta = digitalRead(buttonPin); //0 si se ha pulsado
              //si boton es true,
              if (respuesta == 0){
                cancelar = true;
                break;
              }
            }
            if (!cancelar){
              delay(20000);
              int ciclos = 0;
              while(ciclos!=10){ //afinar la localizacion
                obtenerLocalizacion(); //obtiene la localizacion en la variable global
                ciclos++;
              }
              notifyGPS(); //notificar la posición para que el movil busque el conocido más cercano para notificar.
              notifyEmergency(); //notificar a emergencias
              bloqueada = true;
              veces_por_debajo=0;
            }
            else{
               veces_por_debajo=0; 
               encenderSensorCardiaco();
            }
            apagarGPS();
            
          }
        }
        else{
          veces_por_debajo = 0;
        }
        
        tsLastReport = millis();
      }
    }
    else{
      Serial.println("La pulsera se ha desconectado");
      //TODO Sonar zumbador 1 segundos
      if(veces_zumbador < 5){
          sonarZumbador();
          Serial.println("Zumbador sonando!");
          veces_zumbador = veces_zumbador + 1;
      }
      desconectada=true;
      veces_por_debajo = 0; //reseetear para ser 0
    }
  }
  else{
    while (bloqueada == true){
      Serial.println("Pulsera bloqueada");
      int respuesta = digitalRead(buttonPin); //0 si se ha pulsado
      if (respuesta == 0){
        bloqueada = false;
        encenderSensorCardiaco();
      }
    }
  }
}
