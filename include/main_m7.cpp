//#include <SPI.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "interface.html"
#include "constants.h"
#include <RPC.h>
#include "RPC.h"

#define AP_MODE false

// Inicializando RTOS
using namespace rtos;
Thread sensorThread;

// Configuración de red WiFi
const char *ssid = "Analogue_Hyperlapse_Camera";
const char *password = "CraterLab";

IPAddress ip(192, 168, 8, 4);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 8, 1); //primaryDNS

// Crear el servidor en el puerto 80
WiFiServer server(80);

#define command Serial2

// Motor
int motorSpeedValue = 25;
bool motorIsSpeedActive = false;
int motorIntervalFrames = 1;
int motorIntervalSeconds = 1;
bool motorIsIntervalActive = false;
int motorDirection = 1;

// Shutter
int shutterFadePercent = 0;
int shutterFadeFrames = 50;
bool shutterSyncWithInterval = false;
bool shutterFadeInActive = false;
bool shutterFadeOutActive = false;

// Óptica
int zoomValue = 50;
int focusValue = 50;
float diaphragmValue = 1.8;
bool syncWithIntervalOptics = false;

// Dispositivo 360
int x0Degrees = 0;
int x0Duration = 0;
int x1Degrees = 0;
int x1Duration = 0;
int y0Degrees = 0;
int y0Duration = 0;
bool syncWithInterval360 = false;

int motorSpeedRead = 0;

void requestReading() {
  while (true) {
    delay(25);
    motorSpeedRead = RPC.call("fpsRead").as<int>();
  }
}

void setup()
{
  Serial.begin(115200);
  command.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);
  if (!RPC.begin()) {
    Serial.println("RPC initialization fail");
  }

  #if AP_MODE
    int status = WL_IDLE_STATUS;
    // Create open network. Change this line if you want to create an WEP network:
    WiFi.config(ip, dns, gateway, subnet); 
    status = WiFi.beginAP(ssid, password);
    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
      // don't continue
      while (true)
        ;
    }
  #else
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1000);
      Serial.print(".");
    }
  #endif

  Serial.println();
  Serial.println("Conectado a WiFi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  //sensorThread.start(requestReading);
}

bool flag_send[10] = {false ,false ,false ,false ,false ,false ,false ,false ,false ,false};

void handlePostRequest(String body) {
  // Parseamos el cuerpo del JSON
  StaticJsonDocument<1000> doc;
  deserializeJson(doc, body);
  int save = true;
  //DeserializationError error = deserializeJson(doc, body);
  /*if (error) {
    Serial.println("Error al parsear el JSON");
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }*/

  Serial.println("Procesando JSON...");
  String type = doc["type"];
  if ((type=="save_motor")||(type=="test_motor"))
    {
      Serial.println(type);
      if (type=="save_motor") save = true;
      else save = false;

      /*motorSpeedValue = doc["speedValue"];
      motorIsSpeedActive = doc["isSpeedActive"];
      motorIntervalFrames = doc["intervalFrames"];
      motorIntervalSeconds = doc["intervalSeconds"];
      motorIsIntervalActive = doc["isIntervalActive"];
      String Direction = doc["direction"];*/
      motorSpeedValue = doc["0"];
      motorIsSpeedActive = doc["1"];
      motorIntervalFrames = doc["2"];
      motorIntervalSeconds = doc["3"];
      motorIsIntervalActive = doc["4"];
      String Direction = doc["5"];
      if (Direction=="forward") motorDirection = FORWARD;
      else if (Direction=="backward") motorDirection = BACKWARD;
      if(motorIsSpeedActive)
        {
          RPC.println("/s_motor:4," + String(motorDirection) + "," + String(motorSpeedValue) + "," + String(save));
        }
      else if(motorIsIntervalActive)
        {
          RPC.println("/i_motor:4," + String(motorDirection) + "," + String(motorIntervalFrames) + "," + String(motorIntervalSeconds) + "," + String(save));
        }   
      /*Clave: type, Valor: motor
      Clave: speedValue, Valor: )25
      Clave: isSpeedActive, Valor: false
      Clave: intervalFrames, Valor: 1
      Clave: intervalSeconds, Valor: 1
      Clave: isIntervalActive, Valor: false
      Clave: direction, Valor: forward*/
    }
  else if ((type=="test_shutter")||(type=="save_shutter") )
    {
      Serial.println(type);
      if (type=="save_shutter") save = true;
      else save = false;
      /*shutterFadePercent = doc["fadePercent"];
      shutterFadeFrames = doc["fadeFrames"];
      shutterSyncWithInterval = doc["syncWithInterval"];
      shutterFadeInActive = doc["fadeInActive"];
      shutterFadeOutActive = doc["fadeOutActive"];*/
      shutterFadePercent = doc["0"];
      shutterFadeFrames = doc["1"];
      shutterSyncWithInterval = doc["2"];
      shutterFadeInActive = doc["3"];
      shutterFadeOutActive = doc["4"];
      RPC.println("/fade:" + String(shutterFadePercent) + "," + String(shutterFadeFrames) + "," + String(shutterSyncWithInterval) + "," + String(shutterFadeInActive) + "," + String(shutterFadeOutActive) + "," + String(save));
    }
  else if ((type=="test_optics")||(type=="save_optics"))
    {
      Serial.println(type);
      if (type=="save_optics") save = true;
      else save = false;
      zoomValue = doc["zoomValue"];
      focusValue = doc["focusValue"];
      diaphragmValue = doc["diaphragmValue"];
      syncWithIntervalOptics = doc["syncWithInterval"];
      RPC.println("/optics:" + String(zoomValue) + "," + String(focusValue) + "," + String(diaphragmValue) + "," + String(syncWithIntervalOptics) + "," + String(save));
    }
  else if((type=="test_360")||(type=="save_360"))
    {
      Serial.println(type);
      if (type=="save_360") save = true;
      else save = false;
      String axis = doc["motor"];
      if (axis=="x0")
        {
          x0Degrees = doc["degrees"];
          x0Duration = doc["duration"];
          syncWithInterval360 = doc["syncWithInterval"];
          command.println("/axisA:" + String(x0Degrees) + "," + String(x0Duration) + "," + String(syncWithInterval360) + "," + String(save));
        }
      else if (axis=="x1")  
        {
          x1Degrees = doc["degrees"];
          x1Duration = doc["duration"];
          syncWithInterval360 = doc["syncWithInterval"];
          command.println("/axisX:" + String(x1Degrees) + "," + String(x1Duration) + "," + String(syncWithInterval360) + "," + String(save));
        }
      else if (axis=="y0")
        {
          y0Degrees = doc["degrees"];
          y0Duration = doc["duration"];
          syncWithInterval360 = doc["syncWithInterval"];
          command.println("/axisY:" + String(y0Degrees) + "," + String(y0Duration) + "," + String(syncWithInterval360) + "," + String(save));
        }
    }
  else if (type=="accion")
    {
      Serial.println(type);
      RPC.println("/action:" + String(1));
      command.println("/action:" + String(1));
    }
  else if (type=="corten")
    {
      Serial.println(type);
      RPC.println("/action:" + String(0));
      command.println("/action:" + String(0));
    }
  else if (type=="stop") RPC.println("/s_motor:4," + String(motorDirection) + "," + String(0) + ", 0");
  else Serial.println("No reconocido");
  digitalWrite(LED_BUILTIN, HIGH);
}

unsigned long time_blink=millis();
bool state = false;

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Nuevo cliente conectado");
    String request = "";
    bool isPostRequest = false;

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;

        // Identificar si es una solicitud POST
        if (request.indexOf("POST") >= 0) {
          isPostRequest = true;
        }

        // Si se encuentra el final de la solicitud
        if (c == '\n' && request.endsWith("\r\n\r\n")) {
          Serial.println("Solicitud recibida:");
          Serial.println(request);

          if (isPostRequest) {
            String body = "";
            while (client.available()) {
              body += (char)client.read();
            }

            Serial.println("Cuerpo del mensaje recibido:");
            Serial.println(body);

            // Llamamos a la función para procesar la petición POST
            handlePostRequest(body);

            // Respuesta al cliente
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/plain");
            client.println();
            client.println("Datos enviados correctamente");
          } else if (request.indexOf("GET /motorSpeed") >= 0) { // actualiza fps en interface
            // Respuesta para la ruta /motorSpeed
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println();
            String response = "{\"motorSpeedRead\": " + String(motorSpeedRead) + "}";
            client.print(response); 
          } else {
            // Respuesta para servir la interfaz HTML
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print(htmlTemplate);
          }
          break;
        }
      }
    }
    client.stop();
    Serial.println("Cliente desconectado");
  }

  // Leer datos del RPC
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();
  }

  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
}