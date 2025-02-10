//#include <SPI.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "interface.html"
#include "constants.h"
#include <RPC.h>

#define DEBUG_M7 false
#define AP_MODE false

// Inicializando RTOS
using namespace rtos;
Thread sensorThread;

// Configuración de red WiFi
const char *ssid = "Analogue_Hyperlapse_Camera";
const char *password = "CraterLab";

IPAddress ip(192, 168, 8, 4);
IPAddress remote_ip(192, 168, 8, 3);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 8, 1); //primaryDNS

// Crear el servidor en el puerto 80
WiFiServer server(80);
// Crear conexion UDP para enviar datos
WiFiUDP Udp;
unsigned int remotePort = 2390;      // local port to send
unsigned int localPort = 2392;      // local port to read
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

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

//Sensores
float motorSpeedRead = 0;
float FadePercentRead = 0;
int zoomValueRead = 0;
int focusValueRead = 0;
float diaphragmValueRead = 0;
int x0DegreesRead = 0;
int x1DegreesRead = 0;
int y0DegreesRead = 0;

int read_value[10];
int* decode_values(String inputString, int num_dec)
  {
    String str = inputString.substring(inputString.lastIndexOf(":") + 1);
    read_value[0] = str.substring(0, str.indexOf(',')).toInt();
    for(int i=1; i<num_dec; i++)
      {
        str = str.substring(str.indexOf(',')+1);
        read_value[i] = str.substring(0, str.indexOf(',')).toInt();
      }
    return read_value;
  }

char c_rpc;
String inputString_rpc;

void RPCRead()
  {
     ////////////////////////////////////////////////////////////
     /////// RUTINA TRATAMIENTO DE STRINGS DEL UART   ///////////
     ////////////////////////////////////////////////////////////
     if (RPC.available())
       {
         c_rpc = RPC.read();
         if ((c_rpc == '\r') || (c_rpc == '\n'))
         {
          if (inputString_rpc.startsWith("/sensors:")) //Speed Motor
           {
              int* value = decode_values(inputString_rpc, 5);
              #if DEBUG_M4
                RPC.print("Recibido Speed M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<5; i++) RPC.println(value[i]);
              #endif
              motorSpeedRead = value[0];
              FadePercentRead = value[1];
              zoomValueRead = value[2];
              focusValueRead = value[3];
              diaphragmValueRead = value[4];
            }
           else if(inputString_rpc.startsWith("/debug:"))
            {
              //int* value = decode_values(inputString_rpc, 5);
              //#if DEBUG_M7
                Serial.print("Recibido debug M4: ");
                Serial.println(inputString_rpc);
                //for(int i=0; i<5; i++) Serial.println(value[i]);
              //#endif

            }
           inputString_rpc = String();
         }
         else
           inputString_rpc += c_rpc;
       }
  }

void setup()
{
  Serial.begin(1000000);
  RPC.begin(); //boots M4
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
    WiFi.config(ip, dns, gateway, subnet); 
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
  // Enable Server
  server.begin();
  // Enable UDP
  Udp.begin(localPort);
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
          RPC.println("/s_motor:" + String(FPS_MOTOR) + "," + String(motorDirection) + "," + String(motorSpeedValue) + "," + String(save));
        }
      else if(motorIsIntervalActive)
        {
          RPC.println("/i_motor:" + String(FPS_MOTOR) + ","  + String(motorDirection) + "," + String(motorIntervalFrames) + "," + String(motorIntervalSeconds) + "," + String(save));
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
  else if (type=="accion")
    {
      Serial.println(type);
      RPC.println("/action:" + String(1));
      Udp.beginPacket(remote_ip, remotePort);
      Udp.println("/action:" + String(1));
      Udp.endPacket();
    }
  else if (type=="corten")
    {
      Serial.println(type);
      RPC.println("/action:" + String(0));
      Udp.beginPacket(remote_ip, remotePort);
      Udp.println("/action:" + String(0));
      Udp.endPacket();
    }
  else if (type=="stop") RPC.println("/s_motor:4," + String(motorDirection) + "," + String(0) + ", 0");
  else Serial.println("No reconocido");
}

unsigned long time_blink=millis();
bool state = false;

void loop() {
  WiFiClient client = server.available();
  if (client) {
    #if DEBUG_M7
      Serial.println("Nuevo cliente conectado");
    #endif
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
          #if DEBUG_M7
            Serial.println("Solicitud recibida:");
            Serial.println(request);
          #endif

          if (isPostRequest) {
            String body = "";
            while (client.available()) {
              body += (char)client.read();
            }
            #if DEBUG_M7
              Serial.println("Cuerpo del mensaje recibido:");
              Serial.println(body);
            #endif
            // Llamamos a la función para procesar la petición POST
            handlePostRequest(body);
            // Respuesta al cliente
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/plain");
            client.println();
            client.println("Datos enviados correctamente");
          } else if (request.indexOf("GET /sensors") >= 0) { // actualiza fps en interface
            // Respuesta para la ruta /motorSpeed
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println();
            String response = "{\"sensors\":[" + String(motorSpeedRead) + "," + String(FadePercentRead) + "," 
                                  + String(x0DegreesRead) + "," + String(x1DegreesRead) + "," + String(y0DegreesRead) + "]}";
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
    #if DEBUG_M7
      Serial.println("Cliente desconectado");
    #endif
  }
  //Enviar datos a M4
  RPCRead();

  // Leer datos del RPC
  /*String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();
  }

  if (buffer.length() > 0) {
    Serial.print(buffer);
  }*/
}