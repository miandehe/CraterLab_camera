#include <Arduino.h>
#include <ShiftRegister74HC595.h>
//#include <PID_v1.h> 
//#include <SPI.h>
#include <RPC.h>
#include "constants.h"

#define DEBUG_M4 true
#define MIN_SPEED 110
// Inicializando RTOS
using namespace rtos;
Thread sensorThread;

// Motor
int motorSpeedValue = 25;
bool motorIsSpeedActive = false;
int motorIntervalFrames = 1;
int motorIntervalSeconds = 1;
bool motorIsIntervalActive = false;
bool motorDirection = 1;

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

bool closed=false;
bool isAction=false;

#if PID_ENABLE
  // ************************************************ Variables PID *****************************************************************
  double        Setpoint = 0.0, Input = 0.0, Output = 0.0;  // Setpoint=Posición designada; Input=Posición del motor; Output=Tensión de salida para el motor.
  double        kp = 0.0, ki = 0.0, kd = 0.0;               // Constante proporcional, integral y derivativa.
  double        outMax = 0.0, outMin = 0.0;                 // Límites para no sobrepasar la resolución del PWM.
  // **************************************************** Otras Variables ***********************************************************
  volatile long contador = 0;           // En esta variable se guardará los pulsos del encoder y que interpreremos como distancia (o ángulo si ese fuese el caso).
  byte          ant = 0, act = 0;       // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder. (ant=anterior, act=actual.)
  byte          cmd = 0;                // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
  unsigned int  tmp = 0;                // Variable que utilizaremos para poner el tiempo de muestreo.
  const byte    ledok = 13;             // El pin 13 de los Arduinos tienen un led que utilizo para mostrar que el motor ya ha llegado a la posición designada.
  // ********************************************************************************************************************************

  PID myPID(&Input, &Output, &Setpoint, 0.0, 0.0, 0.0, DIRECT); // Parámetros y configuración para invocar la librería.
#endif

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<1> sr(8, 4, 12);
#define M2A 0
#define M1A 1
#define M0A 2
#define M0B 3
#define M1B 4
#define M3A 5
#define M2B 6
#define M3B 7
#define M4A 31
#define M4B 33
#define M5A 29
#define M5B 27

const int EN_MOTOR[6] = {11, 3, 5, 6, 10, 9};
const int MA_MOTOR[6] = {M0A, M1A, M2A, M3A, M4A, M5A};
const int MB_MOTOR[6] = {M0B, M1B, M2B, M3B, M4B, M5B};

#define sr_enable 7
#define STOP 0
#define CONTINUOUS 1
#define ONE_TURN 2
const long TIME_LINEAL_MAX[6] = {4500, 670, 4250, 6200, 4000, 4000 };  //ms

#define command Serial2


void backward(int motor, int SPEED)
  {
    if(motor<4)
      {
        sr.set(MA_MOTOR[motor], HIGH); 
        sr.set(MB_MOTOR[motor], LOW); 
      }
    else 
      {
        digitalWrite(MA_MOTOR[motor], HIGH); 
        digitalWrite(MB_MOTOR[motor], LOW); 
      }
    analogWrite(EN_MOTOR[motor], SPEED);
  }
 
void forward(int motor, int SPEED)
  {
    if(motor<4)
      {
        sr.set(MA_MOTOR[motor], LOW); 
        sr.set(MB_MOTOR[motor], HIGH); 
      }
    else 
      {
        digitalWrite(MA_MOTOR[motor], LOW); 
        digitalWrite(MB_MOTOR[motor], HIGH); 
      }
    analogWrite(EN_MOTOR[motor], SPEED);
  }

void stop(int motor)
  {
    analogWrite(EN_MOTOR[motor], 0);
    if(motor<4)
      {
        sr.set(MA_MOTOR[motor], LOW); 
        sr.set(MB_MOTOR[motor], LOW); 
      }
    else 
      {
        digitalWrite(MA_MOTOR[motor], LOW); 
        digitalWrite(MB_MOTOR[motor], LOW); 
      }
  }

unsigned long time_move_motor[6] = {0, 0, 0, 0, 0, 0};
unsigned long time_refresh[6] = { millis(), millis(), millis(), millis(), millis(), millis()};
float position_motor[6] = { 0, 0, 0, 0, 0, 0};
bool state_pmotor[6] = { false, false, false, false, false, false};

void config_position_motor(int motor, float position) //Configuracion posicion en % motor
  {
    if((position!=position_motor[motor])&&(!state_pmotor[motor]))
      {
        time_move_motor[motor] = abs(position_motor[motor]-position)*TIME_LINEAL_MAX[motor]/100;
        time_refresh[motor] = millis();
        if(position>position_motor[motor]) forward(motor,255);
        else backward(motor,255);
        position_motor[motor] = position;
        state_pmotor[motor] = true;
        //Serial.println(time_move_motor[motor]);
      }
  }

bool state_smotor[6] = { false, false, false, false, false, false};

void config_speed_motor(int motor, int direction, int speed) //Configuracion velocidad motor
  {
    closed = false;
    if(direction==FORWARD) forward(motor,speed);
    else if(direction==BACKWARD) backward(motor,speed);
    if(speed>0) state_smotor[motor] = true;
    else state_smotor[motor] = false;
  }


volatile float fps_read = 0;
volatile float fps_temp = 0;
unsigned long time_sec= millis();
unsigned long time_fps = micros();

static volatile unsigned long lastTimeDebounce = 0; //Tiempo del rebote
static volatile bool force_stop = false; //Tiempo del rebote

void fps_count_state() {
  if((digitalRead(20) && (micros()-lastTimeDebounce >= 500))&&(!closed))
    {
      lastTimeDebounce = micros();
      fps_temp = 1000000./(float)(micros()-time_fps);
      if(fps_temp<70) fps_read = fps_temp;
      time_fps = micros();
      if((millis()-time_sec)>=1000)
        {
          //RPC.println(fps_read);
          time_sec = millis();
        }
    }
  else if((digitalRead(20)&&(micros()-lastTimeDebounce >= 150))&&(closed))
    {
      lastTimeDebounce = micros();
      closed = false;
      force_stop = true;      
    }
}

int interval_direction[6] = {0, 0, 0, 0, 0, 0};
int interval_frames[6] = {0, 0, 0, 0, 0, 0};
unsigned long interval_time[6] = {0, 0, 0, 0, 0, 0};
int frames_count[6] = {0, 0, 0, 0, 0, 0};
unsigned long time_interval_count[6] = {0, 0, 0, 0, 0, 0};
void config_interval_motor(int motor, int direction, int Frames, int Seconds) //Configuracion velocidad motor
  {
    closed = false;
    interval_direction[motor] = direction;
    interval_frames[motor] = Frames;
    interval_time[motor] = Seconds*1000;
    frames_count[motor] = 0;
    time_interval_count[motor] = millis();
  }

float inc_position_fade = 0;
int fade_direction = 0;

void config_automatic_fade(int shutterFadeFrames, int shutterSyncWithInterval, int shutterFadeInActive, int shutterFadeOutActive) //Configuracion fade
  {
    inc_position_fade = abs(100-position_motor[0])/(float)shutterFadeFrames;
    if((shutterFadeInActive)&&(!shutterFadeOutActive)) fade_direction=IN;
    else if((!shutterFadeInActive)&&(shutterFadeOutActive)) fade_direction=OUT;
    else if((shutterFadeInActive)&&(shutterFadeOutActive)) fade_direction=IN_OUT;
    else fade_direction=STOP;
  }

void config_optics(int zoomValue, int focusValue, int diaphragmValue, int syncWithIntervalOptics) //Configuracion de las opticas
  {
    config_position_motor(1, zoomValue);
    config_position_motor(2, focusValue);
    config_position_motor(3, map(int(diaphragmValue*10), 19, 220, 0, 100));
  }

void secure_stop(int direction)
  {
    config_speed_motor(4, direction, 0);
    delay(150);
    if (!digitalRead(20))
        {
          config_speed_motor(4, direction, MIN_SPEED);
          closed = true;
        }  
  }

void next_frame(int direction)
  {
    config_speed_motor(4, motorDirection, MIN_SPEED);
    closed = true;
  }

void ini_motors()
  {
    backward(0,255);
    backward(1,255);
    backward(2,255);
    backward(3,255);
    delay(TIME_LINEAL_MAX[3]);
    stop(0);
    stop(1);
    stop(2);
    stop(3);
  }

void refresh_position_motors()
  {
    for(int i=0; i<6; i++)
      {
        if(((millis()-time_refresh[i])>time_move_motor[i])&&(state_pmotor[i]))
          {
            stop(i);
            state_pmotor[i] = false;
          }
      }
  }

void refresh_interval_motors()
  {
    if((((millis()-time_interval_count[4])>=interval_time[4])&&(interval_frames[4]!=frames_count[4]))&&(isAction))
      {
        frames_count[4]++;
        time_interval_count[4] = millis();
        if (shutterSyncWithInterval)
          {
            float value_position=0;
            if(fade_direction==IN)
              {
                value_position = position_motor[0] + inc_position_fade;
                if (value_position>100) value_position=100;
              }
            else if(fade_direction==OUT)
              {
                value_position = position_motor[0] - inc_position_fade;
                if (value_position<0) value_position=0;
              }
            else value_position = position_motor[0];
            config_position_motor(0, value_position);
          }
        next_frame(interval_direction[4]);    
      }
  }

float fps_request = 24;
uint8_t speed_request = 0;
unsigned long time_adjust = millis();

void adjust_fps()
  {
    if(fps_temp>=(fps_request+0.5)) 
      {
        if(speed_request>0) speed_request--;
        analogWrite(EN_MOTOR[4], speed_request);
      }
    else if(fps_temp<(fps_request-0.5)) 
      {
        if(speed_request<255) speed_request++;
        analogWrite(EN_MOTOR[4], speed_request);
      }
    
  }

int fpsRead() {
  int result = fps_read;
  return result;
}

int read_value[10];
int* decode_values(String inputString_rpc, int num_dec)
  {
    String str = inputString_rpc.substring(inputString_rpc.lastIndexOf(":") + 1);
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
unsigned long time_led = millis();

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
          digitalWrite(LED_BUILTIN, LOW);
          if (inputString_rpc.startsWith("/s_motor:")) //Speed Motor
           {
              int* value = decode_values(inputString_rpc, 4);
              #if DEBUG_M4
                RPC.print("Recibido Speed M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<4; i++) RPC.println(value[i]);
              #endif
              motorDirection = value[1];
              motorSpeedValue = value[2];
              motorIsSpeedActive = true;
              motorIsIntervalActive = false;
              if (motorSpeedValue>0) motorSpeedValue = map(motorSpeedValue, 24, 48, 146, 192); //Fake fps
              if (value[3]==false) //Si es falso, se ejecuta, sino se guarda sin ejecutar
                {
                  if (motorSpeedValue==0)
                    {
                      isAction = false;
                      secure_stop(motorDirection);
                    } 
                  else config_speed_motor(value[0], motorDirection, motorSpeedValue);
                }
            }
          else if (inputString_rpc.startsWith("/i_motor:")) //Interval Motor
           {
              int* value = decode_values(inputString_rpc, 5);
              #if DEBUG_M4
                RPC.print("Recibido Interval M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<5; i++) RPC.println(value[i]);
              #endif
              motorDirection = value[1];
              motorIntervalFrames = value[2];
              motorIntervalSeconds = value[3];
              motorIsIntervalActive = true;
              motorIsSpeedActive = false;
              if (value[4]==false) //Si es falso, se ejecuta, sino se guarda sin ejecutar
                {
                  isAction = true;
                  config_interval_motor(value[0], motorDirection, motorIntervalFrames, motorIntervalSeconds);
                }
              
            }
          else if (inputString_rpc.startsWith("/fade:")) //Fade configuration
           {
              int* value = decode_values(inputString_rpc, 6);
              #if DEBUG_M4
                RPC.print("Recibido fade M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<6; i++) RPC.println(value[i]);
              #endif
              shutterFadePercent = value[0];
              shutterFadeFrames = value[1];
              shutterSyncWithInterval = value[2];
              shutterFadeInActive = value[3];
              shutterFadeOutActive = value[4];
              config_position_motor(0, shutterFadePercent);
            }
          else if (inputString_rpc.startsWith("/optics:")) //Optics configuration
           {
              int* value = decode_values(inputString_rpc, 5);
              #if DEBUG_M4
                RPC.print("Recibido optics M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<5; i++) RPC.println(value[i]);
              #endif
              zoomValue = value[0];
              focusValue = value[1];
              diaphragmValue = value[2];
              syncWithIntervalOptics = value[3];
              if (value[4]==false) //Si es falso, se ejecuta, sino se guarda sin ejecutar
                {
                  config_optics(zoomValue, focusValue, diaphragmValue, syncWithIntervalOptics);
                }
            }
          else if (inputString_rpc.startsWith("/action:")) //Optics configuration
           {
              int* value = decode_values(inputString_rpc, 1);
              #if DEBUG_M4
                RPC.print("Recibido action M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<1; i++) RPC.println(value[i]);
              #endif
              isAction=value[0];
              if (isAction)
                {
                  if (motorIsSpeedActive)  config_speed_motor(4, motorDirection, motorSpeedValue);
                  else config_interval_motor(4, motorDirection, motorIntervalFrames, motorIntervalSeconds);
                  config_automatic_fade(shutterFadeFrames, shutterSyncWithInterval, shutterFadeInActive, shutterFadeOutActive);
                }
              else
                {
                  secure_stop(motorDirection);
                }
            }
           inputString_rpc = String();
           digitalWrite(LED_BUILTIN, HIGH);
         }
         else
           inputString_rpc += c_rpc;
       }
  }

void requestReading() {
  while (true) {
    //delay(100);
    RPCRead();
  }
}

char c;
String inputString;
void SerialRead()
  {
     ////////////////////////////////////////////////////////////
     /////// RUTINA TRATAMIENTO DE STRINGS DEL UART   ///////////
     ////////////////////////////////////////////////////////////
     if (command.available())
       {
         c = command.read();
         if ((c == '\r') || (c == '\n'))
         {
          if (inputString.startsWith("/s_motor:")) //Speed Motor
           {
             //Serial.print("Rec: ");
             //Serial.println(inputString);
             String str = inputString.substring(inputString.lastIndexOf(":") + 1);
             //Serial.println(str);
             int value[3];
             value[0] = str.substring(0, str.indexOf(',')).toInt();
             //Serial.println(value[0]);
             for(int i=1; i<3; i++)
                  {
                    str = str.substring(str.indexOf(',')+1);
                    value[i] = str.substring(0, str.indexOf(',')).toInt();
                    //Serial.println(value[i]);
                  }
             if (value[0]==4)
              {
                int value_temp = 0;
                if(value[2]==24) value_temp = 146;
                else if(value[2]==48) value_temp = 192;
                else value_temp = 0;
                //config_smotor(value[0], value[1], value_temp);
                if (value_temp==0) secure_stop(value[1]);
                else config_speed_motor(value[0], value[1], value_temp);
              }
            else config_speed_motor(value[0], value[1], value[2]);
             
              
             //config_angle(value[0], value[1], value[2]);
            }
         else if (inputString.startsWith("/p_motor:")) //Position motor
           {
             //Serial.print("Motor: ");
             String str = inputString.substring(inputString.lastIndexOf(":") + 1);
             //Serial.println(str);
             int value[3];
             value[0] = str.substring(0, str.indexOf(',')).toInt();
             str = str.substring(str.indexOf(',')+1);
             value[1] = str.substring(0, str.indexOf(',')).toInt();
             //Serial.println(value[0]);
             //Serial.println(value[1]);
             config_position_motor(value[0], value[1]);
            }
           inputString = String();
         }
         else
           inputString += c;
       }
  }

void setup() {
  //SCB_CleanDCache();
  RPC.begin();
  //delay(5000);
  pinMode(20, INPUT_PULLUP);
  pinMode(sr_enable, OUTPUT);
  for (size_t i = 4; i < 6; i++)
    {
      pinMode(MA_MOTOR[i], OUTPUT);
      pinMode(MB_MOTOR[i], OUTPUT);
    }
  digitalWrite(sr_enable, LOW);
  sr.setAllLow(); // set all pins LOW
  ini_motors();
  secure_stop(FORWARD);
  attachInterrupt(digitalPinToInterrupt(20), fps_count_state, RISING);

  #if PID_ENABLE
    outMax =  255.0;                    // Límite máximo del controlador PID.
    outMin =  128.0;                   // Límite mínimo del controlador PID.
    
    tmp = 5;                           // Tiempo de muestreo en milisegundos.
    
    kp = 10.0;                          // Constantes PID iniciales. Los valores son los adecuados para un encoder de 334 ppr,
    ki = 3.5;                           // pero como el lector de encoder está diseñado como x4 equivale a uno de 1336 ppr. (ppr = pulsos por revolución.)   
    kd = 30.0;
    
    myPID.SetSampleTime(tmp);           // Envía a la librería el tiempo de muestreo.
    myPID.SetOutputLimits(outMin, outMax);// Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
    myPID.SetTunings(kp, ki, kd);       // Constantes de sintonización.
    myPID.SetMode(AUTOMATIC);           // Habilita el control PID (por defecto).
    Setpoint = 24;        // Para evitar que haga cosas extrañas al inciarse, igualamos los dos valores para que comience estando el motor parado.
  #endif

  //config_smotor(4, 1, 146);
  //delay(5000);

  //Bind the sensorRead() function on the M4
  //RPC.bind("fpsRead", fpsRead);
  /*
  Starts a new thread that loops the requestReading() function
  */
  sensorThread.start(requestReading);

  RPC.println("M4 Inicializado.");
}

void loop() 
{
  #if PID_ENABLE
    /*Input = (double)fps_temp;           // Lectura del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones extrenas (pines 2 y 3).
    if(myPID.Compute())
    analogWrite(EN_MOTOR[4], abs(Output)); // Por el primer pin sale la señal PWM.
    */
  #endif
 refresh_position_motors();
 refresh_interval_motors();
 if(force_stop)
  {
    config_speed_motor(4, !motorDirection, MIN_SPEED);
    delay(20);
    stop(4);
    force_stop = false;
  }
}