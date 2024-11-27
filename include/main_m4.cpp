#include <Arduino.h>
#include <ShiftRegister74HC595.h>
//#include <PID_v1.h> 
//#include <SPI.h>
#include <RPC.h>
#include "constants.h"

#define DEBUG_M4 false
#define MIN_SPEED 55

// Motor
int motorSpeedValue = 24;
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
#define EN_M4 33 
#define R_M4 10 
#define L_M4 9 

const int EN_MOTOR[4] = {11, 3, 5, 6};
const int MA_MOTOR[4] = {M0A, M1A, M2A, M3A};
const int MB_MOTOR[4] = {M0B, M1B, M2B, M3B};

#define sr_enable 7
#define STOP 0
#define CONTINUOUS 1
#define ONE_TURN 2
const long TIME_LINEAL_MAX[6] = {4150, 670, 4250, 6200, 4000, 4000 };  //ms

#define command Serial2


void backward(int motor, int SPEED)
  {
    if(motor<4)
      {
        sr.set(MA_MOTOR[motor], HIGH); 
        sr.set(MB_MOTOR[motor], LOW); 
        analogWrite(EN_MOTOR[motor], SPEED);
      }
    else 
      {
        analogWrite(L_M4, SPEED);
        analogWrite(R_M4, 0);
      }
    
  }
 
void forward(int motor, int SPEED)
  {
    if(motor<4)
      {
        sr.set(MA_MOTOR[motor], LOW); 
        sr.set(MB_MOTOR[motor], HIGH); 
        analogWrite(EN_MOTOR[motor], SPEED);
      }
    else 
      {
        analogWrite(L_M4, 0);
        analogWrite(R_M4, SPEED);
      }
    
  }

void stop(int motor)
  {
    
    if(motor<4)
      {
        analogWrite(EN_MOTOR[motor], 0);
        sr.set(MA_MOTOR[motor], LOW); 
        sr.set(MB_MOTOR[motor], LOW); 
      }
    else 
      {
        analogWrite(L_M4, 0);
        analogWrite(R_M4, 0);
      }
  }

float time_move_for_move_position_motor[6] = {0, 0, 0, 0, 0, 0};
unsigned long time_refresh_position_motor[6] = { millis(), millis(), millis(), millis(), millis(), millis()};
float position_motor[6] = { 0, 0, 0, 0, 0, 0};
float position_motor_objective[6] = { 0, 0, 0, 0, 0, 0};
bool moving_position_motor[6] = { false, false, false, false, false, false};

void config_position_motor(int motor, float position) //Configuracion posicion en % motor
  {
    if((position!=position_motor[motor])&&(!moving_position_motor[motor]))
      {
        time_move_for_move_position_motor[motor] = (abs(position_motor[motor]-position)*TIME_LINEAL_MAX[motor])/100.;
        if(position>position_motor[motor]) forward(motor,255);
        else backward(motor,255);
        time_refresh_position_motor[motor] = millis();
        position_motor_objective[motor] = position;
        moving_position_motor[motor] = true;
        //RPC.println("/debug:" + String(position_motor[motor]) + "," + String(position) + "," + String(millis()-time_refresh_position_motor[motor]) + "," + String(time_move_for_move_position_motor[motor]) + "," + "INI");
        //Serial.println(time_move_for_move_position_motor[motor]);
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
  if((digitalRead(20)&&(micros()-lastTimeDebounce>= 100))&&(!closed))
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
void config_interval_motor(int direction, int Frames, int Seconds) //Configuracion velocidad motor
  {
    closed = false;
    interval_direction[FPS_MOTOR] = direction;
    interval_frames[FPS_MOTOR] = Frames;
    interval_time[FPS_MOTOR] = Seconds*1000;
    frames_count[FPS_MOTOR] = 0;
    time_interval_count[FPS_MOTOR] = millis();
  }

float inc_position_fade = 0;
int fade_direction = 0;
void config_automatic_fade(int shutterFadeFrames, int shutterSyncWithInterval, int shutterFadeInActive, int shutterFadeOutActive) //Configuracion fade
  {
    
    if((shutterFadeInActive)&&(!shutterFadeOutActive))
      {
        fade_direction=IN;
        inc_position_fade = (100-position_motor[0])/(float)shutterFadeFrames;
      } 
    else if((!shutterFadeInActive)&&(shutterFadeOutActive))
      {
        fade_direction=OUT;
        inc_position_fade = (position_motor[0])/(float)shutterFadeFrames;
      } 
    else if((shutterFadeInActive)&&(shutterFadeOutActive)) 
      {
        fade_direction=IN_OUT;
        inc_position_fade = (100)/(float)shutterFadeFrames;
      }
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
    config_speed_motor(FPS_MOTOR, direction, 0);
    delay(150);
    if (!digitalRead(20))
        {
          config_speed_motor(FPS_MOTOR, direction, MIN_SPEED);
          closed = true;
        }  
  }

void next_frame(int direction)
  {
    config_speed_motor(FPS_MOTOR, motorDirection, MIN_SPEED);
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
    for(int i=0; i<4; i++)
      {
        if(((millis()-time_refresh_position_motor[i])>(unsigned long)time_move_for_move_position_motor[i])&&(moving_position_motor[i]))
          {
            stop(i);
            position_motor[i] = position_motor_objective[i];
            moving_position_motor[i] = false;
            //RPC.println("/debug:" + String(position_motor[i]) + "," + String(position_motor_objective[i]) + "," + String(millis()-time_refresh_position_motor[i]) + "," + String(time_move_for_move_position_motor[i]) + "," + "END");
          }
      }
  }

int change_direction_inout = 1;

void refresh_interval_motors()
  {
    if((((millis()-time_interval_count[FPS_MOTOR])>=interval_time[FPS_MOTOR])&&(interval_frames[FPS_MOTOR]!=frames_count[FPS_MOTOR]))&&(isAction))
      {
        frames_count[FPS_MOTOR]++;
        time_interval_count[FPS_MOTOR] = millis();
        if(frames_count[FPS_MOTOR]>=interval_frames[FPS_MOTOR]) isAction=false;
        if (shutterSyncWithInterval)
          {
            float value_position=0;
            if(fade_direction==IN)
              {
                value_position = position_motor[0] + inc_position_fade;
                if (value_position>100) value_position=100;
                config_position_motor(0, value_position);
              }
            else if(fade_direction==OUT)
              {
                value_position = position_motor[0] - inc_position_fade;
                if (value_position<0) value_position=0;
                config_position_motor(0, value_position);
              }
            else if(fade_direction==IN_OUT)
              {
                if (((position_motor[0] + inc_position_fade)>100)&&(change_direction_inout==1)) change_direction_inout=-1;
                else if (((position_motor[0] - inc_position_fade)<0)&&(change_direction_inout==-1)) change_direction_inout=1;
                value_position = position_motor[0] + change_direction_inout*inc_position_fade;
                RPC.println("/fade:" + String(position_motor[0]) + "," + String(change_direction_inout) + "," + String(inc_position_fade) + "," + String(0) + "," + String(shutterFadeOutActive));
                config_position_motor(0, value_position);
              }
            //else value_position = position_motor[0];
            
          }
        next_frame(interval_direction[FPS_MOTOR]);    
      }
  }

float fps_request = 24;
uint8_t speed_request = 0;
unsigned long time_adjust = millis();

/*
Functions on the M4 that returns FPS
*/
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
              //if (motorSpeedValue>0) motorSpeedValue = map(motorSpeedValue, 24, 48, 146, 192); //Fake fps
              if (motorSpeedValue>0) motorSpeedValue = map(motorSpeedValue, 0, 48, 0, 255); //Fake fps
              //if (motorSpeedValue>0) motorSpeedValue = 255;
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
                  config_interval_motor(motorDirection, motorIntervalFrames, motorIntervalSeconds);
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
              //RPC.println("/fade:" + String(shutterFadePercent) + "," + String(shutterFadeFrames) + "," + String(shutterSyncWithInterval) + "," + String(shutterFadeInActive) + "," + String(shutterFadeOutActive));
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
                  else config_interval_motor(motorDirection, motorIntervalFrames, motorIntervalSeconds);
                  config_automatic_fade(shutterFadeFrames, shutterSyncWithInterval, shutterFadeInActive, shutterFadeOutActive);
                }
              else
                {
                  secure_stop(motorDirection);
                }
            }
          else if (inputString_rpc.startsWith("/sensors:")) //Optics configuration
           {
              int* value = decode_values(inputString_rpc, 1);
              #if DEBUG_M4
                RPC.print("Recibido fps M4: ");
                RPC.println(inputString_rpc);
                for(int i=0; i<1; i++) RPC.println(value[i]);
              #endif
              if(value[0])
                RPC.println("/sensors:" + String(fps_read) + "," + String(position_motor[0]) + "," + String(position_motor[1]) + "," + String(position_motor[2]) + "," + String(position_motor[3]));
              //RPC.println(fps_read);
            }
           inputString_rpc = String();
           digitalWrite(LED_BUILTIN, HIGH);
         }
         else
           inputString_rpc += c_rpc;
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
  pinMode(EN_M4, OUTPUT);
  digitalWrite(EN_M4, HIGH);
  analogWriteResolution(8);
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

  //config_smotor(FPS_MOTOR, 1, 146);
  //delay(5000);

  //Bind the sensorRead() function on the M4
  //RPC.bind("FPS", fpsRead);
  /*
  Starts a new thread that loops the requestReading() function
  */
  //sensorThread.start(requestReading);
  #if DEBUG_M4
    RPC.println("M4 Inicializado.");
  #endif
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
 RPCRead();
 if(force_stop)
  {
    //config_speed_motor(FPS_MOTOR, !motorDirection, MIN_SPEED);
    //delay(20);
    stop(FPS_MOTOR);
    force_stop = false;
  }
}