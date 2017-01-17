
/*
 Nombre    :  Firm_v1_Pedeler
 Basado en :  VescUartSample.ino
 Creado en :  Diciembre-Enero de 2016
 Author    :	Renzo Varela, Esteban Riquelme
*/

//COMUNICACION BLE.............................   FUNCIONA OK. COPLETADO
//COMUNICACION VESC............................   FUNCIONA OK. FALTA PODER SETEAR PARAMETROS DEL MOTOR/CONTROLADOR
//RPM PEDAL....................................   FUNCIONA OK. FALTA TRABAJAR CON EL UMBRAL DE AYUDA Y EL MODELO MAT.
//RPM RUEDA....................................   NO FUNCIONA.
//ANGULO DE INCLINACION........................   

//Bibliotecas ara VESC
#include "VescUart.h"
#include "datatypes.h"

//Bibliotecas para el calculo de la pendiente
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 accelgyro;

//Biblioteca para controlar el servomotor por el puerto de PWM
#include <Servo.h>
Servo myservo;

//Biblioteca que permite usar (casi) cualquier PIN como puerto UART
#include <SoftwareSerial.h>
SoftwareSerial ble(6,7); // RX, TX

//Estructura con los datos medidos desde VESC
struct bldcMeasure measuredValues;

//  VARIABLES GLOBALES
//------------------------------------------------------------------------------------------------------

//variables para recepcion de mensajes
static char INmsn[5];
char c;
static boolean lectura=false;
static char IntData[2];

//Variables para generacion de mensaje
int NivelBateria = 23.92*100;
int VelBici = 10; // kmh
int Pendiente = 50; //en grados
int Potencia =96; //potencia
int VelTan1 = 40; //  dad tangencial motor
int VelTan2 =30; //velocidad tangencial rueda
int DistAprox = 10;
char Modo = 1; //modo actual
int Regeneracion = 1;//regeneracion
int Ayuda = 1;//ayuda pedaleo
float RadioMotor = 2.00;
float RadioRueda = 2.00;


// Variables para el conteo de RPM
volatile byte rpmcount1 = 0;
volatile byte rpmcount2 = 0;
unsigned int rpm1 = 0; // valor unsigned para rpm
unsigned int rpm2 = 0;
unsigned long timeold1 = 0; // valor unsigned para el timehold o capturador de tiempo
unsigned long timeold2 = 0;
float val2;
float R = 0.3; // Radio de la rueda es de 30 cms = 0.3 mts = 0.03 km
float V;      // V de Velocidad lineal, trasnformación de velocidad angular

//Variables para el calculo de la pendiente
int16_t ax, ay, az, gx, gy, gz;
double ary;
double ary_real;
double ary_inicial=0;

//Definicion de Umbrales
float umbral_velocidad = 50; //la velocidad esta en km/h
float umbral_potencia = 100;
float umbral_pendiente;
float umbral_RPM = 10; // 0.055 
float umbral_RPM_cerro, umbral_RPM_plan;

//Otras variables
float partida = 0;
int giro;

//FUNCIONES
//------------------------------------------------------------------------------------------------------

//Funcion que recibe un mensaje por el modulo BLE
boolean Recepcion_de_bluetooth()
{
  if(ble.available())
  {
    c = ble.read();
    if(c=='#'){lectura=true;}
    if(lectura){INmsn[strlen(INmsn)]=c;}
      
  }
  if(c=='$'&& lectura)
  {
    int inputcount = 0; //Contador buffer.
    
    for (int i = 0; i < strlen(INmsn); i++) 
    { //Separa el mensaje en multiples bytes.
      if (INmsn[i] == ',' || INmsn[i] == '$') 
      { //Separador encontrado.
        IntData[inputcount]=INmsn[i-1]; //Transforma el string y lo almacena en buffer.
        inputcount++;
      }
    }
    
    memset(INmsn,0,sizeof(INmsn)); //Borra el buffer de lectura.
    lectura=false;
    return true;
  }
  return false;
}

//Funcion que refresca datos
void RefrescaDatos(){
  NivelBateria = 600;    //measuredValues.inpVoltage*100;
  VelBici = 10;          //rescatado del sensor
  Pendiente = 0;        //rescatado del sensor      // Pendiente = getPendiente();
  Potencia = 90;        //int(measuredValues.avgMotorCurrent)*NivelBateria; // medida en watts
  VelTan1 =  30;           //(2*3.1416*RadioMotor*measuredValues.rpm)/60;  
  VelTan2 =  46;          //(2*3.1416*RadioRueda*measuredValues.rpm)/60;
  DistAprox = 10;
  
  if (Pendiente > umbral_pendiente){
    //umbral_RPM = umbral_RPM_cerro;
    //partida = 40;
    }
  else{
    //umbral_RPM = umbral_RPM_plan;
    //partida = 0;
    }
  }

//Funcion que crea el mensaje
String CreaMensaje(){
    String inicio_mensaje= "#";
    String ND = "FF";
    String FV = "1";
    String NB = String(NivelBateria, HEX);
    String VB = String(VelBici, HEX);
    String PN = String(Pendiente, HEX);
    String WT = String(Potencia, HEX);
    String VT1 = String(VelTan1, HEX);
    String VT2 = String(VelTan2, HEX);
    String DA = String(DistAprox, HEX);
    String MD = String(Modo, HEX);
    String RG = String(Regeneracion,HEX);
    String AP = String(Ayuda, HEX);    

    String Mensaje = inicio_mensaje+ND+','+FV+','+NB+','+VB+','+PN+','+WT+','+VT1+','+VT2+','+DA+','+MD+','+RG+','+AP+',';

    String finalmensaje= getCheckSum(Mensaje)+'$';
    Mensaje = Mensaje + finalmensaje;
    
    return Mensaje;
    }

/*
//Funcion que crea el mensaje con los datos en decimal
String CreaMensaje(){
    String inicio_mensaje= "#";
    String NB = String(NivelBateria, DEC);    //Nivel Bateria
    String VB = String(VelBici, DEC);
    String PN = String(Pendiente, DEC);
    String WT = String(Potencia, DEC);
    String VT1 = String(VelTan1, DEC);        //Velocidad tangencial de la rueda
    String VT2 = String(VelTan2, DEC);        //Velocidad tangenccial del pedal    

    String Mensaje = inicio_mensaje+NB+','+VB+','+PN+','+WT+','+VT1+','+VT2+',';

    String finalmensaje ='$';
    Mensaje = Mensaje + finalmensaje;
    
    return Mensaje;
    }
*/

// Algoritmo CheckSum8 Xor devuelve un string con el valor HEX del checksum
// comprobar con http://www.scadacore.com/field-applications/programming-calculators/online-checksum-calculator/
String getCheckSum(String string) {
  int i;
  int XOR;
  int c;
  // Calculate checksum ignoring any #'s in the string
  for (XOR = 0, i = 0; i < string.length() ; i++) {
  c = (unsigned char)string[i];
  //if (c == '$') break;
  if (c != '#') XOR ^= c;
  }
  String checksum = String(XOR, HEX);
  return checksum;
}

//Interrupcion que cuenta RPM del pedaleo
void rpm_fun1()
{
   rpmcount1++;   //Each rotation, this interrupt function is run twice
}

//Interrupcion que cuenta RPM del pedaleo
void rpm_fun2()
{
   rpmcount2++;   //Each rotation, this interrupt function is run twice
}


void anguloInicial(){
    //*******Se puede complementar calculando varias veces el valor y promediandolo
    //*******Se tiene que setear con mucho cuidado el 0
    delay(5000);
    //Se obtienen los datos actuales
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // calculate accelerometer angles
    ary_inicial = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
}

void getPendiente(){    
  //Se obtienen los datos actuales
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // calculate accelerometer angles
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));

  ary_real = sqrt(square(ary/ary_inicial)) ;
}

void setup() {
	
	//Puerto UART de comunicacion con VESC
	Serial.begin(9600);
  
  //Puerto UART de comunicacion con BLE
  ble.begin(9600);

  //Se determina el pin que entrega la señal PWM
  myservo.attach(6); 

  //Declaracion de interrupcion para conteo de RPM de la bicicleta
  attachInterrupt(digitalPinToInterrupt(2), rpm_fun1, RISING);

  //Declaracion de interrupcion para conteo de RPM del pedaleo
  attachInterrupt(digitalPinToInterrupt(3), rpm_fun2, RISING);

  // LED para DEBUG
  pinMode(LED_BUILTIN, OUTPUT);

  //Inicio del gyroscopio
  Wire.begin();
  accelgyro.initialize();
  anguloInicial();
}
	
void loop() {
  
  //1. RECIVE MENSAJE
  if(Recepcion_de_bluetooth())
  {
    if (IntData[0]=='A')
    {
      if(IntData[1]=='0')
      {
        ble.println("ayuda pedaleo off");
        Ayuda=0;
      }
      if(IntData[1]=='1')
      {
        ble.println("ayuda pedaleo plano");
        Ayuda=1;
      }
      if(IntData[1]=='2')
      {
        ble.println("ayuda pedaleo plano y subida");
        Ayuda=2;
      }
    }
    if (IntData[0]=='M')
    {
      if(IntData[1]=='E')
      {
        ble.println("Modo ECOLOGICO");
        Modo=0;                         //segun protocolo significa ecologico
        umbral_velocidad = 20;
        umbral_potencia = 30;
      }
      if(IntData[1]=='N')
      {
        ble.println("Modo NORMAL");
        Modo=1;                         //segun protocolo significa 
        umbral_velocidad = 25;
        umbral_potencia = 40;
      }
      if(IntData[1]=='T')
      {
        ble.println("Modo TURBO");
        Modo=2;                         //segun protocolo significa 
        umbral_velocidad = 50;
        umbral_potencia = 50;
      }
    }
    if (IntData[0]=='R')
    {
      if(IntData[1]=='0')
      {
        ble.println("Regeneracion OFF");
        Regeneracion=0;
        //regOff();
      }
      if(IntData[1]=='1')
      {
        ble.println("Regeneracion ON");
        Regeneracion=1;
        //regOn;
      }
    }
    memset(IntData,0,sizeof(IntData));
  }

  //2. ENVIA MENSAJE
	if(1){
	//if (VescUartGetValue(measuredValues)) {
    RefrescaDatos();
    String MensajeEenviado = CreaMensaje();
    ble.println(MensajeEenviado);
	}
	else
	{
		ble.println("Failed to get data!");
	}


  //3. EJECUTA ACCION
  if(Ayuda == 1 || Ayuda == 2 ){                                               // se activa ayuda si es que el usuario lo permite desde la app
  if(VelBici < umbral_velocidad || Potencia < umbral_potencia){ // si se pasa de estos umbrales la ayuda se detiene.  Hay que evaluar si esto se cambia por un while

     if (rpmcount2 >= 6){
       rpm2 = 30*1000/(millis() - timeold2)*rpmcount2; // formula para el conteo de interrupciones y calculo de RPM
       timeold2 = millis(); // timehold igual a los millis instrucción de interrupción para tiempo en milisegundos
       rpmcount2 = 0; // rpm counter se iguala a valor 0
       val2 = rpm2,DEC;            // lectura del valor de rpm en decimales (entre los valores 0 a 40000) 
       //val2 = map(val2, 0, 400, partida, 179);     // mapeo de escalado para usarlo con el servo (valores entre 0 y 180 grados) 
       //val2 = val2*0.00195+0.22;
       val2 = val2*0.0055;
       if(val2 >= 0.055){
          digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
          VescUartSetDuty(0.94);
          delay(500);   //Delay para dar tiempo a que el pedal de otra vuelta  
        }
        else{
          digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level);
          //myservo.write(0);
          //VescUartSetDuty(0.005);
          //delay(400);   //Delay para dar tiempo a que el pedal de otra vuelta  
          }
        delay(500);     //Delay para dar tiempo a que el pedal de otra vuelta  
     }
     else{
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level);
      //myservo.write(0);
      //VescUartSetDuty(0.005);
      delay(500);   //Delay para dar tiempo a que el pedal de otra vuelta  
     }

       
    }
  }

}
