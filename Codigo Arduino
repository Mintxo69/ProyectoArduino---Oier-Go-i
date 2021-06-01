/* PROYECTO FIN DE CURSO: DISPENSADOR DE LACASITOS POR COLORES
 * 
 * Hecho por Oier Goñi
 * Modulo: Sistemas de medida y regulación
 */


//Librerias:
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>      
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>

//Marcas para contadores, loops etc.
int veces=0;
int Marca=0;
int segundos=3;
int num_alarm=3;

//Pines Led RGB:
const int RGB_R=5;
const int RGB_G=6;
const int RGB_B=9; 

//Pin Zumbador:
int buzzer=10;

//Pin Sensor IR:
int SENSOR = 11;

//Colores:
int rosa=0;
int rojo=0;
int azul=0;
int amarillo=0;
int verde=0;
int negro=0;
int total=0;


//Declaro servos:
Servo servoArriba;
Servo servoAbajo;

//Define la largura y las medidas de la pantalla LCD
LiquidCrystal_I2C lcd(0x27,16,2);

/* Inicia con los valores predeterminados (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600); // Activar el monitor serial
  IrReceiver.begin(SENSOR, DISABLE_LED_FEEDBACK);     // inicializa recepcion de datos del sensor IR
  pinMode (buzzer,OUTPUT);  //Define el pin del zumbador que va a ser de salida
  lcd.init ();                      //Inicio la pantalla LCD
  lcd.backlight ();                 //Inicio la luz de la pantalla LCD

//Inicia el programa con el Led apagado
analogWrite (RGB_R,0);
analogWrite (RGB_G,0);
analogWrite (RGB_B,0);

//Si el sensor de color no se inicia da un error que lo escribe en la pantalla lcd y el led se pone en rojo
  if (!tcs.begin()) 
  {
    lcd.clear();
    lcd.print("Error al iniciar");
    lcd.setCursor (0,1);
    lcd.print (" TCS34725");
    while (1) delay(1000);
    analogWrite (RGB_R,255);
    analogWrite (RGB_G,0);
    analogWrite (RGB_B,0);
  }
  
  // Definimos los servos a sus respectivos pines en el arduino
  servoArriba.attach(3);
  servoAbajo.attach(2);

  // Inicializamos los angulos con los que iniciaran los dos servos
  servoArriba.write(80);
  servoAbajo.write(0);
}

void loop()
{
  if (IrReceiver.decode()) {          // si existen datos ya decodificados
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);  // imprime valor en hexadecimal en monitor
    IrReceiver.resume();              // resume la adquisicion de datos

if (veces==0)  // Si es la primera vez que se conecta saldra el menu principal, o tambien una vez haya acabado la parte de recargando
{
  lcd.clear ();
  lcd.print ("1-Start2-colores");
  lcd.setCursor (0,1);
  lcd.println ("3-Calibracion   ");
}

 if (veces>0)  //Si ha acabado el ciclo y habia lacasitos que salga un tiempo de racarga
{
  lcd_cargando ();
  alarma();
  veces=0;
}

if (IrReceiver.decodedIRData.decodedRawData==0xE718FF00) //Al pulsar el boton 2 del mando
  {
    contajes();
  }

if (IrReceiver.decodedIRData.decodedRawData==0xA15EFF00)  //Al pulsar el boton 3 del mando
  {
      alarma();
      lcd.clear();
      lcd.println ("   Calibrando...");
      servoAbajo.write(158);
      delay(500);
      servoAbajo.write(120);
      delay(500);
      servoAbajo.write(78);
      delay(500);
      servoAbajo.write(40);
      delay(500);
      servoAbajo.write(0);
      delay(500);
  }

  //Si esta parado en el menu y esta preparado para iniciar el led estara en verde
  analogWrite (RGB_R,0);
  analogWrite (RGB_G,255);
  analogWrite (RGB_B,0);
  if (IrReceiver.decodedIRData.decodedRawData==0xF30CFF00)  //Si pulsa el boton 1 del mando empieza la secuencia
  {
    alarma(); //Llama al void alarma para sonar el zumbador
    
    //Reset contajes:
    azul=0;
    amarillo=0;
    verde=0;
    rosa=0;
    rojo=0;
    negro=0;
    total=0;

    //Apaga el led
    analogWrite (RGB_R,0);
    analogWrite (RGB_G,0);
    analogWrite (RGB_B,0);

  //Apaga el lcd porque si no parpadea por falta de tensió  
  lcd.noBacklight ();
  lcd.clear();
  lcd.print ("  TRABAJANDO...");
  
  uint16_t r, g, b, c, colorTemp, lux;    //Define los valores del sensor de color

 //Suena un ultimo momento el zumbador
 digitalWrite (buzzer, HIGH);
 delay(200);
 digitalWrite (buzzer, LOW);
 
  while (negro<3)   //Mientras el sensor no detecte el color negro 3 veces sigue haciendo el loop
  {
    agita();  //Llama al void agitar que es para que caigan los lacasitos si se quedan atascados
    servoArriba.write(0);
    delay(100);
    servoArriba.write(90);    //El angulolo 90 es donde esta el sensor de color
    delay(500);

  //Coje los valores del sensor de colores
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  //Escribe los valores del sensor en el monitor serial
  Serial.print("Temperatura color: "); Serial.print(colorTemp, DEC); Serial.println(" K");
  Serial.print("Lux : "); Serial.println(lux, DEC);
  Serial.print("Rojo: "); Serial.println(r, DEC);
  Serial.print("Verde: "); Serial.println(g, DEC);
  Serial.print("Azul: "); Serial.println(b, DEC);
  Serial.print("Clear: "); Serial.println(c, DEC);
  Serial.println(" ");
  delay(100);


  //Si los valores del sensor estan dentro de estos rangos el color es rosa
  if (lux>=0 && lux<=65600 && r>=160 && r<=221 && g>=55 && g<=105 && b>=50 && b<=100 && c>=270 && c<=445)
  {
  Serial.println("Rosa detectado!!!");
  servoAbajo.write(120);    //Se mueve al angulo 20 el servo de abajo
  delay(250);
  rosa=rosa+1;    //Suma 1 al valor del color
  analogWrite (RGB_R,255);
  analogWrite (RGB_G,0);
  analogWrite (RGB_B,255);
  negro=0;
  }


  //Si los valores del sensor estan dentro de estos rangos el color es azul
  if (lux>=20 && lux<=60 && r>=85 && r<=135 && g>=60 && g<=115 && b>=50 && b<=105 && c>=205 && c<=370)
  {
  Serial.println("Azul detectado!!!");
  servoAbajo.write(78);   //Se mueve al angulo 78 el servo de abajo
  delay(250);
  azul=azul+1;    //Suma 1 al valor del color
  analogWrite (RGB_R,0);
  analogWrite (RGB_G,0);
  analogWrite (RGB_B,255);
  negro=0;
  }


  //Si los valores del sensor estan dentro de estos rangos el color es rojo
  if (lux>=65500 && lux<=65530 && r>=110 && r<=170 && g>=15 && g<=35 && b>=15 && b<=29 && c>=135 && c<=225)
  {
  Serial.println("Rojo detectado!!!");
  servoAbajo.write(35);   //Se mueve al angulo 35 el servo de abajo
  delay(250);
  rojo=rojo+1;    //Suma 1 al valor del color
  analogWrite (RGB_R,255);
  analogWrite (RGB_G,0);
  analogWrite (RGB_B,0);
  negro=0;
  }


  //Si los valores del sensor estan dentro de estos rangos el color es verde
  if (lux>=35 && lux<=80 && r>=115 && r<=160 && g>=50 && g<=110 && b>=20 && b<=45 && c>=200 && c<=320)
  {
  Serial.println("Verde detectado!!!");
  servoAbajo.write(0);    //Se mueve al angulo 0 el servo de abajo
  delay(250);
  verde=verde+1;    //Suma 1 al valor del color
  analogWrite (RGB_R,0);
  analogWrite (RGB_G,255);
  analogWrite (RGB_B,0);
  negro=0;
  }


  //Si los valores del sensor estan dentro de estos rangos el color es amarillo
  if (lux>=60 && lux<=140 && r>=140 && r<=310 && g>=100 && g<=180 && b>=35 && b<=60 && c>=360 && c<=600)
  {
  Serial.println("Amarillo detectado!!!");
  servoAbajo.write(158);  //Se mueve al angulo 158 el servo de abajo
  delay(250);
  amarillo=amarillo+1;    //Suma 1 al valor del color
  analogWrite (RGB_R,141);
  analogWrite (RGB_G,73);
  analogWrite (RGB_B,37);
  negro=0;
  }


  //Si los valores del sensor estan dentro de estos rangos el color es negro
  if (lux<=20 && r<=20 && g<=20 && b<=20 && c<=20)
  {
    negro=negro+1;    //Suma 1 al valor del color
  }
  
  //El servo baja a 180 y cae el lacasito
    servoArriba.write(180);
    delay(500);
  }

  //Suma todos los valores una vez acabado el loop para enseñar el total
  total=amarillo+verde+rojo+azul+rosa;

  alarma();
  //Estos valores los escribe en el monitor serie
  Serial.print("Rojos: "); Serial.println(rojo); Serial.print("Rosa: "); Serial.println(rosa); Serial.print("amarillos: "); Serial.println(amarillo);
  Serial.print("Azules: "); Serial.println(azul); Serial.print("Verdes: "); Serial.println(verde); Serial.print("Total: "); Serial.println(total);
  contajes();   //Llama al void contajes para que lea los contajes de cada color en el lcd

/*Si se ha hecho el ciclo y no hay ningun lacasito, no activo esta marca para que no haga la opcion recargando ya que como no habia no se puede recargar,
* y si hay lacasitos activo la marca veces para que si me haga la opcion de cargando.
*/
if (total!=0)
{
veces=veces+1;
}

}
segundos=10;    //Pongo segundos igual a 10 para rearmar los segundos para el siguiente recargando
}
}


//El void agita lo que hace es mover el servo de arriba que esta conectado a un engranaje que mueve un palo conectado al deposito para que caigan los lacasitos
void agita()
{
    servoArriba.write(0);
    delay(55);
    servoArriba.write(60);
    delay(55);
    servoArriba.write(0);
    delay(55);
    servoArriba.write(60);
    delay(55);
    servoArriba.write(0);
    delay(55);
    servoArriba.write(60);
    delay(55);
    servoArriba.write(0);
    delay(55);
}


//Este void es el encargado de leer los contajes de los colores y enseñarlos en la pantalla lcd
void contajes ()
{
    lcd.backlight();
    Marca=0;  //reseteo la marca por si es el segundo ciclo en adelante
    if (total!=0)   //Si no habia lacasitos que no lea los valores ya que son todo zeros
    {
    while (Marca<2)   //Repite dos veces los contajes en el LCD
    {
      lcd.clear ();                //Que se borre todo lo que haya en la pantalla
      lcd.setCursor (0,0);         //Que escriba en la fila de arriba del LCD
      lcd.print ("Rojos: ");
      lcd.print (rojo);
      lcd.setCursor (0,1);        //Que escriba en la fila de abajo del LCD
      lcd.print ("Rosas: ");
      lcd.print (rosa);
      analogWrite (RGB_R,255);
      analogWrite (RGB_G,0);
      analogWrite (RGB_B,0);
      alarma();
      delay(2000);
      analogWrite (RGB_R,255);
      analogWrite (RGB_G,0);
      analogWrite (RGB_B,255);
      delay(2000);
      lcd.setCursor (0,0);
      lcd.print ("Verde: ");
      lcd.print (verde);
      lcd.setCursor (0,1);
      lcd.print ("Azules: ");
      lcd.print (azul);
      analogWrite (RGB_R,141);
      analogWrite (RGB_G,73);
      analogWrite (RGB_B,37);
      delay(2000);
      analogWrite (RGB_R,0);
      analogWrite (RGB_G,0);
      analogWrite (RGB_B,255);
      delay(2000);
      lcd.clear ();
      lcd.setCursor (0,0);
      lcd.print ("Amarillos: ");
      lcd.print (amarillo);
      lcd.setCursor (0,1);
      lcd.print ("Total: ");
      lcd.print (total);
      analogWrite (RGB_R,255);
      analogWrite (RGB_G,255);
      analogWrite (RGB_B,1);
      delay(4000);
      Marca=Marca+1;
    }
  }
  if (total==0)   //Si no habia lacasitos que escriba no hay datos
  {
    lcd.clear ();
      lcd.setCursor (0,0);
      lcd.print ("  No hay datos");
      lcd.setCursor (0,1);
      lcd.print ("    todavia   ");
      alarma();
      delay (3000);
  }
}


//Este void es para que haga la cuenta atras de recargando en el lcd
void lcd_cargando ()
{
  while (segundos!=0)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Recargando...    ");
    lcd.setCursor(0,1);
    lcd.print(segundos);
    lcd.print(" segundos");
    segundos=segundos-1;
    cargando();
  }
  lcd.clear();
}

//Este void hace sonar el buzzer 3  veces
void alarma()
{
  digitalWrite (buzzer, HIGH);
  delay(300);
  digitalWrite (buzzer, LOW);
  delay(200);
  digitalWrite (buzzer, HIGH);
  delay(300);
  digitalWrite (buzzer, LOW);
  delay(200);
  digitalWrite (buzzer, HIGH);
  delay(300);
  digitalWrite (buzzer, LOW);
}


//Este void lo utilizo en la recarga para que el led parpadee en amarillo
void cargando ()
{
  int carga=0;
  while (carga<3)
  {
    analogWrite (RGB_R,255);
    analogWrite (RGB_G,255);
    analogWrite (RGB_B,1);
    digitalWrite (buzzer, HIGH);
    delay (200);
    digitalWrite (buzzer, LOW);
    analogWrite (RGB_R,0);
    analogWrite (RGB_G,0);
    analogWrite (RGB_B,0);
    delay (200);
    carga=carga+1;
}
}
