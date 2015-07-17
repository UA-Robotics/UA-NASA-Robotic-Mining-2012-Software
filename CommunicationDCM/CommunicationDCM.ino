//Data Colection Modual (DCM) code!
//Made By: Richard Johnson
//Fixed By: Duncan Campbell
//V1.0

#include <Wire.h>
#include <digitalWriteFast.h>
#include <EasyTransferI2C.h>

//I2C control stuff
struct DCM_PACKET
{
  float robotvoltage;
  float robotcurrent;
  float comvoltage;
  float comcurrent;
  int angle;
  int distance[5];
};

EasyTransferI2C MCU;

DCM_PACKET dcmData;

//MUX Control Lines
#define M1 2
#define M2 3
#define M3 4
#define M4 5

//sensors 
#define CS A3 //Current Sense
#define ESTOP A2 //Estop input and mesure battery voltage
#define VIN A1 //Comm. Board voltage in
#define MUXIN A0 // MUX input

//ect..
#define LED 13 //Alive LED for MP
#define RS485_S 8//Selection Pin for RS485
#define WD 9 //Watch Dog Timmer Trip Pin

int c = 0; //LIDAR Data updat interval
int i = 0;
int X = 0;
float volts;
int distance1;
int distance2;
int Deg;
int temp;
int comcurrent;
int robotcurrent;
int robotvolt;
int comvolt;

void setup()
{
  //Serial.begin(19200); //Lidar RS485 input
  
  Wire.begin();  //Join I2C bus as address 2 
  MCU.begin(details(dcmData), &Wire);
  
  
  DDRD = DDRD | B111100;
  pinModeFast(M1, OUTPUT);
  pinModeFast(M2, OUTPUT);
  pinModeFast(M3, OUTPUT);
  pinModeFast(M4, OUTPUT);
  pinModeFast(LED, OUTPUT);
  pinModeFast(RS485_S, OUTPUT);
  digitalWriteFast(LED, HIGH); //I am Alive!!!
}

inline void writeData()
{
  static long last = 0;
  static long curr = 0;
  
  curr = millis();
  if(curr - last > 20)
  {
    MCU.sendData(2);
  }
} 

inline void blinky()
{
  static int stat = LOW;
  static unsigned long last = 0;
  static unsigned long curr = 0;
  
  curr = millis();
  if(curr - last > 1000)
  {
    last = curr;
    stat = !stat;
    digitalWriteFast(13, stat);
  }
}

void loop(){
 blinky();
 writeData();
 
 /*
 if( c >= 2000)
 { //update LIDAR data
     digitalWriteFast(RS485_S, HIGH);
     Serial.print('a');
     digitalWriteFast(RS485_S,LOW);
     
     digitalWriteFast(RS485_S, HIGH);
     Serial.print('b');
     digitalWriteFast(RS485_S,LOW);
     c = 0;
 }
 */
    
   if( X >= 10 )
   {
     temp = analogRead(VIN);        //Com. Board Current 
     dcmData.comvoltage = temp / 34; //Com. Board Current 
     dcmData.comcurrent = dcmData.comvoltage / 2; //Voltage comeing in com board
     temp = analogRead(ESTOP);   //Voltage on robot
     dcmData.robotvoltage = temp / 34; //Voltage on robot
     temp = readMux(2) - 512;
     dcmData.robotcurrent = (temp / 102.0) * 200;
     volts = ((readMux(3) / 1023.0) * 5.0) - .01;
     dcmData.angle = volts * (90.0/3.0);
     
     dcmData.distance[0] = readMux(0);
     dcmData.distance[1] = readMux(1);
     dcmData.distance[2] = readMux(4);
     dcmData.distance[3] = readMux(5);
     dcmData.distance[4] = readMux(6);
     
     X = 0;
   }
   c++;
   X++;
}

int readMux(int channel){
  int controlPin[] = {M1, M2, M3, M4};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  //loop through the 4 sig
  for(int r = 0; r < 4; r ++){
    digitalWriteFast(controlPin[r], muxChannel[channel][r]);
  }

  //read the value at the SIG pin
  temp = analogRead(MUXIN);

  //return the value
  return temp;
}
