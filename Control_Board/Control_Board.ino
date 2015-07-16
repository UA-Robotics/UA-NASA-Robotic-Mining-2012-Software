//main board code to run
//created by the Duncan Campbell!!!!
//Version 7

#include <EasyTransfer.h>
#include <EasyTransferI2C.h>
#include <digitalWriteFast.h>
#include <Wire.h>

//printer code
#include "SoftwareSerial.h"
#include "Adafruit_Thermal.h"
#include "qr.cpp"
#include "logo.cpp"

#define MAX_FORWARD 150
#define MAX_BACKWARD 110
#define NEUTRAL 127

#define LEFT_UP 23
#define LEFT_DOWN 22
#define RIGHT_UP 37
#define RIGHT_DOWN 36
#define RIGHT_LEFT 35
#define RIGHT_RIGHT  34
#define AUTO_STOP 28
#define STOP 29
#define ACTUATOR_UP 26
#define ACTUATOR_DOWN 25

//these are the autonomous states
#define MANUAL 0
#define TO_DUMPING 1
#define TO_MINING 2

unsigned long robotVolt = 0;
unsigned long robotCurrent = 0;
unsigned long comVolt = 0;
unsigned long comCurrent = 0;
unsigned long running_count = 0;

struct DCM_PACKET
{
  int pot1;
  int pot2;
  float voltage;
  float current;
};

struct INSTRUCTIONS_PACKET
{
  int leftMotorSpeed;
  int rightMotorSpeed;
  int actuatorSpeed;
  int autoState;
};

struct AUTO_PACKET
{
  int robotvoltage;
  int robotcurrent;
  int comvoltage;
  int comcurrent;
  int angle;
  int distance[2];
};

EasyTransfer ET;
EasyTransfer ET2;
EasyTransferI2C DCM;

INSTRUCTIONS_PACKET netData;
AUTO_PACKET autoData;
DCM_PACKET dcmData;

Adafruit_Thermal printer(43,48);

inline void getDCMData()
{ 
  DCM.receiveData();
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(38400);
  Serial3.begin(9600);
  ET.begin(details(netData), &Serial1);
  ET2.begin(details(autoData), &Serial1);
  
  Wire.begin(2);
  DCM.begin(details(dcmData), &Wire);
  Wire.onReceive(receive);
  
  pinModeFast(12, OUTPUT);
  pinModeFast(13, OUTPUT);
  digitalWriteFast(12, HIGH);
  
  pinModeFast(LEFT_UP, INPUT);
  pinModeFast(LEFT_DOWN, INPUT);
  pinModeFast(RIGHT_UP, INPUT);
  pinModeFast(RIGHT_DOWN, INPUT);
  pinModeFast(RIGHT_LEFT, INPUT);
  pinModeFast(RIGHT_RIGHT, INPUT);
  pinModeFast(AUTO_STOP, INPUT);
  pinModeFast(STOP, INPUT);
  pinModeFast(ACTUATOR_UP, INPUT);
  pinModeFast(ACTUATOR_DOWN, INPUT);
}

void receive(int howMany) {}

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

void processScreenData()
{
  char temp = Serial3.read();
  
  if(temp == '#')
  {
    temp = Serial3.read();
    
    if(temp == 'p')
    {
      processPrinterData();
    }
    else if(temp == 'd')
    {
      netData.autoState = TO_DUMPING;
    }
    else if(temp == 'm')
    {
      netData.autoState == TO_MINING;
    }
  }
  
  Serial3.write('$');
  Serial3.write(23);
  Serial3.write((autoData.robotvoltage / 1023.0) * 255);
  Serial3.write((autoData.robotcurrent / 1023.0) * 255);
  Serial3.write((autoData.distance[0] / 1023.0) * 255);
  Serial3.write((autoData.distance[1] / 1023.0) * 255);
}

void processPrinterData()
{
  
  unsigned int robotVoltAverage = robotVolt / running_count;
  unsigned int robotCurrentAverage = robotCurrent / running_count;
  unsigned int comVoltAverage = comVolt / running_count;
  unsigned int comCurrentAverage = comCurrent / running_count;
  unsigned int volt = robotVoltAverage + comVoltAverage;
  unsigned int current = robotCurrentAverage + (comVoltAverage / 1000);
  unsigned int energy = volt * current * 10; 
  
  printer.begin();
  delay(500);
  printer.justify('C'); 
  printer.printBitmap(180, 182, logo);
  delay(500);
  printer.boldOn();
  printer.print("FEAR THE ROOBOT!");
  printer.println(" ");
  printer.boldOff();  
  delay(500); 
  printer.underlineOn(); 
  printer.print("DATA:");
  printer.underlineOff();  
  printer.println(" ");
  delay(500);
  printer.justify('L'); 
  printer.print("Energy Usage: ");
  printer.print(energy, DEC);
  printer.print(" Amp Minutes");
  printer.println(" ");
  delay(500);
  printer.justify('C');
  delay(200);
  printer.boldOn(); 
  printer.println("Have a good day!");
  printer.println(" ");
  printer.println("-From S.T.E.V.E.");
  printer.boldOff(); 
  printer.println(" ");
  printer.println("Check out our Facebook with");
  printer.println("the QR code below!");
  printer.println(" ");
  printer.printBitmap(132, 132, qr);
}

void processInputs()
{
  if(!digitalReadFast(LEFT_UP))
  {
    netData.leftMotorSpeed = NEUTRAL - dcmData.pot1;
  }
  else if(!digitalReadFast(LEFT_DOWN))
  {
    netData.leftMotorSpeed = NEUTRAL + dcmData.pot1;
  }
  else
  {
    netData.leftMotorSpeed = NEUTRAL;
  }
  
  if(!digitalReadFast(RIGHT_UP))
  {
    netData.rightMotorSpeed = NEUTRAL - dcmData.pot2;
  }
  else if(!digitalReadFast(RIGHT_DOWN))
  {
    netData.rightMotorSpeed = NEUTRAL + dcmData.pot2;
  }
  else
  {
    netData.rightMotorSpeed = NEUTRAL;
  }
  
  if(!digitalReadFast(ACTUATOR_UP))
  {
    netData.actuatorSpeed = 255;
  }
  else if(!digitalReadFast(ACTUATOR_DOWN))
  {
    netData.actuatorSpeed = 1;
  }
  else
  {
    netData.actuatorSpeed = 127;
  }
  
  if(!digitalReadFast(AUTO_STOP))
  {
    netData.leftMotorSpeed = 0;
    netData.rightMotorSpeed = 0;
    netData.actuatorSpeed = 0;
    netData.autoState = MANUAL;
  }
  
  if(!digitalReadFast(STOP))
  {
    netData.leftMotorSpeed = 0;
    netData.rightMotorSpeed = 0;
    netData.actuatorSpeed = 0;
    netData.autoState = MANUAL;
  }
}

void loop()
{
  while(1)
  {
    blinky();
    processInputs();
    processScreenData();
    getDCMData();
    ET.sendData();
    if(ET2.receiveData())
    {
      robotVolt += autoData.robotvoltage;
      robotCurrent += autoData.robotcurrent;
      comVolt += autoData.comvoltage;
      comCurrent += autoData.comcurrent;
      running_count++;
    }
    
    Serial.print("M:");
    Serial.print(netData.leftMotorSpeed, DEC);
    Serial.print(" ");
    Serial.print(netData.rightMotorSpeed, DEC);
    Serial.print(" ");
    Serial.print(netData.actuatorSpeed, DEC);
    Serial.print(" ");
    Serial.print(autoData.angle, DEC);
    Serial.print(" ");
    Serial.print(netData.autoState, DEC);
    Serial.print(" ");
    Serial.print(dcmData.pot1, DEC);
    Serial.print(" ");
    Serial.print(dcmData.pot2, DEC);  
    Serial.println();
  }
}
