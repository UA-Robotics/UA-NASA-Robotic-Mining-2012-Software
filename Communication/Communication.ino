/*
    Created By: Duncan Campbell and Richard Johnson
    Copyright 2012

    This file is part of UA NASA Robotic Mining Competion Robot STeve.

    STeve is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Steve is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Steve.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <EasyTransfer.h>
#include <EasyTransferI2C.h>
#include <digitalWriteFast.h>
#include <Wire.h>

//these are the switches that limit the pots
#define LIMIT_SWITCH1 22
#define LIMIT_SWITCH2 23

//these are the autonomous states
#define MANUAL 0
#define TO_DUMPING 1
#define TO_MINING 2

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

struct DCM_PACKET
{
    float robotvoltage;
    float robotcurrent;
    float comvoltage;
    float comcurrent;
    int angle;
    int distance[5];
};

int leftMotorSpeed = 127;
int rightMotorSpeed = 127;
int actuatorSpeed = 127;

int angle = 0;

EasyTransfer ET;
EasyTransfer ET2;
EasyTransferI2C DCM;

INSTRUCTIONS_PACKET netData;
AUTO_PACKET autoData;
DCM_PACKET dcmData;

void sendMotorCommand()
{
    Serial2.print('$');
    Serial2.print(leftMotorSpeed);
    Serial2.print(',');
    Serial2.print(rightMotorSpeed);
    Serial2.print(',');
    Serial2.print(actuatorSpeed);
    Serial2.println();

    Serial.print(leftMotorSpeed, DEC);
    Serial.print(" ");
    Serial.print(rightMotorSpeed, DEC);
    Serial.print(" ");
    Serial.print(actuatorSpeed, DEC);
    Serial.print(" ");
    Serial.print(dcmData.angle, DEC);
    Serial.print(" ");
    Serial.print(dcmData.distance[0]);
    Serial.print(" ");
    Serial.print(dcmData.distance[1]);
    Serial.println();
}

inline void comSafety(boolean check)
{
    static unsigned long current = 0;
    static unsigned long last_com = 0;

    current = millis();

    if(check)
    {
        last_com = current;
    }

    if(current - last_com > 1000)
    {
        leftMotorSpeed = 127;
        rightMotorSpeed = 127;
        actuatorSpeed = 127;
    }
}

inline void getDCMData()
{
    if(DCM.receiveData())
    {
        autoData.robotvoltage = dcmData.robotvoltage;
        autoData.robotcurrent = dcmData.robotcurrent;
        autoData.comvoltage = dcmData.comvoltage;
        autoData.comcurrent = dcmData.comcurrent;
        autoData.angle = dcmData.angle;
        autoData.distance[0] = dcmData.distance[0];
        autoData.distance[1] = dcmData.distance[1];
    }
}

void manualControl()
{
    if(ET.receiveData())
    {
        leftMotorSpeed = netData.leftMotorSpeed;
        rightMotorSpeed = netData.rightMotorSpeed;
        actuatorSpeed = netData.actuatorSpeed;
        comSafety(true);
    }

    if(!digitalReadFast(LIMIT_SWITCH1) && actuatorSpeed == 0)
    {
        actuatorSpeed = 127;
    }
    else if(!digitalReadFast(LIMIT_SWITCH2) && actuatorSpeed == 255)
    {
        actuatorSpeed = 127;
    }

    if(netData.autoState == TO_DUMPING)
    {
        toDumping();
    }
    else if(netData.autoState == TO_MINING)
    {
        toMining();
    }

    sendMotorCommand();
}

void receive(int howMany)
{

}

void setup()
{
    Serial.begin(9600);
    Serial1.begin(38400);
    Serial2.begin(9600);
    ET.begin(details(netData), &Serial1);
    ET2.begin(details(autoData), &Serial1);

    Wire.begin(2);
    DCM.begin(details(dcmData), &Wire);
    Wire.onReceive(receive);

    pinModeFast(A15, OUTPUT);
    pinModeFast(13, OUTPUT);
    pinModeFast(12, OUTPUT);
    digitalWriteFast(A15, HIGH);
    digitalWriteFast(12, HIGH);
}

//blinks hte led every second
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

void loop()
{
    while(1)
    {
        blinky();
        manualControl();
        getDCMData();
        comSafety(false);
        ET2.sendData();
    }
}
