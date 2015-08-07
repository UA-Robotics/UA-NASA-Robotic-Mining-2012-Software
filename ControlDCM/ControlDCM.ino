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

#include <Wire.h>
#include <digitalWriteFast.h>
#include <EasyTransferI2C.h>

EasyTransferI2C MCU;

struct DCM_PACKET
{
    int pot1;
    int pot2;
    float voltage;
    float current;
};

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

int c = 0; //LIDAR Data updat interval
int i = 0;
int X = 0;
int volts;
int temp;
float comcurrent;
float comvolt;

void setup()
{
    Wire.begin();  //Join I2C bus as address 2
    MCU.begin(details(dcmData), &Wire);

    Serial.begin(9600);

    //DDRD = DDRD | B111100;
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); //I am Alive!!!
}

inline void writeData()
{
    static long last = 0;
    static long curr = 0;

    curr = millis();
    if(curr - last > 200)
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

    if( X >= 300 )
    {
        temp = analogRead(VIN);     //Voltage comeing in com board
        comvolt = (temp/1023.00)*5.00; //Voltage comeing in com board
        comcurrent = comvolt / 2 * 100;
        dcmData.voltage = comvolt;
        dcmData.current = comcurrent;
        dcmData.pot1 = ((float)readMux(3) / 1023.0) * 120; //((float)readMux(4) / 1023.0) * 50;
        dcmData.pot2 = ((float)readMux(4) / 1023.0) * 120;

        if(dcmData.pot1 > 100)
        {
            dcmData.pot1 = 0;
        }
        if(dcmData.pot2 > 100)
        {
            dcmData.pot2 = 0;
        }

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
        digitalWrite(controlPin[r], muxChannel[channel][r]);
    }

    //read the value at the SIG pin
    temp = analogRead(MUXIN);

    //return the value
    return temp;
}
