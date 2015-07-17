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

#define DELTA_S 1 //change step numbr

#include "Tlc5940.h"
#include "tlc_servos.h"

#define MOTOR_L   3
#define MOTOR_R   4
#define ACT   0
#define INTERVAL 1000

int LED = 500;

int ledState = HIGH;
unsigned char hold = 0;  // 1 byte in Arduino
long previousMillis = 0;
unsigned long currentMillis;
long last = 0;
long last2 = 0;
int Dchange = 0;
int mtr = 0;
int prevmtr = 90;
int Dchange2 = 0;
int mtl = 0;
int prevmtl = 90;
int Dchange3 = 0;
int act = 0;
int prevact = 90;
long trip;
#define SERIAL_BUFFERSIZE 80
char buffer[SERIAL_BUFFERSIZE];
int bufferidx;
char *parseptr;


inline boolean lockDown()
{
  static boolean set = false;

  if(mtr == 0 || mtl == 0 || act == 0)
  {
    LED = 100;
    set = true;

    tlc_setServo(MOTOR_L,90);
    tlc_setServo(MOTOR_R,90);
    tlc_setServo(ACT,90);
    Tlc.set(MOTOR_L, 0);
    Tlc.set(MOTOR_R, 0);
    Tlc.set(ACT, 0);
    Dchange = 0;
    mtl = 0;
    Dchange2 = 0;
    mtr = 0;
    Dchange3 = 0;
    act = 0;

    return false;
  }

  if(mtr != 0 && mtl != 0 && act != 0)
  {
    LED = 500;

    if(set)
    {
      set = false;
      last = 0;
      last2 = 0;
      Dchange = 0;
      mtr = 0;
      prevmtr = 90;
      Dchange2 = 0;
      mtl = 0;
      prevmtl = 90;
      Dchange3 = 0;
      act = 0;
      prevact = 90;
    }
    return true;
  }
}

void setup()
{
  Serial.begin(9600);
  tlc_initServos();  // Note: this will drop the PWM freqency down to 50Hz.
  tlc_setServo(MOTOR_L,90);
  tlc_setServo(MOTOR_R,90);
  tlc_setServo(ACT,90);
  Tlc.update();
  pinMode(7, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  Serial.print("Ready");
}

void loop(){
   currentMillis = millis();
   Serial_mode_read();
   if(currentMillis - previousMillis > LED) {
      previousMillis = currentMillis;
      if(ledState == LOW){
         ledState = HIGH;
         digitalWrite(7, ledState);
        }
      else{
         ledState = LOW;
         digitalWrite(7, ledState);
        }
      }
   if(currentMillis - last > 5){
       last = currentMillis;
       trip++;
       //Serial.print("called");
       if(Dchange != 0){        //ISR or time delay for this "ramp" fcn LEFT ramp
       if(Dchange > 0 ){
          Dchange = Dchange - DELTA_S; //This is used to count the Delta change required for the value
          prevmtl = prevmtl + DELTA_S; //This is the actual manipulation of the data fff new numb > old
          //send out to motor comand here:
          tlc_setServo(MOTOR_L,prevmtl);
          //Tlc.update();
         }
       else if(Dchange < 0 ){
          Dchange = Dchange + DELTA_S; //This is used to count the Delta change required for the value
          prevmtl = prevmtl - DELTA_S; //This is the actual manipulation of the data : fff new numb < old
          //send out to motor comand here:
          tlc_setServo(MOTOR_L,prevmtl);
          //Tlc.update();
         }
      }
      if(Dchange2 != 0){        //ISR or time delay for this "ramp" fcn LEFT ramp
       if(Dchange2 > 0 ){
          Dchange = Dchange - DELTA_S; //This is used to count the Delta change required for the value
          prevmtr = prevmtr + DELTA_S; //This is the actual manipulation of the data fff new numb > old
          //send out to motor comand here:
          tlc_setServo(MOTOR_R,prevmtr);
          //Tlc.update();
         }
       else if(Dchange2 < 0 ){
          Dchange = Dchange + DELTA_S; //This is used to count the Delta change required for the value
          prevmtr = prevmtr - DELTA_S; //This is the actual manipulation of the data : fff new numb < old
          //send out to motor comand here:
          tlc_setServo(MOTOR_R,prevmtr);
          Tlc.update();
         }
        }

      Serial.print(prevmtr);
      Serial.print(",");
      Serial.println(prevmtl);

      //Tlc.update();
    }

   if(currentMillis - last2 > 5){
       last2 = currentMillis;
       if(Dchange3 != 0){        //ISR or time delay for this "ramp" fcn LEFT ramp
        if(Dchange3 > 0 ){
          Dchange = Dchange - 1; //This is used to count the Delta change required for the value
          prevact = prevact + 1; //This is the actual manipulation of the data fff new numb > old
          //send out to motor comand here:
          tlc_setServo(ACT,prevact);
          //Tlc.update();
         }
        else if(Dchange3 < 0 ){
          Dchange = Dchange + 1; //This is used to count the Delta change required for the value
          prevact = prevact - 1; //This is the actual manipulation of the data : fff new numb < old
          //send out to motor comand here:
          tlc_setServo(ACT,prevact);
          //Tlc.update();
         }
        }
        /*
      Serial.println(prevact);
      */
      Tlc.update();
    }

    if(currentMillis - previousMillis > INTERVAL){
       //no data : lockdown
       if(trip > 8){
         tlc_setServo(MOTOR_L,90);
         tlc_setServo(MOTOR_R,90);
         tlc_setServo(ACT,90);
         Tlc.set(MOTOR_L, 0);
         Tlc.set(MOTOR_R, 0);
         Tlc.set(ACT, 0);
         Dchange = 0;
         mtl = 0;
         Dchange2 = 0;
         mtr = 0;
         Dchange3 = 0;
         act = 0;
         //Serial.println("trip");
         Serial.flush();
        }
      }
}

// We read the control values from serial port
void Serial_mode_read(){
char c;
int numc;
int i;

numc = Serial.available();
//Serial.print(numc);
if (numc > 0)
for (i=0;i<numc;i++)
{
c = Serial.read();
if (c == '$'){ // Line Start
bufferidx = 0;
buffer[bufferidx++] = c;
continue;
}
if (c == '\r'){ // Line End
buffer[bufferidx++] = 0;
parse_serial();
}
else {
if (bufferidx < (SERIAL_BUFFERSIZE-1)){
buffer[bufferidx++] = c;
}
else
bufferidx=0; // Buffer overflow : restart
}
}
}

// Parse the serial port strings
void parse_serial()
{
  parseptr = strchr(buffer, '$')+1;
  mtl = parsedecimal(parseptr,3);
  parseptr = strchr(parseptr, ',')+1;
  mtr = parsedecimal(parseptr,3);
  parseptr = strchr(parseptr, ',')+1;
  act = parsedecimal(parseptr,3);
  trip = 0;

  if(lockDown())
  {
    mtl = map(mtl,1,255,0,180);
    mtr = map(mtr,1,255,0,180);
    act = map(act,1,255,0,180);
    if(mtl > 180){
      mtl = 180;
    }
    if(mtr > 180){
      mtr = 180;
    }
    if(act > 180){
      act = 180;
    }
    Dchange = mtl - prevmtl; //This tels us what the DELTA change from prev val to new value
    if(abs(Dchange) % 2 == 1){ // this seamed to fix the issue with code runaway (IE: dosn't like Dchg to have neg #'s)
      Dchange = Dchange + 1;
    }
    Dchange2 = mtr - prevmtr; //This tels us what the DELTA change from prev val to new value
    if(abs(Dchange2) % 2 == 1){ // this seamed to fix the issue with code runaway (IE: dosn't like Dchg to have neg #'s)
      Dchange2 = Dchange2 + 1;
    }
    Dchange3 = act - prevact; //This tels us what the DELTA change from prev val to new value
    if(abs(Dchange3) % 2 == 1){ // this seamed to fix the issue with code runaway (IE: dosn't like Dchg to have neg #'s)
      Dchange3 = Dchange3 + 1;
    }
  }
}

// Decimal number parser
long parsedecimal(char *str,byte num_car) {
long d = 0;
byte i;

i = num_car;
while ((str[0] != 0)&&(i>0)) {
if (str[0]=='-')
d = -1*d;
else if ((str[0] > '9') || (str[0] < '0'))
return d;
d *= 10;
d += str[0] - '0';
str++;
i--;
}
return d;
}
