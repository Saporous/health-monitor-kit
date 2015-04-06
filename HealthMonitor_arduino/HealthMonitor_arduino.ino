
/*
>> Pulse Sensor Amped 1.2 <<
This code is for Pulse Sensor Amped by Joel Murphy and Yury Gitman
    www.pulsesensor.com 
    >>> Pulse Sensor purple wire goes to Analog Pin 0 <<<
Pulse Sensor sample aquisition and processing happens in the background via Timer 2 interrupt. 2mS sample rate.
PWM on pins 3 and 11 will not work when using this code, because we are using Timer 2!
The following variables are automatically updated:
Signal :    int that holds the analog signal data straight from the sensor. updated every 2mS.
IBI  :      int that holds the time interval between beats. 2mS resolution.
BPM  :      int that holds the heart rate value, derived every beat, from averaging previous 10 IBI values.
QS  :       boolean that is made true whenever Pulse is found and BPM is updated. User must reset.
Pulse :     boolean that is true when a heartbeat is sensed then false in time with pin13 LED going out.

This code is designed with output serial data to Processing sketch "PulseSensorAmped_Processing-xx"
The Processing sketch is a simple data visualizer. 
All the work to find the heartbeat and determine the heartrate happens in the code below.
Pin 13 LED will blink with heartbeat.
If you want to use pin 13 for something else, adjust the interrupt handler
It will also fade an LED on pin fadePin with every beat. Put an LED and series resistor from fadePin to GND.
Check here for detailed code walkthrough:
http://pulsesensor.myshopify.com/pages/pulse-sensor-amped-arduino-v1dot1

Code Version 1.2 by Joel Murphy & Yury Gitman  Spring 2013
This update fixes the firstBeat and secondBeat flag usage so that realistic BPM is reported.

*/
#include <SoftwareSerial.h>
#include <OneWire.h> 
#include <Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

//  VARIABLES
SoftwareSerial mySerial(11, 10);   // RX, TX
OneWire ds(2);                     // on digital pin 2
int pulsePin = 0;                  // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                 // pin to blink led at each beat
int fadePin = 5;                   // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                  // used to fade LED on with PWM on fadePin
int count,countX,countY,countZ;    // counters for XYZ axis
int delayCount;                    // 
int stepUp,stepDown;               // flag to signal whether a step was already detected
int emergencyCount,emergencyFlag;  // flag to determine pressed emergency button press
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float tempTemperature=30.00,temperature=30.00,farenheit;

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  //pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  mySerial.begin(9600);
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
   // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
   // AND APPLY THAT VOLTAGE TO THE A-REF PIN
   //analogReference(EXTERNAL);   
}



void loop(){
  if(emergencyButton){
    emergencyCount++;
  }
  if(emergencyCount > 5){
    emergencyFlag = 1;
  }
  tempTemperature = getTemp();
  if(tempTemperature > 10 && tempTemperature < 50){
    temperature = tempTemperature;
  }
  farenheit = (temperature * 9/5) + 32;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //float TmpF=(Tmp/340.00+36.53)*9/5 + 32; // Unused onboard temperature sensor in Farenheit
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  if(GyX > 5000 || GyX < -5000 && !stepDown){
    countX++;
    stepDown = 1;
    stepUp = 0;
  }
  if(GyY > 5000 || GyY < -5000 && !stepDown){
    countY++;
    stepDown = 1;
    stepUp = 0;
  }
  if(GyZ > 5000 || GyZ < -5000 && !stepDown){
    countZ++;
    stepDown = 1;
    stepUp = 0;
  }
  
  if(GyX < 5000 && GyX > -5000 && countX){
    countX = 0;
    stepDown = 0;
    stepUp = 1;
  }
  if(GyY < 5000 && GyY > -5000 && countY){
    countY = 0;
    stepDown = 0;
    stepUp = 1;
  }
  if(GyZ < 5000 && GyZ > -5000 && countZ){
    countZ = 0;
    stepDown = 0;
    stepUp = 1;
  }
  
  if(stepUp){
    count++;
    stepDown = 0;
    stepUp = 0;
  }
  
  if(delayCount > 10){
    //Serial.print("GyX"); Serial.print(GyX);
    Serial.print("<<<<<<<<<<"); 
    Serial.print(":T"); Serial.print(farenheit);
    Serial.print(":S"); Serial.print(count/2);
    Serial.print(":H"); Serial.print(BPM);
    Serial.print(":T"); Serial.print(farenheit);
    Serial.print(":S"); Serial.print(count/2);
    Serial.print(":H"); Serial.print(BPM);
    Serial.print(":T"); Serial.print(farenheit);
    Serial.print(":S"); Serial.print(count/2);
    Serial.print(":H"); Serial.print(BPM);
    Serial.print(":T"); Serial.print(farenheit);
    Serial.print(":S"); Serial.print(count/2);
    Serial.print(":H"); Serial.print(BPM);
    Serial.println(":>>>>>>>>>>");

    mySerial.print("<<<<<<<<<<"); 
    mySerial.print(":T"); mySerial.print(farenheit);
    mySerial.print(":S"); mySerial.print(count/2);
    mySerial.print(":H"); mySerial.print(BPM);
    mySerial.print(":T"); mySerial.print(farenheit);
    mySerial.print(":S"); mySerial.print(count/2);
    mySerial.print(":H"); mySerial.print(BPM);
    mySerial.print(":T"); mySerial.print(farenheit);
    mySerial.print(":S"); mySerial.print(count/2);
    mySerial.print(":H"); mySerial.print(BPM);
    mySerial.print(":T"); mySerial.print(farenheit);
    mySerial.print(":S"); mySerial.print(count/2);
    mySerial.print(":H"); mySerial.print(BPM);
    mySerial.println(":>>>>>>>>>>");
    
    delayCount=0;
  }
  delayCount++;
  delay(200);                             //  take a break
}

void ledFadeToBeat(){
    fadeRate -= 15;                         //  set LED fade value
    fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
    analogWrite(fadePin,fadeRate);          //  fade LED
  }

void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
    Serial.println(data);                // the data to send culminating in a carriage return
  }

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}






