#include <SoftwareSerial.h>

#include <OneWire.h> 
SoftwareSerial mySerial(10, 11); // RX, TX

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
int counter = 0;    // counter for packetizing
char message[] = "";

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello, world!");
    mySerial.begin(9600);
  mySerial.println("Hello, world!");
}

void loop(void) {
  float temperature = getTemp();
  float farenheit = (temperature * 9 / 5) + 32;
  message = message + 'F' + farenheit;

  delay(100); //just here to slow down the output so it is easier to read
  if(counter > 5){
    mySerial.print(farenheit);
    mySerial.println(" F");
  
    if(Serial.available()){
      Serial.print(temperature);
      Serial.print(" C / ");
      Serial.print(farenheit);
      Serial.println(" F");
    }
    counter = 0;
    message = "";
  }
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


