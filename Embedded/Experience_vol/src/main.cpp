#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "tatelem.h"

#define SpreadingFactor 8
#define PinLED 3
#define SLAVE_ADDR 8
#define SS 10

char data[70];
uint8_t dataLength = 0;
char LoRaBuffer[70];
uint8_t LoraBufferSize = 0;
bool FrametoSend = false;

byte byte1 = 0;
byte byte2 = 0;
byte byte3 = 0;
byte byte4 = 0;
byte byte5 = 0;
byte byte6 = 0;

byte trame_end_of_frame1 = 0xD5;
byte trame_end_of_frame2 = 0xAE;
byte trame_end_of_frame3 = 0x77;
byte trame_end_of_frame4 = 0xBD;
byte trame_end_of_frame5 = 0x42;
byte trame_end_of_frame6 = 0xCD;

int VisUI_start_of_frame = 0xBB;
int VisUI_end_of_frame1 = 0x0A;
int VisUI_end_of_frame2 = 0xAA;
int VisUI_end_of_frame3 = 0xFA;

void setup() {
  Serial.begin(9600);

  Wire1.begin();
  //Wire1.setClock(400000);

}

byte start_of_frame = 0xBB;

byte end_of_frame1 = 0x0A;
byte end_of_frame2 = 0xAA;
byte end_of_frame3 = 0xFA;

int16_t valeur1 = 0;
int16_t valeur2 = 100;
int16_t valeur3 = 200;

#define slave_addr 8

uint8_t uint8_t_=0;
int8_t int8_t_value=0;
uint16_t uint16_t_=0;
int16_t int16_t_=0;
uint32_t uint32_t_=0;
int32_t int32_value =0;

void loop(){

  String Belissama_frame = "";

  Belissama_frame += (char)start_of_frame;

  //Serial.println(Belissama_frame.length());

  //bool as byte
  bool s[] = {0, 1, 0, 1, 0, 1, 1, 1};
  Belissama_frame = convertToByteArray(s, Belissama_frame);

  //uint8_t
  uint8_t_ += 1;
  Belissama_frame = convertToByteArray(uint8_t_, Belissama_frame);
  
  //int8_t
  int8_t_value += 1;
  Belissama_frame = convertToByteArray(int8_t_value, Belissama_frame);
  
  //uint16_t
  uint16_t_ += 1;
  Belissama_frame = convertToByteArray(uint16_t_, Belissama_frame);
  
  //int16_t
  int16_t_ += 1;
  Belissama_frame = convertToByteArray(int16_t_, Belissama_frame);

  //uint32_t
  uint32_t_ += 1;
  Belissama_frame = convertToByteArray(uint32_t_, Belissama_frame);
    
  //int32_t
  int32_value += 1;
  Belissama_frame = convertToByteArray(int32_value, Belissama_frame);

  //float
  float valeur1_b = 1.23;
  Belissama_frame = convertToByteArray(valeur1_b, Belissama_frame, false);
  
  //h_float
  float valeur2_b = 2.71;
  Belissama_frame = convertToByteArray(valeur2_b, Belissama_frame,true);
  
  //byte
  byte byte_value = '\xf8';
  Belissama_frame = convertToByteArray(byte_value, Belissama_frame);

  //char
  char char_value = 'A';
  Belissama_frame = convertToByteArray(char_value, Belissama_frame);

  //end of frame
  Belissama_frame += (char)trame_end_of_frame1;
  Belissama_frame += (char)trame_end_of_frame2;
  Belissama_frame += (char)trame_end_of_frame3;
  Belissama_frame += (char)trame_end_of_frame4;
  Belissama_frame += (char)trame_end_of_frame5;
  Belissama_frame += (char)trame_end_of_frame6;
  
  //Serial.println(Belissama_frame.length());

  // print RSSI of packet
  Wire1.beginTransmission(slave_addr);  // Address of the slave device (change as needed)
  for (int i = 0; i < Belissama_frame.length(); i++) {
/*     if(i>=32){
      Wire1.endTransmission();  // End the transmission
      Wire1.beginTransmission(slave_addr);  // Address of the slave device (change as needed)
    } */
    Wire1.write(Belissama_frame[i]);
    //Serial.print(Belissama_frame[i]);
    delayMicroseconds(1);
  }
  Wire1.endTransmission();  // End the transmission
}