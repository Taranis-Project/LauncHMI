#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>

#define SpreadingFactor 8
#define PinLED 3
#define SLAVE_ADDR 8

volatile size_t index = 0;
volatile char receivedChar;
char receivedString[50];

char data[31];
uint8_t dataLength = 0;
char LoRaBuffer[31];
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

void receiveEvent(int howMany) {
  for (int i = 0; i < howMany; i++) {
    byte1 = byte2;
    byte2 = byte3;
    byte3 = byte4;
    byte4 = byte5;
    byte5 = byte6;
    byte6 = Wire.read();
    data[dataLength] = byte6;
    dataLength++;

    if (byte6 == trame_end_of_frame6 && byte5 == trame_end_of_frame5 && byte4 == trame_end_of_frame4 && byte3 == trame_end_of_frame3 && byte2 == trame_end_of_frame2 && byte1 == trame_end_of_frame1) {
      for (int k = 0; k < dataLength-6; k++) {
        LoRaBuffer[k] = data[k];
    }
      LoraBufferSize = dataLength-6;
      FrametoSend = true;
      dataLength = 0;
    }
    else if (dataLength > 57) {
      dataLength = 0;
    }
  }
}

void setup() {
  pinMode(PinLED, OUTPUT);

  Serial.begin(9600); // Adjust baud rate as needed
  
  Wire.begin(8);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvent);

  SPI.begin();           // Initialize SPI as slave
  SPI.attachInterrupt(); // Enable SPI interrupt

  if (!LoRa.begin(869.5E6)) {
    while (1) {
      digitalWrite(PinLED, HIGH);
      delay(100);
      digitalWrite(PinLED, LOW);
      delay(100);
    }
  }

  LoRa.setFrequency(869.5E6);
  LoRa.setSpreadingFactor(SpreadingFactor);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  //LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setTxPower(17,PA_OUTPUT_RFO_PIN);                      //500mW POUR 869.4/869.65

  digitalWrite(PinLED, HIGH);
}

void loop() {
  if (FrametoSend) {
    Serial.print(LoRaBuffer);
    LoRa.beginPacket();
    LoRa.write(LoRaBuffer, LoraBufferSize);
    LoRa.endPacket();

//    for (int i = 0; i < LoraBufferSize; i++) {
//      Serial.print((byte)LoRaBuffer[i]);
//      Serial.print(',');
//    }
//    Serial.println("");

    FrametoSend = false;
    dataLength = 0;
  }
}