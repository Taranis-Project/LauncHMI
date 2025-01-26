#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

//---------------------Telem--------------------
byte start_of_frame = 0xBB;

byte end_of_frame1 = 0x0A;
byte end_of_frame2 = 0xAA;
byte end_of_frame3 = 0xFA;

#define SpreadingFactor 8
void setup() {
  Serial.begin(2000000);
  while (!Serial);

  //Serial.println("LoRa Receiver");

  if (!LoRa.begin(869.5E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setFrequency(869.5E6);               //définir la fréquence utilisé
  LoRa.setSpreadingFactor(SpreadingFactor);           // ranges from 6-12,default 7 see API docs
  LoRa.setSignalBandwidth(250E3);           // 500 Hz = 0.0005 MHz Plus étaler = plus de vitesse de transmission et moins de portée (car plus de bruit). 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
  LoRa.setCodingRate4(5);                   // entre 5 et 8 plus la valeurs est elevée plus les erreurs sont faible (portée augmente) débit est réduit 4/5 4/8
  LoRa.setPreambleLength(8);            //Taille des données anti erreurs plus le chiffre est grand plus la portée augmente (et la vitesse diminue)
  LoRa.setTxPower(17,PA_OUTPUT_RFO_PIN);                      //500mW POUR 869.4/869.65
  //LoRa.setTxPower(20,PA_OUTPUT_PA_BOOST_PIN);      //500mW POUR 869.4/869.65
  //LoRa.enableCrc();                       //Cyclic redundancy check premet de réparer les erreurs mais prend de la bande passante.
  //LoRa.disableCrc();
  //LoRa.enableInvertIQ();                //Pour éviter des problèmes d'interfèrence avec d'autres suystèmes Lora, les phases modulée I et Q peuvent être inversé (à tester si pb au C'Space) 
  //LoRa.disableInvertIQ();
  //LoRa.setGain(gain);                   //Défini la sensibilité du gain de récéption. Cela est determinée par défaut par la puce Lora
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize>0) {
    // received a packet
    // read packet
    String Belissama_frame = "";
    Belissama_frame += (char)start_of_frame;
    while (LoRa.available()) {
      Belissama_frame += (char)LoRa.read();
    }
    int16_t LoraRSSI = LoRa.packetRssi();

    //Serial.println(LoraRSSI);

    byte RSSIByte[2];
    
    convertToByteArray(LoraRSSI, &RSSIByte[0]);
    Belissama_frame += (char)RSSIByte[0];
    Belissama_frame += (char)RSSIByte[1];
    
    Belissama_frame += (char)end_of_frame1;
    Belissama_frame += (char)end_of_frame2;
    Belissama_frame += (char)end_of_frame3;
    
    // print RSSI of packet
    Serial.print(Belissama_frame);
//    for(int i = 0; i < Belissama_frame.length();i++)
//    {
//      Serial.print((byte)Belissama_frame.charAt(i), HEX);
//      Serial.print(',');
//    }
//    Serial.println("");
  }
}

void convertToByteArray(int16_t intValue, byte *byteArray) {
  uint8_t *p = (uint8_t *)&intValue;
  *byteArray++ = *(p + 1);
  *byteArray++ = *p;
}
