#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#define SS_PIN 18
#define RST_PIN 23
#define DIO0_PIN 26
#define SCL 22
#define SDA 21

//---------------------Telem--------------------
byte start_of_frame = 0xBB;

byte end_of_frame1 = 0x0A;
byte end_of_frame2 = 0xAA;
byte end_of_frame3 = 0xFA;

void convertFloatToByteArray(float floatValue, byte *byteArray, bool Half_Float) ;
//---------------------Telem--------------------

const long FREQUENCY = 868063E3;

struct TelemetryData {
  float corr_ax;
  float corr_ay;
  float corr_az;
  float corr_gx;
  float corr_gy;
  float corr_gz;
  float lat;
  float lng;
  float speed;
  float corr_pressure;
  //float rssi;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //Serial.println("LoRa Receiver");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  while (!LoRa.begin(FREQUENCY)) {
    //Serial.println("LoRa initialization failed. Retrying...");
    delay(500);
  }

  LoRa.setSyncWord(0xF3);

  //Serial.println("LoRa Initializing OK!");
}

void loop() {
  if (LoRa.parsePacket()) {
    TelemetryData receivedData;

    //receivedData.rssi = LoRa.packetRssi(); // stocker la valeur RSSI dans la structure

    // Read the packet and parse telemetry data
    String packetData = LoRa.readString();
    sscanf(packetData.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
           &receivedData.corr_ax, &receivedData.corr_ay, &receivedData.corr_az,
           &receivedData.corr_gx, &receivedData.corr_gy, &receivedData.corr_gz,
           &receivedData.lat, &receivedData.lng,
           &receivedData.speed, &receivedData.corr_pressure);

    // Set isSignalGood based on RSSI threshold
    //receivedData.isSignalGood = (receivedData.rssi > RSSI_THRESHOLD);

    // Display received telemetry data
    /*Serial.println("Received packet:");
    Serial.print("Accel X: "); Serial.println(receivedData.corr_ax);
    Serial.print("Accel Y: "); Serial.println(receivedData.corr_ay);
    Serial.print("Accel Z: "); Serial.println(receivedData.corr_az);
    Serial.print("Gyro X: "); Serial.println(receivedData.corr_gx);
    Serial.print("Gyro Y: "); Serial.println(receivedData.corr_gy);
    Serial.print("Gyro Z: "); Serial.println(receivedData.corr_gz);
    Serial.print("Mx: "); Serial.println(receivedData.mx);
    Serial.print("My: "); Serial.println(receivedData.my);
    Serial.print("Mz: "); Serial.println(receivedData.mz);
    Serial.print("Temperature: "); Serial.println(receivedData.temperature);
    Serial.print("Pression: "); Serial.println(receivedData.corr_pressure);*/
    //Serial.print("Lat: "); Serial.println(receivedData.lat, 6);
    //Serial.print("Long: "); Serial.println(receivedData.lng, 6);
    //Serial.print("RSSI: "); Serial.println(receivedData.rssi);
    //Serial.print("Signal is "); Serial.println(receivedData.isSignalGood ? "good" : "bad");*/
    //Serial.println("----------------------");

    float roll = atan2(receivedData.corr_ay, receivedData.corr_az);
    float pitch = atan2(-receivedData.corr_ax, sqrt(receivedData.corr_ay * receivedData.corr_ay + receivedData.corr_az * receivedData.corr_az));
    roll *= 180.0 / PI; // Conversion en degrés
    pitch *= 180.0 / PI; // Conversion en degrés

    // Debut du paquet :
    String msg_frame = "";               // Initialisation du buffer
    msg_frame += (char)start_of_frame;   // Start Flag

    // Ajout des datas :
    // Gyro x
    byte dataAsByte[4];   // Creation du tableau dans lequel on va enregistrer les octets du float
    convertToByteArray(receivedData.corr_gx,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Gyro y
    convertToByteArray(receivedData.corr_gy,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Gyro z
    convertToByteArray(receivedData.corr_gz,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Accel x
    convertToByteArray(receivedData.corr_ax,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Accel y
    convertToByteArray(receivedData.corr_ay,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Accel z
    convertToByteArray(receivedData.corr_az,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Speed
    convertToByteArray(receivedData.speed,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Pressure
    convertToByteArray(receivedData.corr_pressure,&dataAsByte[0],false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Lng
    convertToByteArray(receivedData.lng, &dataAsByte[0], false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Lat
    convertToByteArray(receivedData.lat, &dataAsByte[0], false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Roll
    convertToByteArray(roll, &dataAsByte[0], false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // Pitch
    convertToByteArray(pitch, &dataAsByte[0], false);
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];

    // RSSI
    /*convertToByteArray(receivedData.rssi, &dataAsByte[0], false); // Add this line
    msg_frame += (char)dataAsByte[0];
    msg_frame += (char)dataAsByte[1];
    msg_frame += (char)dataAsByte[2];
    msg_frame += (char)dataAsByte[3];*/

    // Fin du message :
    msg_frame += (char)end_of_frame1;
    msg_frame += (char)end_of_frame2;
    msg_frame += (char)end_of_frame3;

    // Envoi du message
    Serial.print(msg_frame);
  }
}

//C++ Function
// Union for an easier conversion
union FloatConverter {
  float f;
  uint32_t i;
};
void convertToByteArray(float floatValue, byte *byteArray, bool Half_Float) {
  if (Half_Float)
  {
    FloatConverter converter;
    converter.f = floatValue;
    uint32_t floatbuff = converter.i;

    uint32_t sign = (floatbuff >> 16) & 0x8000;
    uint32_t exponent = ((floatbuff >> 23) & 0xFF) - 127;
    uint32_t fraction = floatbuff & 0x007FFFFF;

    // special values
    if (exponent == 128) {
      // Infinity or NaN
      exponent = 31;
      if (fraction != 0) {
        // NaN
        fraction >>= 13;
        fraction |= 0x2000;
      }
    } else if (exponent == -127) {
      // Zero ou denormalized
      exponent = 0;
      fraction >>= 13;
    } else {
      // normalized
      exponent += 15;
      fraction >>= 13;
    }

    uint16_t halfValue = static_cast<uint16_t>(sign | (exponent << 10) | fraction);

    *byteArray++ = static_cast<uint8_t>(halfValue >> 8);
    *byteArray++ = static_cast<uint8_t>(halfValue & 0xFF);

  }
  else
  {
    uint8_t *p = (uint8_t *)&floatValue;
    *byteArray++ = *(p + 3);
    *byteArray++ = *(p + 2);
    *byteArray++ = *(p + 1);
    *byteArray++ = *p;
  }
}