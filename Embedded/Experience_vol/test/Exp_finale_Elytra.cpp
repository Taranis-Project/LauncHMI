#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <MPU9250.h>
//#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TinyGPSPlus.h>
#include <mySD.h>

#define SS_PIN 18
#define RST_PIN 23
#define DIO0_PIN 26
// If LILYGO 18, 23, 26
// 5, 14, 2
#define SCL 22
#define SDA 21

File SDdata;

// Define the I2C addresses for the sensors
#define MPU6050_ADDR 0x68  // MPU-6050
#define MPU9250_ADDR 0x69  // MPU-9250
//#define AK8963_ADDRESS 0x0C // AK8963 (magnetometer) I2C address
#define BMP_ADDR 0x76   // BMP280

// Telem variables
const long FREQUENCY = 868E6;
const int DELAY_BETWEEN_TRANSMISSIONS =50;
int SPREADING_FACTOR = 7;
int TX_POWER = 2;

// MPU variables
float corr_ax, corr_ay, corr_az, corr_gx, corr_gy, corr_gz;
float corr_az_16g;
// Magneto variables
//float mx, my, mz;

// Define the scale factors for the MPU sensors
float accelScaleFactor = 16384.0; // For +/- 2g
float gyroScaleFactor = 32.8; // Scale factor: 32.8 LSB/°/s for 1000 degrees/sec

// Initialize the BMP280 sensor
Adafruit_BMP280 bmp;

// BMP variables
float temperature;
float pressure, corr_pressure, last_pressure;
#define N 10 // Moyenne glissante sur N valeurs
float HistoriqueSignal[N];
float TotalBufferCirculaireSignal=0; // Stockage du total de toutes les valeurs du buffer.
byte Plus_ancienSignal=0;

// Pitot
const float R = 8.314; // Constante universelle de gaz parfaits 
const float rho = 1.204; // Masse volumique de l'air
const float temp = 293.15; // Température ambiante
const float voltageMax = 3.3; // voltage maximal (correspondant à la plage du pin analogique de l'ESP32)
const float kpaRangeTopVoltage = 3.33; // valeur maximale renvoyée par le capteur, par l'intermédiaire d'un pont diviseur de tension. 

const int numReadings = 10; // Nombre de lectures pour la moyenne glissante
int readings[numReadings] = {0}; // Tableau pour stocker les lectures des capteurs
int currentIndex = 0; // Index actuel pour le tableau des lectures
int total = 0; // Somme des lectures pour le calcul de la moyenne
int sensorPin = 36; // Pin analogique où le capteur est connecté

int sensorValue = 0, sensorMax = 4096, sensorOffset = 0; // Variables pour la valeur du capteur, valeur max du capteur, et l'offset
float voltage = 0, kpa = 0, speed = 0, lastspeed = 0, mach = 0; // Variables pour la tension, pression en kPa, vitesse, dernière vitesse et nombre de Mach
// bool calibrationDone = false; // Booléen pour vérifier si le calibrage est effectué

// GPS
static const uint32_t GPSBaud = 9600;
double lat, lng;
TinyGPSPlus gps;

// General
#define LED 25 // LED Bleue
#define SIGNAL 0 // Signal parachute
#define S_SEQ 4 // Signal Séquenceur
#define SW_TELEM 12 // Switch télem
bool signal_seq_recu;
bool apogee_sent;
uint16_t t0;

// Structuration télémetrie
struct TelemetryData {
  float corr_ax;
  float corr_ay;
  float corr_az;
  float corr_gx;
  float corr_gy;
  float corr_gz;
  /*float mx;
  float my; 
  float mz;*/
  float lat;
  float lng;
  float temperature;
  float corr_pressure;
};

// ==================================================== SETUP

void setup() {

  /////////////////////////////////////////////////////
  // TELEM INITIALIZATION [BEGIN]
  Serial.begin(115200);
  while (!Serial);

  // Separate SPI configurations for LoRa
  SPI.begin(5, 19, 27, 18); //SCK, MISO, MOSI, CS
  //5, 19, 27, 18
  //18, 19, 23, 5
  Serial.println("LoRa Sender");
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  
  while (!LoRa.begin(FREQUENCY)) {
    Serial.println("LoRa initialization failed. Retrying...");
    delay(500);
  }

  LoRa.setSyncWord(0xF3);
  SPREADING_FACTOR = constrain(SPREADING_FACTOR, 7, 12); // Limits of SPREADING_FACTOR
  TX_POWER = constrain(TX_POWER, -3, 15); // Limmits of TX_POWER

  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setTxPower(TX_POWER);
  LoRa.setSignalBandwidth(125E3);           // 500 Hz = 0.0005 MHz Plus étaler = plus de vitesse de transmission et moins de portée (car plus de bruit). 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
  LoRa.setCodingRate4(5);                   // entre 5 et 8 plus la valeurs est elevée plus les erreurs sont faible (portée augmente) débit est réduit 4/5 4/8
  LoRa.setPreambleLength(8);            //Taille des données anti erreurs plus le chiffre est grand plus la portée augmente (et la vitesse diminue)
  Serial.println("LoRa Initializing OK!");
  /////////////////////////////////////////////////////

  // Pins setup
  pinMode(LED, OUTPUT);
  pinMode(SIGNAL, OUTPUT);
  pinMode(S_SEQ, INPUT);
  digitalWrite(SIGNAL, LOW);
  pinMode(SW_TELEM, INPUT_PULLUP);
  signal_seq_recu = false;
  apogee_sent = false;
  
  // Start the serial communication and the I2C bus
  Serial.begin(GPSBaud);
  Wire.begin();

  // Initialize the MPU sensors
  initializeMPU(MPU6050_ADDR);
  initializeMPU(MPU9250_ADDR);
  //initializeMAGNETO(AK8963_ADDRESS); // MAGNETO NE SORT PAS DE VALEURS

  // Initialize the BMP280 sensor
  if (!bmp.begin(BMP_ADDR)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1){blinkLED();};
  }

  // Calibration Pitot
  sensorOffset = adjustOffset(sensorPin, sensorOffset);

  // Initialize SD library with SPI pins
  if (!SD.begin(13,15,2,14)) {            //T1:13,15,2,14  T2: 23,5,19,18 M5：4,23,19,18 uint8_t csPin, int8_t mosi, int8_t miso, int8_t sck
    Serial.println("initialization failed!");
    while (1) {
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
      delay(1000);
    }
  }

  SDdata = SD.open("DATA.txt", O_CREAT | O_WRITE | O_APPEND);
  SDdata.println("start :");
  SDdata.close();
  Serial.println("SD Ok");
  
  delay(3000);
  digitalWrite(LED, HIGH);
  Serial.println("Setup done!");
  t0 = millis();

}

// ==================================================== LOOP

void loop() {

  /////////////////////////////////////////////////////
  SPREADING_FACTOR = constrain(SPREADING_FACTOR, 7, 12);
  TX_POWER = constrain(TX_POWER, -3, 15);

  TelemetryData data;
  /////////////////////////////////////////////////////

  int signal_seq = digitalRead(S_SEQ);
  // Read the data from the MPU sensors and the BMP280 sensors
  readMPU9250Data(MPU9250_ADDR);
  readMPU6050Data(MPU6050_ADDR);
  //readMAGNETO(AK8963_ADDRESS);
  sensorValue = analogRead(sensorPin) - sensorOffset; // Calcul de la valeur renvoyée par le capteur après calibrage
  updateReadings(sensorValue); // Mise à jour des lectures du capteur
  float smoothedSensorValue = getSmoothedSensorValue(); // Obtention de la valeur lissée du capteur
  calculateAndPrintValues(smoothedSensorValue); // Calcul et affichage des valeurs
  last_pressure = corr_pressure;
  readBMPData();

  // Détection de descente
  if ((last_pressure + 0.5) < corr_pressure){
    digitalWrite(SIGNAL, HIGH); // Signal envoyé au séquenceur
    //digitalWrite(LED, HIGH);
    apogee_sent = true;
  } else {
      digitalWrite(SIGNAL, LOW);
      //digitalWrite(LED, LOW);
      apogee_sent = false;
  }
  
  // Réception signal du séquenceur
  if (signal_seq == HIGH) {
    signal_seq_recu = true;
  } else {
    signal_seq_recu = false;
  }

  // Read Data from the GPS
  readGPS();

  /////////////////////////////////////////////////////
  // Serialize data into a string
  String serializedData = String(corr_ax) + "," + String(corr_ay) + "," + String(corr_az) + ","
                        + String(corr_gx) + "," + String(corr_gy) + "," + String(corr_gz) + ","
                        + String(lat*1000000) + "," + String(lng*1000000) + ","  
                        + String(temperature) + "," + String(corr_pressure);

  // Send data via LoRa
  LoRa.beginPacket();
  LoRa.print(serializedData);
  LoRa.endPacket();
  /////////////////////////////////////////////////////

  SDdata = SD.open("DATA.txt", O_WRITE | O_APPEND);;
  if(SDdata) {
    SDdata.print(millis()-t0);
    SDdata.print(',');
    SDdata.print(signal_seq_recu);
    SDdata.print(',');
    SDdata.print(apogee_sent);
    SDdata.print(',');
    SDdata.print(corr_ax);
    SDdata.print(',');
    SDdata.print(corr_ay);
    SDdata.print(',');
    SDdata.print(corr_az);
    SDdata.print(',');
    SDdata.print(corr_gx);
    SDdata.print(',');
    SDdata.print(corr_gy);
    SDdata.print(',');
    SDdata.print(corr_gz);
    SDdata.print(',');
    SDdata.print(corr_az_16g);
    SDdata.print(',');
    SDdata.print(temperature);
    SDdata.print(',');
    SDdata.print(corr_pressure);
    SDdata.print(',');
    SDdata.print(lat, 10);
    SDdata.print(',');
    SDdata.println(lng, 10);
    SDdata.close();
  }


  // VARIABLES SD :
  // millis()
  // corr_ax, corr_ay, corr_az, corr_gx, corr_gy, corr_gz, mx, my, mz;
  // corr_az_16g;
  // lat, lng
  // signal_seq_recu, apogee_sent
  // temperature, corr_pressure
  // voltage, kpa, speed

  // VARIABLES TELEM :
  // millis()
  // corr_ax, corr_ay, corr_az, corr_gx, corr_gy, corr_gz, mx, my, mz;
  // lat, lng
  // corr_pressure
  // speed

  /////////////////////////////////////////////////////
  // Important !
  //delay(DELAY_BETWEEN_TRANSMISSIONS);
  /////////////////////////////////////////////////////
}

// ==================================================== FONCTIONS

void initializeMPU(uint8_t address) {
  // Wake up the MPU sensor
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU6050)
  Wire.endTransmission(true);

  if (address == MPU9250_ADDR) {
  // Set the gyro configuration register
  Wire.beginTransmission(address);
  Wire.write(0x1B);  // Gyro configuration register
  Wire.write(2 << 3); // Set the range to 1000 degrees/sec: 0b10 (2) shifted to bits 4 and 3 = 2 << 3 = 00010000
  Wire.endTransmission(true);
  } else {
    // Set the accelerometer configuration register
    Wire.beginTransmission(address);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(3 << 3); // Set the range to ±16g: 0b11 (3) shifted left 3 bits
    Wire.endTransmission(true);
    }
}

/*void initializeMAGNETO(uint8_t address) {
  // Configure the magnetometer (AK8963)
  Wire.beginTransmission(address);
  Wire.write(0x0A); // Control register 1
  Wire.write(0x16); // Set to 16-bit output and 100Hz continuous measurement mode
  Wire.endTransmission(true);
}*/

void readMPU9250Data(uint8_t address) {
  // Read the raw data from the MPU sensor
  int16_t ax, ay, az, gx, gy, gz;

  Wire.beginTransmission(address);
  Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);  // Request a total of 14 registers

  ax = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H and ACCEL_XOUT_L
  ay = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H and ACCEL_YOUT_L
  az = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H and ACCEL_ZOUT_L
  Wire.read();  // Skip TEMP_OUT_H
  Wire.read();  // Skip TEMP_OUT_L
  gx = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H and GYRO_XOUT_L
  gy = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H and GYRO_YOUT_L
  gz = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H and GYRO_ZOUT_L

  corr_ax = ax / accelScaleFactor;
  corr_ay = az / accelScaleFactor;
  corr_az = ay / accelScaleFactor;
  corr_gx = gx / gyroScaleFactor-2.40;
  corr_gy = gz / gyroScaleFactor+2.40;
  corr_gz = gy / gyroScaleFactor+1.00; // Invert Z and Y axis
  
  // Print the acceleration and gyro data for the MPU sensor
  /*Serial.print("MPU9250 - Acceleration: ");
  Serial.print("X = "); Serial.print(ax / accelScaleFactor);
  Serial.print(", Y = "); Serial.print(az / accelScaleFactor);
  Serial.print(", Z = "); Serial.println(ay / accelScaleFactor);

  Serial.print("MPU9250 - Gyro: ");
  Serial.print("X = "); Serial.print(gx / gyroScaleFactor-2.40);
  Serial.print(", Y = "); Serial.print(gz / gyroScaleFactor+2.40);
  Serial.print(", Z = "); Serial.println(gy / gyroScaleFactor+1.00);*/
}

void readMPU6050Data(uint8_t address) {
  // Read the raw data from the MPU sensor
  int16_t ay_16g;

  Wire.beginTransmission(address);
  Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 4, true);  // Request a total of 4 registers

  Wire.read();  // Skip ax_OUT_H
  Wire.read();  // Skip ax_OUT_L
  ay_16g = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H and ACCEL_YOUT_L

  corr_az_16g = ay_16g / 2048.0;

  //Serial.print("MPU6050 - Acceleration: ");
  //Serial.print("Z_16g = "); Serial.println(ay_16g / 2048.0); // 2048.0 scale factor for 16g
  
}

/*void readMAGNETO(uint8_t address) {
  // Read magnetometer data
  Wire.beginTransmission(address);
  Wire.write(0x03); // Starting register for magnetometer data
  Wire.endTransmission(false);
  Wire.requestFrom(address, 7, true); // Request 7 registers

  int16_t mx_raw, my_raw, mz_raw;
  mx_raw = Wire.read() | Wire.read() << 8; // MAG_XOUT_L and MAG_XOUT_H
  my_raw = Wire.read() | Wire.read() << 8; // MAG_YOUT_L and MAG_YOUT_H
  mz_raw = Wire.read() | Wire.read() << 8; // MAG_ZOUT_L and MAG_ZOUT_H

  Wire.read(); // Skip ST2 register

  Serial.print(" | Mag raw: ");
  Serial.print(mx_raw);
  Serial.print(", ");
  Serial.print(my_raw);
  Serial.print(", ");
  Serial.print(mz_raw);
  Serial.println();
}*/

void readBMPData() {
  // Read the temperature and pressure from the BMP280 sensor
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();  // Pressure in Pa

  // Moyenne glissante
  TotalBufferCirculaireSignal = TotalBufferCirculaireSignal + pressure - HistoriqueSignal[Plus_ancienSignal];
  HistoriqueSignal[Plus_ancienSignal] = pressure;
  Plus_ancienSignal ++;
  if(Plus_ancienSignal == N) Plus_ancienSignal = 0;
  corr_pressure = (TotalBufferCirculaireSignal/N);

  // Print the temperature and pressure data for the BMP280 sensor
  /*Serial.print("BMP280 - Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(corr_pressure);
  Serial.println(" Pa");*/
  //Serial.println(corr_pressure);
}

void blinkLED() {
  if(millis() % 150<75){
    digitalWrite(LED, LOW);
  }
  else{
    digitalWrite(LED, HIGH);
  }  
}

// Fonctions Pitot
int adjustOffset(int sensorPin, int sensorOffset) { // Fonction "adjustOffset", permettant de calibrer le capteur pitot 
  Serial.println("Commencement de l'etalonnage"); // Affichage pour le développement 
  while (true) {
    sensorValue = analogRead(sensorPin) - sensorOffset; // Lecture de la valeur du capteur, ajustée par l'offset
    voltage = sensorValue * (voltageMax / sensorMax); // Convertion de la valeur analogique en tension
    kpa = ((voltage / kpaRangeTopVoltage) - 0.04) / 0.018; // Convertir la tension en pression (kPa) selon le datasheet du capteur MPX5050DP : https://www.nxp.com/docs/en/data-sheet/MPX5050.pdf

    if (kpa > 0.05) { // si la pression est supérieure à 0.05 kPa...
      sensorOffset++; // ...Augmentation de l'offset 
    } else if (kpa < -0.05) { // si la pression est inférieure à -0.05 kPa...
      sensorOffset--; // ... Diminution de l'offset 
    } else {
      Serial.println("Etalonnage termine"); // Affichage d'un message de fin d'étalonnage
      break;
    }
    Serial.println("Etalonnage en cours..."); // Affichage d'un message pendant l'étalonnage
  }
  return sensorOffset; // Retourner la nouvelle valeur d'offset ajustée
}

void updateReadings(int newValue) { // Fonction pour mettre à jour les lectures du capteur
  total -= readings[currentIndex]; // Soustraction de l'ancienne valeur à la somme totale
  readings[currentIndex] = newValue; // Mise à jour de la valeur actuelle dans le tableau
  total += readings[currentIndex]; // Ajout de la nouvelle valeur à la somme totale
  currentIndex = (currentIndex + 1) % numReadings; // Mise à jour de l'index actuel en bouclant si nécessaire
}

float getSmoothedSensorValue() { // Fonction pour obtenir la valeur moyenne lissée
  return total / numReadings; // Calcul de la moyenne des lectures
}

void calculateAndPrintValues(float smoothedSensorValue) { // Fonction pour calculer et afficher les valeurs
  voltage = smoothedSensorValue * (voltageMax / sensorMax); // Conversion de la valeur du capteur en tension
  kpa = ((voltage / kpaRangeTopVoltage) - 0.04) / 0.018; // Conversion de la tension en pression en kPa

  mach = lastspeed / (20.05 * sqrt(temp)); // Calcul du nombre de Mach
  speed = sqrt((2 * kpa * pow(10,3)) / (rho * (1 + (pow(mach, 2) / 4)))); // Calcul de la vitesse
  lastspeed = speed; // Mise à jour de la dernière vitesse

  Serial.print("Voltage: "); // Affichage de la tension
  Serial.println(voltage, 3); // Affichage de la tension avec 3 décimales

  Serial.print("Kpa: "); // Affichage de la pression en kPa
  Serial.println(kpa, 1); // Affichage de la pression avec 1 décimale

  Serial.print("vitesse :"); // Affichage de la vitesse
  Serial.println(speed); // Affichage de la vitesse
}

void readGPS(){
  if (Serial.available() > 0) {
    gps.encode(Serial.read());
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();

      Serial.print(F("Latitude: "));
      Serial.print(gps.location.lat(), 6); // Latitude in degrees (6 decimal places)
      Serial.print(F(" Longitude: "));
      Serial.println(gps.location.lng(), 6); // Longitude in degrees (6 decimal places)*/
    }
  }

}