#include <Arduino.h>
//----------------Cablage------------------
#define PIN_Declenchement  7
#define Pin_LED 8
#define CS_SD 3
#define PIN_Moteur 5
#define Vbatmot 14
#define Vbatexp 15

#define Valeurbatexp 683  //11,1 V : batterie 3s 764=12,4 V
#define Valeurbatmot 783 //14,8 V : batterie 4s 874=16.5 V

//----------------Flash-------------------

//#include<SPIMemory.h>
//#define CS_Flash 2
//uint32_t strAddr = 10;
//SPIFlash flash(CS_Flash);

//----------------MPU+I2C------------------

#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>        //Il faut aussi trouver le yaw pitch roll

MPU9250_DMP imu;

//------------------BME--------------------
#include <BMP280_DEV.h>  
#define I2C_ADDRESS_BME 0x76

//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMP280_DEV bmp280;

//-----------------GPS--------------------

#include <TinyGPSPlus.h>

//Default baud of NEO-6M is 9600
TinyGPSPlus gps;

//-------------------Roulis--------------------
#include <Servo.h>
Servo Moteur;
float corr;
bool sec = 0;

//---------------------SD--------------------

#include "SdFat.h"
// File system object.
SdFat sd;

// Log file.
SdFile data;

struct LogData {
  byte time_overflow;
  uint32_t t;
  byte s;
  float p;
  float E1;
  float com1bis;
  uint16_t Voltage_mot;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
  float ax;
  float ay;
  float az;
  float corrGyrRawx;
  float corrGyrRawy;
  float corrGyrRawz;
  float Mxyz[3];
  float Axyz[3];
  float q[4];
  float dmp_qw;
  float dmp_qx;
  float dmp_qy;
  float dmp_qz;
  uint16_t Voltage_exp;
  float BME_temp;
  uint16_t overRun;
};

struct LogGPS {
  uint32_t teensy_time;
  uint8_t  GPS_minute;
  uint8_t  GPS_second;
  uint8_t  GPS_centisecond;
  float GPS_altitude;
  bool GPS_altitudeValid;
  float GPS_lat;
  float GPS_lng;
  uint32_t GPS_age;
  float GPS_deg;
  bool GPS_degValid;
  float GPS_speed;
  bool GPS_speedValid;
  float GPS_satellites;
  bool GPS_satellitesValid;
  float GPS_hdop;
  bool GPS_hdopValid;

};

uint16_t overRun = 0;



//---------------------Telem--------------------

int LoRaCount = 0;

//---------------------RTOS--------------------
#include "ChRt.h"

// Shared data, use volatile to insure correct access.

// Mutex for atomic access to data.
MUTEX_DECL(dataMutex);

volatile byte vol_s;


volatile float vol_GPSlat;
volatile float vol_GPSlng;

volatile unsigned long t_demarrage;
volatile unsigned long t;
byte time_overflow;
volatile float com1 = 0;
volatile float com1bis = 0;
volatile uint16_t com2 = 1500;
volatile float E;
volatile float E1;
volatile float S1 = 0.01;
volatile float q[4] = {1.0, 0.0, 0.0, 0.0};
volatile float commande;
volatile float wind_up = 0;

volatile thread_t * thread_motor;
volatile thread_t * thread_GPS;
volatile thread_t * thread_SD;

// Fifo definitions.

// Size of fifo in records.
const size_t FIFO_SIZE = 128;

// Count of data records in fifo.
SEMAPHORE_DECL(fifoData, 0);

// Count of free buffers in fifo.
SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

// Array of fifo items.
LogData DataFifo[FIFO_SIZE];

SEMAPHORE_DECL(fifoGPS, 0);

// Count of free buffers in fifo.
SEMAPHORE_DECL(fifoGPSSpace, FIFO_SIZE);

// Array of fifo items.
LogGPS GPSFifo[FIFO_SIZE];

#define ExpThreadPeriod 30

#define GPSThreadPeriod 500

//---------------------RTOS--------------------

float pression_memory[10] = {1013.25, 1013.25, 1013.25, 1013.25, 1013.25, 1013.25, 1013.25, 1013.25, 1013.25, 1013.25};

float pression_lisse[5] = {1013.25, 1013.25, 1013.25, 1013.25, 1013.25};

uint32_t tdepart;

int Last = 0;

int Lisse = 0;

//----------------------------------------------------------------------FUNCTIONS------------------------------------------------------------------

void convertToByteArray(uint32_t intValue, byte *byteArray) {
  uint8_t *p = (uint8_t *)&intValue;
  *byteArray++ = *(p + 3);
  *byteArray++ = *(p + 2);
  *byteArray++ = *(p + 1);
  *byteArray++ = *p;
}

void convertToByteArray(int16_t intValue, byte *byteArray) {
  uint8_t *p = (uint8_t *)&intValue;
  *byteArray++ = *(p + 1);
  *byteArray++ = *p;
}

void convertToByteArray(uint16_t intValue, byte *byteArray) {
  uint8_t *p = (uint8_t *)&intValue;
  *byteArray++ = *(p + 1);
  *byteArray++ = *p;
}

void convertToByteArray(uint8_t byteValue, byte *byteArray) {
  *byteArray++ = byteValue;
}
// Union pour faciliter la conversion
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


    // Gestion des valeurs spéciales
    if (exponent == 128) {
      // Infinity ou NaN
      exponent = 31;
      if (fraction != 0) {
        // NaN - on peut changer les bits de la fraction pour obtenir différentes valeurs NaN
        fraction >>= 13;
        fraction |= 0x2000;
      }
    } else if (exponent == -127) {
      // Zéro ou dénormalisé
      exponent = 0;
      fraction >>= 13;
    } else {
      // Normalisé
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

String byteArrayToString(byte* byteArray, int len) {
  String result = "";
  for (int i = 0; i < len; i++) {
    //Serial.println(byteArray[i]);
    result += (char)byteArray[i];
  }
  return result;
}

void byteArrayToWrite(byte* byteArray, int len) {

  for (int i = 0; i < len; i++) {
    //Serial.println(byteArray[i]);
    //Serial.write(byteArray[i]);
  }

}

void bytesToChars(byte* bytes, char* chars, int len) {
  chars = new char[len];
  for (int i = 0; i < len; i++) {
    *chars++ = (char) bytes[i];
  }
}

void mergeByteArrays(byte* array1, byte* array2, byte* result, int len1, int len2) {
  for (int i = 0; i < len1; i++) {
    result[i] = array1[i];
  }
  for (int j = 0; j < len2; j++) {
    result[len1 + j] = array2[j];
  }
}

void sendPacket(byte * packet, byte len) {
  for (byte i = 0; i < len; i++)
  {
    Serial5.write(packet[i]);
  }
}
bool declenche = 0;

//----------------------------------------------------------------------RTOS TASKS------------------------------------------------------------------
//----------------------------------------------------------------------RTOS TASKS------------------------------------------------------------------
//----------------------------------------------------------------------RTOS TASKS------------------------------------------------------------------

//------------------------------------------------------------------------------
// Thread 1, high priority to read sensor.
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread1, 2048);

static THD_FUNCTION(Thread1, arg) {
  (void)arg;


  systime_t wakeTime = chVTGetSystemTime();

  // Index of record to be filled.
  size_t fifoDataHead = 0;

  while (!chThdShouldTerminateX()) {
    // Sleep until next second.
    wakeTime += TIME_MS2I(ExpThreadPeriod);
    chThdSleepUntil(wakeTime);

    unsigned long t2;
    t2 = t;
    t = micros();

    if (t < t2)
    {
      time_overflow += 1;
    }

    //--------------Declaration-----------------
    char s =56;

    chMtxLock(&dataMutex);

    float GPS_lat = vol_GPSlat;
    float GPS_lng = vol_GPSlng;

    // Unlock data access.
    chMtxUnlock(&dataMutex);


    //----------Données Séq----------------

    if (Serial1.available()) {
      s = Serial1.read();
      //Serial.println(s);
      Serial1.flush();
    }

    //-----------------Données------------------

    float p;

    float BME_temp;
    
    float altitude;    
    
    bmp280.getMeasurements(BME_temp, p, altitude);
    
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    int short Voltage_exp = analogRead(Vbatexp);
    int short Voltage_mot = analogRead(Vbatmot);    

    //direct à modifier en fonction de la fusée !!!!

    float Mxyz[3];
    float Axyz[3];

    Axyz[0] = imu.ax-0;         //x
    Axyz[1] = -imu.az-0;       //y
    Axyz[2] = imu.ay+0;         //z

    Mxyz[0] = imu.mx-0;        //x
    Mxyz[1] = -imu.mz-0;       //y
    Mxyz[2] = imu.my-0;         //z

    float offsetgyrox = 116;
    float offsetgyroz = -28;
    float offsetgyroy = -132;

    float corrGyrRawx = (imu.gx - offsetgyrox); //Axe x fusée = Axe capteur x
    float corrGyrRawz = (imu.gy - offsetgyroy); //Axe z fusée = Axe capteur y
    float corrGyrRawy = -(imu.gz - offsetgyroz); //Axe y fusée = -Axe capteur z

    
    //Serial.println(s);

    // Integrate rate of change of quaternion, q cross gyro term
    float deltat_half = 0.015;
    float qa = q[0];
    float qb = q[1];
    float qc = q[2];

    float corrGyrRawxq = corrGyrRawx * 2000.0 / 32768.0*0.03;
    float corrGyrRawyq = corrGyrRawx * 2000.0 / 32768.0*0.03;
    float corrGyrRawzq = corrGyrRawx * 2000.0 / 32768.0*0.03;

    q[0] += ((-qb*corrGyrRawxq)-(qc*corrGyrRawyq)-(q[3]*corrGyrRawzq));
    q[1] += (qa*corrGyrRawxq+qc*corrGyrRawzq-q[3]*corrGyrRawyq);
    q[2] += (qa*corrGyrRawyq-qb*corrGyrRawzq+q[3]*corrGyrRawxq);
    q[3] += (qa*corrGyrRawzq+qb*corrGyrRawyq-qc*corrGyrRawxq);
    
    // renormalise quaternion
    float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    
    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
    q[3] = q[3] * recipNorm;

    long dmp_qw;
    long dmp_qx;
    long dmp_qy;
    long dmp_qz;

    if ( imu.dmpUpdateFifo() == INV_SUCCESS) {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      dmp_qw = imu.qw;
      dmp_qx = imu.qx;
      dmp_qy = imu.qy;
      dmp_qz = imu.qz;
    }

    //--------------sauvegarde des données-------------------

    //temps micro (71 minutes, Etat, Pression, Temp, qDMP0, qDMP1, qDMP2, qDMP3, gx,gy,gz,magx,magy,magz,ax,ay,az,q[0],q[1],q[2],q[3],rodre_mot,sortie,sortie1,sortie2,entrée2,volt_mot,volt_exp

    if (chSemWaitTimeout(&fifoSpace, TIME_IMMEDIATE) != MSG_OK) overRun++; // Fifo full, indicate missed point.

    LogData* record = &DataFifo[fifoDataHead];

    record->time_overflow = time_overflow;
    record->t = t;
    record->s = s;
    record->p = p;
    record->com1bis = com1bis;
    record->E1 = E1;
    record->Voltage_mot = Voltage_mot;
    record->gx = imu.gx;
    record->gy = -imu.gz;
    record->gz = imu.gy;
    record->mx = imu.mx;
    record->my = -imu.mz;
    record->mz = imu.my;
    record->ax = imu.ax;
    record->ay = -imu.az;
    record->az = imu.ay;
    record->q[0] = q[0];
    record->q[1] = q[1];
    record->q[2] = q[2];
    record->q[3] = q[3];
    record->corrGyrRawx = corrGyrRawx;
    record->corrGyrRawy = corrGyrRawy;
    record->corrGyrRawz = corrGyrRawz;
    record->Mxyz[0] = Mxyz[0];
    record->Mxyz[1] = Mxyz[1];
    record->Mxyz[2] = Mxyz[2];
    record->Axyz[0] = Axyz[0];
    record->Axyz[1] = Axyz[1];
    record->Axyz[2] = Axyz[2];
    record->dmp_qw = dmp_qw;
    record->dmp_qx = dmp_qx;
    record->dmp_qy = dmp_qy;
    record->dmp_qz = dmp_qz;
    record->BME_temp = BME_temp;
    record->Voltage_exp = Voltage_exp;
    record->overRun = overRun;

    // Signal new data.
    chSemSignal(&fifoData);



    // Advance FIFO index.
    fifoDataHead = fifoDataHead < (FIFO_SIZE - 1) ? fifoDataHead + 1 : 0;


    
    //------------------------Envois télém-------------------
//    byte error;
//    Wire.beginTransmission(0x76);
//    error = Wire.endTransmission();
//    Serial.println(LoRaCount);
    if (LoRaCount == 0 && s != '7') {
      //temps micro,Etat,Pression,GPS_lat,GPS_lng,Temp,gx,gy,gz,magx,magy,magz,ax,ay,az,q1,q2,q3,q1inty,intz,rodre_mot,sortie,sortie1,sortie2,entrée2,volt_mot,volt_exp
      //byte globalByteArray[54] = {start_of_frame};
      byte globalByteArray[60];
      int len = 0;
  
      byte end_of_frame1 = 0xD5;
  
      byte end_of_frame2 = 0xAE;
  
      byte end_of_frame3 = 0x77;

      byte end_of_frame4 = 0xBD;
      
      byte end_of_frame5 = 0x42;

      byte end_of_frame6 = 0xCD;
      
      convertToByteArray(t, &globalByteArray[len]);
      len += 4;
  
      convertToByteArray(p, &globalByteArray[len],1);
      len += 2;
  
      convertToByteArray((byte)s, &globalByteArray[len]);
      len += 1;
  
      convertToByteArray(GPS_lat, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(GPS_lng, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(com1bis, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(E1, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Voltage_mot, &globalByteArray[len]);
      len += 2;
  
      convertToByteArray(corrGyrRawx, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(corrGyrRawy, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(corrGyrRawz, &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Mxyz[0], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Mxyz[1], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Mxyz[2], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Axyz[0], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Axyz[1], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(Axyz[2], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(q[0], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(q[1], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(q[2], &globalByteArray[len], 1);
      len += 2;
  
      convertToByteArray(q[3], &globalByteArray[len], 1);
      len += 2;
      
      convertToByteArray(Voltage_exp, &globalByteArray[len]);
      len += 2;
  
      convertToByteArray(BME_temp, &globalByteArray[len],1);
      len += 2;
  
      convertToByteArray(end_of_frame1, &globalByteArray[len]);
      len += 1;
  
      convertToByteArray(end_of_frame2, &globalByteArray[len]);
      len += 1;
  
      convertToByteArray(end_of_frame3, &globalByteArray[len]);
      len += 1;

      convertToByteArray(end_of_frame4, &globalByteArray[len]);
      len += 1;
  
      convertToByteArray(end_of_frame5, &globalByteArray[len]);
      len += 1;
  
      convertToByteArray(end_of_frame6, &globalByteArray[len]);
      len += 1;
  
      String string_to_send = byteArrayToString(&globalByteArray[0], len);
      
      //bytesToChars(&globalByteArray[0], chars_to_send, len);
      //Serial.print(string_to_send);
      //Serial.println(len);
  
      Wire.beginTransmission(2); // transmit to device
      for(int k = 0; k < 32; k++){
      Wire.write(globalByteArray[k]);        // sends
      delayMicroseconds(1);
      }
      Wire.endTransmission();    // stop   transmitting
      Wire.beginTransmission(2); // transmit to device
      for(int k = 32; k < len; k++){
      Wire.write(globalByteArray[k]);        // sends
      delayMicroseconds(1);
      }
      Wire.endTransmission();    // stop   transmitting
    }
    LoRaCount = (LoRaCount+1)%2;
    //-----------Algo_contrôle_de_roulis-------------


    //Serial.println(s);
    //Serial.println("Activation roulis");

//Serial.println((byte)s);
    commande = 380; //1 tour 15.5 s test
    digitalWrite(Pin_LED, HIGH);


    float pwmmin = 1000;    //vitesse de commande max du moteur sens inverse
    float pwmmax = 2000;   //vitesse de commande max du moteur sens normal

    float constante_S1 = 1;
    float constante_E = 0.023667;
    float constante_E1 = -0.0225451842;

   
    if (s == '2') {
      E1 = E;
      E = commande + corrGyrRawz;
      com1bis = com1bis + E * constante_E + E1 * (constante_E1 + wind_up * 0.00003);
      com1 = com1bis * 874.0 / Voltage_mot;
      if (com1 > 500)
      {
        wind_up = 500 * Voltage_mot / 874 - com1bis;
        com1 = 500;
      }
      else if (com1 < -500)
      {
        wind_up = -500 * Voltage_mot / 874 - com1bis;
        com1 = -500;
      }
      else
      {
        wind_up = 0;
      }
      com2 = 1500 + com1;
      if(com2 < 1580 & com2 > 1420)
      {
        com2 = 1500;
      }
    }
    else if(s == '3' | s == '4' | s == '5') {}
    else if(s == '6') 
    {
       if( (com2 < 1540 & com2 > 1500) | (com2 > 1440 & com2 < 1500) )
       {
        com2 = 1500;
       }
       else if( com2 > 1500)
       {
        com2 = com2 - 1;
       }
       else if( com2 < 1500)
       {
        com2 = com2 + 1;
       }
    }
    else {
      com2 = 1500;
      com1 = 0;
      com1bis = 0;
    }


    Moteur.writeMicroseconds(com2);

    //---------------Algo ouverture para-----------------

      //Serial.println("activation mode ouverture para");
      float seuil = 0.3;


      float average_pressure = p;
      for (int i = 4; i >= 0; i--)
      {
        average_pressure = average_pressure + pression_lisse[i];
      }
      average_pressure = average_pressure / 6 ;


      if (Lisse == 4)
      {
        Lisse = 0;
        pression_lisse[Lisse] = average_pressure;
      }
      else
      {
        Lisse++;
        pression_lisse[Lisse] = average_pressure;
        
      }
     

      if (Last == 9)
      {
        Last = 0;
        pression_memory[Last] = pression_lisse[Lisse];
      }
      else
      {
        Last++;
        pression_memory[Last] = pression_lisse[Lisse];;
      }


      float max1 = pression_memory[Last];
      for (int j = 0; j < 10; j++)
      {
        //Serial.println(pression_memory[j]);
        if (max1 < pression_memory[j])
        {
          max1 = pression_memory[j];
        }
      }
      
      float min1 = pression_memory[Last];
      for (int j = 0; j < 10; j++)
      {
        if (min1 > pression_memory[j])
        {
          min1 = pression_memory[j];
        }
      }


      if ( (max1 - min1 < seuil) and s == '3' ) {
        digitalWrite(PIN_Declenchement, LOW);
      }



  }
}

//------------------------------------------------------------------------------
// Thread 2, high priority to read sensor.
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread2, 512);

static THD_FUNCTION(Thread2, arg) {
  (void)arg;

  systime_t wakeTime = chVTGetSystemTime();

  // Index of record to be filled.
  size_t fifoGPSHead = 0;

  while (!chThdShouldTerminateX()) {
    // Sleep until next second.
    wakeTime += TIME_MS2I(GPSThreadPeriod);
    chThdSleepUntil(wakeTime);


    //--------------GPS---------------------

float Offset_lat=43.2184361; //N+ 10^-7 ok précision
float Offset_lng=-0.04733333; //Ouest donc négatig
    while (Serial5.available() > 0) {
      if (gps.encode(Serial5.read())) {
        
        if (gps.location.isValid()) {

          if (chSemWaitTimeout(&fifoGPSSpace, TIME_IMMEDIATE) != MSG_OK) overRun++; // Fifo full, indicate missed point.

          LogGPS* GPS = &GPSFifo[fifoGPSHead];

          // get the byte data from the GPS
          GPS->teensy_time = millis();
          GPS->GPS_minute = gps.time.minute();
          GPS->GPS_second = gps.time.second();
          GPS->GPS_centisecond = gps.time.centisecond();
          GPS->GPS_lat = (gps.location.lat() - Offset_lat)*111120; //nombre de mêtres dans 60 miles nautiques (1°)
          GPS->GPS_lng = (gps.location.lng() - Offset_lng)*111120; //nombre de mêtres dans 60 miles nautiques (1°)
          GPS->GPS_age = gps.location.age();
          GPS->GPS_speed = gps.speed.kmph();
          GPS->GPS_speedValid = gps.speed.isValid();
          GPS->GPS_deg = gps.course.deg();
          GPS->GPS_degValid = gps.course.isValid();
          GPS->GPS_altitude = gps.altitude.meters();
          GPS->GPS_altitudeValid = gps.altitude.isValid();
          GPS->GPS_satellites = gps.satellites.value();
          GPS->GPS_satellitesValid = gps.satellites.isValid();
          GPS->GPS_hdop = gps.hdop.hdop();
          GPS->GPS_hdopValid = gps.hdop.isValid();

          // Signal new data.
          chSemSignal(&fifoGPS);


          // Advance FIFO index.
          fifoGPSHead = fifoGPSHead < (FIFO_SIZE - 1) ? fifoGPSHead + 1 : 0;

          chMtxLock(&dataMutex);

          vol_GPSlat = GPS->GPS_lat;
          vol_GPSlng = GPS->GPS_lng;

          // Unlock data access.
          chMtxUnlock(&dataMutex);
        }
        
      }
      
    }

  }

}




void chSetup() {
  // Start analgRead() thread.

  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 3, Thread1, NULL);

  // Start print thread.
  chThdCreateStatic(waThread2, sizeof(waThread2),
                    NORMALPRIO + 1, Thread2, NULL);


}


//----------------------------------------------------------------------SETUP------------------------------------------------------------------

void setup() {
  //----------------Cablage------------------

  pinMode(Pin_LED, OUTPUT);
  pinMode(Vbatmot, INPUT);
  pinMode(Vbatexp, INPUT);

  //--------------------------Serial Séq----------------

  Serial1.begin(9600);
  if (!Serial1) {
    Serial.print("Problème connexion Sérial Séquenceur");
    while (1) {
      digitalWrite(Pin_LED, HIGH);
      delay(100);
      digitalWrite(Pin_LED, LOW);
      delay(100);
    }
  }

  //-------------------Batteries_test------------------

  //Test batterie expérience
  Serial.println(analogRead(Vbatexp));
  if (analogRead(Vbatexp) < Valeurbatexp) {
    Serial.println("Batterie Expérience faible !");
    Serial.println((float)analogRead(Vbatexp) / 1024 * 3.3 / 2.6 * 13);
//      while (1) {
//        digitalWrite(Pin_LED, HIGH);
//        delay(100);
//        digitalWrite(Pin_LED, LOW);
//        delay(100);
//      }
  }
  Serial.println("Batterie Expérience ok !");
  //Test batterie moteur
  Serial.println(analogRead(Vbatmot));
  if (analogRead(Vbatmot) < Valeurbatmot) {
    Serial.println("Batterie Moteur faible !");
    Serial.println((float)analogRead(Vbatmot) / 1024 * 3.3 / 3 * 17.5);
//      while (1) {
//        digitalWrite(Pin_LED, HIGH);
//        delay(100);
//        digitalWrite(Pin_LED, LOW);
//        delay(100);
//      }
  }
  Serial.println("Batterie Moteur ok !");

  //-----------------------TELEM--------------------

  Wire.begin();
 // Wire.setClock(400000);

  if (!bmp280.begin(0x76))
  {
      while (1) {
        digitalWrite(Pin_LED, HIGH);
        delay(100);
        digitalWrite(Pin_LED, LOW);
        delay(100);
      }
  }
  // Default initialisation, place the BMP280 into SLEEP_MODE 
  //bmp280.setPresOversampling(OVERSAMPLING_X1);    // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  bmp280.setIIRFilter(IIR_FILTER_OFF);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_62MS);     // Set the standby time to 2 seconds
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE  
  //-----------------Flash+sauvegarde données précédentes---------------------

  //  flash.begin(); //probleme connexion flash

  //  if (!flash.begin()) {
  //    Serial.println("Echec connexion Flash");
  //    while (1) {
  //      digitalWrite(Pin_LED, HIGH);
  //      delay(100);
  //      digitalWrite(Pin_LED, LOW);
  //      delay(100);
  //    }
  //      int line_break = 0;
  //      uint32_t i=5;
  //      while(line_break < 15)
  //      {
  //        byte Byte = flash.readByte(i);
  //        if( (Byte == 0 || Byte == 255) && line_break == 0)
  //        {
  //          line_break++;
  //        }
  //        else if(Byte != 0 || Byte != 255)
  //        {
  //          donnees.println((char)Byte);
  //          line_break = 0;
  //        }
  //        flash.writeByte(i, Byte = 255);
  //        i++;
  //      }
  //  }

  //-------------------------SD------------------------

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(CS_SD, SD_SCK_MHZ(50))) {
    Serial.println("Echec connextion SD");
    while (1) {
      digitalWrite(Pin_LED, HIGH);
      delay(100);
      digitalWrite(Pin_LED, LOW);
      delay(100);
    }
  }
  //Sauvegarde des données
  data.open("data.txt", O_CREAT | O_WRITE | O_APPEND);

  data.println("start:");

  data.close();

  data.open("GPS.txt", O_CREAT | O_WRITE | O_APPEND);

  data.println("start:");

  data.close();

  Serial.println("Réussite connextion SD");

  //------------------------MPU+I2C--------------------

  byte error;

  Wire.beginTransmission(0x68);
  error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("MPU9250 ne répond pas");
    while (1) {
      digitalWrite(Pin_LED, HIGH);
      delay(100);
      digitalWrite(Pin_LED, LOW);
      delay(100);
    }
  }
  else {
    Serial.println("MPU9250 est connecté");
  }

  imu.begin();

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10); // Set DMP FIFO rate to 10 Hz

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  imu.setGyroFSR(2000); // Set gyro to 2000 dps

  imu.setAccelFSR(16); // Set accel to +/-2g
  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you
     need to select the bandwidth, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
     but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
     MPU9250_BW_WO_DLPF_3600
     MPU9250_BW_WO_DLPF_8800
  */

  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
     MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7

     DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
       0         250            0.97             8
       1         184            2.9              1
       2          92            3.9              1
       3          41            5.9              1
       4          20            9.9              1
       5          10           17.85             1
       6           5           33.48             1
       7        3600            0.17             8

       You achieve lowest noise using level 6
    imu.setLPF(5); // Set LPF corner frequency to 5Hz
  */

  imu.setLPF(20); // Set LPF corner frequency to 20Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(20); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(20); // Set mag rate to 10Hz

  //--------------------------BME---------------------

  Wire.beginTransmission(0x76);
  error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("BME280 ne répond pas");
    while (1) {
      digitalWrite(Pin_LED, HIGH);
      delay(100);
      digitalWrite(Pin_LED, LOW);
      delay(100);
    }
  }
  else {
    Serial.println("BME280 est connecté");
  }

  //--------------------------GPS---------------------  Start the Arduino hardware serial port at 9600 baud
  Serial5.begin(9600);

  byte Nav5_Fix[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0xfe};
  byte NMEA[] = {0xb5, 0x62, 0x06, 0x17 , 0x14, 0x00, 0x00, 0x23, 0x00 , 0x0a , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x5f , 0xa5};
  byte PWR[] = {0xb5, 0x62 , 0x06 , 0x86 , 0x08 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x94 , 0x5a};
  byte Rate[] = {0xb5 , 0x62 , 0x06 , 0x08 , 0x06 , 0x00 , 0xf4 , 0x01 , 0x01 , 0x00 , 0x01 , 0x00 , 0x0b , 0x77};
  byte SBAS[] = {0xb5 , 0x62 , 0x06 , 0x16 , 0x08 , 0x00 , 0x01 , 0x01 , 0x01 , 0x00 , 0x59 , 0x08 , 0x00 , 0x00 , 0x88 , 0x1b};
  byte FXN[] = {0xb5 , 0x62 , 0x06 , 0x0e , 0x24 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xc0 , 0xd4 , 0x01 , 0x00 , 0xc0 , 0xd4 , 0x01 , 0x00 , 0x20 , 0xbf , 0x02 , 0x00 , 0x20 , 0xbf , 0x02 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x40 , 0x77 , 0x1b , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xf6 , 0xfb};

  delay(100);
  
  sendPacket(Nav5_Fix, sizeof(Nav5_Fix));
  delay(100);
  Serial5.flush();

  sendPacket(NMEA, sizeof(NMEA));
  delay(100);
  Serial5.flush();

  sendPacket(PWR, sizeof(PWR));
  delay(100);
  Serial5.flush();

  sendPacket(Rate, sizeof(Rate));
  delay(100);
  Serial5.flush();


  sendPacket(SBAS, sizeof(SBAS));
  delay(100);
  Serial5.flush();
  sendPacket(FXN, sizeof(FXN));
  delay(100);
  Serial5.flush();

  //-------------------Ouverture para---------------

  pinMode(PIN_Declenchement, OUTPUT);
  digitalWrite(PIN_Declenchement, HIGH);

  //---------------Contrôle de roulis---------------
  Moteur.attach(PIN_Moteur);

  Moteur.writeMicroseconds(1500);

  //--------------fin initialisation----------------

  digitalWrite(Pin_LED, HIGH);
  Serial.println("Réussite initialisation");

  chBegin(chSetup);

}

  //----------------------------------------------------------------------LOOP IDLE------------------------------------------------------------------

  // FIFO index for record to be written.
  size_t fifoDataTail = 0;

  // FIFO index for record to be written.
  size_t fifoGPSTail = 0;

  void loop() {

    chSemWait(&fifoData);



    LogData* ExpData = &DataFifo[fifoDataTail];
    if (fifoDataTail >= FIFO_SIZE) fifoDataTail = 0;

    data.open("data.txt", O_WRITE | O_APPEND);
    data.print(ExpData->time_overflow);
    data.print(',');
    data.print(ExpData->t);
    data.print(',');
    data.print(ExpData->s);
    data.print(',');
    data.print(ExpData->p);
    data.print(',');
    data.print(ExpData->E1);
    data.print(',');
    data.print(ExpData->com1bis);
    data.print(',');
    data.print(ExpData->Voltage_mot);
    data.print(',');
    data.print(ExpData->gx);
    data.print(',');
    data.print(ExpData->gy);
    data.print(',');
    data.print(ExpData->gz);
    data.print(',');
    data.print(ExpData->mx);
    data.print(',');
    data.print(ExpData->my);
    data.print(',');
    data.print(ExpData->mz);
    data.print(',');
    data.print(ExpData->ax);
    data.print(',');
    data.print(ExpData->ay);
    data.print(',');
    data.print(ExpData->az);
    data.print(',');
    data.print(ExpData->q[0]);
    data.print(',');
    data.print(ExpData->q[1]);
    data.print(',');
    data.print(ExpData->q[2]);
    data.print(',');
    data.print(ExpData->q[3]);
    data.print(',');
    data.print(ExpData->corrGyrRawx);
    data.print(',');
    data.print(ExpData->corrGyrRawy);
    data.print(',');
    data.print(ExpData->corrGyrRawz);
    data.print(',');
    data.print(ExpData->Mxyz[0]);
    data.print(',');
    data.print(ExpData->Mxyz[1]);
    data.print(',');
    data.print(ExpData->Mxyz[2]);
    data.print(',');
    data.print(ExpData->Axyz[0]);
    data.print(',');
    data.print(ExpData->Axyz[1]);
    data.print(',');
    data.print(ExpData->Axyz[2]);
    data.print(',');
    data.print(ExpData->dmp_qw);
    data.print(',');
    data.print(ExpData->dmp_qx);
    data.print(',');
    data.print(ExpData->dmp_qy);
    data.print(',');
    data.print(ExpData->dmp_qz);
    data.print(',');
    data.print(ExpData->Voltage_exp);
    data.print(',');
    data.print(ExpData->BME_temp);
    data.print(',');
    data.println(ExpData->overRun);
    data.close();

    // Release record.
    chSemSignal(&fifoSpace);

    // Advance FIFO index.
    fifoDataTail = fifoDataTail < (FIFO_SIZE - 1) ? fifoDataTail + 1 : 0;


    if (chSemWaitTimeout(&fifoGPS, TIME_IMMEDIATE) == MSG_OK)
    {
      LogGPS* GPSData = &GPSFifo[fifoGPSTail];
      if (fifoGPSTail >= FIFO_SIZE) fifoGPSTail = 0;
      
      data.open("GPS.txt", O_WRITE | O_APPEND);
      data.print(GPSData->teensy_time);
      data.print(',');
      data.print(GPSData->GPS_minute);
      data.print(',');
      data.print(GPSData->GPS_second);
      data.print(',');
      data.print(GPSData->GPS_centisecond);
      data.print(',');
      data.print(GPSData->GPS_lat);
      data.print(',');
      data.print(GPSData->GPS_lng);
      data.print(',');
      data.print(GPSData->GPS_age);
      data.print(',');
      data.print(GPSData->GPS_deg);
      data.print(',');
      data.print(GPSData->GPS_degValid);
      data.print(',');
      data.print(GPSData->GPS_speed);
      data.print(',');
      data.print(GPSData->GPS_speedValid);
      data.print(',');
      data.print(GPSData->GPS_altitude);
      data.print(',');
      data.print(GPSData->GPS_altitudeValid);
      data.print(',');
      data.print(GPSData->GPS_satellites);
      data.print(',');
      data.print(GPSData->GPS_satellitesValid);
      data.print(',');
      data.print(GPSData->GPS_hdop);
      data.print(',');
      data.println(GPSData->GPS_hdopValid);

      data.close();

      // Release record.
      chSemSignal(&fifoGPSSpace);

      // Advance FIFO index.
      fifoGPSTail = fifoGPSTail < (FIFO_SIZE - 1) ? fifoGPSTail + 1 : 0;
    }


  }
