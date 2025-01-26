#include "tatelem.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//----------------------------------------------------------------------FUNCTIONS------------------------------------------------------------------

/* void convertToByteArray(uint32_t value, byte* byteArray) {
  byteArray[0] = (value >> 24) & 0xFF;
  byteArray[1] = (value >> 16) & 0xFF;
  byteArray[2] = (value >> 8) & 0xFF;
  byteArray[3] = value & 0xFF;
} */


union FloatConverter {
  float f;
  uint32_t i;
};

String convertToByteArray(const bool* bool_table, String byteString) {
    int value = 0;
    for (size_t i = 0; i < 8; i++) {  
        value = (value << 1);  
        if (bool_table[i]) {  
            value++;  
        }
    }
    byteString += (char)value;
    return byteString;
}

String convertToByteArray(float floatValue, String byteString, bool Half_Float) {
  byte dataAsByte[4];
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

    dataAsByte[0] = static_cast<uint8_t>(halfValue >> 8);
    dataAsByte[1] = static_cast<uint8_t>(halfValue & 0xFF);
    byteString += (char)dataAsByte[0];
    byteString += (char)dataAsByte[1];
  }
  else
  {
    uint8_t *p = (uint8_t *)&floatValue;
    dataAsByte[0] = *(p + 3);  // Most significant byte
    dataAsByte[1] = *(p + 2);  // Second byte
    dataAsByte[2] = *(p + 1);  // Third byte
    dataAsByte[3] = *p;        // Least significant byte
    byteString += (char)dataAsByte[0];
    byteString += (char)dataAsByte[1];
    byteString += (char)dataAsByte[2];
    byteString += (char)dataAsByte[3];
  }
  return byteString;
}

String convertToByteArray(uint32_t intValue, String byteString) {
  byte dataAsByte[4];
  uint8_t *p = (uint8_t *)&intValue;
  dataAsByte[0] = *(p + 3);  // Most significant byte
  dataAsByte[1] = *(p + 2);  // Second byte
  dataAsByte[2] = *(p + 1);  // Third byte
  dataAsByte[3] = *p;        // Least significant byte
  byteString += (char)dataAsByte[0];
  byteString += (char)dataAsByte[1];
  byteString += (char)dataAsByte[2];
  byteString += (char)dataAsByte[3];
  return byteString;
} 

String convertToByteArray(int32_t intValue, String byteString) {
  byte dataAsByte[4];
  uint8_t *p = (uint8_t *)&intValue;
  dataAsByte[0] = *(p + 3);  // Most significant byte
  dataAsByte[1] = *(p + 2);  // Second byte
  dataAsByte[2] = *(p + 1);  // Third byte
  dataAsByte[3] = *p;        // Least significant byte
  byteString += (char)dataAsByte[0];
  byteString += (char)dataAsByte[1];
  byteString += (char)dataAsByte[2];
  byteString += (char)dataAsByte[3];

  return byteString;
} 

String convertToByteArray(uint16_t intValue, String byteString) {
  byte dataAsByte[4];
  uint8_t *p = (uint8_t *)&intValue;
  dataAsByte[0] = *(p + 1);  // High byte
  dataAsByte[1] = *p;        // Low byte
  byteString += (char)dataAsByte[0];
  byteString += (char)dataAsByte[1];
  return byteString;
}

String convertToByteArray(int16_t intValue, String byteString) {
  byte dataAsByte[4];
  uint8_t *p = (uint8_t *)&intValue;
  dataAsByte[0] = *(p + 1);  // High byte
  dataAsByte[1] = *p;        // Low byte
  byteString += (char)dataAsByte[0];
  byteString += (char)dataAsByte[1];
  return byteString;
}

/* String convertToByteArray(uint8_t intValue, String byteString) {
  byte dataAsByte[4];
  dataAsByte[0] = intValue;
  byteString += (char)dataAsByte[0];
  return byteString;
} */

String convertToByteArray(int8_t intValue, String byteString) {
  byte dataAsByte[4];
  dataAsByte[0] = intValue;
  byteString += (char)dataAsByte[0];
  return byteString;
}

String convertToByteArray(byte byteValue, String byteString) {
  byte dataAsByte[4];
  dataAsByte[0] = byteValue;  // Store byteValue at the first position in the byteArray
  byteString += (char)dataAsByte[0];
  return byteString;
}

String convertToByteArray(char char_value, String byteString){
  byteString += (char)char_value;
  return byteString;
}



/* void convertToByteArray(double doubleValue, byte *byteArray) {
  uint8_t *p = (uint8_t *)&doubleValue;
  *byteArray++ = *(p + 7);
  *byteArray++ = *(p + 6);
  *byteArray++ = *(p + 5);
  *byteArray++ = *(p + 4);
  *byteArray++ = *(p + 3);
  *byteArray++ = *(p + 2);
  *byteArray++ = *(p + 1);
  *byteArray++ = *p;

} */

/* void sendPacket(byte * packet, byte len) {
  for (byte i = 0; i < len; i++)
  {
    Serial5.write(packet[i]);
  }
} */

