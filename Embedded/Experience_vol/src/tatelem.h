#include <Arduino.h>
#define tatelen_h

String convertToByteArray(const bool* bool_table, String byteArray);

String convertToByteArray(uint32_t intValue, String byteArray);

String convertToByteArray(int32_t intValue, String byteArray);

String convertToByteArray(int16_t intValue, String byteArray);

String convertToByteArray(uint16_t intValue, String byteArray);

/* String convertToByteArray(uint8_t intValue, String byteArray);
 */
String convertToByteArray(int8_t intValue, String byteArray);

String convertToByteArray(float floatValue, String byteArray, bool Half_Float);

String convertToByteArray(byte byteValue, String byteArray);

String convertToByteArray(unsigned char charValue, String byteArray);
String convertToByteArray(char charValue, String byteArray);

String byteArrayToString(byte* byteArray, int len);

String byteArrayToWrite(byte* byteArray, int len);

void bytesToChars(byte* bytes, char* chars, int len);

void mergeByteArrays(byte* array1, byte* array2, byte* result, int len1, int len2);

void sendPacket(byte * packet, byte len);

void sendViaWire(String byteString, int slave_addr);

//void convertToByteArray(double doubleValue, byte *byteArray);