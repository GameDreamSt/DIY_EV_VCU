
#include <Arduino.h>
#include <vector>

void PrintSerialMessage(String outputMsg);
void PrintSerialMessageHEX(String message, int hexData);
void PrintSerialMessage(String message, int intData);

void TickSerialWriter();

String FloatToString(float var, int precision);
String ToString(int var);
String BoolToString(bool var);
String IntToHex(int n);
String BytesToString(std::vector<unsigned char> data);
String BytesToString(unsigned char *data, int length);