
#include <vector>
#include <string>

void PrintSerialMessage(std::string outputMsg);
void PrintSerialMessageHEX(std::string message, int hexData);
void PrintSerialMessage(std::string message, int intData);

void TickSerialWriter();

std::string FloatToString(float var, int precision);
std::string ToString(int var);
std::string BoolToString(bool var);
std::string IntToHex(int n);
std::string BytesToString(std::vector<unsigned char> data);
std::string BytesToString(unsigned char *data, int length);