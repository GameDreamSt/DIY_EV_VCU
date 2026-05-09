
#include <stdint.h>

struct can_frame;
class MCP2515;

class CAN
{
  private:
    MCP2515 *mcp2515;

  public:
    static bool printReceive;

    CAN(int chipSelectPin, int interruptPin);
    ~CAN();

    void Transmit(can_frame canFrame);
    void Transmit(int ID, uint8_t length, uint8_t *data);
    bool GetCanData(can_frame &output);
    bool HasData();
};