
#include <stdint.h>

class HardwareSerial;
class LIN_Master_HardwareSerial;

#define LIN_MESSAGE_COUNT 8

struct LIN_Message
{
    uint8_t ID;
    uint8_t dataSize;
    uint8_t data[8];
    bool isReceiver;
    bool dataValid; // Received data at least once

    LIN_Message()
    {
        Clear();
    }

    void Clear()
    {
        ID = 0;
        dataSize = 0;
        for(int i = 0; i < 8; i++)
        {
            data[i] = 0;
        }
        isReceiver = false;
        dataValid = false;
    }
};

class LIN_Manager
{
    private:
    LIN_Master_HardwareSerial* LIN_NET;

    LIN_Message messages[LIN_MESSAGE_COUNT];

    int counter;
    bool debug;
    
    void Read_LIN_Data();
    void Write_LIN_Data();
    LIN_Message FindMessage(uint8_t ID);
    void SetData(LIN_Message data);

    public:
    LIN_Manager() { LIN_NET = nullptr; }; 
    LIN_Manager(int linIndex);
    void Tick();

    void SetRequest(uint8_t ID, uint8_t dataSize, uint8_t data[8]);
    void SetReceiver(uint8_t ID, uint8_t dataSize);
    void RemoveMessage(uint8_t ID);
    bool TryGetData(uint8_t ID, LIN_Message &msg);
    bool ToggleDebugMode();
};