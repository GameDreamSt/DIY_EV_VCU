
#include "HardwareSerial.h"
#include "LIN_Manager.h"
#include "LIN_master_HardwareSerial.h"

#define LIN_PAUSE 200 // pause [ms] between LIN frames
#define SERIAL_CONSOLE Serial

#define LIN_NETWORK_COUNT 1

LIN_Master_HardwareSerial LIN_NETWORKS[LIN_NETWORK_COUNT] = {LIN_Master_HardwareSerial(Serial1, "Master")};

LIN_Manager::LIN_Manager(int linIndex)
{
    LIN_NET = nullptr;
    debug = false;

    if (linIndex >= LIN_NETWORK_COUNT)
    {
        String str = "Trying to start a LIN network with out of bounds index! {" + String(linIndex) + "/" +
                     String(LIN_NETWORK_COUNT) + "}";
        SERIAL_CONSOLE.println(str);
        return;
    }

    LIN_NET = &LIN_NETWORKS[linIndex];
    LIN_NET->begin(19200);

    for (int i = 0; i < LIN_MESSAGE_COUNT; i++)
    {
        messages[i].Clear();
    }

    counter = 0;
}

void LIN_Manager::Tick()
{
    if (LIN_NET == nullptr)
        return;

    LIN_NET->handler();

    Read_LIN_Data();

    static uint32_t lastLINFrame = 0;
    if (millis() - lastLINFrame > LIN_PAUSE)
    {
        lastLINFrame = millis();
        Write_LIN_Data();
    }
}

void LIN_Manager::Write_LIN_Data()
{
    // SERIAL_CONSOLE.print("LIN: Current state ID: ");
    // SERIAL_CONSOLE.println((int)LIN_NET->getState());

    bool hasMessage = false;
    int firstValidMessageIndex = 0;
    for (int i = 0; i < LIN_MESSAGE_COUNT; i++)
    {
        if (messages[i].ID != 0)
        {
            hasMessage = true;
            firstValidMessageIndex = i;
            break;
        }
    }

    if (!hasMessage)
    {
        SERIAL_CONSOLE.println("LIN: No messages are registered!");
        return;
    }

    // SERIAL_CONSOLE.print("Counter: ");
    // SERIAL_CONSOLE.println(counter);

    LIN_Message msg = messages[counter];

    bool nextMessageFound = false;
    for (int i = counter + 1; i < LIN_MESSAGE_COUNT; i++)
    {
        if (messages[i].ID != 0)
        {
            nextMessageFound = true;
            counter = i;
            break;
        }
    }

    if (!nextMessageFound)
        counter = firstValidMessageIndex;

    if (msg.isReceiver)
    {
        /*SERIAL_CONSOLE.print("LIN: Preparing to receive slave response ID:");
        SERIAL_CONSOLE.print(msg.ID, HEX);
        SERIAL_CONSOLE.print(", size:");
        SERIAL_CONSOLE.println(msg.dataSize);*/
        LIN_NET->receiveSlaveResponse(LIN_Master_Base::LIN_V2, msg.ID, msg.dataSize);
    }
    else
    {
        /*SERIAL_CONSOLE.print("LIN: Preparing to send master request ID:");
        SERIAL_CONSOLE.print(msg.ID, HEX);
        SERIAL_CONSOLE.print(", size:");
        SERIAL_CONSOLE.print(msg.dataSize);
        SERIAL_CONSOLE.print(", data: ");
        for (int i = 0; i < 8; i++)
        {
            SERIAL_CONSOLE.print(msg.data[i], HEX);
            SERIAL_CONSOLE.print(" ");
        }
        SERIAL_CONSOLE.println();*/
        LIN_NET->sendMasterRequest(LIN_Master_Base::LIN_V2, msg.ID, msg.dataSize, msg.data);
    }
}

void LIN_Manager::Read_LIN_Data()
{
    LIN_Master_Base::frame_t Type;
    LIN_Master_Base::error_t error;
    uint8_t Id;
    uint8_t NumData;
    uint8_t Data[8];

    if (LIN_NET->getState() != LIN_Master_Base::STATE_DONE)
        return;

    LIN_NET->getFrame(Type, Id, NumData, Data);
    error = LIN_NET->getError();

    bool hasError = error != LIN_Master_Base::NO_ERROR;
    bool canDebug = debug || hasError;

    if (canDebug)
    {
        SERIAL_CONSOLE.print(LIN_NET->nameLIN);

        if (Type == LIN_Master_Base::MASTER_REQUEST)
            SERIAL_CONSOLE.print(", request, ID=0x");
        else
            SERIAL_CONSOLE.print(", response, ID=0x");

        SERIAL_CONSOLE.print(Id, HEX);

        if (hasError)
        {
            SERIAL_CONSOLE.print(", err=0x");
            SERIAL_CONSOLE.print(error, HEX);

            switch (error)
            {
            default:
                SERIAL_CONSOLE.println(" no error");
                break;
            case LIN_Master_Base::ERROR_STATE:
                SERIAL_CONSOLE.println(" error in LIN state machine");
                break;
            case LIN_Master_Base::ERROR_ECHO:
                SERIAL_CONSOLE.println(" error reading response echo");
                break;
            case LIN_Master_Base::ERROR_TIMEOUT:
                SERIAL_CONSOLE.println(" frame timeout error");
                break;
            case LIN_Master_Base::ERROR_CHK:
                SERIAL_CONSOLE.println(" LIN checksum error");
                break;
            case LIN_Master_Base::ERROR_MISC:
                SERIAL_CONSOLE.println(" misc error, should not occur");
                break;
            }
        }
        else
        {
            SERIAL_CONSOLE.print(", data=");
            for (uint8_t i = 0; (i < NumData); i++)
            {
                SERIAL_CONSOLE.print("0x");
                SERIAL_CONSOLE.print((int)Data[i], HEX);
                SERIAL_CONSOLE.print(" ");
            }
            SERIAL_CONSOLE.println();
        }
    }

    if (!hasError)
    {
        LIN_Message msg = FindMessage(Id);
        if (msg.ID != 0)
        {
            if (msg.isReceiver)
            {
                msg.dataSize = NumData;
                memcpy(msg.data, Data, 8);
            }
            msg.dataValid = true;
            SetData(msg);
        }
        else
        {
            SERIAL_CONSOLE.print("Received a message, but the message ID isn't registered! ");
            SERIAL_CONSOLE.print(Id, HEX);
            SERIAL_CONSOLE.println();
        }
    }

    // reset state machine & error
    LIN_NET->resetStateMachine();
    LIN_NET->resetError();
}

LIN_Message LIN_Manager::FindMessage(uint8_t ID)
{
    for (int i = 0; i < LIN_MESSAGE_COUNT; i++)
    {
        if (messages[i].ID == ID)
            return messages[i];
    }

    return LIN_Message();
}

void LIN_Manager::SetData(LIN_Message msg)
{
    for (int i = 0; i < LIN_MESSAGE_COUNT; i++) // Find existing one
    {
        if (messages[i].ID == msg.ID)
        {
            messages[i] = msg;
            return;
        }
    }

    for (int i = 0; i < LIN_MESSAGE_COUNT; i++) // Find a place for a new one
    {
        if (messages[i].ID == 0)
        {
            messages[i] = msg;
            return;
        }
    }
}

void LIN_Manager::SetRequest(uint8_t ID, uint8_t dataSize, uint8_t data[8])
{
    LIN_Message msg = FindMessage(ID);

    if (msg.ID == 0)
    {
        SERIAL_CONSOLE.print("LIN: setting NEW request for ID:");
        SERIAL_CONSOLE.print(ID, HEX);
        SERIAL_CONSOLE.print(", of size:");
        SERIAL_CONSOLE.println(dataSize);

        LIN_Message newMsg;
        newMsg.ID = ID;
        newMsg.dataSize = dataSize;
        memcpy(newMsg.data, data, 8);
        SetData(newMsg);
        return;
    }

    /*SERIAL_CONSOLE.print("LIN: setting request for ID:");
    SERIAL_CONSOLE.print(ID, HEX);
    SERIAL_CONSOLE.print(", of size:");
    SERIAL_CONSOLE.println(dataSize);*/

    msg.dataSize = dataSize;
    memcpy(msg.data, data, 8);
    msg.dataValid = false;
    msg.isReceiver = false;
    SetData(msg);
}

void LIN_Manager::SetReceiver(uint8_t ID, uint8_t dataSize)
{
    LIN_Message msg = FindMessage(ID);

    if (msg.ID == 0)
    {
        SERIAL_CONSOLE.print("LIN: setting NEW receiver for ID:");
        SERIAL_CONSOLE.print(ID, HEX);
        SERIAL_CONSOLE.print(", of size:");
        SERIAL_CONSOLE.println(dataSize);

        LIN_Message newMsg;
        newMsg.ID = ID;
        newMsg.dataSize = dataSize;
        newMsg.isReceiver = true;
        SetData(newMsg);
        return;
    }

    /*SERIAL_CONSOLE.print("LIN: setting request for ID:");
    SERIAL_CONSOLE.print(ID, HEX);
    SERIAL_CONSOLE.print(", of size:");
    SERIAL_CONSOLE.println(dataSize);*/

    msg.dataSize = dataSize;
    msg.dataValid = false;
    msg.isReceiver = true;
    SetData(msg);
}

void LIN_Manager::RemoveMessage(uint8_t ID)
{
    for (int i = 0; i < LIN_MESSAGE_COUNT; i++)
    {
        if (messages[i].ID == ID)
        {
            messages[i].Clear();
            break;
        }
    }
}

bool LIN_Manager::TryGetData(uint8_t ID, LIN_Message &returnMsg)
{
    returnMsg = FindMessage(ID);
    return returnMsg.ID != 0;
}

bool LIN_Manager::ToggleDebugMode()
{
    debug = !debug;
    return debug;
}