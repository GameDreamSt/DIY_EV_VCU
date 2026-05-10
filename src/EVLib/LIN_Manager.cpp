
#include "HardwareSerial.h"
#include "LIN_Manager.h"
#include "LIN_master_HardwareSerial.h"
#include "SerialPrint.h"

#include <string>

using namespace std;

#define LIN_PAUSE 200 // pause [ms] between LIN frames

#define LIN_NETWORK_COUNT 1

LIN_Master_HardwareSerial LIN_NETWORKS[LIN_NETWORK_COUNT] = {LIN_Master_HardwareSerial(Serial2, "Master")};

LIN_Manager::LIN_Manager(int linIndex)
{
    LIN_NET = nullptr;
    debug = false;

    if (linIndex >= LIN_NETWORK_COUNT)
    {
        string str = "Trying to start a LIN network with out of bounds index! {" + ToString(linIndex) + "/" +
                     ToString(LIN_NETWORK_COUNT) + "}";
        PrintSerialMessage(str);
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
    // PrintSerialMessage("LIN: Current state ID: " + ToString((int)LIN_NET->getState()));

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
        PrintSerialMessage("LIN: No messages are registered!");
        return;
    }

    // PrintSerialMessage("LIN: Counter: " + ToString(counter));

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
        //PrintSerialMessage("LIN: Preparing to receive slave response ID: 0x" + IntToHex(msg.ID) + ", size:" + ToString(msg.dataSize));
        LIN_NET->receiveSlaveResponse(LIN_Master_Base::LIN_V2, msg.ID, msg.dataSize);
    }
    else
    {
        /*PrintSerialMessage("LIN: Preparing to send master request ID: 0x" + IntToHex(msg.ID) +
        ", size:" + ToString(msg.dataSize) + ", data: " + BytesToString(&msg.data[0], 8));*/
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

    string debugOut = "LIN:";

    if (canDebug)
    {
        debugOut += string(LIN_NET->nameLIN) + ", ";

        if (Type == LIN_Master_Base::MASTER_REQUEST)
            debugOut += "request";
        else
            debugOut += "response";

        debugOut += ", ID=0x" + IntToHex(Id);

        if (hasError)
        {
            debugOut += ", err=0x" + IntToHex(error);

            switch (error)
            {
            default:
                debugOut += " no error";
                break;
            case LIN_Master_Base::ERROR_STATE:
                debugOut += " error in LIN state machine";
                break;
            case LIN_Master_Base::ERROR_ECHO:
                debugOut += " error reading response echo";
                break;
            case LIN_Master_Base::ERROR_TIMEOUT:
                debugOut += " frame timeout error";
                break;
            case LIN_Master_Base::ERROR_CHK:
                debugOut += " LIN checksum error";
                break;
            case LIN_Master_Base::ERROR_MISC:
                debugOut += " misc error, should not occur";
                break;
            }
        }
        else
        {
            debugOut += ", data=";
            for (uint8_t i = 0; (i < NumData); i++)
            {
                debugOut += "0x" + IntToHex((int)Data[i]) + " ";
            }
            PrintSerialMessage(debugOut);
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
            PrintSerialMessage("LIN: Received a message, but the message ID isn't registered! " + IntToHex(Id));
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
        PrintSerialMessage("LIN: setting NEW request for ID:" + IntToHex(ID) + ", of size:" + ToString(dataSize));

        LIN_Message newMsg;
        newMsg.ID = ID;
        newMsg.dataSize = dataSize;
        memcpy(newMsg.data, data, 8);
        SetData(newMsg);
        return;
    }

    //PrintSerialMessage("LIN: setting request for ID:" + IntToHex(ID) + ", of size:" + ToString(dataSize));

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
        PrintSerialMessage("LIN: setting NEW receiver for ID:" + IntToHex(ID) + ", of size:" + ToString(dataSize));

        LIN_Message newMsg;
        newMsg.ID = ID;
        newMsg.dataSize = dataSize;
        newMsg.isReceiver = true;
        SetData(newMsg);
        return;
    }

    //PrintSerialMessage("LIN: setting receiver for ID:" + IntToHex(ID) + ", of size:" + ToString(dataSize));

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