
#include "CAN.h"
#include "SerialPrint.h"
#include "mcp2515.h" // autowp-mcp2515

#include <SPI.h>
#include <string>

using namespace std;

bool CAN::printReceive = false;

string ErrToStr(MCP2515::ERROR errType)
{
    switch (errType)
    {
    default:
        return "Unknown error code!";

    case MCP2515::ERROR_OK:
        return "No error";

    case MCP2515::ERROR_FAIL:
        return "Something failed!";

    case MCP2515::ERROR_ALLTXBUSY:
        return "All TX busy!";

    case MCP2515::ERROR_FAILINIT:
        return "Failed to initialize!";

    case MCP2515::ERROR_FAILTX:
        return "TX failure!";

    case MCP2515::ERROR_NOMSG:
        return "No message received";
    }
}

void PrintIfFail(MCP2515::ERROR errType, string error)
{
    if(errType != MCP2515::ERROR_OK)
        PrintSerialMessage("MCP2515 " + error + " " + ErrToStr(errType));
}

CAN::CAN(int chipSelectPin)
{
    mcp2515 = new MCP2515(chipSelectPin);

    PrintIfFail(mcp2515->reset(), "reset failed!");
    PrintIfFail(mcp2515->setBitrate(CAN_SPEED::CAN_500KBPS, CAN_CLOCK::MCP_8MHZ), "setBirtate failed!");
    PrintIfFail(mcp2515->setNormalMode(), "setNormalMode failed!");
}

CAN::~CAN()
{
    delete mcp2515;
}

bool CAN::GetCanData(can_frame &output)
{
    MCP2515::ERROR errFlag = mcp2515->readMessage(&output);

    if(output.can_id == 0 || output.can_dlc == 0 || errFlag == MCP2515::ERROR_NOMSG)
        return false;

    if (errFlag != MCP2515::ERROR_OK)
    {
        PrintIfFail(errFlag, "readMessage failed!");
        return false;
    }

    if (CAN::printReceive)
        PrintSerialMessage("Got CAN MSG ID: 0x" + IntToHex(output.can_id) + " Bytes: " + BytesToString(output.data, output.can_dlc));

    return true;
}

void CAN::Transmit(can_frame canFrame)
{
    Transmit(canFrame.can_id, canFrame.can_dlc, canFrame.data);
}
void CAN::Transmit(int ID, uint8_t length, uint8_t *data)
{
    // PrintSerialMessage("Sending 0x" + IntToHex(ID) + " Size:"+ToString(length) + " Data:" + BytesToString(data, length));
    can_frame tempFrame;
    tempFrame.can_id = ID;
    tempFrame.can_dlc = length;
    for (int i = 0; i < length; i++)
        tempFrame.data[i] = data[i];
    PrintIfFail(mcp2515->sendMessage(&tempFrame), "sendMessage failed!");
}