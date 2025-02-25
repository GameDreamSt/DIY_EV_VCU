
#include "CAN.h"
#include "SerialPrint.h"
#include "mcp2515.h"

#include <SPI.h>

bool CAN::printReceive = false;

CAN::CAN(int chipSelectPin, int interruptPin)
{
    mcp2515 = new MCP2515(chipSelectPin);

    mcp2515->reset();
    mcp2515->setBitrate(CAN_SPEED::CAN_500KBPS, CAN_CLOCK::MCP_8MHZ);
    mcp2515->setNormalMode();
}

CAN::~CAN()
{
    delete mcp2515;
}

String ErrToStr(MCP2515::ERROR errType)
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

bool CAN::GetCanData(can_frame &output)
{
    MCP2515::ERROR errFlag = mcp2515->readMessage(&output);

    if (errFlag != MCP2515::ERROR_OK || output.can_id == 0 || output.can_dlc == 0)
    {
        // PrintSerialMessage("CAN error! " + ErrToStr(errFlag));
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
    mcp2515->sendMessage(&tempFrame);
}