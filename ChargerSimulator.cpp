
#include "CAN.h"

#define byte unsigned char

static CAN *can;

void ProvideCanCommunicator(CAN *canCommunicator)
{
    can = canCommunicator;
}

void SendCanMessage(int ID, uint8_t length, byte *data)
{
    can->Transmit(ID, length, data);
}

static uint8_t counter_11a = 0;
static uint8_t counter_1f2 = 0;
static uint8_t counter_1d4 = 0;
static uint8_t counter_1dc = 0;
static uint8_t counter_1db = 0;
static uint8_t counter_55b = 0;
static uint8_t counter_5bc = 0;
// Can message definitions
// 11A 8 bytes including prun & crc
byte message11A_0[8] = {0x01, 0x80, 0x00, 0xaa, 0xc0, 0x00, 0x00, 0x60};
byte message11A_1[8] = {0x01, 0x80, 0x00, 0x55, 0x00, 0x00, 0x01, 0x76};
byte message11A_2[8] = {0x01, 0x80, 0x00, 0x55, 0x40, 0x00, 0x02, 0xbb};
byte message11A_3[8] = {0x01, 0x80, 0x00, 0xaa, 0x80, 0x00, 0x03, 0xad};
// 1d4 8 bytes including prun & crc
byte message1d4_0[8] = {0x6E, 0x6E, 0x00, 0x00, 0x42, 0x06, 0xE0, 0x0D};
byte message1d4_1[8] = {0x6E, 0x6E, 0x00, 0x00, 0xb2, 0x06, 0xE0, 0xC1};
byte message1d4_2[8] = {0x6E, 0x6E, 0x00, 0x00, 0xC2, 0x06, 0xE0, 0xD6};
byte message1d4_3[8] = {0x6E, 0x6E, 0x00, 0x00, 0x02, 0x06, 0xE0, 0xCA};
// 1db 8 bytes including prun & crc
byte message1db_0[8] = {0x01, 0x20, 0xbe, 0xab, 0x2b, 0x00, 0x00, 0x37};
byte message1db_1[8] = {0x01, 0x20, 0xbe, 0xab, 0x2b, 0x00, 0x01, 0xb2};
byte message1db_2[8] = {0x01, 0x20, 0xbe, 0xab, 0x2b, 0x00, 0x02, 0xb8};
byte message1db_3[8] = {0x01, 0x20, 0xbe, 0xab, 0x2b, 0x00, 0x03, 0x3d};
// 1dc 8 bytes including prun & crc, modified from Jack Bauer
byte message1dc_0[8] = {0x6E, 0x0A, 0x05, 0xD5, 0x00, 0x00, 0x00, 0x2b};
byte message1dc_1[8] = {0x6E, 0x0A, 0x05, 0xD5, 0x00, 0x00, 0x01, 0xae};
byte message1dc_2[8] = {0x6E, 0x0A, 0x05, 0xD5, 0x00, 0x00, 0x02, 0xa4};
byte message1dc_3[8] = {0x6E, 0x0A, 0x05, 0xD5, 0x00, 0x00, 0x03, 0x21};
// 1f2 8 bytes including prun & crc
byte message1f2_0[8] = {0x30, 0xA0, 0x20, 0xac, 0x00, 0x3c, 0x00, 0x80};
byte message1f2_1[8] = {0x30, 0xA0, 0x20, 0xac, 0x00, 0x3c, 0x01, 0x05};
byte message1f2_2[8] = {0x30, 0xA0, 0x20, 0xac, 0x00, 0x3c, 0x02, 0x0f};
byte message1f2_3[8] = {0x30, 0xA0, 0x20, 0xac, 0x00, 0x3c, 0x03, 0x8a};
// 50b 7 bytes no prun or crc
byte message50b[7] = {0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00};
// 55b 8 bytes including prun & crc
byte message55b_0[8] = {0xa4, 0x40, 0xAA, 0x00, 0xDF, 0xC0, 0x00, 0x76};
byte message55b_1[8] = {0xa4, 0x40, 0xAA, 0x00, 0xDF, 0xC0, 0x01, 0xf3};
byte message55b_2[8] = {0xa4, 0x40, 0xAA, 0x00, 0xDF, 0xC0, 0x02, 0xf9};
byte message55b_3[8] = {0xa4, 0x40, 0xAA, 0x00, 0xDF, 0xC0, 0x03, 0x7c};
// 59e 8 bytes no prun or crc
byte message59e[8] = {0x00, 0x00, 0x0b, 0x94, 0xc0, 0x00, 0x00, 0x00};
// 5bc 8 bytes no prun or crc
byte message5bc[8] = {0x3d, 0x80, 0xf0, 0x64, 0xb0, 0x01, 0x00, 0x32};

// Send 11A message every 10ms
void VCM_11a_Charging_Message()
{
    switch (counter_11a)
    {
    case 0:
        SendCanMessage(0x11a, 8, message11A_0);
        break;
    case 1:
        SendCanMessage(0x11a, 8, message11A_1);
        break;
    case 2:
        SendCanMessage(0x11a, 8, message11A_2);
        break;
    case 3:
        SendCanMessage(0x11a, 8, message11A_3);
        break;
    }
    counter_11a++;
    if (counter_11a >= 4)
        counter_11a = 0;
}

// Send 1F2 message every 10ms
void VCM_1F2_Charging_Message()
{

    switch (counter_1f2)
    {
    case 0:
        SendCanMessage(0x1f2, 8, message1f2_0);
        break;
    case 1:
        SendCanMessage(0x1f2, 8, message1f2_1);
        break;
    case 2:
        SendCanMessage(0x1f2, 8, message1f2_2);
        break;
    case 3:
        SendCanMessage(0x1f2, 8, message1f2_3);
        break;
    }
    counter_1f2++;
    if (counter_1f2 >= 4)
        counter_1f2 = 0;
}

// Send 1d4 message every 10ms
void VCM_1d4_Charging_Message()
{

    switch (counter_1d4)
    {
    case 0:
        SendCanMessage(0x1d4, 8, message1d4_0);
        break;
    case 1:
        SendCanMessage(0x1d4, 8, message1d4_1);
        break;
    case 2:
        SendCanMessage(0x1d4, 8, message1d4_2);
        break;
    case 3:
        SendCanMessage(0x1d4, 8, message1d4_3);
        break;
    }
    counter_1d4++;
    if (counter_1d4 >= 4)
        counter_1d4 = 0;
}

// Send 1dc message every 10ms
void VCM_1DC_Charging_Message()
{

    switch (counter_1dc)
    {
    case 0:
        SendCanMessage(0x1dc, 8, message1dc_0);
        break;
    case 1:
        SendCanMessage(0x1dc, 8, message1dc_1);
        break;
    case 2:
        SendCanMessage(0x1dc, 8, message1dc_2);
        break;
    case 3:
        SendCanMessage(0x1dc, 8, message1dc_3);
        break;
    }
    counter_1dc++;
    if (counter_1dc >= 4)
        counter_1dc = 0;
}

// Send 1db message every 10ms
void HVB_1DB_Charging_Message()
{

    switch (counter_1db)
    {
    case 0:
        SendCanMessage(0x1db, 8, message1db_0);
        break;
    case 1:
        SendCanMessage(0x1db, 8, message1db_1);
        break;
    case 2:
        SendCanMessage(0x1db, 8, message1db_2);
        break;
    case 3:
        SendCanMessage(0x1db, 8, message1db_3);
        break;
    }
    counter_1db++;
    if (counter_1db >= 4)
        counter_1db = 0;
}

// Send 55b message every 100ms
void HVB_55B_Charging_Message()
{

    switch (counter_55b)
    {
    case 0:
        SendCanMessage(0x55b, 8, message55b_0);
        break;
    case 1:
        SendCanMessage(0x55b, 8, message55b_1);
        break;
    case 2:
        SendCanMessage(0x55b, 8, message55b_2);
        break;
    case 3:
        SendCanMessage(0x55b, 8, message55b_3);
        break;
    }
    counter_55b++;
    if (counter_55b >= 4)
        counter_55b = 0;
}

// Send 5bc message every 100ms
void HVB_5BC_Charging_Message()
{
    SendCanMessage(0x5bc, 8, message5bc);
}

// Send 50b message every 100ms
void VCM_50B_Charging_Message()
{
    SendCanMessage(0x50b, 7, message50b);
}

// Send 59e message every 500ms
void HVB_59E_Charging_Message()
{
    SendCanMessage(0x59e, 8, message59e);
}

void SendChargeStateCan10msMessage()
{
    VCM_11a_Charging_Message();
    VCM_1F2_Charging_Message();
    VCM_1d4_Charging_Message();
    HVB_1DB_Charging_Message();
    VCM_1DC_Charging_Message();
}

void SendChargingSimBattery100msMessage()
{
    HVB_55B_Charging_Message();
    HVB_5BC_Charging_Message();
    VCM_50B_Charging_Message();
}

void SendChargingSimBattery500msMessage()
{
    HVB_59E_Charging_Message();
}