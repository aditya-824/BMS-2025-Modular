#include "BMS_CAN.h"

#define CCS 10 // MCP chip select pin

mcp2518fd CAN(CCS); // CAN object for MCP2518FD

void initializeMCP()
{
    Serial.println("CAN Initializing....");

    CAN.setMode(CAN_CLASSIC_MODE);

    while (CAN_OK != CAN.begin(CAN20_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("CAN INITIALIZATION FAILED");
    }
    Serial.println("CAN SUCCESS");
}

bool canSend(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_cell, uint8_t total_temp, uint16_t *all_data, uint16_t vsHVin, uint16_t vsBat, float soc, uint16_t current_reading)
{
    uint8_t can_msg[8];
    uint8_t can_msg_index = 0;
    uint8_t can_id_index = 0;
    uint16_t index = 0;
    uint8_t failed_msgs = 0;

    const byte can_ids[46] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                              0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
                              0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46}; // CAN IDs for each Message

    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        for (uint8_t cell = 0; cell < total_cell; cell++)
        {
            all_data[index++] = bms_ic[current_ic].cells.c_codes[cell];
        }

        for (uint8_t temp = 0; temp < total_temp; temp++)
        {
            all_data[index++] = bms_ic[current_ic].aux.a_codes[temp];
        }
    }

    all_data[index++] = current_reading;
    all_data[index++] = vsHVin;
    all_data[index++] = vsBat;
    all_data[index++] = (uint16_t)(soc * 100);

    for (uint16_t i = 0; i < 184; i++)
    {
        can_msg[can_msg_index++] = lowByte(all_data[i]);
        can_msg[can_msg_index++] = highByte(all_data[i]);

        if (can_msg_index == 8)
        {
            if (can_id_index < 46)
            {
                if (CAN.sendMsgBuf(can_ids[can_id_index], 0, 8, can_msg) != CAN_OK)
                {
                    failed_msgs++;
                }
                can_id_index++;
            }
            can_msg_index = 0;
        }
    }
    if (failed_msgs == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool charging()
{
    unsigned char BMSA[8] = {0x11, 0xB8, 0x00, 0x64, 0x00, 0x00, 0xFF, 0xFF}; // Byte 0 and 1 -> Max TS Voltage. Byte 2 and 3 -> Max Charging current.
                                                                              // Byte 4 and 5 -> Charger related settings. Byte 6 and 7 -> Retain.
    bool charger_status;
    uint32_t buf2[8];
    buf2[2] << 8;
    CAN.sendMsgBuf(0x1806E5F4, 1, 8, BMSA);
    uint16_t chargerCANrx = CAN.getCanId();
    if (chargerCANrx == 0x18FF50E5)
    {
        charger_status = true;
    }
    else
    {
        charger_status = false;
    }

    return charger_status;
}