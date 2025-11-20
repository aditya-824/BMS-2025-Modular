#include "BMS_PRINT.h"

void printStackVoltage(uint8_t total_ic, cell_asic *BMS_IC)
{
    float packVoltage = 0;
    const uint8_t kPrintOrder[3][6] = {
        {18, 13, 12, 7, 6, 1},
        {17, 14, 11, 8, 5, 2},
        {16, 15, 10, 9, 4, 3}};

    for (uint8_t row = 0; row < 3; row++)
    {
        for (uint8_t col = 0; col < 6; col++)
        {
            uint8_t ic = kPrintOrder[row][col] - 1;

            // Print stack number with alignment
            Serial.print("S");
            if (kPrintOrder[row][col] < 10)
                Serial.print("0"); // pad single digit stack numbers
            Serial.print(kPrintOrder[row][col]);
            Serial.print(": ");

            if (ic >= total_ic)
            {
                // Error message
                Serial.print("NotConn ");
            }
            else
            {
                float voltage = BMS_IC[ic].stat.stat_codes[0] * 0.0001 * 20;
                packVoltage += voltage;
                // Voltage always between 0.0000 and 25.2000, so fixed width
                Serial.print(voltage, 4);
            }

            if (col < 5)
                Serial.print(" | ");
        }
        Serial.println();
    }
    Serial.print("Total Pack Voltage: ");
    Serial.println(packVoltage, 4);
}

void print(uint8_t ic, uint8_t cell, uint16_t value, bool temp = false, bool fault = false)
{
    Serial.print("S");
    Serial.print(ic + 1, DEC);
    Serial.print("C");
    Serial.print(cell + 1, DEC);
    Serial.print(": ");

    if (temp == false)
    {
        Serial.print(value * 0.0001, 4);
    }
    else
    {
        Serial.print(value * 0.01);
    }
    if (fault)
    {
        Serial.print(" !FAULT! ");
    }
    Serial.print(", ");
}