#include "BMS_CALC.h"
#include <Arduino.h>
#include <stdint.h>
#include "pinModes.h"
#include "LTC681x.h"

uint16_t tempCalc(cell_asic *bms_ic, uint8_t ic, uint8_t temp)
{
    const double r_inf = 10000 * exp(-3435 / 298.15); // Variable for temperature conversion
    double R; // Variable for temperature conversion
    double vRef2; // Reference voltage for temperature conversions

    vRef2 = bms_ic[ic].aux.a_codes[5] * 0.0001;
    R = bms_ic[ic].aux.a_codes[temp] / (vRef2 - (bms_ic[ic].aux.a_codes[temp] * 0.0001));
    return ((3435 / log(R / r_inf)) - 273.15) * 100;
}

double minmaxCV(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_cell)
{
    double minCV, maxCV; // Variables for min and max cell voltage
    minCV = bms_ic[0].cells.c_codes[0];
    maxCV = minCV;

    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        for (uint8_t cell = 0; cell < total_cell; cell++)
        {
            uint16_t voltage = bms_ic[current_ic].cells.c_codes[cell];
            minCV = (voltage < minCV) ? voltage : minCV;
            maxCV = (voltage > maxCV) ? voltage : maxCV;
        }
    }
    Serial.print("Maximum Cell Voltage: ");
    Serial.print(maxCV * 0.0001, 4);
    Serial.println(" V");
    Serial.print("Minimum Cell Voltage: ");
    Serial.print(minCV * 0.0001, 4);
    Serial.println(" V");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");

    return minCV;
}

void minmaxCT(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_temp)
{
    uint8_t current_ic;
    double minCT, maxCT; // Variables for min and max cell temperature

    minCT = maxCT = tempCalc(bms_ic, 0, 0);
    for (current_ic = 0; current_ic < total_ic; current_ic++)
    {
        for (uint8_t temp = 0; temp < total_temp; temp++)
        {
            uint16_t temperature = tempCalc(bms_ic, current_ic, temp);
            minCT = (temperature < minCT) ? temperature : minCT;
            maxCT = (temperature > maxCT) ? temperature : maxCT;
        }
    }
    Serial.print("Maximum Cell Temperature: ");
    Serial.print(maxCT * 0.01);
    Serial.println(" C");
    Serial.print("Minimum Cell Temperature: ");
    Serial.print(minCT * 0.01);
    Serial.println(" C");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

float readCurrent()
{
    const float sensorSensitivity = 0.0057; // Sensor sensitivity in V/A
    const float vRef = 2.4929;              // Reference voltage
    const float analogResolution = 5.0 / 1023.0;

    uint16_t currentValue = analogRead(CSOUT);            // changed from int to uint16_t
    float voltage = currentValue * analogResolution;      // Converts to Voltage
    float current = (voltage - vRef) / sensorSensitivity; // Calculates Current
    // Serial.print("Voltage: ");
    // Serial.println(voltage);
    return current;
}