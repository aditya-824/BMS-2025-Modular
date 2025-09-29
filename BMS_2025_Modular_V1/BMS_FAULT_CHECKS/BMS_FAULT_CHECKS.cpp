#include "BMS_FAULT_CHECKS.h"
#include "Arduino.h"
#include <stdint.h>
#include "pinModes.h"
#include "LTC681x.h"
#include "BMS_CALC.h"

void check_error(int error)
{
    if (error == -1)
    {
        Serial.println(F("A PEC error was detected in the received data"));
    }
}

void faultCounter(uint8_t total_ic, uint8_t total_cell, uint16_t uv, uint16_t ov, uint16_t ut, uint16_t ot, uint16_t volt_faultCounter[][TOTAL_CELL], uint16_t temps_faultCounter[][TEMPS], uint16_t value, bool is_temp, uint8_t ic, uint8_t cell_no)
{
    if (!is_temp)
    {
        if (value > ov || value < uv)
        {
            volt_faultCounter[ic][cell_no] += 1;
        }
        else
        {
            volt_faultCounter[ic][cell_no] = 0;
        }
    }
    else
    {
        if (value > ot || value < ut)
        {
            temps_faultCounter[ic][cell_no] += 1;
        }
        else
        {
            temps_faultCounter[ic][cell_no] = 0;
        }
    }
}

bool voltage_faultCheck(uint8_t total_ic, uint8_t total_cell, uint16_t volt_faultCounter[][TOTAL_CELL], bool voltfaultStatus[][TOTAL_CELL])
{
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        for (uint8_t cell = 0; cell < total_cell; cell++)
        {
            if (volt_faultCounter[current_ic][cell] == 10)
            {
                voltfaultStatus[current_ic][cell] = true;
                volt_faultCounter[current_ic][cell] = 0;
                digitalWrite(progLED1, HIGH);
                return true;
            }
            else
            {
                voltfaultStatus[current_ic][cell] = false;
                digitalWrite(progLED1, LOW);
            }
        }
    }
    return false;
}

bool temperature_faultCheck(uint8_t total_ic, uint8_t temps, uint16_t temps_faultCounter[][TEMPS], bool tempfaultStatus[][TEMPS], cell_asic *BMS_IC)
{
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        for (uint8_t temp = 0; temp < temps; temp++)
        {
            uint16_t tempValue = tempCalc(BMS_IC, current_ic, temp);
            if (temps_faultCounter[current_ic][temp] == 10 || tempValue > 50000)
            {
                tempfaultStatus[current_ic][temp] = true;
                temps_faultCounter[current_ic][temp] = 0;
                digitalWrite(progLED2, HIGH);
                if (tempValue > 50000)
                {
                    Serial.println("NTC DISCONNECTED");
                }
                return true;
            }
            else
            {
                tempfaultStatus[current_ic][temp] = false;
                digitalWrite(progLED2, LOW);
            }
        }
    }
    return false;
}

bool csFault_check()
{
    double currentSensor; // Variable to store current sensor reading
    const double csFault_value = 10.00; // Variable for current sensor fault check => Change based on CS testing.

    currentSensor = readCurrent();
    Serial.print("Current sensor value:");
    Serial.print(currentSensor);
    Serial.print(" Amps.");
    Serial.println();
    if (currentSensor > csFault_value || currentSensor < -400)
    {
        digitalWrite(progLED3, HIGH);
        Serial.println("Current sensor fault");
        Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
        return true;
    }
    else
    {
        digitalWrite(progLED3, LOW);
        return false;
    }
}