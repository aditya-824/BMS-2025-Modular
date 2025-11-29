#include "BMS_FAULT_CHECKS.h"

void check_error(int error, bool gui)
{
    if (error == -1)
    {
        if (gui)
        {
            Serial.print("PECError, ");
        }
        else
        {
            Serial.println(F("A PEC error was detected in the received data"));
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

bool temperature_faultCheck(uint8_t total_ic, uint8_t temps, uint16_t temps_faultCounter[][TEMPS], bool tempfaultStatus[][TEMPS], cell_asic *BMS_IC, bool gui)
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
                if (tempValue > 50000 && !gui)
                {
                    Serial.println("NTC DISCONNECTED");
                }
                return true;

                if (gui)
                {
                }
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

bool csFault_check(bool gui)
{
    double currentSensor;             // Variable to store current sensor reading
    const double kCsFaultVal = 10.00; // Variable for current sensor fault check => Change based on CS testing.

    currentSensor = readCurrent();
    if (gui)
    {
        Serial.print(currentSensor);
        Serial.print(", ");
    }
    else
    {
        Serial.print("Current sensor value:");
        Serial.print(currentSensor);
        Serial.print(" Amps.");
        Serial.println();
    }
    if (currentSensor > kCsFaultVal || currentSensor < -400)
    {
        digitalWrite(progLED3, HIGH);
        if (!gui)
        {
            Serial.println("Current sensor fault");
            Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
        }
        return true;

        if (gui)
        {
        }
    }
    else
    {
        digitalWrite(progLED3, LOW);
        return false;
    }
}

float lvData(bool gui)
{
    const float kAnalogResolution = 5.0 / 1023.0;
    float pcr_done = 0, air_p = 0, air_n = 0, pcr_5v = 0, vs_bat = 0, v_check = 0, t_check = 0, ams_fault_out = 0, k2 = 0, k1 = 0, ss_final = 0, green_in = 0, green_out = 0, aux_p = 0, aux_n = 0, pcr_aux_in = 0;
    uint16_t vs_bat_in = 0, vs_hv_in = 0;
    double vs_bat_voltage, vs_hv_voltage;

    pcr_done = digitalRead(PCRDONE);
    air_p = digitalRead(AIRP);
    air_n = digitalRead(AIRN);
    pcr_5v = digitalRead(PCR5V);
    vs_bat = digitalRead(VSBAT);
    t_check = digitalRead(TCHECK);
    vs_bat_in = analogRead(VS_BAT_IN);
    vs_hv_in = analogRead(VS_HV_IN);
    ams_fault_out = digitalRead(AMS_FAULT_SDC);
    k1 = digitalRead(K1);
    k2 = digitalRead(K2);
    ss_final = digitalRead(SSFINAL);
    green_in = digitalRead(GRNIN);
    green_out = digitalRead(GRNOUT);
    aux_p = digitalRead(AUXP);
    aux_n = digitalRead(AUXN);
    pcr_aux_in = digitalRead(PCRAUXIN);
    v_check = digitalRead(VCHECK);

    double vs_hv_op = vs_hv_in * kAnalogResolution;
    vs_hv_voltage = (vs_hv_op - 2.529) / 0.001326;

    double vs_bat_op = vs_bat_in * kAnalogResolution;
    vs_bat_voltage = (vs_bat_op - 2.529) / 0.001326;

    if (vs_bat_in >= 0.95 * vs_hv_in)
    {
        digitalWrite(VCHECK, HIGH);
    }

    if (gui)
    {
        Serial.print(air_p);
        Serial.print(", ");
        Serial.print(aux_p);
        Serial.print(", ");
        Serial.print(aux_p);
        Serial.print(", ");
        Serial.print(aux_n);
        Serial.print(", ");
        Serial.print(pcr_5v);
        Serial.print(", ");
        Serial.print(pcr_aux_in);
        Serial.print(", ");
        Serial.print(vs_bat);
        Serial.print(", ");
        Serial.print(ss_final);
        Serial.print(", ");
        Serial.print(green_in);
        Serial.print(", ");
        Serial.print(green_out);
        Serial.print(", ");
        Serial.print(vs_bat_voltage);
        Serial.print(", ");
        Serial.print(vs_hv_voltage);
        Serial.print(", ");
        Serial.print(pcr_done);
        Serial.print(", ");
        Serial.print(v_check);
        Serial.print(", ");
        Serial.print(t_check);
        Serial.print(", ");
        Serial.print(k2);
        Serial.print(", ");
        Serial.print(k1);
    }
    else
    {
        Serial.print("AIR +ve: ");
        Serial.print(air_p);
        Serial.print(" || AUX +ve: ");
        Serial.print(aux_p);
        Serial.print(" || AIR -ve: ");
        Serial.print(aux_p);
        Serial.print(" || AUX -ve: ");
        Serial.print(aux_n);
        Serial.print(" || PCR: ");
        Serial.print(pcr_5v);
        Serial.print(" || PCR AUX: ");
        Serial.print(pcr_aux_in);
        Serial.print(" || vs_bat: ");
        Serial.print(vs_bat);
        Serial.println();

        Serial.print("ss_final: ");
        Serial.print(ss_final);
        Serial.print(" || green_in: ");
        Serial.print(green_in);
        Serial.print(" || green_out: ");
        Serial.print(green_out);
        Serial.print(" || vs_bat_in: ");
        Serial.print(vs_bat_voltage);
        Serial.print(" || vs_hv_in: ");
        Serial.print(vs_hv_voltage);
        Serial.println();

        Serial.print("pcr_done: ");
        Serial.print(pcr_done);
        Serial.print(" || Vcheck: ");
        Serial.print(v_check);
        Serial.print(" || t_check: ");
        Serial.print(t_check);
        Serial.print(" || K2: ");
        Serial.print(k2);
        Serial.print(" || K1: ");
        Serial.print(k1);
        Serial.println();
        Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
    }

    return vs_bat;
}