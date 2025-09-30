#include "BMS_LTC.h"

const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; // ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED;              // Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;   // Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;     // Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;   // Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;              // Register Selection

void serialPrintHex(uint8_t data)
{
    if (data < 16)
    {
        Serial.print("0");
        Serial.print((byte)data, HEX);
    }
    else
    {
        Serial.print((byte)data, HEX);
    }
}

void initializeLTC(cell_asic *bms_ic, uint8_t total_ic, bool refon, bool adcopt, bool *gpio_bits_a, bool *dcc_bits_a, uint8_t dcto_bits, uint16_t uv, uint16_t ov)
{
    spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
    LTC6811_init_cfg(total_ic, bms_ic);
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        LTC6811_set_cfgr(current_ic, bms_ic, refon, adcopt, gpio_bits_a, dcc_bits_a, dcto_bits, uv, ov);
    }
    LTC6811_reset_crc_count(total_ic, bms_ic);
    LTC6811_init_reg_limits(total_ic, bms_ic);
}

void print_wrconfig(cell_asic *bms_ic, uint8_t total_ic)
{
    Serial.println(F("Written Config:"));
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        Serial.print(F("CFGA IC "));
        Serial.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serialPrintHex(bms_ic[current_ic].config.tx_data[i]);
        }
        Serial.print(F(", Calculated PEC: 0x"));
        int cfg_pec = pec15_calc(6, &bms_ic[current_ic].config.tx_data[0]);
        serialPrintHex((uint8_t)(cfg_pec >> 8));
        Serial.print(F(", 0x"));
        serialPrintHex((uint8_t)(cfg_pec));
        Serial.println();
    }
}

void print_rxconfig(cell_asic *bms_ic, uint8_t total_ic)
{
    Serial.println(F("Received Config:"));
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
    {
        Serial.print(F("CGFA IC "));
        Serial.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serialPrintHex(bms_ic[current_ic].config.rx_data[i]);
        }
        Serial.print(F(", Received PEC: 0x"));
        serialPrintHex(bms_ic[current_ic].config.rx_data[6]);
        Serial.print(F(", 0x"));
        serialPrintHex(bms_ic[current_ic].config.rx_data[7]);
        Serial.println();
    }
}

void measurementLoop(uint8_t TOTAL_IC, cell_asic *BMS_IC)
{
    int8_t error;

    wakeup_idle(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);

    wakeup_idle(TOTAL_IC);
    LTC6811_adax(SEL_ALL_REG, AUX_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);

    wakeup_idle(TOTAL_IC);
    LTC6811_adstat(SEL_ALL_REG, STAT_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
}