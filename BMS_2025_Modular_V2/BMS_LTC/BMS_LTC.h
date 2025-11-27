#ifndef BMS_LTC_H
#define BMS_LTC_H

#include <Arduino.h>
#include <SPI.h>
#include "BMS_FAULT_CHECKS.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include "LT_SPI.h"

void serialPrintHex(uint8_t);
void initializeLTC(cell_asic *bms_ic, uint8_t total_ic, bool refon, bool adcopt, bool *gpio_bits_a, bool *dcc_bits_a, uint8_t dcto_bits, uint16_t uv, uint16_t ov);
void print_wrconfig(cell_asic *bms_ic, uint8_t total_ic);
void print_rxconfig(cell_asic *bms_ic, uint8_t total_ic);
void measurementLoop(uint8_t TOTAL_IC, cell_asic *BMS_IC);

#endif // BMS_LTC_H