#ifndef BMS_PRINT_H
#define BMS_PRINT_H

#include "Arduino.h"
#include <stdint.h>
#include "BMS_PRINT.h"
#include "LTC681x.h"

void printStackVoltage(uint8_t total_ic, cell_asic *BMS_IC);
void print(uint8_t ic, uint8_t cell, uint16_t value, bool temp = false, bool fault = false);

#endif // BMS_PRINT_H