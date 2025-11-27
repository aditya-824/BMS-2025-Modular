#ifndef BMS_PRINT_H
#define BMS_PRINT_H

#include "Arduino.h"
#include <stdint.h>
#include <stdbool.h>
#include "BMS_PRINT.h"
#include "LTC681x.h"

void printStackVoltage(uint8_t total_ic, cell_asic *BMS_IC, bool gui);
void print(uint8_t ic, uint8_t cell, uint16_t value, bool temp = false, bool fault = false, bool gui = false);

#endif // BMS_PRINT_H