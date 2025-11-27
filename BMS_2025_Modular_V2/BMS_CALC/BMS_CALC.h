#ifndef BMS_CALC_H
#define BMS_CALC_H

#include <Arduino.h>
#include <stdint.h>
#include "pinModes.h"
#include "LTC681x.h"

uint16_t tempCalc(cell_asic *bms_ic, uint8_t ic, uint8_t temp);
double minmaxCV(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_cell);
void minmaxCT(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_temp);
float readCurrent(void);

#endif // BMS_CALC_H