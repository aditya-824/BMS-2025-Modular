#ifndef BMS_FAULT_CHECKS_H
#define BMS_FAULT_CHECKS_H

#include "Arduino.h"
#include <stdint.h>
#include "pinModes.h"
#include "LTC681x.h"
#include "BMS_CALC.h"

#define TOTAL_CELL 6
#define TEMPS 4

void check_error(int error);
void faultCheck(void);
bool voltage_faultCheck(uint8_t total_ic, uint8_t total_cell, uint16_t volt_faultCounter[][TOTAL_CELL], bool voltfaultStatus[][TOTAL_CELL]);
bool temperature_faultCheck(uint8_t total_ic, uint8_t temps, uint16_t temps_faultCounter[][TEMPS], bool tempfaultStatus[][TEMPS], cell_asic *BMS_IC);
bool csFault_check(void);
float lvData(void);

#endif // BMS_FAULT_CHECKS_H