#ifndef BMS_CAN_H
#define BMS_CAN_H

#include "mcp2518fd_can.h"
#include <Arduino.h>
#include <SPI.h>
#include "LTC681x.h"

void initializeMCP();
bool canSend(cell_asic *bms_ic, uint8_t total_ic, uint8_t total_cell, uint8_t total_temp, uint16_t *all_data, uint16_t vsHVin, uint16_t vsBat, float soc, uint16_t current_reading);
void charging(); 

#endif // BMS_CAN_H