/* Standard Includes */
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

/* LTC Includes */
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"

/* Custom Includes */
#include "pinModes.h"
#include "BMS_CAN.h"
#include "BMS_LTC.h"
#include "BMS_CALC.h"
#include "BMS_FAULT_CHECKS.h"
#include "BMS_PRINT.h"
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Macros */
#define TOTAL_IC 18               // Total number of ICs/Total number of stacks
#define TOTAL_CELL 6              // Total number of cells per stack => Used for reading voltage values
#define TEMPS 4                   // Total number of cells per stack => Used for reading temperature values
#define UV 30000                  // Under voltage limit
#define OV 41000                  // Over voltage limit
#define UT 1000                   // Under temperature limit
#define OT 4500                   // Over temperature limit
#define MEASUREMENT_LOOP_TIME 20  // Sampling period per iteration for mainLoop()
#define cartId_pin 9
#define chargerAux_pin 6
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Global Variables */

// Booleans
bool voltfaultStatus[TOTAL_IC][TOTAL_CELL];  // Fault status variable => Will trigger AMS Fault
bool tempfaultStatus[TOTAL_IC][TEMPS];       // Fault status variable => Will trigger AMS Fault
bool chargerCAN_status;

// Counters
uint8_t mainLoopCounter = 0;  // Counter variable to check whether main loop has run 10 times

// Timers
static uint32_t next_loopTime;            // Timer vairable to hold prev loop time
unsigned long start_timer, current_time;  // Timer for SoC Estimation

// Readings
uint16_t avg_cellVoltage, avg_cellTemperature;  // Variable to hold average of 10 readings of cell voltage and temperature
float soc = 100.0;                              // SoC Variable

// Constants
const float maxCapacity = 4.5;  // Maximum Capacity of MOLICEL INR-21700-P45Bc
const int period = 200;         // To calculate SoC every 200ms
const float analogResolution = 5.0 / 1023.0;

// Variables
float vsBat = 0;      // Variables for lvData
uint16_t vsHVin = 0;  // Changed these two from float to uint16_t;
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Arrays */
// Readings
uint32_t cellVoltage[TOTAL_IC][TOTAL_CELL];                     // Array to hold voltage values for each cell
uint32_t cellTemperature[TOTAL_IC][TEMPS];                      // Array to hold temperature values for each cell
uint16_t allData[TOTAL_IC * (TEMPS + TOTAL_CELL) + 4] = { 0 };  // Array to hold the values of voltage and Temperature readings

// Counters
uint16_t volt_faultCounter[TOTAL_IC][TOTAL_CELL] = { 0 };  // Array to hold voltage fault count for each cell
uint16_t temps_faultCounter[TOTAL_IC][TEMPS] = { 0 };      // Array to hold temperature fault count for each cell
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Struct initialization */
cell_asic BMS_IC[TOTAL_IC];  // Initializing main struct to hold cell voltage, temperature values and stack voltage value.
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Configuration bit setup */
bool REFON = true;                                       // Reference Powered Up Bit
bool ADCOPT = false;                                     // ADC Mode option bit
bool gpioBits_a[5] = { true, true, true, true, false };  // GPIO Pin Control => Gpio {1,2,3,4,5} /* CHANGE GPIO 5 TO TRUE AND CHECK. IT CAN BE CONFIGURED AS SPI MASTER -> Didn't make a difference*/
bool dccBits_a[12] = { false, false, false, false,
                       false, false, false, false,
                       false, false, false, false };  // Discharge cell switch => Dcc {1,2,3,4,5,6,7,8,9,10,11,12}
bool dctoBits[4] = { false, false, false, false };    // Discharge time value => Dcto {0,1,2,3}. Programed for 4 min
/* !!Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet!! */
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* User Defined Functions */
void BMSData(void);
void AvgBMSData(void);
void faultCounter(uint16_t, uint16_t, bool, uint8_t, uint8_t);

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);  // Setting baud rate for Serial Monitor
  while (!Serial)
    ;
  pinMode(16, OUTPUT);  // isoSPI LED
  pinMode(10, OUTPUT);  //MCP CS

  initializeMCP();
  initializeLTC(BMS_IC, TOTAL_IC, REFON, ADCOPT, gpioBits_a, dccBits_a, dctoBits, UV, OV);  // Initialize LTC

  int8_t error;

  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  print_wrconfig(BMS_IC, TOTAL_IC);

  wakeup_sleep(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
  check_error(error);
  print_rxconfig(BMS_IC, TOTAL_IC);

  digitalWrite(VCHECK, HIGH);
}

void loop() {
  int cartId = digitalRead(cartId);
  int chargerAux = digitalRead(chargerAux);
  if (millis() - next_loopTime >= MEASUREMENT_LOOP_TIME) {
    measurementLoop(TOTAL_IC, BMS_IC);
    BMSData();
    if (cartId == 1) {
      if (chargerAux == 1) {
        chargerCAN_status = charging();
      } else {
        Serial.println("Not Charging");
      }
    }
    mainLoopCounter += 1;
    if (mainLoopCounter >= 10) {
      bool voltageFault = voltage_faultCheck(TOTAL_IC, TOTAL_CELL, volt_faultCounter, voltfaultStatus);
      bool tempFault = temperature_faultCheck(TOTAL_IC, TEMPS, temps_faultCounter, tempfaultStatus, BMS_IC);
      bool csFault = csFault_check();
      if (voltageFault == true || tempFault == true) {
        digitalWrite(AMS_FAULT_SDC, LOW);
      } else {
        digitalWrite(AMS_FAULT_SDC, HIGH);
      }
      Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
      AvgBMSData();
      minmaxCV(BMS_IC, TOTAL_IC, TOTAL_CELL);
      minmaxCT(BMS_IC, TOTAL_IC, TEMPS);
      uint16_t currentReading = analogRead(CSOUT);
      if (canSend(BMS_IC, TOTAL_IC, TOTAL_CELL, TEMPS, allData, vsHVin, vsBat, soc, currentReading) == true) {
        Serial.println("DATA SENT SUCCESSFULLY");
      }
      if (chargerCAN_status == true) {
        Serial.println("Recieved Correct CAN ID from charger.");
      } else {
        Serial.println("(CHARGER): No CAN id message recieved");
      }
      Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
      vsBat = lvData();
      mainLoopCounter = 0;
    }
    spi_enable(SPI_CLOCK_DIV16);
    next_loopTime += MEASUREMENT_LOOP_TIME;
  }
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void BMSData() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    bool isTemp = false;
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      cellVoltage[currentIc][cell] += BMS_IC[currentIc].cells.c_codes[cell];
      faultCounter(BMS_IC[currentIc].cells.c_codes[cell], isTemp, currentIc, cell);
    }
    isTemp = true;
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      cellTemperature[currentIc][temp] += tempCalc(BMS_IC, currentIc, temp);
      faultCounter(tempCalc(BMS_IC, currentIc, temp), isTemp, currentIc, temp);
    }
  }
}

void AvgBMSData() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    bool isTemp = false;
    Serial.println("Voltage Readings");
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      avg_cellVoltage = cellVoltage[currentIc][cell] / 10;
      print(currentIc, cell, avg_cellVoltage, isTemp, voltfaultStatus[currentIc][cell]);
      cellVoltage[currentIc][cell] = 0;
    }
    Serial.println();
    Serial.println("Temperature Readings");
    isTemp = true;
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      avg_cellTemperature = cellTemperature[currentIc][temp] / 10;
      print(currentIc, temp, avg_cellTemperature, isTemp, tempfaultStatus[currentIc][temp]);
      cellTemperature[currentIc][temp] = 0;
    }
    Serial.println();
    Serial.println();
  }
  Serial.println("Stack Voltage");
  printStackVoltage(TOTAL_IC, BMS_IC);
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

void faultCounter(uint16_t value, bool isTemp, uint8_t ic, uint8_t cellno) {
  if (!isTemp) {
    if (value > OV || value < UV) {
      volt_faultCounter[ic][cellno] += 1;
    } else {
      volt_faultCounter[ic][cellno] = 0;
    }
  } else {
    if (value > OT || value < UT) {
      temps_faultCounter[ic][cellno] += 1;
    } else {
      temps_faultCounter[ic][cellno] = 0;
    }
  }
}