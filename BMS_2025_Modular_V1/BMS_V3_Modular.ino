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
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* LTC Defines */
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/* User Defines */
#define TOTAL_IC 18               // Total number of IC's/Total number of stacks
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
// ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED;          // ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;  // ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED;               // Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;    // Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;      // Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;    // Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;               // Register Selection
const uint8_t SEL_REG_A = REG_1;                   // Register Selection
const uint8_t SEL_REG_B = REG_2;                   // Register Selection

const uint8_t WRITE_CONFIG = ENABLED;  // This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = ENABLED;   // This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED;  // This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED;   // This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = ENABLED;  // This is to ENABLED or DISABLED reading the status registers in a continuous loop
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* User Variables */

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
float pcrDone = 0, airP = 0, airN = 0, pcr5v = 0, vsBat = 0,  // Variables for lvData
  vCheck = 0, tCheck = 0, amsFaultout = 0, k2 = 0, k1 = 0, ssFinal = 0, grnIn = 0,
      grnOut = 0, auxP = 0, auxN = 0, pcrAuxin = 0;
uint16_t vsBatin = 0, vsHVin = 0;  // Changed these two from float to uint16_t;
double vsBATvoltage, vsHVvoltage;
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* User Defined Arrays */
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
// User Defined Functions
void measurementLoop();
void printStackVoltage(void);
void print(uint8_t, uint8_t, uint16_t, bool, bool);
void BMSData(void);
void AvgBMSData(void);
void SoC(void);
void lvData(void);
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);  // Setting baud rate for Serial Monitor
  while (!Serial)
    ;
  pinMode(16, OUTPUT);  // isoSPI LED
  pinMode(10, OUTPUT);  //MCP CS

  initializeLTC(BMS_IC, TOTAL_IC, REFON, ADCOPT, gpioBits_a, dccBits_a, dctoBits, UV, OV);  // Initialize LTC

  int8_t error;

  if (WRITE_CONFIG == ENABLED) {
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    print_wrconfig(BMS_IC, TOTAL_IC);
  }
  if (READ_CONFIG == ENABLED) {
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig(BMS_IC, TOTAL_IC);
  }
  digitalWrite(VCHECK, HIGH);
}

void loop() {
  int cartId = digitalRead(cartId);
  int chargerAux = digitalRead(chargerAux);
  if (millis() - next_loopTime >= MEASUREMENT_LOOP_TIME) {
    measurementLoop();
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
      if (voltageFault == true || tempFault == true || csFault == true) {
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
      lvData();
      mainLoopCounter = 0;
    }
    spi_enable(SPI_CLOCK_DIV16);
    next_loopTime += MEASUREMENT_LOOP_TIME;
  }
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* LTC Defined Functions */
void measurementLoop() {
  int8_t error;

  if (MEASURE_CELL == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }

  if (MEASURE_AUX == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adax(SEL_ALL_REG, AUX_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }

  if (MEASURE_STAT == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adstat(SEL_ALL_REG, STAT_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }
}

void printStackVoltage() {
  double accuVoltage = 0;
  const uint8_t printOrder[3][6] = {
    { 18, 13, 12, 7, 6, 1 },
    { 17, 14, 11, 8, 5, 2 },
    { 16, 15, 10, 9, 4, 3 }
  };

  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 6; col++) {
      uint8_t ic = printOrder[row][col] - 1;

      if (ic >= TOTAL_IC) {
        // Print stack number with alignment
        Serial.print("S");
        if (printOrder[row][col] < 10) Serial.print("0");  // pad single digit stack numbers
        Serial.print(printOrder[row][col]);
        Serial.print(": ");

        // Error message
        Serial.print("NotConn ");
      } else {
        float voltage = BMS_IC[ic].stat.stat_codes[0] * 0.0001 * 20;
        // Print stack number with alignment
        Serial.print("S");
        if (printOrder[row][col] < 10) Serial.print("0");  // pad single digit stack numbers
        Serial.print(printOrder[row][col]);
        Serial.print(": ");

        // Voltage always between 0.0000 and 25.2000, so fixed width
        Serial.print(voltage, 4);
      }

      if (col < 5) Serial.print(" | ");
    }
    Serial.println();
  }
}

void print_conv_time(uint32_t conv_time) {
  uint16_t m_factor = 1000;  // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* User defined functions */
void print(uint8_t ic, uint8_t cell, uint16_t value, bool temp = false, bool fault = false) {
  Serial.print("S");
  Serial.print(ic + 1, DEC);
  Serial.print("C");
  Serial.print(cell + 1, DEC);
  Serial.print(": ");

  if (temp == false) {
    Serial.print(value * 0.0001, 4);
  } else {
    Serial.print(value * 0.01);
  }
  if (fault) {
    Serial.print(" !FAULT! ");
  }
  Serial.print(", ");
}

void BMSData() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    bool isTemp = false;
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      cellVoltage[currentIc][cell] += BMS_IC[currentIc].cells.c_codes[cell];
      faultCounter(TOTAL_IC, TOTAL_CELL, UV, OV, UT, OT, volt_faultCounter, temps_faultCounter, BMS_IC[currentIc].cells.c_codes[cell], isTemp, currentIc, cell);
    }
    isTemp = true;
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      cellTemperature[currentIc][temp] += tempCalc(BMS_IC, currentIc, temp);
      faultCounter(TOTAL_IC, TOTAL_CELL, UV, OV, UT, OT, volt_faultCounter, temps_faultCounter, tempCalc(BMS_IC, currentIc, temp), isTemp, currentIc, temp);
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
  printStackVoltage();
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

void SoC() {
  float totalCoulombs;
  current_time = millis();  // Get the current time

  // Check if the time interval has elapsed
  if (current_time - start_timer >= period) {
    // Measure the current
    float current = readCurrent();

    // Calculate charge in Coulombs (Current (A) * Time (s))
    float deltaTime = period / 1000.0;        // Convert milliseconds to seconds
    float chargeDelta = current * deltaTime;  // Charge change in Coulombs

    // Update total charge
    totalCoulombs += chargeDelta;

    // Calculate State of Charge (SoC)
    float totalAh = totalCoulombs / 3600.0;         // Convert Coulombs to Ah
    soc = 100.0 * (1.0 - (totalAh / maxCapacity));  // Remaining charge as percentage
    soc = constrain(soc, 0.0, 100.0);               // Constrain SoC to valid range

    Serial.print("State of Charge (SoC): ");
    Serial.print(int(soc));
    Serial.println(" %");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
    // Update start time
    start_timer = current_time;
  }
}

void lvData() {
  pcrDone = digitalRead(PCRDONE);
  airP = digitalRead(AIRP);
  airN = digitalRead(AIRN);
  pcr5v = digitalRead(PCR5V);
  vsBat = digitalRead(VSBAT);
  tCheck = digitalRead(TCHECK);
  vsBatin = analogRead(VS_BAT_IN);
  vsHVin = analogRead(VS_HV_IN);
  amsFaultout = digitalRead(AMS_FAULT_SDC);
  k1 = digitalRead(K1);
  k2 = digitalRead(K2);
  ssFinal = digitalRead(SSFINAL);
  grnIn = digitalRead(GRNIN);
  grnOut = digitalRead(GRNOUT);
  auxP = digitalRead(AUXP);
  auxN = digitalRead(AUXN);
  pcrAuxin = digitalRead(PCRAUXIN);
  vCheck = digitalRead(VCHECK);

  double vsHVop = vsHVin * analogResolution;
  vsHVvoltage = (vsHVop - 2.529) / 0.001326;

  double vsBATop = vsBatin * analogResolution;
  vsBATvoltage = (vsBATop - 2.529) / 0.001326;

  if (vsBatin >= 0.95 * vsHVin) {
    digitalWrite(VCHECK, HIGH);
  }

  Serial.print("AIR +ve: ");
  Serial.print(airP);
  Serial.print(" || AUX +ve: ");
  Serial.print(auxP);
  Serial.print(" || AIR -ve: ");
  Serial.print(airN);
  Serial.print(" || AUX -ve: ");
  Serial.print(auxN);
  Serial.print(" || PCR: ");
  Serial.print(pcr5v);
  Serial.print(" || PCR AUX: ");
  Serial.print(pcrAuxin);
  Serial.print(" || VSBAT: ");
  Serial.print(vsBat);
  Serial.println();

  Serial.print("ssfinal: ");
  Serial.print(ssFinal);
  Serial.print(" || grnin: ");
  Serial.print(grnIn);
  Serial.print(" || grnout: ");
  Serial.print(grnOut);
  Serial.print(" || VSBatIn: ");
  Serial.print(vsBATvoltage);
  Serial.print(" || VSHVIn: ");
  Serial.print(vsHVvoltage);
  // Serial.print(" || vshvin difference: ");
  // Serial.print(abs(vsHVin - vsBatin));
  Serial.println();

  Serial.print("PCRDONE: ");
  Serial.print(pcrDone);
  Serial.print(" || Vcheck: ");
  Serial.print(vCheck);
  Serial.print(" || TCheck: ");
  Serial.print(tCheck);
  Serial.print(" || K2: ");
  Serial.print(k2);
  Serial.print(" || K1: ");
  Serial.print(k1);
  Serial.println();
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}