# BMS-2025-Modular
## BMS_2025_Modular_V1
Libraries created:
- BMS_CALC: Temperature, minimum & maximum cell voltage & temperature calculation and current calculation
- BMS_CAN: Initialization of MCP, sending CAN data and charging function
- BMS_FAULT_CHECKS: Error check, fault counter, fault check, voltage and temperature fault check and current fault check
- BMS_LTC: Hex number conversion, LTC initialization and printing written and read configs
## BMS_2025_Modular_V2
- Moved measurementLoop to BMS_LTC
- Created new BMS_PRINT library for all print funtions
## BMS_2025_Modular_V3
- GUI mode toggle to swtich between full serial output and live GUI output
- All serial prints modified to fit one final GUI input string
- Barebones Debug Mode toggle to disable 'print write config' and 'print read config' functions for GUI