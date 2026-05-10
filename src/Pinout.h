
// Using ESP32-WROOM-32

#define VCCVoltage 3.3f
#define ANALOG_RES 12

// Pins:
#define APIN_CHARGER_PP 36 // ADC INPUT ONLY, G36=SP Proximity Pilot
#define APIN_Throttle1 39 // ADC INPUT ONLY, G39=SN, Analog throttle channel 1
#define APIN_Throttle2 34 // ADC INPUT ONLY, Analog throttle channel 2
#define APIN_VACUUM 35 // ADC INPUT ONLY, Vacuum sensor
#define PIN_PRECHARGE 32 // ADC, P1 Precharge relay control
#define PIN_NEG_CONTACTOR 33 // ADC, P3 Negative contactor control
#define PIN_WATER_PUMP 25 // ADC, P5 Water pump activation
#define PIN_CHARGER_IGN 26 // ADC, P7 Charger turn on signal
#define PIN_REVERSE 27 // G27 ADC
#define PIN_VACUUM_PUMP 14 // ADC JTAG, P4 Vacuum pump activation
#define PIN_DCDC_ENABLE 12 // ADC STRAPPING PD FLASH BOOT, P6 DC/DC enable signal
// G13 ADC JTAG
// G09 ADC SD Data RX1 FLASH
// G10 ADC SD Data TX1 FLASH
// G11 ADC SD CMD FLASH

// USED FOR SPI - G23 SPI MOSI
// G22 I2C CLK
// G01 TX0 USB CONSOLE
// G03 RX0 USB CONSOLE
#define PIN_POS_CONTACTOR 21 // I2C DATA, P2 Positive contactor control
// USED FOR SPI - G19 SPI MISO
// USED FOR SPI - G18 SPI SCK
// G5 SPI SS/CS STRAPPING BOOT SDIO SLAVE
#define PIN_LIN_TX 17 // G17 TX2 FLASH, We actually just use Serial2 for LIN bus
#define PIN_LIN_RX 16 // G16 RX2 FLASH, We actually just use Serial2 for LIN bus
#define PIN_DRIVE_MODE 4 // ADC STRAPPING BOOT CONFIG, Single pulse of 12V from car key starter signal
#define PIN_IGNITION 0 // ADC STRAPPING BOOT, Constant 12V from car key ignition
#define MKRCAN_MCP2515_CS_PIN 2 // ADC STRAPPING RESET, Slave Select pin (SS) /	Chip Select Pin (CS)
#define PIN_BRAKES 15 // ADC STRAPPING BOOT PU
// G08 SD Data FLASH
// G07 SD Data FLASH
// G06 SD CLK