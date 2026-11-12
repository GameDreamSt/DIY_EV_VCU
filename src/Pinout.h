
// Using ESP32-WROOM-32

#define VCCVoltage 3.3f
#define ANALOG_RES 12

#define CHANNEL_THROTTLE1 0
#define CHANNEL_THROTTLE2 1
#define CHANNEL_VACUUM 2
#define CHANNEL_PP 3

// Pins:
#define PIN_AC_CONTROL 36 // G36/SP ADC INPUT ONLY
#define PIN_BRAKES 39 // G39/SN ADC INPUT ONLY
#define PIN_IGNITION 34 // G34 ADC INPUT ONLY
#define PIN_REVERSE 35 // G35 ADC INPUT ONLY
#define PIN_PRECHARGE 32 // G32 ADC, P1 Precharge relay control
#define PIN_NEG_CONTACTOR 33 // G33 ADC, P3 Negative contactor control
#define PIN_WATER_PUMP 25 // G25 ADC, P5 Water pump activation
#define PIN_CHARGER_IGN 26 // ADC, P7 Charger turn on signal
#define PIN_VACUUM_PUMP 27 // G27 ADC, P4 Vacuum pump activation
#define PIN_RADIATOR_FAN 14 // G14 ADC JTAG
#define PIN_DCDC_ENABLE 12 // G12 ADC STRAPPING PD FLASH BOOT, P6 DC/DC enable signal
#define PIN_POS_CONTACTOR 13 // G13 ADC JTAG, P2 Positive contactor control
// G09 ADC SD Data RX1 FLASH
// G10 ADC SD Data TX1 FLASH
// G11 ADC SD CMD FLASH

// USED FOR SPI - G23 SPI MOSI
// USED FOR I2C - G22 I2C CLK
// G01 TX0 USB CONSOLE
// G03 RX0 USB CONSOLE
// USED FOR I2C - G21 I2C DATA
// USED FOR SPI - G19 SPI MISO
// USED FOR SPI - G18 SPI SCK
#define MKRCAN_MCP2515_CS_PIN 5 // G05 SPI SS/CS STRAPPING BOOT SDIO SLAVE
#define PIN_LIN_TX 17 // G17 TX2 PSRAM, We actually just use Serial2 for LIN bus
#define PIN_LIN_RX 16 // G16 RX2 PSRAM, We actually just use Serial2 for LIN bus
#define PIN_DRIVE_MODE 4 // G04 ADC STRAPPING BOOT CONFIG, Single pulse of 12V from car key starter signal
// G00 ADC STRAPPING BOOT, HIGH for normal setup, LOW for programming, Constant 12V from car key ignition
// G02 ADC STRAPPING RESET
// G15 ADC STRAPPING BOOT PU
// G08 SD Data FLASH
// G07 SD Data FLASH
// G06 SD CLK