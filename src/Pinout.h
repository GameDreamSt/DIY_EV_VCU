
// Using ESP32-WROOM-32

#define VCCVoltage 3.3f
#define ANALOG_RES 12

// DIGITAL INPUT
#define PIN_DRIVE_MODE 4        // G4 Single pulse of 12V from car key starter signal
#define PIN_IGNITION 0          // G0 Constant 12V from car key ignition

// DIGITAL OUTPUT
#define PIN_PRECHARGE 32        // G32 P1 Precharge relay control
#define PIN_POS_CONTACTOR 21    // G21 P2 Positive contactor control
#define PIN_NEG_CONTACTOR 33    // G33 P3 Negative contactor control
#define PIN_VACUUM_PUMP 14      // G14 P4 Vacuum pump activation
#define PIN_WATER_PUMP 25       // G25 P5 Water pump activation
#define PIN_DCDC_ENABLE 12      // G12 P6 DC/DC enable signal
#define PIN_CHARGER_IGN 26      // G26 P7 Charger turn on signal
// G13 P8 unused

#define MKRCAN_MCP2515_CS_PIN 2  // Slave Select pin (SS) /	Chip Select Pin (CS)
#define MKRCAN_MCP2515_INT_PIN 5 // Interrupt pin for CAN messages (not used)

// ANALOG TO DIGITAL
#ifdef CONFIG_IDF_TARGET_ESP32
#define APIN_VACUUM 35          // G35 Vacuum sensor
#define APIN_Throttle1 39       // G39=SN Analog throttle channel 1
#define APIN_Throttle2 34       // G34 Analog throttle channel 2
#define APIN_CHARGER_PP 36      // G36=SP Proximity Pilot
#else
#define APIN_VACUUM 7           // ADC7 G35 Vacuum sensor
#define APIN_Throttle1 3        // ADC3 G39=SN Analog throttle channel 1
#define APIN_Throttle2 6        // ADC6 G34 Analog throttle channel 2
#define APIN_CHARGER_PP 0       // ADC0 G36=SP Proximity Pilot
#endif