
// Using Arduino Due

#define VCCVoltage 3.3f
#define ANALOG_RES 10

// LEDs
// RX, TX, Built-in
#define DEBUG_LED LED_BUILTIN // onboard DEBUG_LED for diagnosis

// DIGITAL
#define PIN_DRIVE_MODE 2        // Single pulse of 12V from car key starter signal
#define PIN_IGNITION 3          // Constant 12V from car key ignition
#define PIN_PRECHARGE 4         // Precharge relay control
#define PIN_POS_CONTACTOR 5     // Positive contactor control
#define PIN_NEG_CONTACTOR 6     // Negative contactor control
#define PIN_VACUUM_PUMP 7       // Vacuum pump activation
#define PIN_WATER_PUMP 8        // Water pump activation
#define PIN_DCDC_ENABLE 9       // DC/DC enable signal

#define MKRCAN_MCP2515_CS_PIN 10 // Slave Select pin (SS) /	Chip Select Pin (CS)
#define MKRCAN_MCP2515_INT_PIN 11 // Interrupt pin for CAN messages (not used)

// ANALOG TO DIGITAL
#define APIN_VACUUM A0  // Vacuum sensor
#define APIN_Throttle1 A1 // Analog throttle channel 1
#define APIN_Throttle2 A2 // Analog throttle channel 2