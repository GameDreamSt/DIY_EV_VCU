
// Using Arduino Due

#define VCCVoltage 3.3f
#define ANALOG_RES 10

// LEDs
// RX, TX, Built-in
#define DEBUG_LED LED_BUILTIN // onboard DEBUG_LED for diagnosis

// DIGITAL
#define PIN_IGNITION 2          // Constant 12V from car key ignition
#define PIN_DRIVE_MODE 3        // Single pulse of 12V from car key starter signal
#define PIN_PRECHARGE 4         // Precharge relay control
#define PIN_POS_CONTACTOR 5     // Positive contactor control
#define PIN_NEG_CONTACTOR 6     // Negative contactor control
#define PIN_AUX_CONTROL 7       // Some other 12V 5A control

#define MKRCAN_MCP2515_CS_PIN 10 // Slave Select pin (SS) /	Chip Select Pin (CS)
#define MKRCAN_MCP2515_INT_PIN 11 // Interrupt pin for CAN messages (not used)

// ANALOG TO DIGITAL
#define APIN_BAT_VOLT A0  // Battery voltage (Currently not used since first experiments fried my board... We read the voltage from the Inverter)
#define APIN_Throttle1 A1 // Analog throttle channel 1
#define APIN_Throttle2 A2 // Analog throttle channel 2