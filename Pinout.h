
// Using Arduino Due

#define VCCVoltage 3.3f
#define ANALOG_RES 10

// LEDs
// RX, TX, Built-in
#define DEBUG_LED LED_BUILTIN // onboard DEBUG_LED for diagnosis

// DIGITAL
#define MKRCAN_MCP2515_INT_PIN 2 // CAN Interrupt
#define PIN_Brake 3              // Brake pedal
#define PIN_IGNITION 4           // Is ignition on?
#define PIN_PRECHARGE 5          // Precharge relay control
#define PIN_POS_CONTACTOR 6      // Positive contactor control (negative is assumed to be always on)
#define PIN_INV_POWER 7          // Inverter + Power Delivery Module 12V relay

#define MKRCAN_MCP2515_CS_PIN 10 // Slave Select pin (SS) /	Chip Select Pin (CS)

// ANALOG TO DIGITAL
#define APIN_BAT_VOLT A0  // Battery voltage (Currently not used since first experiments fried my board... We read the voltage from the Inverter)
#define APIN_Throttle1 A1 // Analog throttle channel 1
#define APIN_Throttle2 A2 // Analog throttle channel 2