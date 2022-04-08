// Debug Defines
//#define DEBUG_RELAY_FINE_STEPS
#define DEBUG_FINE_STEP
//#define DEBUG_RELAY_STATE
#define DEBUG_COARSE_TUNE_STATUS
//#define DEBUG_TUNE_SUMMARY
//#define DEBUG_status

// We need the printStatus() subroutine for any of these debugs
#if defined  DEBUG_COARSE_TUNE_STATUS
#define PRINT_STATUS
#elif defined DEBUG_TUNE_SUMMARY
#define PRINT_STATUS
#elif defined DEBUG_RELAY_FINE_STEPS
#define PRINT_STATUS
//#elif defined DEBUG_RELAY_STATE
//#define PRINT_STATUS
#endif

#define DEBUG_BUTTON_ARRAY
//#define DEBUG_BUTTON_INFO
//#define DEBUG_BUTTONS
//#define DEBUG_STEP_BUTTON

const uint8_t numLrelays =  9;    // There are actually 9 states 0,1,2,4,8,16,32,64,128,256
const uint8_t numCrelays =  9;
#define TX_LEVEL_THRESHOLD 20
#define CAPS_at_INPUT       LOW   //For digitalWrites to Capacitor I/O changeover relay
#define CAPS_at_OUTPUT      HIGH
#define SWR_AVERAGE_COUNT   8     // Number of analog readings to find a voice peak
#define ARRAY_SIZE          25

// Shift Register for L & C driver Pin assign
#define outputEnable  4     // Pin 13 of 74HC164's U4 & U5 to pin 7 of Arduino Nano

#define Clock        8     // Pin 11 of 74HC595's U4 & U5 to pin 11 of Arduino Nano
#define Latch        7     // Pin 12 of 74HC595's U4 & U5 to pin 10 of Arduino Nano
#define Data         6     // Pin 14 of 74HC595 U4 to pin 9 of Arduino Nano

#define coRelay       9    // Capacitor set c/o relay
#define swrGain       12    // Switchable gain for swr amplifiers
#define BUTTON_PIN    A0    // Push Button (Analog pin used as digital)
#define LEDpin        13    // A LED is connected to this pin, use for heartbeat
#define forward       A2    // Measure forward SWR on this pin
#define reverse       A3    // Measure reverse SWR on this pin

#define Button_Debounce_Millis 20   // Delay for pushbutton debounce settle time ms
#define Relay_Settle_Millis    20   // Delay for relay debounce settle time ms

#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final
#define hiZ           true    // L network set for high impedence loads (capacitors at output side)
#define loZ           false   // L network set for low Z loads (caps at input side i.e. relay not operated)
#define hi            true    // Gain setting for swr amplifier
#define lo            false   // Gain setting for swr amplifier
#define printHeader   true    // Tell printStatus() to print the header line
#define printBody     false   // Tell printStatus() to print the status data
#define OK_SWR        120000
#define XSize 5       // Capacitance Relay array to search over when fine tuning.
#define YSize 5       // Inductance Relay array to search over when fine tuning
// const int valuesCentre = valuesSize/2; // e.g. 9 / 2 = 4
#define valuesCentre int(valuesSize/2)

// Analog pushbutton settings
#define analog_buttons_pin A6
#define num_of_analog_buttons 4
#define analog_buttons_r1 10  // Resistor value connected to button chain in "K's"
#define analog_buttons_r2 1.2 // Value of each resistor in the chain in "K's"
#define LONG_PRESS_TIME 800 //msec before button considered a long press
#define analog_Button_Debounce_Millis 10

// Setup LCD stuff
#define lcdNumCols 16 // -- number of columns in the LCD
#define lcdNumRows  2 // -- number of rowss in the LCD

// Housekeeping defines
#define baudRate 115200
