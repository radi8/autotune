//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// plugin Defines
#define plugIn_2W_LCD

// Debug Defines
//#define DEBUG_RELAY_FINE_STEPS
//#define DEBUG_RELAY_STATE
#define DEBUG_COARSE_TUNE_STATUS
//#define DEBUG_TUNE_SUMMARY
//#define DEBUG_status.rawSWR_VALUES
//#define DEBUG_SHIFT

#define DEBUG_BUTTON_ARRAY
//#define DEBUG_BUTTON_INFO
#define DEBUG_BUTTONS
#define DEBUG_STEP_BUTTON

#define TX_LEVEL_THRESHOLD 5
#define CAPS_at_INPUT      LOW    //For digitalWrites to Capacitor I/O changeover relay
#define CAPS_at_OUTPUT     HIGH
#define SWR_AVERAGE_COUNT  10     // Number of analog readings to combat jitter

// Shift Register for L & C driver Pin assign
#define Cclock 2           // Pin 8 of 74HC164 U4, pin 20 of Arduino nano
#define Cdata 3            // Pin 2 of 74HC164 U4, pin 21 of Arduino nano
#define Lclock 4           // Pin 8 of 74HC164 U3, pin 22 of Arduino nano
#define Ldata 5            // Pins 1 & 2 of 74HC164 U3, pin 23 of Arduino nano

#define coRelay        7   // Capacitor set c/o relay
#define swrGain        8   // Switchable gain for swr amplifiers
#define BUTTON_PIN     6   // Push Button
#define LEDpin        13   // A LED is connected to this pin
#define forward       A0   // Measure forward SWR on this pin
#define reverse       A1   // Measure reverse SWR on this pin

#define DELAY         50   // Delay for relay debounce settle time ms

#define C             true    // Capacitor relay set
#define L             false   // Inductor relay set
#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final
#define hiZ           true    // L network set for high impedence loads (capacitors at output side)
#define loZ           false   // L network set for low Z loads (caps at input side i.e. relay not operated)
#define hi            true    // Gain setting for swr amplifier
#define lo            false   // Gain setting for swr amplifier
#define printHeader   true    // Tell printStatus() to print the header line
#define printBody     false   // Tell printStatus() to print the status data
#define OK_SWR        120000


// Analog pushbutton settings
#define analog_buttons_pin A2
#define num_of_analog_buttons 4
#define analog_buttons_r1 7.5 //Resistor value connected to button // with internal pullup in "K's"
#define analog_buttons_r2 1.2
#define LONG_PRESS_TIME 800 //msec before button considered a long press
#define analog_Button_Debounce_Millis 10

// Comment out block below out if not using LCD.   TODO ... Check if compiler can do this automatically
#include <Wire.h>              // used for I2C functionality and serial LCD
#include <LiquidCrystal_SR.h>  // Using Liquid Crystal display in 2 wire mode
LiquidCrystal_SR lcd(8,7,TWO_WIRE); // Change pins to suit schematic
//                   | |
//                   | \-- Clock Pin
//                   \---- Data/Enable Pin

// Global variables always start with an underscore
int _Button_array_max_value[num_of_analog_buttons];
int _Button_array_min_value[num_of_analog_buttons];

struct status {
  unsigned int fwd;
  unsigned int rev;
  unsigned long rawSWR;
  boolean ampGain;
  byte C_relays;
  byte L_relays;
  unsigned int totC;
  unsigned int totL;
  boolean outputZ;
} _status;

  //       Inductor definitions     L1   L2   L3   L4    L5    L6    L7    L8   
const unsigned int  _inductors[] = { 6,  17,  35,  73,  136,  275,  568, 1099 };  // inductor values in nH
const unsigned int _strayL = 0;
  //        Capacitor definitions   C1   C2   C3   C4    C5    C6    C7    C8
const unsigned int _capacitors[] = { 6,  11,  22,  44,   88,  168,  300,  660 };  // capacitor values in pF
const unsigned int _strayC = 0;

enum commandMode {TUNED, TUNE, TUNING};
byte _cmd = 0;  // Holds the command to be processed

/**********************************************************************************************************/

void setup() { 
  pinMode(Cclock, OUTPUT); // make the Capacitor clock pin an output
  pinMode(Cdata , OUTPUT); // make the Capacitor data pin an output
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output  
  pinMode(coRelay, OUTPUT);
  pinMode(swrGain, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(swrGain, LOW); // Turns off fet shunting swr Start with highest gain for amps.voltages
  _status.ampGain = hi;
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  digitalWrite(analog_buttons_pin, HIGH); // pull-up activated
  digitalWrite(Cclock, LOW);
  digitalWrite(LEDpin, LOW);
  _status.C_relays = 0;
  _status.L_relays = 0;
  _status.outputZ = loZ; // Caps switched to input side of L network
  setRelays(); // Switch off all the relays & set c/o relay to input.

  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Arduino antenna tuner ver 0.1.1");
  Serial.println("Copyright (C) 2015, Graeme Jury ZL2APV");
  Serial.println();
  initialize_analog_button_array();
  
  #ifdef plugIn_2W_LCD
    lcd.begin(16,2);               // initialize the lcd
    lcd.home ();                   // go home
    lcd.print("Arduino Autotune");
    lcd.setCursor ( 0, 1 ); // go to second line (position, line_number)
    lcd.print("ZL2APV (c) 2015");
  #endif // plugIn_2W_LCD
  
} 
/**********************************************************************************************************/

void loop(){
  byte buttonNumber;
  

  getSWR();
  _cmd = processCommand(_cmd);
//  buttonNumber = check_step_buttons();
  buttonNumber = getAnalogButton();
  if(buttonNumber != 0) { // 0x00 is returned with no button press
    _cmd = TUNED; // Stop any automatic tuning process with any button press
    if(buttonNumber <= num_of_analog_buttons) {  
      // A short press trailing edge detected
      processShortPressTE(buttonNumber);
            
    } 
    else if(buttonNumber <= (num_of_analog_buttons + num_of_analog_buttons)) {
      // A long press leading edge detected
      buttonNumber = buttonNumber - num_of_analog_buttons;
      processLongPressLE(buttonNumber);
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A long press leading edge detected on button ");
      Serial.println(buttonNumber);
#endif      
    } 
    else {
      // A long press trailing edge detected
      buttonNumber = buttonNumber - (num_of_analog_buttons + num_of_analog_buttons);
      processLongPressTE(buttonNumber);
    }  
  }

  // This button press will auto step the selected Capacitor or Inductor relays
  // handle button
  boolean button_pressed = handle_button();
  if (button_pressed) {
    if(_cmd != TUNED) {
      _cmd = TUNED; // Halt a pending tune with no RF applied
    } else _cmd = TUNE;
  }
}

// Subroutines start here
/**********************************************************************************************************/
byte processCommand(byte cmd)
{
  unsigned long SWRtmp;
  int bestSWR = 0;
  byte C_RelaysTmp; // Holds map of operated relays with C/O on input
  byte L_RelaysTmp; //  0 = released and 1 = operated
  boolean bestZ;
  
  switch (cmd) {
    case TUNED: 
    { // Update LCD display
//      Serial.print("Got to TUNED, cmd = ");
//      Serial.println(cmd);
      break;
    }
    case TUNE:
    { // Wait for sufficient RF fwd pwr then start tuning
//      Serial.print("Got to TUNE, cmd = ");
//      Serial.println(cmd);
      if(_status.fwd > TX_LEVEL_THRESHOLD) {
        cmd = TUNING;
        break;
      } else break;
    }
    case TUNING: 
    { // Tuning is under way so process until finished
      tryPresets();
      if(_status.rawSWR > 150000) {
        _status.outputZ = hiZ;
     doRelayCourseSteps();
     //Save SWR and relay states and see if better with C/O relay on output
     C_RelaysTmp = _status.C_relays;
     L_RelaysTmp = _status.L_relays;
     bestZ = _status.outputZ;
     getSWR();
     SWRtmp = _status.rawSWR;
     
     if(_status.rawSWR > 120000) { // Only try again if swr needs improving
       _status.outputZ = loZ;
       doRelayCourseSteps(); //Run it again and see if better with C/O relay operated
       //If not better restore relays to input state
       getSWR();
       if(SWRtmp <= _status.rawSWR) {             //Capacitors on Input side gave best result so
         _status.C_relays = C_RelaysTmp;       // set relays back to where they were on input.
         _status.L_relays = L_RelaysTmp;
         _status.outputZ = bestZ;
         setRelays();
       }
       getSWR();
       }
      }
      doRelayFineSteps();
      cmd = TUNED;
    }
    default: cmd = TUNED; 
  }
  return cmd;
}

/**********************************************************************************************************/
void tryPresets()
{
  status statusTemp;
  
  // Here I pre-load some settings for each band and see if swr is low enough to indicate a
    // suitable starting point for a tune

    // Presets for wire antenna

    // Try 80 M wire antenna centred on 3.525 mHz
    _status.C_relays = B01110111; // Debug settings for C and L relays
    _status.L_relays = B00001111;
    _status.outputZ  = hiZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    statusTemp = _status;

    // Try 80 M wire antenna centred on 3.6 mHz
    _status.C_relays = B00001110; // Debug settings for C and L relays
    _status.L_relays = B00000101;
    _status.outputZ  = hiZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }
    
    // Try 80 M wire antenna centred on 3.8 mHz
    _status.C_relays = B11000011; // Debug settings for C and L relays
    _status.L_relays = B00001011;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 40 M wire antenna centred on 7.05 mHz
    _status.C_relays = B01000001; // Debug settings for C and L relays
    _status.L_relays = B00001100;
    _status.outputZ  = hiZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 30 M wire antenna centred on 10.025 mHz
    _status.C_relays = B01001011; // Debug settings for C and L relays
    _status.L_relays = B00001101;
    _status.outputZ  = hiZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 20 M wire antenna centred on 14.025 mHz
    _status.C_relays = B00111111; // Debug settings for C and L relays
    _status.L_relays = B00000011;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 17 M wire antenna centred on 18.09 mHz
    _status.C_relays = B00010000; // Debug settings for C and L relays
    _status.L_relays = B00001000;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 15 M wire antenna centred on 21.025 mHz
    _status.C_relays = B00010110; // Debug settings for C and L relays
    _status.L_relays = B00000001;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Try 12 M wire antenna centred on 21.025 mHz
    _status.C_relays = B00011010; // Debug settings for C and L relays
    _status.L_relays = B00000011;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Presets for beam antenna

    // Try 20 M beam antenna centred on 14.025 mHz
    _status.C_relays = B00011100; // Debug settings for C and L relays
    _status.L_relays = B00000011;
    _status.outputZ  = loZ;
    setRelays();
    getSWR();
    Serial.println(_status.rawSWR);
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    // Fallback of no relays operated

    _status.C_relays = B00000000; // Debug settings for C and L relays
    _status.L_relays = B00000000;
    _status.outputZ  = hiZ;
    setRelays();
    Serial.println("Relays set at zero");
    if(_status.rawSWR < statusTemp.rawSWR) {
      statusTemp = _status;
    }

    _status = statusTemp;
    setRelays();
    getSWR();    
}

/**********************************************************************************************************/
           
void doRelayCourseSteps(){

  unsigned long bestSWR = 99900000; // Dummy value to force bestSWR to be written from
  byte bestC;
  byte bestL;
  boolean bestZ;
  byte cnt = 0;
  byte cnt_L = 0;

  // Initialise with no relays operated, changeover relay set by caller.
  _status.C_relays = 0;
  _status.L_relays = 0;
  setRelays(); // Switch off all the relays

#ifdef DEBUG_COARSE_TUNE_STATUS
    Serial.print("doRelayCourseSteps():  Doing Capacitor sequence with caps at ");
  if(_status.outputZ == hiZ) Serial.println("Output"); 
  else Serial.println("Input");
  Serial.print("cnt");
  Serial.print(" ");
  Serial.print("bestSWR");
  Serial.print("\t");
  printStatus(printHeader);
#endif

  getSWR();  //Get SWR with no relays operated at this point.

  // currentSWR first time through for loop
  // here we set the capacitor relays one by one from 0 relays operated (cnt = 0)
  // through first to 8th relay (cnt = 1 to 8), checking to see which relay produces
  // the lowest SWR

    for(cnt_L = 0; cnt_L < 9; cnt_L++) {
    if(cnt_L > 0){
      _status.L_relays = 0;
      bitSet(_status.L_relays,cnt_L - 1);
      _status.C_relays = 0; //Start C_Relay loop at b'0000_0000'
      setRelays(); // Stepping through the Inductor relays
      getSWR();
    }
    for(cnt = 0; cnt < 9; cnt++){
      if(cnt > 0){
        _status.C_relays = 0;
        bitSet(_status.C_relays,cnt - 1);
        setRelays(); // Stepping through the Capacitor relays
        getSWR();
      }
      //      Serial.print(currentSWR, 4); Serial.print(" and "); Serial.println(bestSWR, 4);
      if(_status.rawSWR <= bestSWR){
        bestSWR = _status.rawSWR;
        bestC = _status.C_relays;
        bestL = _status.L_relays;
        bestZ = _status.outputZ;
      }

#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(cnt);
      Serial.print("   ");
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print("\t");
      printStatus(printBody);
#endif
    } // end of inner for loop

  } // end of outer for loop
  _status.C_relays = bestC;
  _status.L_relays = bestL;
  _status.outputZ = bestZ;
  setRelays();
  getSWR();
#ifdef DEBUG_COARSE_TUNE_STATUS  
  printStatus(printHeader);
  printStatus(printBody);
#endif  
}

/**********************************************************************************************************/
void doRelayFineSteps() {

  unsigned long bestSWR;  
  unsigned long swrTemp;

  getSWR(); // Get swr and relay status up to date.
  swrTemp = _status.rawSWR;

#ifdef DEBUG_RELAY_FINE_STEPS
  int cnt = 1;
  Serial.print("doRelayFineSteps: entry, _status.rawSWR = "); 
  Serial.print(_status.rawSWR);
  Serial.print("\t("); 
  Serial.print(float(_status.rawSWR)/100000, 5); 
  Serial.println(")");
#endif

  do {
    bestSWR = swrTemp;
    swrTemp = fineStep_L(); // Returns best SWR obtained from stepping L both up and down
    swrTemp = fineStep_C(); // Starts with best from stepping L and returns best C swr.
#ifdef DEBUG_RELAY_FINE_STEPS    
    Serial.print("doRelayFineSteps():  Been through loop "); 
    Serial.print(cnt); 
    Serial.println(" times.");
    cnt++;
    Serial.println("-----------------------------------------------------------------------------------");
#endif    
  } 
  while(swrTemp < bestSWR); // If swr was improved, go again

  Serial.println("Exiting doRelayFineSteps(): values on exit ...");
  printStatus(printHeader);
  printStatus(printBody);
}

/**********************************************************************************************************/
unsigned long fineStep_C(){ // Enter with swr and relay status up to date

  unsigned long bestSWR;
  byte C_RelaysTmp = _status.C_relays; //Later will compare the value of _status.C_relays to see if we changed.

#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("fineStep_C():  Stepping capacitors up."); 
  Serial.print("bestSWR\t"); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the C relays. We will increase capacitance as first try.
  if(_status.C_relays != B11111111) { // Step to next capacitor value only if it won't step beyond maximum C.
    do {
      bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t");
      printStatus(printBody);
#endif      
      _status.C_relays++;
      setRelays();
      getSWR();
    } 
    while(_status.rawSWR <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
    // We have not printed the values which caused the loop to exit so do it now
    Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit. 
    Serial.print("\t");
    printStatus(printBody);
#endif

    _status.C_relays--; // On exit, we have stepped one capacitor step too far, so back up one to best value.
    setRelays();
    getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println("Values on exit from capacitor fine steps up. The extra step has been corrected.");
    Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print("\t");
    printStatus(printBody);
    //  Serial.print("C_RelaysTmp, _status.C_relays = "); Serial.print(C_RelaysTmp); Serial.print("' ");Serial.println(_status.C_relays);
#endif
  } // end if(_status.C_relays != B11111111)
  else {
    // Relays were at b'1111_1111' so stepping them up would have rolled over to b'0000_0000' therefore
    // we do nothing and leave the C_Relays at entry state.
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("_status.C_Relays = b1111_1111 so are not going to step C_Relays up one");
    Serial.print(float(bestSWR)/100000, 4); 
    Serial.print("\t"); 
    printStatus(printBody);
#endif
  }
  //------------------------------------------------------------------
  if(C_RelaysTmp == _status.C_relays) { // We didn't improve by trying to increase C so try reducing it.
    bestSWR = _status.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("fineStep_C():  Stepping capacitors down."); 
    Serial.print("bestSWR\t"); 
    printStatus(printHeader);
#endif
    if(_status.C_relays != B00000000) { // Step next capacitor down only if it won't roll up to maximum C.
      do {
        bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        Serial.print(float(bestSWR)/100000, 4); 
        Serial.print("\t");
        printStatus(printBody);
#endif        
        _status.C_relays--;
        setRelays();
        getSWR();
      } 
      while(_status.rawSWR <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
      // We have not printed the values which caused the loop to exit so do it now
      Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit.
      Serial.print("\t");
      printStatus(printBody);
#endif
      _status.C_relays++; // On exit, we have stepped one capacitor step too far, so back up one to best value.
      setRelays();
      getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println("Values on exit from capacitor fine steps down. The extra step has been corrected.");
      Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print("\t");
      printStatus(printBody);
#endif
    } // end if(_status.C_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the C_Relays at entry state.
#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println("_status.C_Relays = b'0000_0000' so are not going to step C_Relays down one");
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t"); 
      printStatus(printBody);
#endif
    }
  } // end if(C_RelaysTmp == _status.C_relays)

  return _status.rawSWR;
}

/**********************************************************************************************************/
unsigned long fineStep_L() { // Enter with swr and relay status up to date

  unsigned long bestSWR;
  byte L_RelaysTmp = _status.L_relays; //Later will compare the value of _status.L_relays to see if we changed.

#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("fineStep_L():  Stepping inductors up."); 
  Serial.print("bestSWR\t"); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the L relays. We will increase inductance as first try.
  if(_status.L_relays != B11111111) { // Step to next inductor value only if it won't step beyond maximum L.
    do {      
      bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t");
      printStatus(printBody);
#endif      
      _status.L_relays++;
      setRelays();
      getSWR();
    } 
    while(_status.rawSWR <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
    // We have not printed the values which caused the loop to exit so do it now
    Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit. 
    Serial.print("\t");
    printStatus(printBody);
#endif
    _status.L_relays--; // On exit, we have stepped one inductor step too far, so back up one to best value.
    setRelays();
    getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println("Values on exit from inductor fine steps up. The extra step has been corrected.");
    Serial.print(float(_status.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print("\t");
    printStatus(printBody);
#endif
  } // end if(_status.L_relays != B11111111)
  else {
    // Relays were at b'1111_1111' so stepping them up would have rolled over to b'0000_0000' therefore
    // we do nothing and leave the L_Relays at entry state; swrTemp has already been set to best SWR
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("_status.L_Relays = b1111_1111 so are not going to step L_Relays up one");
    Serial.print(float(bestSWR)/100000, 4); 
    Serial.print("\t"); 
    printStatus(printBody);
#endif
  }
  //------------------------------------------------------------------
  if(L_RelaysTmp == _status.L_relays) { // We didn't improve by trying to increase L so try reducing it.
    bestSWR = _status.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("fineStep_L():  Stepping inductors down."); 
    Serial.print("bestSWR\t"); 
    printStatus(printHeader);
#endif
    if(_status.L_relays != B00000000) { // Step next inductor down only if it won't roll up to maximum L.
      do {
        bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        Serial.print(float(bestSWR)/100000, 4); 
        Serial.print("\t");
        printStatus(printBody);
#endif        
        _status.L_relays--;
        setRelays();
        getSWR();
      } 
      while(_status.rawSWR <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
      // We have not printed the values which caused the loop to exit so do it now
      Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit.
      Serial.print("\t");
      printStatus(printBody);
#endif
      _status.L_relays++; // On exit, we have stepped one inductor step too far, so back up one to best value.
      setRelays();
      getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println("Values on exit from inductor fine steps down. The extra step has been corrected.");
      Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print("\t");
      printStatus(printBody);
#endif
    } // end if(_status.L_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the L_Relays at entry state.
#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println("_status.L_Relays = b'0000_0000' so are not going to step L_Relays down one");
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t"); 
      printStatus(printBody);
#endif
    }
  } // end if(L_RelaysTmp == _status.L_relays)

  return _status.rawSWR;
}

/**********************************************************************************************************/
void printStatus(boolean doHeader) {

  if(doHeader) {
    Serial.println("C_relays\tL_relays\ttotC\ttotL\tfwdVolt\trevVolt\tGain\toutZ\trawSWR");
  } 
  else {
    print_binary(_status.C_relays, 8); 
    Serial.print("\t"); 
    print_binary(_status.L_relays, 8); 
    Serial.print("\t");
    Serial.print(calcXvalue(C)); 
    Serial.print("\t"); 
    Serial.print(calcXvalue(L)); 
    Serial.print("\t");
    Serial.print(_status.fwd);
    Serial.print("\t"); 
    Serial.print(_status.rev); 
    Serial.print("\t");
    if(_status.ampGain == hi) Serial.print("High"); 
    else Serial.print(" Low"); 
    Serial.print("\t");
    if(_status.outputZ == hiZ) Serial.print("HiZ"); 
    else Serial.print(" LoZ"); 
    Serial.print("\t");  
    Serial.println(_status.rawSWR);
  }
}

/**********************************************************************************************************/
void printFineSteps(float bestSWR) {
  Serial.print(bestSWR, 4); 
  Serial.print("\t"); 
  Serial.print(_status.fwd);
  Serial.print("\t");
  Serial.print(_status.rev);
  Serial.print("\t");
  Serial.print(calcXvalue(C));
  Serial.print("\t");
  Serial.print(calcXvalue(L));
  Serial.print("\t");
  print_binary(_status.C_relays, 8);
  Serial.print("\t");
  print_binary(_status.L_relays, 8);
  Serial.println();
}

/**********************************************************************************************************/
// We calculate the total values of L or C. CorL is a flag to determine which reactance to sum up.
unsigned int calcXvalue(bool CorL){
  unsigned int val = 0;

  for (byte cnt = 0; cnt < 8; cnt++) {
    if (CorL) {   // True do capacitors, false do inductors
      if(bitRead(_status.C_relays, cnt)) val = val + _capacitors[cnt]; // add reactance assigned to set bits only.
    } 
    else {     
      if(bitRead(_status.L_relays, cnt)) val = val + _inductors[cnt];
    } 
  }
  if (CorL) {
    val = val + _strayC;
  }
  else {
    val = val + _strayL;
  }
  return val;
}

/**********************************************************************************************************/

void setRelays() {
  // Writes the bytes from _status.C_relays and _status.L_relays to the associated shift register. Following
  // this the HiZ/LoZ changeover is set to match _status.outputZ (hiZ = true; loZ = false). The capacitor
  // changeover relay switches caps to input (not operated) for hiZ loads and output (operated) for loZ.

  // Set the C Relays from _status.C_relays;
  shiftOut(Cdata, Cclock, MSBFIRST, _status.C_relays); // send this binary value to the Capacitor shift register

  // Set the L Relays from _status.L_relays;
  shiftOut(Ldata, Lclock, MSBFIRST, _status.L_relays); // send this binary value to the Inductor shift register

  // Set the HiZ/LoZ Changeover Relay from the value of _status.outputZ
  if(_status.outputZ) {
    digitalWrite(coRelay, CAPS_at_OUTPUT); // HiZ loads, relay operated

  } 
  else {
    digitalWrite(coRelay, CAPS_at_INPUT); // LoZ loads, relay not operated
  }
#ifdef DEBUG_RELAY_STATE
  dbugRelayState();
#endif
  delay(DELAY); // Let the relays do their contact bounce settling
}
/**********************************************************************************************************/

void getSWR() {
  // Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
  // (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.

  //       We are using the GLOBAL struct _swr
  //       All globals are prefixed with an underscore e.g. _status.fwd

  unsigned long fwd = 0;
  unsigned long rev = 0;

  for(byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
    fwd = fwd + analogRead(forward);
    rev = rev + analogRead(reverse);
  }
  fwd = fwd / SWR_AVERAGE_COUNT;
  rev = rev / SWR_AVERAGE_COUNT;
  delay(1);

  if((fwd < 300) && (_status.ampGain == lo)){
    digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
    _status.ampGain = hi;
    delay(1);  
    fwd = analogRead(forward);
  }
  else if(fwd == 1023) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    _status.ampGain = lo;
    delay(1);
    fwd = analogRead(forward);
  }
  if(fwd > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    fwd = 0;
    rev = 0;
    for(byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
      fwd = fwd + analogRead(forward);
      rev = rev + analogRead(reverse);
    }
    fwd = fwd / SWR_AVERAGE_COUNT;
    rev = rev / SWR_AVERAGE_COUNT;
    if(rev == 0) rev = 1;
    if (fwd <= rev) fwd = (rev + 1); //Avoid division by zero or negative.
    _status.fwd = fwd;
    _status.rev = rev;
    _status.rawSWR = ((fwd + rev)*100000) / (fwd - rev);
  } 
  else {
    _status.fwd = 0;
    _status.rev = 0;
    _status.rawSWR = 99900000;
  }
#ifdef DEBUG_status.rawSWR_VALUES
  Serial.print("getSWR: fwd, rev, floatSWR, rawSWR = ");
  Serial.print(_status.fwd);
  Serial.print(", ");
  Serial.print(_status.rev);
  Serial.print(", ");
  Serial.print(float(_status.rawSWR) / 100000, 6);
  Serial.print(", ");
  Serial.println(_status.rawSWR);
#endif
}
/**********************************************************************************************************/
// Pads an integer number for printing to be right justified over 4 digits
// Expects a positive integer up to 4 digits i.e. 0 ... 9999
void formatINT(int number)
{
  if (number < 10)
  {
    Serial.print("   ");
  }
  else if (number <100)
  {
    Serial.print("  ");
  }
  else if (number <1000)
  {
    Serial.print(" ");
  }
} //Exit with no padded spaces if number >= 1000

/**********************************************************************************************************/

// PRINT_BINARY - Arduino
//
// Prints a positive integer in binary format with a fixed withdth
//
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07

void print_binary(int v, int num_places)
{
  int mask=0, n;

  for (n=1; n<=num_places; n++)
  {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places

  while(num_places)
  {

    if (v & (0x0001 << num_places-1))
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }

    --num_places;
    if(((num_places%4) == 0) && (num_places != 0))
    {
      Serial.print("_");
    }
  }
}

/**********************************************************************************************************/

void dbugRelayState(){
  Serial.print("_status.C_relays value = ");
  Serial.print(_status.C_relays);
  Serial.print(", C = ");
  Serial.print(calcXvalue(C));
  Serial.print(" pF");
  Serial.print("      ... _status.L_relays value = ");
  Serial.print(_status.L_relays);
  Serial.print(", L = ");
  Serial.print(calcXvalue(L));
  Serial.println(" nH");
  Serial.println("C Relay# 1 2 3 4 5 6 7 8 ... L Relay# 1 2 3 4 5 6 7 8");
  Serial.print("         ");
  for(int x = 0; x < 8; x++){
    Serial.print(bitRead(_status.C_relays, x));
    Serial.print(" ");
  }
  Serial.print("             ");
  for(int x = 0; x < 8; x++){
    Serial.print(bitRead(_status.L_relays, x));
    Serial.print(" ");
  }
  Serial.print("SWR = ");
  getSWR();
  Serial.println(float(_status.rawSWR) / 10000);
}

/**********************************************************************************************************/
boolean handle_button() {
  static boolean button_was_pressed = false;
  boolean event;

  int button_now_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
  event = button_now_pressed && !button_was_pressed;
  if (event) { // Check if button changed
    delay(10);
    int button_now_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
    event = button_now_pressed && !button_was_pressed;
  }
  button_was_pressed = button_now_pressed;
  return event;
}

/**********************************************************************************************************/

void initialize_analog_button_array() {
  /* 
   typical button values:
   
   0: -56 - 46
   1: 47 - 131
   2: 132 - 203
   3: 203 - 264
   */
  int button_value;
  int lower_button_value;
  int higher_button_value;

  for (int x = 0;x < num_of_analog_buttons;x++) {
    button_value = int(1023 * (float(x * analog_buttons_r2)/float((x * analog_buttons_r2) + analog_buttons_r1)));
    lower_button_value = int(1023 * (float((x-1) * analog_buttons_r2)/float(((x-1) * analog_buttons_r2) + analog_buttons_r1)));
    higher_button_value = int(1023 * (float((x+1) * analog_buttons_r2)/float(((x+1) * analog_buttons_r2) + analog_buttons_r1)));

    _Button_array_min_value[x] = (button_value - ((button_value - lower_button_value)/2));
    _Button_array_max_value[x] = (button_value + ((higher_button_value - button_value)/2));

#ifdef DEBUG_BUTTON_ARRAY    
    Serial.print("initialize_analog_button_array: ");
    Serial.print(x);
    Serial.print(":  ");
    Serial.print(_Button_array_min_value[x]);
    Serial.print(" - ");
    Serial.println(_Button_array_max_value[x]);
#endif //DEBUG_BUTTON_ARRAY/*  
  }
}

/**********************************************************************************************************/
byte getAnalogButton()
{
// Buttons are checked for either a short or a long press. The first time the poll detects a button press it saves
// the button info and waits 10 msec to re-sample the button. A short press is determined by a release being
// detected within LONG_PRESS_TIME. A Long press by checking button is held for more than LONG_PRESS_TIME.
// Returns: 0 if no button is pressed 
//          0 for short pressed button leading edge or 'short press time' button hold.
//          button number if short press trailing edge
//          button Number plus Number of buttons if Long press leading edge
//          button Number plus (Number of buttons * 2) if Long press trailing edge
  
  static unsigned long lastButtonTime = 0;
  static unsigned long longPressTimer = 0;
  static boolean longPress = false;
  static byte lastButtonValue = 0;
  static byte currentButton = 0;
  int analogButtonValue = 0;
  int analog_read_temp = 0;
  byte thisButton;
  byte cnt;
  byte retVal;

  //sample the analog input only every 'analog_Button_Debounce_Millis' intervals
  if((millis() - lastButtonTime) < analog_Button_Debounce_Millis){
    return 0;
  }
  lastButtonTime = millis(); // Set timer for next sample period
  //See if a button was pressed
  //  if (analogRead(analog_buttons_pin) <= button_array_high_limit[num_of_analog_buttons-1]) {

  // 32 reads of button effectively averages it
  for (cnt = 0; cnt < 32; cnt++){
    analogButtonValue = analogButtonValue + analogRead(analog_buttons_pin);
  }
  analogButtonValue = analogButtonValue / cnt;
  // Now determine which button was pressed if any
  if (analogButtonValue <= _Button_array_max_value[num_of_analog_buttons-1]) {
    for (cnt = 0; cnt < num_of_analog_buttons; cnt++) {
      if  ((analogButtonValue > _Button_array_min_value[cnt]) &&
          (analogButtonValue <=  _Button_array_max_value[cnt])) {
        thisButton = cnt + 1;
      }
    }  
  } else thisButton = 0;
  // See if we got 2 identical samples in a row
  if(thisButton != lastButtonValue) {
    lastButtonValue = thisButton; // No but setting up now for next sample match.
  } else { // We have a valid button press or a valid button release
    if(thisButton != 0) { // It is a press so save the button and check for a long press
      if(currentButton != thisButton) {
        currentButton = thisButton;
        longPressTimer = millis();
      }
      if((millis() - longPressTimer) > LONG_PRESS_TIME) {
        retVal = currentButton + num_of_analog_buttons;
        longPress = true;
      } else retVal = 0;
    } else { // We are releasing the button so check if it is from a short or long press
      if(longPress) {
//        Serial.println("At ... if((millis() - longPressTimer) > LONG_PRESS_TIME)");
        retVal = currentButton + num_of_analog_buttons + num_of_analog_buttons;
        currentButton = 0;
        longPress = false;
      } else {
        retVal = currentButton;
        currentButton = 0;
        longPressTimer = 0;
      }
    }
  } 
  return retVal;  
}

/**********************************************************************************************************/

void processShortPressTE(byte button)
{
  static unsigned long repeatValue = 0;
  
  if((millis() - repeatValue) > 60) {      
      switch (button) {
      case 1: 
        {
          _status.C_relays++;
          setRelays();
          break;
        }
      case 2: 
        {
          _status.C_relays--;
          setRelays();
          break;
        }
      case 3: 
        {
          _status.L_relays++;
          setRelays();
          break;
        }
      case 4: 
        {
          _status.L_relays--;
          setRelays();
        } 
      }
  }
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A short press trailing edge detected on button ");
      Serial.println(button);
#endif
#ifdef DEBUG_TUNE_SUMMARY
      printStatus(printHeader);
      printStatus(printBody);
#endif
}

/**********************************************************************************************************/
void processLongPressLE(byte button)
{
  static unsigned long repeatValue = 0;
  
  if((millis() - repeatValue) > 60) {
    repeatValue = millis();
      switch (button) {
      case 1: 
        {
          _status.C_relays++;
          setRelays();
          break;
        }
      case 2: 
        {
          _status.C_relays--;
          setRelays();
          break;
        }
      case 3: 
        {
          _status.L_relays++;
          setRelays();
          break;
        }
      case 4: 
        {
          _status.L_relays--;
          setRelays();
        } 
      }
  }
}
/**********************************************************************************************************/      
void processLongPressTE(byte button)
{
  static unsigned long repeatValue = 0;
  
  if((millis() - repeatValue) > 60) {
    
  }
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A long press trailing edge detected on button ");      
      Serial.println(button);
#endif
}

/**********************************************************************************************************/ 
