//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// Debug Defines
#define DEBUG_RELAY_FINE_STEPS
//#define DEBUG_RELAY_STATE
#define DEBUG_COARSE_TUNE_STATUS
#define DEBUG_TUNE_SUMMARY
//#define DEBUG_swr.rawSWR_VALUES
//#define DEBUG_SHIFT

#define DEBUG_BUTTON_ARRAY
#define DEBUG_BUTTON_INFO
//#define DEBUG_BUTTONS
//#define DEBUG_STEP_BUTTON

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
#define hiZ           true    // L network set for high impedence loads (capacitors at input side)
#define loZ           false   // L network set for low impedence loads (capacitors at output side)
#define hi            true    // Gain setting for swr amplifier
#define lo            false   // Gain setting for swr amplifier
#define printHeader   true    // Tell printStatus() to print the header line
#define printBody     false   // Tell printStatus() to print the status data
#define OK_SWR        120000


// Analog pushbutton settings
#define analog_buttons_pin A2
#define analog_buttons_number_of_buttons 4
#define analog_buttons_r1 12 //Resistor value connected to button in "K's"
#define analog_buttons_r2 1.2
#define LONG_PRESS_TIME 1000 //msec before button considered a long press
int button_array_high_limit[analog_buttons_number_of_buttons];
int button_array_low_limit[analog_buttons_number_of_buttons];
long button_last_add_to_send_buffer_time = 0;

// Global variables always start with an underscore
//byte _status.C_relays = 0; // Holds map of operated relays with
//byte _status.L_relays = 0; // 0 = released and 1 = operated

struct swr {
  unsigned int fwd;
  unsigned int rev;
  unsigned long rawSWR;
  boolean ampGain;
} 
_swr;

struct status {
  byte C_relays;
  byte L_relays;
  unsigned int totC;
  unsigned int totL;
  boolean outputZ;
} 
_status;

  //       Inductor definitions     L1   L2   L3   L4    L5    L6    L7    L8   
const unsigned int  _inductors[] = { 6,  16,  32,  64,  125,  250,  500, 1000 };  // inductor values in nH
  //        Capacitor definitions   C1   C2   C3   C4    C5    C6    C7    C8
const unsigned int _capacitors[] = { 5,  11,  22,  44,   88,  168,  300,  660 };  // capacitor values in pF

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
  _swr.ampGain = hi;
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  digitalWrite(Cclock, LOW);
  digitalWrite(LEDpin, LOW);
  _status.C_relays = 0;
  _status.L_relays = 0;
  _status.outputZ = hiZ;
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
} 
/**********************************************************************************************************/

void loop(){
  byte C_RelaysTmp; // Holds map of operated relays with C/O on input
  byte L_RelaysTmp; //  0 = released and 1 = operated
  byte buttonNumber;
  unsigned long SWRtmp;
  boolean bestZ;

  buttonNumber = check_step_buttons();
  //  Serial.print("MainLoop:  buttonNumber = ");
  //  Serial.println(buttonNumber);
  if(buttonNumber != 255) { // 0xFF (255) is returned with no button press
    if(buttonNumber < analog_buttons_number_of_buttons) {  
      // A short press trailing edge detected
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A short press trailing edge detected on button ");
      Serial.println(buttonNumber);
#endif      
      switch (buttonNumber) {
      case 0: 
        {
          _status.C_relays++;
          setRelays();
          break;
        }
      case 1: 
        {
          _status.C_relays--;
          setRelays();
          break;
        }
      case 2: 
        {
          _status.L_relays++;
          setRelays();
          break;
        }
      case 3: 
        {
          _status.L_relays--;
          setRelays();
        } 
      }
      getSWR();
#ifdef DEBUG_TUNE_SUMMARY
      printStatus(printHeader);
      printStatus(printBody);
#endif      
    } 
    else if(buttonNumber < (analog_buttons_number_of_buttons + analog_buttons_number_of_buttons)) {
      // A long press leading edge detected
      buttonNumber = buttonNumber - analog_buttons_number_of_buttons;
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A long press leading edge detected on button ");
      Serial.println(buttonNumber);
#endif      
    } 
    else {
      // A long press trailing edge detected
      buttonNumber = buttonNumber - (analog_buttons_number_of_buttons + analog_buttons_number_of_buttons);
#ifdef DEBUG_BUTTON_INFO      
      Serial.print("Loop:  A long press trailing edge detected on button ");      
      Serial.println(buttonNumber);
#endif      
    }  
  }

  // The button press will step the selected Capacitor or Inductor relays
  // handle button
  boolean button_pressed = handle_button();
  if (button_pressed) {
    Serial.println("button_pressed");
    _status.outputZ = hiZ;
    doRelayCourseSteps();
    //Save SWR and relay states and see if better with C/O relay on output
    C_RelaysTmp = _status.C_relays;
    L_RelaysTmp = _status.L_relays;
    bestZ = _status.outputZ;
    getSWR();
    SWRtmp = _swr.rawSWR;

    if(_swr.rawSWR > 1.2) { // Only try again if swr no good
      _status.outputZ = loZ;
      doRelayCourseSteps(); //Run it again and see if better with C/O relay operated
      //If not better restore relays to input state
      getSWR();
      if(SWRtmp <= _swr.rawSWR) {             //Capacitors on Input side gave best result so
        _status.C_relays = C_RelaysTmp;       // set relays back to where they were on input.
        _status.L_relays = L_RelaysTmp;
        _status.outputZ = hiZ;
        setRelays();
      }
      getSWR();
    }
/*
    // Here I pre-load some settings for each band and see if swr is low enough to indicate a
    // suitable starting point for a tune
    do {
      // Try 80 M wire antenna centred on 3.525 mHz
      _status.C_relays = B10001100; // Debug settings for C and L relays
      _status.L_relays = B00001101;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 80 M wire antenna centred on 3.6 mHz
      _status.C_relays = B01010110; // Debug settings for C and L relays
      _status.L_relays = B00001010;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 40 M wire antenna centred on 7.05 mHz
      _status.C_relays = B00111111; // Debug settings for C and L relays
      _status.L_relays = B00000011;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 30 M wire antenna centred on 10.025 mHz
      _status.C_relays = B01010100; // Debug settings for C and L relays
      _status.L_relays = B00000101;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 20 M wire antenna centred on 14.025 mHz
      _status.C_relays = B01010000; // Debug settings for C and L relays
      _status.L_relays = B00000101;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 17 M wire antenna centred on 18.09 mHz
      _status.C_relays = B01010000; // Debug settings for C and L relays
      _status.L_relays = B00000101;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        break;
      }
      // Try 15 M wire antenna centred on 21.025 mHz
      _status.C_relays = B01010000; // Debug settings for C and L relays
      _status.L_relays = B00000101;
      _status.outputZ  = loZ;
      setRelays();
      getSWR();
      Serial.println(_swr.rawSWR);
      if(_swr.rawSWR < OK_SWR) {
        //      Serial.println("18.09 MHz preset values");
        break;      
      }
      _status.C_relays = B00000000; // Debug settings for C and L relays
      _status.L_relays = B00000000;
      _status.outputZ  = hiZ;
      setRelays();
      Serial.println("Relays set at zero");
    }
    while(false);

    doRelayFineSteps();
*/
#ifdef DEBUG_TUNE_SUMMARY
    printStatus(printHeader);
    printStatus(printBody);
#endif
  }
}

// Subroutines start here
/**********************************************************************************************************/

void doRelayCourseSteps(){

//  unsigned long currentSWR;
  float bestSWR = 999;
//  float C_RelayBestSWR = 999;
  byte bestC;
  byte bestL;
  byte bestC_temp;
  byte bestL_temp;
  byte bestCnt = 0;
  byte cnt = 0;
  byte cnt_L = 0;
  boolean bestZ;

  // Initialise with no relays operated, changeover relay set by caller.
  _status.C_relays = 0;
  _status.L_relays = 0;
  setRelays(); // Switch off all the relays

#ifdef DEBUG_COARSE_TUNE_STATUS
    Serial.print("doRelayCourseSteps():  Doing Capacitor sequence with caps at ");
  if(_status.outputZ == hiZ) Serial.println("Input"); 
  else Serial.println("Output");
  Serial.print("cnt");
  Serial.print(" ");
//  Serial.print("curSWR");
//  Serial.print("\t");
  Serial.print("bestSWR");
  Serial.print("\t");
//  Serial.print("C_RelayBestSWR");
  printStatus(printHeader);
#endif

  getSWR();  //Get SWR with no relays operated at this point.
//  currentSWR = _swr.rawSWR;
  bestSWR = 9900000; // Dummy value to force bestSWR to be written from
  // currentSWR first time through for loop
  // here we set the capacitor relays one by one from 0 relays operated (cnt = 0)
  // through first to 8th relay (cnt = 1 to 8), checking to see which relay produces
  // the lowest SWR

  for(cnt_L = 0; cnt_L < 9; cnt_L++) {
    if(cnt_L > 0){
      _status.L_relays = 0;
      bitSet(_status.L_relays,cnt_L - 1);
      setRelays(); // Stepping through the Inductor relays
    }
    for(cnt = 0; cnt < 9; cnt++){
      if(cnt > 0){
        _status.C_relays = 0;
        bitSet(_status.C_relays,cnt - 1);
        setRelays(); // Stepping through the Capacitor relays
        getSWR();
//        currentSWR = _swr.rawSWR; // TODO _swr.rawSWR should be able to be used directly
      }
      //      Serial.print(currentSWR, 4); Serial.print(" and "); Serial.println(bestSWR, 4);
      if(_swr.rawSWR <= bestSWR){
        bestSWR = _swr.rawSWR;
        bestCnt = cnt;
        bestC_temp = _status.C_relays;
        bestL_temp = _status.L_relays;
        bestZ = _status.outputZ;
      }

#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(cnt);
      Serial.print("   ");
//      Serial.print(float(currentSWR)/100000, 4);
//      Serial.print("\t");
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print("\t");
//      Serial.print(float(C_RelayBestSWR)/100000, 4);
//      Serial.print("\t");
      printStatus(printBody);
#endif
    } // end of inner for loop
/*
    if(C_RelayBestSWR < bestSWR) {
//      bestSWR = C_RelayBestSWR;
      bestC = bestC_temp;
      bestL = bestL_temp;
      _status.C_relays = 0;
    }
*/    
  } // end of outer for loop
  _status.C_relays = bestC;
  _status.L_relays = bestL;
  _status.outputZ = bestZ;
  setRelays();
  getSWR();
}

/**********************************************************************************************************/
void doRelayFineSteps() {

  unsigned long bestSWR;  
  unsigned long swrTemp;
  byte CrelaysTemp = _status.C_relays;
  byte LrelaysTemp = _status.L_relays;

  getSWR(); // Get swr and relay status up to date.
  swrTemp = _swr.rawSWR;

#ifdef DEBUG_RELAY_FINE_STEPS
  int cnt = 1;
  Serial.print("doRelayFineSteps: entry, _swr.rawSWR = "); 
  Serial.print(_swr.rawSWR);
  Serial.print("\t("); 
  Serial.print(float(_swr.rawSWR)/100000, 5); 
  Serial.println(")");
#endif

  do {
    bestSWR = swrTemp;
    swrTemp = fineStep_C(); // Returns best SWR obtained from stepping C both up and down
    swrTemp = fineStep_L(); // Starts with best from stepping C and returns best L swr.
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

  /*
  if(_swr.rawSWR > bestSWR) {
   _swr.rawSWR = bestSWR;
   Serial.print("Doing if clause:  _swr.rawSWR = "); 
   Serial.print(_swr.rawSWR);
   Serial.print("; and swrTemp = "); 
   Serial.println(swrTemp);
   }
   else {
   // Leave original _swr.rawSWR untouched
   _status.C_relays = CrelaysTemp; // Restore C & L Relays
   _status.L_relays = LrelaysTemp;
   setRelays();
   Serial.print("Doing else clause:  _swr.rawSWR = "); 
   Serial.print(_swr.rawSWR);
   Serial.print("; and swrTemp = "); 
   Serial.println(swrTemp);
   }
   
   */
}

/**********************************************************************************************************/
unsigned long fineStep_C(){ // Enter with swr and relay status up to date

  unsigned long swrTemp;
  unsigned long bestSWR;
  byte C_RelaysTmp = _status.C_relays;

  bestSWR = _swr.rawSWR;
  swrTemp = _swr.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("fineStep_C():  Stepping capacitors up."); 
  Serial.print("bestSWR\t"); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the C relays. We will increase capacitance as first try.
  if(_status.C_relays != B11111111) { // Step to next capacitor value only if it won't step beyond maximum C.
    do {
      bestSWR = swrTemp; // 1st time through, it is already equal
#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t");
      printStatus(printBody);
#endif      
      _status.C_relays++;
      setRelays();
      getSWR();
      swrTemp = _swr.rawSWR; 
    } 
    while(swrTemp <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
    // We have not printed the values which caused the loop to exit so do it now
    Serial.print(float(bestSWR)/100000, 4); // Print the status values on entry 
    Serial.print("\t");
    printStatus(printBody);
#endif

    _status.C_relays--; // On exit, we have stepped one capacitor step too far, so back up one to best value.
    setRelays();
    getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println("Values on exit from capacitor fine steps up. The extra step has been corrected.");
    Serial.print(float(_swr.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print("\t");
    printStatus(printBody);
    //  Serial.print("C_RelaysTmp, _status.C_relays = "); Serial.print(C_RelaysTmp); Serial.print("' ");Serial.println(_status.C_relays);
#endif
  } // end if(_status.C_relays != B11111111)
  else {
    // Relays were at b'1111_1111' so stepping them up would have rolled over to b'0000_0000' therefore
    // we do nothing and leave the C_Relays at entry state; swrTemp has already been set to best SWR
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("_status.C_Relays = b1111_1111 so are not going to step C_Relays up one");
    Serial.print(float(bestSWR)/100000, 4); 
    Serial.print("\t"); 
    printStatus(printBody);
#endif
  }
  swrTemp = _swr.rawSWR;
  //------------------------------------------------------------------
  if(C_RelaysTmp == _status.C_relays) { // We didn't improve by trying to increase C so try reducing it.

    bestSWR = _swr.rawSWR;
    swrTemp = _swr.rawSWR;

#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("fineStep_C():  Stepping capacitors down."); 
    Serial.print("bestSWR\t"); 
    printStatus(printHeader);
#endif
    if(_status.C_relays != B00000000) { // Step next capacitor down only if it won't roll up to maximum C.
      do {
        bestSWR = swrTemp; // 1st time through, it is already equal
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        Serial.print(float(bestSWR)/100000, 4); 
        Serial.print("\t");
        printStatus(printBody);
#endif        
        _status.C_relays--;
        setRelays();
        getSWR();
        swrTemp = _swr.rawSWR; 
      } 
      while(swrTemp <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
      // We have not printed the values which caused the loop to exit so do it now
      Serial.print(float(bestSWR)/100000, 4); // Print the status values on entry 
      Serial.print("\t");
      printStatus(printBody);
#endif
      _status.C_relays++; // On exit, we have stepped one capacitor step too far, so back up one to best value.
      setRelays();
      getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println("Values on exit from capacitor fine steps down. The extra step has been corrected.");
      Serial.print(float(_swr.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print("\t");
      printStatus(printBody);
      //  Serial.print("C_RelaysTmp, _status.C_relays = "); Serial.print(C_RelaysTmp); Serial.print("' ");Serial.println(_status.C_relays);
#endif
    } // end if(_status.C_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the C_Relays at entry state; swrTemp has already been set to best SWR
#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println("_status.C_Relays = b'0000_0000' so are not going to step C_Relays down one");
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t"); 
      printStatus(printBody);
#endif
    }
    swrTemp = _swr.rawSWR;
  } // end if(C_RelaysTmp == _status.C_relays)
  return swrTemp; //TODO Do we need to return this or is rawSWR already holding this value?
}

/**********************************************************************************************************/
unsigned long fineStep_L() { // Enter with swr and relay status up to date

  unsigned long swrTemp;
  unsigned long bestSWR;
  byte L_RelaysTmp = _status.L_relays;

  bestSWR = _swr.rawSWR;
  swrTemp = _swr.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("fineStep_L():  Stepping inductors up."); 
  Serial.print("bestSWR\t"); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the L relays. We will increase inductance as first try.
  if(_status.L_relays != B11111111) { // Step to next inductor value only if it won't step beyond maximum L.
    do {
      bestSWR = swrTemp; // 1st time through, it is already equal
#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t");
      printStatus(printBody);
#endif   
      _status.L_relays++;
      setRelays();
      getSWR();
      swrTemp = _swr.rawSWR; 
    } 
    while(swrTemp <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
    // We have not printed the values which caused the loop to exit so do it now
    Serial.print(float(bestSWR)/100000, 4); // Print the status values on entry 
    Serial.print("\t");
    printStatus(printBody);
#endif

    _status.L_relays--; // On exit, we have stepped one inductor step too far, so back up one to best value.
    setRelays();
    getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println("Values on exit from inductor fine steps up. The extra step has been corrected.");
    Serial.print(float(_swr.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print("\t");
    printStatus(printBody);
    //  Serial.print("L_RelaysTmp, _status.L_relays = "); Serial.print(L_RelaysTmp); Serial.print("' ");Serial.println(_status.L_relays);
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
  swrTemp = _swr.rawSWR;
  //------------------------------------------------------------------
  if(L_RelaysTmp == _status.L_relays) { // We didn't improve by trying to increase C so try reducing it.

    bestSWR = _swr.rawSWR;
    swrTemp = _swr.rawSWR;

#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println("fineStep_L():  Stepping inductors down. originalBestSWR = "); 
    Serial.print("bestSWR\t"); 
    printStatus(printHeader);
#endif
    if(_status.L_relays != B00000000) { // Step next inductor down only if it won't roll up to maximum L.
      do {
        bestSWR = swrTemp; // 1st time through, it is already equal
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        Serial.print(float(bestSWR)/100000, 4); 
        Serial.print("\t");
        printStatus(printBody);
#endif        
        _status.L_relays--;        
        setRelays();
        getSWR();
        swrTemp = _swr.rawSWR; 
      } 
      while(swrTemp <= bestSWR);
#ifdef DEBUG_RELAY_FINE_STEPS
      // We have not printed the values which caused the loop to exit so do it now
      Serial.print(float(bestSWR)/100000, 4); // Print the status values on entry 
      Serial.print("\t");
      printStatus(printBody);
#endif

      _status.L_relays++; // On exit, we have stepped one capacitor step too far, so back up one to best value.
      setRelays();
      getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println("Values on exit from inductor fine steps down. The extra step has been corrected.");
      Serial.print(float(_swr.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print("\t");
      printStatus(printBody);
      //  Serial.print("L_RelaysTmp, _status.L_relays = "); Serial.print(L_RelaysTmp); Serial.print("' ");Serial.println(_status.L_relays);
#endif
    } // end if(_status.L_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the L_Relays at entry state; swrTemp has already been set to best SWR
#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println("_status.L_Relays = b'0000_0000' so are not going to step L_Relays down one");
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print("\t"); 
      printStatus(printBody);
#endif
    }
    swrTemp = _swr.rawSWR;
  } // end if(L_RelaysTmp == _status.L_relays)
  return swrTemp; //TODO Do we need to return this or is rawSWR already holding this value?
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
    Serial.print(_swr.fwd);
    Serial.print("\t"); 
    Serial.print(_swr.rev); 
    Serial.print("\t");
    if(_swr.ampGain == hi) Serial.print("High"); 
    else Serial.print(" Low"); 
    Serial.print("\t");
    if(_status.outputZ == hiZ) Serial.print("HiZ"); 
    else Serial.print(" LoZ"); 
    Serial.print("\t");  
    Serial.println(_swr.rawSWR);
  }
}

/**********************************************************************************************************/
void printFineSteps(float bestSWR) {
  Serial.print(bestSWR, 4); 
  Serial.print("\t"); 
  Serial.print(_swr.fwd);
  Serial.print("\t");
  Serial.print(_swr.rev);
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
  return val;
}

/**********************************************************************************************************/

void setRelays() {
  // Writes the bytes from _status.C_relays and _status.L_relays to the associated shift register. Following
  // this the HiZ/LoZ changeover is set to match _status.outputZ (hiZ = true; loZ = false.

  // Set the C Relays from _status.C_relays;
  shiftOut(Cdata, Cclock, MSBFIRST, _status.C_relays); // send this binary value to the Capacitor shift register

  // Set the L Relays from _status.L_relays;
  shiftOut(Ldata, Lclock, MSBFIRST, _status.L_relays); // send this binary value to the Inductor shift register

  // Set the HiZ/LoZ Changeover Relay from the value of _status.outputZ
  if(_status.outputZ) {
    digitalWrite(coRelay, CAPS_at_INPUT); // HiZ loads
  } 
  else {
    digitalWrite(coRelay, CAPS_at_OUTPUT); // LoZ loads
  }
#ifdef DEBUG_RELAY_STATE
  dbugRelayState();
#endif
  delay(DELAY); // Let the relays do their contact bounce settling
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

void getSWR() {
  // Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
  // (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.

  //       We are using the GLOBAL struct _swr
  //       All globals are prefixed with an underscore e.g. _swr.fwd

  unsigned long fwd = 0;
  unsigned long rev = 0;

  for(byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
    fwd = fwd + analogRead(forward);
    rev = rev + analogRead(reverse);
  }
  fwd = fwd / SWR_AVERAGE_COUNT;
  rev = rev / SWR_AVERAGE_COUNT;
  delay(1);

  if((fwd < 300) && (_swr.ampGain == lo)){
    digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
    _swr.ampGain = hi;
    delay(1);  
    fwd = analogRead(forward);
  }
  else if(fwd == 1023) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    _swr.ampGain = lo;
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
    _swr.fwd = fwd;
    _swr.rev = rev;
    _swr.rawSWR = ((fwd + rev)*100000) / (fwd - rev);
  } 
  else {
    _swr.fwd = 0;
    _swr.rev = 0;
    _swr.rawSWR = 99900000;
  }
#ifdef DEBUG_swr.rawSWR_VALUES
  Serial.print("getSWR: fwd, rev, floatSWR, rawSWR = ");
  Serial.print(_swr.fwd);
  Serial.print(", ");
  Serial.print(_swr.rev);
  Serial.print(", ");
  Serial.print(float(_swr.rawSWR) / 100000, 6);
  Serial.print(", ");
  Serial.println(_swr.rawSWR);
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
  Serial.println(float(_swr.rawSWR) / 10000);
}

/**********************************************************************************************************/

// Buttons are checked for either a short or a long press. The first time the poll detects a button press it saves
// the button info and waits 50 msec to re-check the button. A short press is determined by a release being
// detected within LONG_PRESS_TIME. A Long press by checking button is held for more than LONG_PRESS_TIME.
// Returns: 255 if button not pressed or a button still being pressed within the 50 msec delay is detected
//          button number if short press trailing edge
//          button Number plus Number of buttons if Long press leading edge
//          button Number plus (Number of buttons * 2) if Long press trailing edge
byte check_step_buttons()
{
  static long last_button_action  = 0; //Time last button was actioned (pressed or released)
  static byte last_button_pressed = 255; //Button number or 255 for no press
  byte        analogbuttontemp    = analogbuttonpressed(); // returns 255 for no press or button number

  // Check if a trailing edge has been detected. We will detect a released button (255) while
  // last_button_pressed is still holding a valid button number.
  if((analogbuttontemp == 255) && (last_button_pressed < analog_buttons_number_of_buttons)) {

    if((millis() - last_button_action) < LONG_PRESS_TIME) {
#ifdef DEBUG_CMD_BUTTON      
      Serial.print("Trailing edge detected. Short button press of ");
      //      Serial.print(millis() - button_depress_time);
      Serial.print(millis() - last_button_action);
      Serial.print(" msec detected on button ");
      Serial.println(last_button_pressed);
#endif      
      analogbuttontemp = last_button_pressed;
      last_button_pressed  = 255;
    } 
    else {
#ifdef DEBUG_CMD_BUTTON      
      Serial.print("Trailing edge detected. Long button press of ");
      //      Serial.print(millis() - button_depress_time);
      Serial.print(millis() - last_button_action);
      Serial.print(" msec detected on button ");
      Serial.println(last_button_pressed);
#endif      
      analogbuttontemp = last_button_pressed + analog_buttons_number_of_buttons + analog_buttons_number_of_buttons;
      last_button_pressed  = 255;      
    }
    last_button_pressed = 255;
    last_button_action = millis(); // Can't see another button press for 50 mSec from now
  }
  // Only do anything if a VALID button was pressed for MORE than 50 msec since last press. The leading edge of
  // a button press will pass this test so we can set up the debounce timing then following do a debounce delay.
  else if ((analogbuttontemp < analog_buttons_number_of_buttons) && ((millis() - last_button_action) > 300)) {
    // Here we are testing for the leading edge of a new button press
    if (last_button_pressed != analogbuttontemp) {  // First detection of an analogButton press so start the
      last_button_action = millis();                // button change timer. Only do once per press.
#ifdef DEBUG_CMD_BUTTON
      Serial.print("analogbuttontemp = ");
      Serial.print(analogbuttontemp);
      Serial.print(", last_button_action = ");
      Serial.println(last_button_action);
#endif
    } // endif (last_button_pressed != analogbuttontemp)


    // Check for a long press. If detected we add analog_buttons_number_of_buttons to the button number to
    // identify the button is currently long pressed. This is returned to the caller each time we are polled
    // until the button is released. The short press TE won't be sent now but a long press TE will.
    if ((millis() - last_button_action) > LONG_PRESS_TIME) {
#ifdef DEBUG_STEP_BUTTON
      Serial.print("LONG press of ");
      Serial.print(millis() - button_depress_time);
      Serial.print(" msec detected on button ");
      Serial.println(analogbuttontemp);
#endif
      analogbuttontemp = last_button_pressed + analog_buttons_number_of_buttons;
    } 
    else
    {
      // Record the button number. While the button is held down, multiple calls will be made to the subroutine
      // so we save the number multiple times but don't use it until the trailing edge is detected.
      last_button_pressed = analogbuttontemp;
      // We dont want any button action while the short press button is pressed so a dummy return is made
      analogbuttontemp = 255;
    }
  } 
  else analogbuttontemp = 255;  
  return analogbuttontemp;
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

  for (int x = 0;x < analog_buttons_number_of_buttons;x++) {
    button_value = int(1023 * (float(x * analog_buttons_r2)/float((x * analog_buttons_r2) + analog_buttons_r1)));
    lower_button_value = int(1023 * (float((x-1) * analog_buttons_r2)/float(((x-1) * analog_buttons_r2) + analog_buttons_r1)));
    higher_button_value = int(1023 * (float((x+1) * analog_buttons_r2)/float(((x+1) * analog_buttons_r2) + analog_buttons_r1)));

    button_array_low_limit[x] = (button_value - ((button_value - lower_button_value)/2));
    button_array_high_limit[x] = (button_value + ((higher_button_value - button_value)/2));

#ifdef DEBUG_BUTTON_ARRAY    
    Serial.print("initialize_analog_button_array: ");
    Serial.print(x);
    Serial.print(":  ");
    Serial.print(button_array_low_limit[x]);
    Serial.print(" - ");
    Serial.println(button_array_high_limit[x]);
#endif //DEBUG_BUTTON_ARRAY/*  
  }
}

/**********************************************************************************************************/

// Check to see if a button has been pressed. Return button number (0 .. n) or 0xFF if no press.
//button needs to match after a 20 mSec delay to be considered debounced
byte analogbuttonpressed() {

  int analog_line_read_average = 0;
  int analog_read_temp = 0;
  byte cnt;

  //See if a button was pressed
  if (analogRead(analog_buttons_pin) <= button_array_high_limit[analog_buttons_number_of_buttons-1]) {

    // 64 reads of button effectively averageses it
    for (cnt = 0; cnt < 64; cnt++){
      analog_read_temp = analogRead(analog_buttons_pin);
      if (analog_read_temp <= button_array_high_limit[analog_buttons_number_of_buttons-1]){
        analog_line_read_average = (analog_line_read_average + analog_read_temp) / 2;
      }
    }
    // Now determine which button was pressed
    for (cnt = 0; cnt < analog_buttons_number_of_buttons; cnt++) {
      if ((analog_line_read_average > button_array_low_limit[cnt]) &&
        (analog_line_read_average <=  button_array_high_limit[cnt])) {
#ifdef DEBUG_BUTTONS
        Serial.print(F(" analogbuttonpressed: returning: "));
        Serial.println(cnt);
#endif    
        return cnt;
      }  
    }    
  }
  return 255; //No buttons were pressed
}

/**********************************************************************************************************/

// Check a specific button for press. Return 1 if pressed 0 if not.
byte analogbuttonread(byte button_number) {

  // button numbers start with 0

  int analog_line_read = analogRead(analog_buttons_pin);

#ifdef DEBUG_BUTTONS
  static byte debug_flag = 0;
#endif

  if (analog_line_read < 1000) {  
    if ((analog_line_read > button_array_low_limit[button_number])&& (analog_line_read <  button_array_high_limit[button_number])) {
#ifdef DEBUG_BUTTONS
      if (!debug_flag) {
        Serial.print(F("\nanalogbuttonread: analog_line_read: "));
        Serial.print(analog_line_read);
        Serial.print(F("  button pressed: "));
        Serial.println(button_number);
        debug_flag = 1;
      }
#endif
      return 1;
    }  
  }
#ifdef DEBUG_BUTTONS
  debug_flag = 0;
#endif  
  return 0;
}
/**********************************************************************************************************/
/*

 void tuneSummary() {
 Serial.print("C_Relays");
 Serial.print("  ");
 Serial.print("L_Relays");
 Serial.print("  ");
 Serial.print("fwdVolts");
 Serial.print("\t");
 Serial.print("revVolts");
 Serial.print("\t");
 Serial.println("SWR");
 print_binary(_status.C_relays, 8);
 Serial.print("  ");
 print_binary(_status.L_relays, 8);
 Serial.print("  ");
 formatINT(_swr.fwd);
 Serial.print( _swr.fwd);
 Serial.print("\t");
 formatINT(_swr.rev);
 Serial.print(_swr.rev);
 Serial.print("\t\t");
 Serial.println(float(_swr.rawSWR)/100000, 4);
 Serial.print("Total Capacitance = ");
 Serial.print(calcXvalue(C));
 Serial.print(" pF and Total Inductance = ");
 Serial.print(calcXvalue(L));
 Serial.print(" nH");
 Serial.print("\tThe capacitors connect to the ");
 if(digitalRead(coRelay) == LOW) Serial.println("Input"); 
 else Serial.println("Output");
 Serial.println();
 }
/**********************************************************************************************************/



