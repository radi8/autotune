//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// Debug Defines
#define DEBUG_RELAY_FINE_STEPS
//#define DEBUG_RELAY_STATE
//#define DEBUG_COARSE_TUNE_STATUS
#define DEBUG_TUNE_SUMMARY
//#define DEBUG_CURRENT_FUNCTION
//#define DEBUG_SWR_VALUES
//#define DEBUG_SHIFT

#define DEBUG_BUTTON_ARRAY
#define DEBUG_BUTTON_INFO
//#define DEBUG_BUTTONS
//#define DEBUG_STEP_BUTTON

#define TX_LEVEL_THRESHOLD 5
#define CAPS_at_INPUT      LOW    //For digitalWrites to Capacitor I/O changeover relay
#define CAPS_at_OUTPUT     HIGH

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

#define DELAY         50   // Delay per loop in ms

#define C             true    // Capacitor relay set
#define L             false   // Inductor relay set
#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final

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
byte _C_Relays = 0; // Holds map of operated relays with
byte _L_Relays = 0; // 0 = released and 1 = operated
int _fwdVolts;
int _revVolts;
float _SWR;
enum _C_STATE{C_at_Input, C_at_Output};
//       Inductor definitions      L1  L2  L3  L4    L5    L6    L7    L8   
const unsigned int _inductors[] = { 6, 16, 32, 64,  125,  250,  500, 1000 };        // inductor values in nH
//        Capacitor definitions     C1   C2    C3    C4    C5     C6    C7    C8
const unsigned int _capacitors[] = { 5,  11,   22,   44,   88,   168,  300,  660 };  // capacitor values in pF

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

  digitalWrite(coRelay, LOW); // Set capacitor C/O relay to input side
  digitalWrite(swrGain, LOW); // Turns off fet shunting swr Start with highest gain for amps.voltages
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  digitalWrite(Cclock, LOW);
  digitalWrite(LEDpin, LOW);

  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Arduino antenna tuner ver 0.1.1");
  Serial.println("Copyright (C) 2015, Graeme Jury ZL2APV");
  Serial.println();
  initialize_analog_button_array();
  setRelays(C); // Switch off all the Capacitor relays ( _C_Relays = 0 )
  setRelays(L); // Switch off all the Inductor relays ( _L_Relays = 0 )
} 
/**********************************************************************************************************/

void loop(){
  byte C_RelaysTmp; // Holds map of operated relays with C/O on input
  byte L_RelaysTmp; //  0 = released and 1 = operated
  byte buttonNumber;
  float SWRtmp;

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
      case 0: {
          _C_Relays++;
          setRelays(C);
          break;
        }
      case 1: {
          _C_Relays--;
          setRelays(C);
          break;
        }
      case 2: {
          _L_Relays++;
          setRelays(L);
          break;
        }
      case 3: {
          _L_Relays--;
          setRelays(L);
        } 
      }
      _SWR = getSWR();
#ifdef DEBUG_TUNE_SUMMARY
      tuneSummary();
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
    doRelayCourseSteps(C_at_Input);
    //Save SWR and relay states and see if better with C/O relay on output
    C_RelaysTmp = _C_Relays;
    L_RelaysTmp = _L_Relays;
    getSWR();
    SWRtmp = _SWR;
#ifdef DEBUG_TUNE_SUMMARY
    tuneSummary();
#endif
    if(_SWR > 1.05) {
      doRelayCourseSteps(C_at_Output); //Run it again and see if better with C/O relay operated
      //If not better restore relays to input state
      getSWR();
      if(SWRtmp <= _SWR) {             //Capacitors on Input side gave best result so
        _C_Relays = C_RelaysTmp;       // set relays back to where they were on input.
        _L_Relays = L_RelaysTmp;
        digitalWrite(coRelay, CAPS_at_INPUT);
        setRelays(C);
        setRelays(L);
      }
    }
    getSWR();
    doRelayFineSteps();
#ifdef DEBUG_TUNE_SUMMARY
    tuneSummary();
#endif
  }
//  delay(DELAY);
}

// Subroutines start here
/**********************************************************************************************************/
void doRelayFineSteps() {
  
  float bestSWR;  
  float swrTemp;
  byte CrelaysTemp = _C_Relays;
  byte LrelaysTemp = _L_Relays;
#ifdef DEBUG_RELAY_FINE_STEPS
  int cnt = 1;
#endif

  Serial.print("doRelayFineSteps: entry, _SWR = "); Serial.println(_SWR, 4);
  swrTemp = _SWR;
  do {
    bestSWR = swrTemp;
    swrTemp = fineStep_C(swrTemp);
    swrTemp = fineStep_L(swrTemp);
#ifdef DEBUG_RELAY_FINE_STEPS    
    Serial.print("doRelayFineSteps():  Been through loop "); Serial.print(cnt); Serial.println(" times.");
    cnt++;
#endif    
  } while(swrTemp < bestSWR);
  
/*  Serial.print("doRelayFineSteps: entry, _SWR = "); Serial.println(_SWR, 4);
  swrTemp = fineStep_C(_SWR);
  Serial.print("doRelayFineSteps: 1st C, swrTemp = "); Serial.println(swrTemp, 4);
  swrTemp = fineStep_L(swrTemp);  
  Serial.print("doRelayFineSteps: 1st L, swrTemp = "); Serial.println(swrTemp, 4);
  swrTemp = fineStep_C(swrTemp);
  Serial.print("doRelayFineSteps: 2nd C, swrTemp = "); Serial.println(swrTemp, 4);
  swrTemp = fineStep_L(swrTemp);
  Serial.print("doRelayFineSteps: 2nd L, swrTemp = "); Serial.println(swrTemp, 4);
*/  
  
  if(_SWR > bestSWR) {
    _SWR = bestSWR;
    Serial.print("Doing if clause:  _SWR = "); Serial.print(_SWR, 7);
    Serial.print("; and swrTemp = "); Serial.println(swrTemp, 7);
  }
  else {
    // Leave original _SWR untouched
    _C_Relays = CrelaysTemp; // Restore C & L Relays
    _L_Relays = LrelaysTemp;
    setRelays(C);
    setRelays(L);
    Serial.print("Doing else clause:  _SWR = "); Serial.print(_SWR, 7);
    Serial.print("; and swrTemp = "); Serial.println(swrTemp, 7);
  }
}

/**********************************************************************************************************/
float fineStep_C(float bestSWR){
  
  float swrTemp;
  bool improved;
  
//  bestSWR = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.print("fineStep_C():  bestSWR = "); Serial.println(bestSWR, 4);
  Serial.println("bestSWR\tfwdVolt\trevVolt\ttotC\ttotL\tC_relays\tL_relays\tCapacitor step up.");
#endif
//Start off by tweaking the C relays. We will increase capacitance as first try.
  improved = false;
  swrTemp = bestSWR;
  while(swrTemp <= bestSWR) { // We got an improvement
#ifdef DEBUG_RELAY_FINE_STEPS
  printFineSteps(bestSWR);
#endif
    if(swrTemp < bestSWR){ // Will be equal to, not less than, on entry so improved is not set.
      improved = true;
    }
    bestSWR = swrTemp; // 1st time through, it is already equal
    if(_C_Relays < B11111111) { // Step to next capacitor value only if it won't step beyond maximum C.
      _C_Relays++;
      setRelays(C);
    } else {
      _C_Relays++; // Dummy step because of the _C_Relays-- after exit from while loop
      break;
    }
    swrTemp = getSWR();        
  }  //endwhile 
  _C_Relays--; // When we exit, we have stepped one capacitor step too far, so back up one to best value.
  setRelays(C);
  swrTemp = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("Values on exit from capacitor fine steps up.");
  printFineSteps(swrTemp);
#endif  
  
  
  if(!improved){ // We were going the wrong way by increasing C so try reducing it.
    Serial.println("bestSWR\tfwdVolt\trevVolt\ttotC\ttotL\tC_relays\tL_relays\tDoing Capacitor step down.");
    swrTemp = bestSWR;
    while(swrTemp <= bestSWR) { // We got an improvement
#ifdef DEBUG_RELAY_FINE_STEPS
  printFineSteps(bestSWR);
#endif
      if(swrTemp < bestSWR){ // Will be equal to, not less than, on entry so improved is not set.
        improved = true;
      }
      bestSWR = swrTemp; // 1st time through, already equal
      if(_C_Relays < B00000000) { // Step down to next capacitor value only if it won't step below minimum C.
        _C_Relays--;
        setRelays(C);
      } else {
        _C_Relays--; // Dummy step as _C_Relays stepped up 1 on exit from while loop
        break;
      }
      swrTemp = getSWR();        
    }  //endwhile 
    _C_Relays++;
    setRelays(C);
    swrTemp = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("Values on exit from capacitor fine steps down.");
  printFineSteps(swrTemp);
#endif 
  }
  return swrTemp;
}

/**********************************************************************************************************/
float fineStep_L(float bestSWR){

  float swrTemp;
  bool improved;
  
//  bestSWR = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.print("fineStep_L():  bestSWR = "); Serial.println(bestSWR, 4);
  Serial.println("bestSWR\tfwdVolt\trevVolt\ttotC\ttotL\tC_relays\tL_relays\tInductor step up.");
#endif
//Start off by tweaking the L relays. We will increase inductance as first try.
  improved = false;
  swrTemp = bestSWR;
  while(swrTemp <= bestSWR) { // We got an improvement
#ifdef DEBUG_RELAY_FINE_STEPS
  printFineSteps(bestSWR);
#endif
    if(swrTemp < bestSWR){ // Will be equal to, not less than, on entry so improved is not set.
      improved = true;
    }
    bestSWR = swrTemp; // 1st time through, already equal
    if(_L_Relays < B11111111) { // Step to next inductor value only if it won't step beyond maximum L.
      _L_Relays++;
      setRelays(L);
    } else {
      _L_Relays++; // Dummy step as _L_Relays will be stepped back 1 on exit from while loop
      break;
    }
    swrTemp = getSWR();        
  }  //endwhile 
  _L_Relays--; // When we exit, we have stepped one inductor step too far, so back up one to best value.
  setRelays(L);
  swrTemp = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("Values on exit from inductor fine steps up.");
  printFineSteps(swrTemp);
#endif  
  
  
  if(!improved){ // We were going the wrong way by increasing L so try reducing it.
    Serial.println("bestSWR\tfwdVolt\trevVolt\ttotC\ttotL\tC_relays\tL_relays\tDoing Inductor step down.");
    swrTemp = bestSWR;
    while(swrTemp <= bestSWR) { // We got an improvement
#ifdef DEBUG_RELAY_FINE_STEPS
  printFineSteps(bestSWR);
#endif
      if(swrTemp < bestSWR){ // Will be equal to, not less than, on entry so improved is not set.
        improved = true;
      }
      bestSWR = swrTemp; // 1st time through, already equal
      if(_L_Relays < B00000000) { // Step down to next inductor value only if it won't step below minimum L.
        _L_Relays--;
        setRelays(L);
      } else {
        _L_Relays--; // Dummy step as _L_Relays will be stepped up 1 on exit from while loop
        break;
      }
      swrTemp = getSWR();        
    }  //endwhile 
    _L_Relays++;
    setRelays(L);
    swrTemp = getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println("Values on exit from inductor fine steps down.");
  printFineSteps(swrTemp);
#endif 
  }
  return swrTemp;  
}

/**********************************************************************************************************/
void printFineSteps(float bestSWR) {
  Serial.print(bestSWR, 4);Serial.print("\t");Serial.print(_fwdVolts);Serial.print("\t");Serial.print(_revVolts);
  Serial.print("\t");Serial.print(calcXvalue(C));Serial.print("\t");Serial.print(calcXvalue(L));Serial.print("\t");
  print_binary(_C_Relays, 8);Serial.print("\t");print_binary(_L_Relays, 8);Serial.println();
}

/**********************************************************************************************************/

void doRelayCourseSteps(byte position){

  float currentSWR;
  float bestSWR;
  float co_bestSWR;
  byte bestC;
  byte bestL;
  byte bestCnt = 0;
  byte cnt = 0;

  // Initialise with no relays operated, no changeover relay and SWR at this state
  _C_Relays = 0;
  _L_Relays = 0;
  setRelays(L); // Switch off all the Inductor relays
  setRelays(C); // Switch off all the Capacitor relays
  if(position == C_at_Input) {
    digitalWrite(coRelay, CAPS_at_INPUT);
  } 
  else {
    digitalWrite(coRelay, CAPS_at_OUTPUT); // Switch capacitors to output side of L network
  }
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.print("doRelayCourseSteps:  Doing Capacitor sequence with caps at ");
  if(position == C_at_Input) Serial.println("Input"); 
  else Serial.println("Output");
  Serial.print("cnt");
  Serial.print("\t");
  Serial.print("bestCnt");
  Serial.print("\t");
  Serial.print("_C_Relays");
  Serial.print("  ");
  Serial.print("_L_Relays");
  Serial.print("  ");
  Serial.print("fwdVolt");
  Serial.print("\t");
  Serial.print("revVolt");
  Serial.print("\t");
  Serial.print("curSWR");
  Serial.print("\t");
  Serial.println("bestSWR");
#endif

  currentSWR = getSWR();  //Get SWR with no relays operated at this point.
  bestSWR = currentSWR + 0.0001; // Dummy value to force bestSWR to be written from
  // currentSWR first time through for loop
  // here we set the capacitor relays one by one from 0 relays operated (cnt = 0)
  // through first to 8th relay (cnt = 1 to 8), checking to see which relay produces
  // the lowest SWR

  for(cnt = 0; cnt < 9; cnt++){
    if(cnt > 0){
      _C_Relays = 0;
      bitSet(_C_Relays,cnt - 1);
      setRelays(C); // Stepping through the Capacitor relays
      currentSWR = getSWR();
    }
    if(currentSWR < bestSWR){
      bestSWR = currentSWR;
      bestCnt = cnt;
    }
#ifdef DEBUG_COARSE_TUNE_STATUS
    Serial.print(cnt);
    Serial.print("\t");
    Serial.print(bestCnt);
    Serial.print("\t");
    print_binary(_C_Relays, 8);
    Serial.print("  ");
    print_binary(_L_Relays, 8);
    Serial.print("  ");
    formatINT(_fwdVolts);
    Serial.print(_fwdVolts);
    Serial.print("\t");
    formatINT(_revVolts);
    Serial.print(_revVolts);
    Serial.print("\t");
    Serial.print(currentSWR, 3);
    Serial.print("\t");
    Serial.println(bestSWR, 3);
#endif
  }
  _C_Relays = 0;
  if(bestCnt > 0) bitSet(_C_Relays, bestCnt - 1);
  setRelays(C); // Leave capacitors with best capacitor set

  // At this point we have found the capacitor which gives the lowest SWR. Now try Inductors.
  // Inductor relays are all released and we have stored bestSWR for this state.
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.println();
  Serial.print("doRelayCourseSteps:  Doing Inductor sequence with caps at ");
  if(position == C_at_Input) Serial.println("Input"); 
  else Serial.println("Output");
#endif
  bestCnt = 0;  //Start by assuming no Inductor Relays gives best SWR
  for(cnt = 0; cnt < 9; cnt++){
    if(cnt > 0){
      _L_Relays = 0;
      bitSet(_L_Relays, cnt - 1);
      setRelays(L); // Stepping through the Inductor relays
    }
    // Here check for an inductor which gives better SWR than capacitors alone. If a better SWR can't
    // be found then bestSWR is not altered, _L_Relays is left at 0 and bestCnt also left at 0.
    currentSWR = getSWR();
    if(currentSWR < bestSWR){
      bestSWR = currentSWR;
      bestCnt = cnt;
    }
#ifdef DEBUG_COARSE_TUNE_STATUS
    Serial.print(cnt);
    Serial.print("\t");
    Serial.print(bestCnt);
    Serial.print("\t");
    print_binary(_C_Relays, 8);
    Serial.print("  ");
    print_binary(_L_Relays, 8);
    Serial.print("  ");
    formatINT(_fwdVolts);
    Serial.print(_fwdVolts);
    Serial.print("\t");
    formatINT(_revVolts);
    Serial.print(_revVolts);
    Serial.print("\t");
    Serial.print(currentSWR, 3);
    Serial.print("\t");
    Serial.println(bestSWR, 3);
#endif
  }
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.println("Capacitors are connected to L network ");
  if(position == C_at_Input) {
    Serial.println("Input");
  } 
  else {
    Serial.println("Output");
  }
  Serial.println();
#endif
  _L_Relays = 0;
  if(bestCnt > 0) bitSet(_L_Relays, bestCnt - 1);
  setRelays(L); // Best Inductor now set.
  _SWR = getSWR(); 
}

/**********************************************************************************************************/
// We calculate the total values of L or C. CorL is a flag to determine which reactance to sum up.
unsigned int calcXvalue(bool CorL){
  unsigned int val = 0;

  for (byte cnt = 0; cnt < 8; cnt++) {
    if (CorL) {   // True do capacitors, false do inductors
      if(bitRead(_C_Relays, cnt)) val = val + _capacitors[cnt]; // add reactance assigned to set bits only.
    } else {     
      if(bitRead(_L_Relays, cnt)) val = val + _inductors[cnt];
    } 
  }
  return val;
}

/**********************************************************************************************************/

void setRelays(boolean relaySet) {
  // Writes a byte either _C_Relays or _L_Relays to a shift register. The shift register chosen
  // depends on "relaySet" value where true = C and false = L.

  //  byte relays;

  if(relaySet){
    //    relays = _C_Relays;
    shiftOut(Cdata, Cclock, MSBFIRST, _C_Relays); // send this binary value to the Capacitor shift register
#ifdef DEBUG_CURRENT_FUNCTION
    Serial.print("Current function = setRelays(byte value, boolean relaySet) where relaySet = ");
    Serial.print(relaySet);
    Serial.print(" and _C_Relays value = ");
    Serial.println(_C_Relays);
#endif    
  } 
  else {
    //    relays = _L_Relays;
    shiftOut(Ldata, Lclock, MSBFIRST, _L_Relays); // send this binary value to the Inductor shift register
#ifdef DEBUG_CURRENT_FUNCTION
    Serial.print("Current function = setRelays(byte value, boolean relaySet) where relaySet = ");
    Serial.print(relaySet);
    Serial.print(" and _L_Relays value = ");
    Serial.println(_L_Relays);
#endif    
  }
#ifdef DEBUG_RELAY_STATE
  dbugRelayState();
#endif
  delay(DELAY);
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

float getSWR() {
  // Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
  // (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.

  //       We are using the GLOBAL VARIABLES _fwdVolts, _revVolts, _SWR
  //       All globals are prefixed with an underscore e.g. _fwdVolts
  float swrTemp;
  
  digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
  digitalWrite(LEDpin, HIGH);     // Indicate state of amplifier gain
  delay(20); // Settling time after FET switching to stabilize analog input
  _fwdVolts = analogRead(forward);
//  Serial.print("getSWR: before amp gain, _fwdVolts = "); Serial.println(_fwdVolts); // Temporary debug
  if(_fwdVolts == 1023) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    digitalWrite(LEDpin, LOW);   // Indicate switched to low gain
    delay(20);
    _fwdVolts = analogRead(forward); // and re-read the forward power
  }
//  Serial.print("getSWR: after amp gain, _fwdVolts = "); Serial.println(_fwdVolts); // Temporary debug
  _revVolts = analogRead(reverse);
  if(_fwdVolts > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    if (_fwdVolts <= _revVolts) _revVolts = (_fwdVolts - 1); //Avoid division by zero or negative.
    swrTemp = float((_fwdVolts + _revVolts)) / float((_fwdVolts - _revVolts));
  } 
  else {
    swrTemp = 100;
  }
#ifdef DEBUG_SWR_VALUES
  Serial.print("getSWR: fwd, rev, swr = ");
  Serial.print(_fwdVolts);
  Serial.print(", ");
  Serial.print(_revVolts);
  Serial.print(", ");
  Serial.println(_SWR, 4);
#endif
  return swrTemp;
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
  Serial.print("_C_Relays value = ");
  Serial.print(_C_Relays);
  Serial.print(", C = ");
  Serial.print(calcXvalue(C));
  Serial.print(" pF");
  Serial.print("      ... _L_Relays value = ");
  Serial.print(_L_Relays);
  Serial.print(", L = ");
  Serial.print(calcXvalue(L));
  Serial.println(" nH");
  Serial.println("C Relay# 1 2 3 4 5 6 7 8 ... L Relay# 1 2 3 4 5 6 7 8");
  Serial.print("         ");
  for(int x = 0; x < 8; x++){
    Serial.print(bitRead(_C_Relays, x));
    Serial.print(" ");
  }
  Serial.print("             ");
  for(int x = 0; x < 8; x++){
    Serial.print(bitRead(_L_Relays, x));
    Serial.print(" ");
  }
  Serial.print("SWR = ");
  Serial.println(getSWR());
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
      } else
      {
      // Record the button number. While the button is held down, multiple calls will be made to the subroutine
      // so we save the number multiple times but don't use it until the trailing edge is detected.
        last_button_pressed = analogbuttontemp;
      // We dont want any button action while the short press button is pressed so a dummy return is made
        analogbuttontemp = 255;
      }
  } else analogbuttontemp = 255;  
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
void tuneSummary() {
  Serial.print("_C_Relays");
  Serial.print("  ");
  Serial.print("_L_Relays");
  Serial.print("  ");
  Serial.print("fwdVolts");
  Serial.print("\t");
  Serial.print("revVolts");
  Serial.print("\t");
  Serial.println("SWR");
  print_binary(_C_Relays, 8);
  Serial.print("  ");
  print_binary(_L_Relays, 8);
  Serial.print("  ");
  formatINT(_fwdVolts);
  Serial.print(_fwdVolts);
  Serial.print("\t");
  formatINT(_revVolts);
  Serial.print(_revVolts);
  Serial.print("\t\t");
  Serial.println(_SWR, 4);
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
