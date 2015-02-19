//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Controller for the EB104.ru Auto Antenna Tuner
//Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// Debug Defines
//#define DEBUG_RELAY_STATE
#define DEBUG_COARSE_TUNE_STATUS
#define DEBUG_TUNE_SUMMARY
//#define DEBUG_CURRENT_FUNCTION
//#define DEBUG_SWR_VALUES
#define DEBUG_SHIFT

#define TX_LEVEL_THRESHOLD 5
#define CAPS_at_INPUT      LOW    //For digitalWrites to Capacitor I/O changeover relay
#define CAPS_at_OUTPUT     HIGH

// Shift Register for L & C driver Pin assign
#define Cclock 2  //Pin 8 of 74HC164 U4, pin 20 of Arduino nano
#define Cdata 3   //Pin 2 of 74HC164 U4, pin 21 of Arduino nano
#define Lclock 4  //Pin 8 of 74HC164 U3, pin 22 of Arduino nano
#define Ldata 5   //Pins 1 & 2 of 74HC164 U3, pin 23 of Arduino nano
#define LEDpin 13 //A LED is connected to this pin

#define coRelay       7    // Capacitor set c/o relay
#define swrGain       8    //Switchable gain for swr amplifiers
#define BUTTON_PIN    6    // Push Button

#define forward       A0  // Measure forward SWR on this pin
#define reverse       A1  // Measure reverse SWR on this pin

#define DELAY         50  // Delay per loop in ms

#define C             true    // Capacitor relay set
#define L             false   // Inductor relay set
#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final

// Global variables always start with an underscore
byte _C_Relays = 0; // Holds map of operated relays with
byte _L_Relays = 0; // 0 = released and 1 = operated
int _fwdVolts;
int _revVolts;
float _SWR;
enum _C_STATE{C_at_Input, C_at_Output};
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
  setRelays(C); // Switch off all the Capacitor relays ( _C_Relays = 0 )
  setRelays(L); // Switch off all the Inductor relays ( _L_Relays = 0 )
} 
/**********************************************************************************************************/

void loop(){
  byte C_RelaysTmp; // Holds map of operated relays with C/O on input
  byte L_RelaysTmp; // 0 = released and 1 = operated
  float SWRtmp;
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
      Serial.println("main Loops:  After caps on input been completed summary");
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
      Serial.print("\t");
      Serial.print(_SWR, 4);
      Serial.print("\tThe capacitors are connect to the ");
      if(digitalRead(coRelay) == LOW) Serial.println("Input"); else Serial.println("Output");
      Serial.println();
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
    #ifdef DEBUG_TUNE_SUMMARY
      Serial.println("main Loops:  After tuning has been completed summary");
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
      Serial.print("\t");
      Serial.print(_SWR, 4);
      Serial.print("\tThe capacitors are connect to the ");
      if(digitalRead(coRelay) == LOW) Serial.println("Input"); else Serial.println("Output");
      Serial.println();
    #endif
  }
  delay(DELAY);
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
  } else {
    digitalWrite(coRelay, CAPS_at_OUTPUT); // Switch capacitors to output side of L network
  }
    #ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print("doRelayCourseSteps:  Doing Capacitor sequence with caps at ");
      if(position == C_at_Input) Serial.println("Input"); else Serial.println("Output");
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
    if(position == C_at_Input) Serial.println("Input"); else Serial.println("Output");
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
    } else {
      Serial.println("Output");
    }
    Serial.println();
  #endif
  _L_Relays = 0;
  if(bestCnt > 0) bitSet(_L_Relays, bestCnt - 1);
  setRelays(L); // Best Inductor now set. 
}
/**********************************************************************************************************/

void doRelayFineSteps() {
  
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
  } else {
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
  
  digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
  digitalWrite(LEDpin, HIGH);     // Indicate state of amplifier gain
  _fwdVolts = analogRead(forward);
  if(_fwdVolts > 950) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    digitalWrite(LEDpin, LOW);
    _fwdVolts = analogRead(forward); // and re-read the forward power
  }
  _revVolts = analogRead(reverse);
  if(_fwdVolts > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    if (_fwdVolts <= _revVolts) _revVolts = (_fwdVolts - 1); //Avoid division by zero or negative.
    _SWR = float((_fwdVolts + _revVolts)) / float((_fwdVolts - _revVolts));
  } else {
    _SWR = 100;
  }
  #ifdef DEBUG_SWR_VALUES
    Serial.print("getSWR: fwd, rev, swr = ");
    Serial.print(_fwdVolts);
    Serial.print(", ");
    Serial.print(_revVolts);
    Serial.print(", ");
    Serial.println(_SWR, 4);
  #endif
  return _SWR;
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
  Serial.print("      ... _L_Relays value = ");
  Serial.println(_L_Relays);
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
  Serial.println();
}
