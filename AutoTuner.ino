//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Controller for the EB104.ru Auto Antenna Tuner
//Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// Debug Defines
#define DEBUG_RELAY_STATE
#define DEBUG_CURRENT_FUNCTION
#define DEBUG_SWR_VALUES
// Shift Register for L & C driver Pin assign
#define Cdata 2
#define Cclock 3
#define Ldata 4
#define Lclock 5

#define BUTTON_PIN    10   // Push Button
#define coRelay       11   // Capacitor set c/o relay

#define forward       A6  // Measure forward SWR on this pin
#define reverse       A7  // Measure reverse SWR on this pin

#define DELAY         20  // Delay per loop in ms

#define C             true    // Capacitor relay set
#define L             false   // Inductor relay set
#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final

// Global variables
byte _C_Relays = 0; // Holds map of operated relays with
byte _L_Relays = 0; // 0 = released and 1 = operated
unsigned int _SWR;

void setup() { 
  pinMode(Cclock, OUTPUT); // make the Capacitor clock pin an output
  pinMode(Cdata , OUTPUT); // make the Capacitor data pin an output
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output  
  pinMode(coRelay, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  digitalWrite(coRelay, LOW); // Set capacitor C/O relay to input side
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  setRelays(C); // Switch off all the Capacitor relays ( _C_Relays = 0 )
  setRelays(L); // Switch off all the Inductor relays ( _L_Relays = 0 )
} 

void loop(){
  // The button press will step the selected Capacitor or Inductor relays
  // handle button
  boolean button_pressed = handle_button();
  if (button_pressed) {
    Serial.print("button_pressed");
    doRelayCourseSteps();
  }
  delay(DELAY);
}

void doRelayCourseSteps(){
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
  digitalWrite(coRelay, LOW); // Set capacitors to input side of L network
  currentSWR = getSWR();
  bestSWR = currentSWR + 1; // Dummy value to force bestSWR to be written from
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
        // debug
    if(cnt == 6) currentSWR = bestSWR - 5; // Make currentSWR worse than bestSWR
    if(currentSWR < bestSWR){
      bestSWR = currentSWR;
      bestCnt = cnt;
    }
    Serial.print("cnt = ");
    Serial.print(cnt);
    Serial.print(",  bestCnt = ");
    Serial.print(bestCnt);
    Serial.print(",  _C_Relays = ");
    Serial.print(_C_Relays);
    Serial.print(",  currentSWR = ");
    Serial.print(currentSWR);
    Serial.print(",  bestSWR = ");
    Serial.println(bestSWR);
    Serial.println("******************************************************");
  }
  Serial.print("############## Current value of cnt = ");
  Serial.println(cnt);
  _C_Relays = 0;
  if(bestCnt > 0) bitSet(_C_Relays, bestCnt - 1);
  setRelays(C); // Leave capacitors with best capacitor set

  // At this point we have found the capacitor which gives the lowest SWR. Now try Inductors.
  // Inductor relays are all released and we have bestSWR for this state.
  bestCnt = 0;
  for(cnt = 0; cnt < 9; cnt++){
    if(cnt > 0){
      _L_Relays = 0;
      bitSet(_L_Relays,cnt - 1);
      setRelays(L); // Stepping through the Capacitor relays
      currentSWR = getSWR();
    }
        // debug
    if(cnt == 1) currentSWR = bestSWR - 5; // Make currentSWR worse than bestSWR
    if(currentSWR < bestSWR){
      bestSWR = currentSWR;
      bestCnt = cnt;
    }
    Serial.print("cnt = ");
    Serial.print(cnt);
    Serial.print(",  bestCnt = ");
    Serial.print(bestCnt);
    Serial.print(",  _L_Relays = ");
    Serial.print(_L_Relays);
    Serial.print(",  currentSWR = ");
    Serial.print(currentSWR);
    Serial.print(",  bestSWR = ");
    Serial.println(bestSWR);
    Serial.println("******************************************************");
  }
  Serial.print("############## Current value of bestCnt = ");
  Serial.println(bestCnt);
  _L_Relays = 0;
  if(bestCnt > 0) bitSet(_L_Relays, bestCnt - 1);
  setRelays(L); // Best Inductor now set and we are ready for finding if coRelay needed  
}

// Writes a byte either _C_Relays or _L_Relays to a shift register. The shift register chosen
// depends on "relaySet" value where true = C and false = L.
void setRelays(boolean relaySet) {
  byte relays;
  
  #ifdef DEBUG_CURRENT_FUNCTION
    Serial.print("Current function = setRelays(byte value, boolean relaySet) where relaySet = ");
    Serial.print(relaySet);
    Serial.print(" and _C_Relays value = ");
    Serial.println(_C_Relays);
  #endif
  if(relaySet){
    relays = _C_Relays;
    shiftOut(Cdata, Cclock, MSBFIRST, relays); // send this binary value to the Capacitor shift register
  } else {
    relays = _L_Relays;
    shiftOut(Ldata, Lclock, MSBFIRST, relays); // send this binary value to the Inductor shift register
  }
  #ifdef DEBUG_RELAY_STATE
    dbugRelayState();
  #endif
//  Serial.println();
  delay(DELAY);
}

/*
// relaySet = true, do Capacitor relay set
void step_CorL(boolean relaySet) {
  // The first time called no relays would have been operated so we want to operate #1
  int count = 0;  // General loop counter etc.
  byte relays;
  
  #ifdef DEBUG_CURRENT_FUNCTION
    Serial.println("Arrived at step_CorL(boolean relaySet) with value of relaySet = ");
    Serial.println(relaySet);
  #endif
  
  if(relaySet){
    relays = _C_Relays; // _C_Relays is a byte holding the state of the capacitor relay set
  } else {
    relays = _L_Relays; // _L_Relays is a byte holding the state of the inductor relay set
  }
  
  Serial.println("Relay# 1 2 3 4 5 6 7 8");
  Serial.print("       ");
  // Get the current state of the relays as 0 .. 7 or 8 if no relays on.
   for(count = 0; count < 8; count++){    // Find which bit is set
    Serial.print(bitRead(relays, count));
    Serial.print(" ");
    if(bitRead(relays, count) == 1) break;
  }
  Serial.println();
  // count now points to currently operated relay (0 .. 7) or 8 if no relays.
  count++; // Step to next relay. count - (1 .. 8) or 9
  // if count is 9, we had been in the no relays operated state so we want to
  if(count == 9) count = 0; // operate the first relay in the 0 .. 7 sequence.
  Serial.print("The value of count = "); // If previously the last relay was
  Serial.println(count);                 // operated count will now be 8 which
                                         // is the no relays operated indicator.
  // count is now 0 ..7 or 8 if no relays are to be operated
  if(relaySet){
    _C_Relays = 0;
    if(count < 8) bitSet(_C_Relays, count); // If 8 we clear relays only
  } else {
    _L_Relays = 0;
    if(count < 8) bitSet(_L_Relays, count);
  }
  Serial.print("_C_Relays, _L_Relays values =   ");
  Serial.print(_C_Relays);
  Serial.print(",   ");
  Serial.println(_L_Relays);
  
  setRelays(relaySet); // Operate the appropriate relay

  delay(DELAY);
  Serial.println("--------------------------------------------------------------------------------");
}
*/
boolean handle_button() {
  static boolean button_was_pressed = false;
  boolean event;
  
  int button_now_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
  event = button_now_pressed && !button_was_pressed;
  if (event) { // Check if button changed
    delay(DELAY);
    int button_now_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
    event = button_now_pressed && !button_was_pressed;
  }
  button_was_pressed = button_now_pressed;
  return event;
}

void fineSteps_C(boolean dir) {
  /*
 if (data & (1<<y)) {
                    // turn on the LED at location (x,y)
 } else {
                    // turn off the LED at location (x,y)
 }
 */
}
/*
// We set each relay in turn and check to see if SWR is better or worse. If better do
// the next relay, if worse go back to previous relay. Find best "C" relay first then
// best "L" relay. Retry with capacitor relays at other end if inductors

void courseSteps() {
  unsigned int bestSWR;
  unsigned int temp;
  unsigned int co_bestSWR;
  byte bestC;
  byte bestL;
  byte co_bestL;
  
  
  //Initialise with no relays operated, no changeover relay and SWR at this state
  _C_Relays = 0;
  _L_Relays = 0;
  setRelays(C); // Switch off all the Capacitor relays
  setRelays(L); // Switch off all the Inductor relays
  digitalWrite(coRelay, LOW);
  bestSWR = getSWR();
  
  Serial.print("Initial swr reading  = ");
  Serial.println(bestSWR);
  
  // Need to insert a loop here to keep redoing this until SWR
  // stops getting better. Could use ...
  // if (bestSWR better than initialSWR ...
  // or while(bestSWR better than initial ... 
  
  // Start with the capacitors
  for(int i =0; i < 8; i++){
    _C_Relays = 0;
    bitSet(_C_Relays, i);
    setRelays(C);
    temp = getSWR();
    if (temp < bestSWR) {
     bestSWR = temp;
     bestC = _C_Relays;
    } else {
      break; // Break when SWR gets worse leaving best swr settings stored
    }
  }
  
  Serial.print("SWR after capacitors = ");
  Serial.println(bestSWR);
/*  
  // Now the inductors
  for(int i =0; i < 8; i++){
    _L_Relays = 0;
    bitSet(_L_Relays, i);
    setRelays(L);
    temp = getSWR();
    if (temp < bestSWR) { 
     bestSWR = temp; 
     bestL = _L_Relays;
    } else {    // As soon as SWR gets worse, stop adding L and
      break;    // exit leaving best swr settings.
    }
  }
  
  Serial.print("SWR after inductors  = ");
  Serial.println(bestSWR);
  
  // Now swap ends with C and retry the inductors
  digitalWrite(coRelay, HIGH);
  co_bestSWR = bestSWR;
  for(int i =0; i < 8; i++){
    _L_Relays = 0;
    bitSet(_L_Relays, i);
    setRelays(L);
    temp = getSWR();
    if (temp < co_bestSWR) {
     co_bestSWR = temp;
     co_bestL = _L_Relays;
    }
    if(co_bestSWR < bestSWR) {
      bestSWR = co_bestSWR;
      _L_Relays = co_bestL;
      Serial.println("C/O relay operated");
    } else {
      digitalWrite(coRelay, LOW);
      Serial.println("C/O relay released");
    }
  }
  
  Serial.print("SWR after c/o relay  = ");
  Serial.println(bestSWR);
  
}
*/
// Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
// (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.
float getSWR() {
  float fwdPwr;
  float revPwr;
  unsigned int swr;
  
  revPwr = analogRead(reverse);
  fwdPwr = analogRead(forward);
  if(fwdPwr == 0) fwdPwr = 0.01; // Keep away from values causing divide by 0 errors
  if(revPwr == 0) revPwr = 0.01;
  if (fwdPwr <= revPwr) fwdPwr = (revPwr + 0.001); //Avoid division by zero or negative.
  swr = (fwdPwr + revPwr) / (fwdPwr - revPwr);// Multiply by 1000 avoids floats
  #ifdef DEBUG_SWR_VALUES
    Serial.print("SWR readings fwd, rev, swr = "); // & keeps precision.
    Serial.print(fwdPwr);
    Serial.print(", ");
    Serial.print(revPwr);
    Serial.print(", ");
//    Serial.print((fwdPwr + revPwr) * 1000);
//    Serial.print(", ");
    Serial.println(swr);
  #endif
  
  return swr;
}

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
