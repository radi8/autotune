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
#define Cclock 2
#define Cdata 3
#define Lclock 4
#define Ldata 5

#define coRelay       7    // Capacitor set c/o relay
#define swrGain       8    //Switchable gain for swr amplifiers
#define BUTTON_PIN    10   // Push Button

#define forward       A0  // Measure forward SWR on this pin
#define reverse       A1  // Measure reverse SWR on this pin

#define DELAY         20  // Delay per loop in ms

#define C             true    // Capacitor relay set
#define L             false   // Inductor relay set
#define Up            true    // Debug item, remove in final
#define Dn            false   // Debug item, remove in final

// Global variables always start with an underscore
byte _C_Relays = 0; // Holds map of operated relays with
byte _L_Relays = 0; // 0 = released and 1 = operated
unsigned int _SWR;
/**********************************************************************************************************/

void setup() { 
  pinMode(Cclock, OUTPUT); // make the Capacitor clock pin an output
  pinMode(Cdata , OUTPUT); // make the Capacitor data pin an output
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output  
  pinMode(coRelay, OUTPUT);
  pinMode(swrGain, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  digitalWrite(coRelay, LOW); // Set capacitor C/O relay to input side
  digitalWrite(swrGain, LOW); // Turns off fet shunting swr Start with highest gain for amps.voltages
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  setRelays(C); // Switch off all the Capacitor relays ( _C_Relays = 0 )
  setRelays(L); // Switch off all the Inductor relays ( _L_Relays = 0 )
} 
/**********************************************************************************************************/

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
/**********************************************************************************************************/

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
/**********************************************************************************************************/

void doRelayFineSteps() {
  
}
/**********************************************************************************************************/

void setRelays(boolean relaySet) {
// Writes a byte either _C_Relays or _L_Relays to a shift register. The shift register chosen
// depends on "relaySet" value where true = C and false = L.

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
  delay(DELAY);
}
/**********************************************************************************************************/

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
/**********************************************************************************************************/

float getSWR() {
// Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
// (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.

  float fwdPwr;
  float revPwr;
  unsigned int swr;
  
  digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
  fwdPwr = analogRead(forward);
  if(fwdPwr > 950) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    fwdPwr = analogRead(forward); // and re-read the forward power
  }
  revPwr = analogRead(reverse);
  if(fwdPwr == 0) fwdPwr = 0.01; // Keep away from values causing divide by 0 errors
  if(revPwr == 0) revPwr = 0.01;
  if (fwdPwr <= revPwr) fwdPwr = (revPwr + 0.001); //Avoid division by zero or negative.
  swr = (fwdPwr + revPwr) / (fwdPwr - revPwr);
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
