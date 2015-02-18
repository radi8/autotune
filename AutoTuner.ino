//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Controller for the EB104.ru Auto Antenna Tuner
//Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

// Debug Defines
//#define DEBUG_RELAY_STATE
//#define DEBUG_CURRENT_FUNCTION
#define DEBUG_SWR_VALUES
#define DEBUG_SHIFT

#define TX_LEVEL_THRESHOLD 5

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
float _SWR;
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
  Serial.println("Arduino antenna tuner ver 0.1.0");
  Serial.println("(C) 2015, Graeme Jury ZL2APV");
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
    digitalWrite(coRelay, LOW); // Set capacitors to input side of L network
    doRelayCourseSteps();
    //Save SWR and relay states and see if better with C/O relay on output
    C_RelaysTmp = _C_Relays;
    L_RelaysTmp = _L_Relays;
    SWRtmp = _SWR;
    if(_SWR > 1.05) {
      digitalWrite(coRelay, HIGH); // Set capacitors to output side of L network
      Serial.println("Set capacitors to output side of L network");
      doRelayCourseSteps(); //Run it again and see if better with C/O relay operated
      //If not better restore relays to input state
      if(SWRtmp >= _SWR) {
        _C_Relays = C_RelaysTmp;
        _L_Relays = L_RelaysTmp;
        setRelays(C);
        setRelays(L);
      } else digitalWrite(coRelay, LOW);
    }
//    #ifdef DEBUG_RELAY_STATE
      Serial.print("SWR = ");
      Serial.println(_SWR);
      dbugRelayState();
      
//    #endif
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
//  digitalWrite(coRelay, LOW); // Set capacitors to input side of L network
  currentSWR = getSWR();
  bestSWR = currentSWR + 0.0001; // Dummy value to force bestSWR to be written from
                            // currentSWR first time through for loop
  // here we set the capacitor relays one by one from 0 relays operated (cnt = 0)
  // through first to 8th relay (cnt = 1 to 8), checking to see which relay produces
  // the lowest SWR
  Serial.println("Doing Capacitor sequence");
  for(cnt = 0; cnt < 9; cnt++){
    if(cnt > 0){
      _C_Relays = 0;
      bitSet(_C_Relays,cnt - 1);
      setRelays(C); // Stepping through the Capacitor relays
      currentSWR = getSWR();
    }
        // debug
//    if(cnt == 6) currentSWR = bestSWR - 5; // Make currentSWR worse than bestSWR
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
    Serial.print(currentSWR, 4);
    Serial.print(",  bestSWR = ");
    Serial.println(bestSWR, 4);
    Serial.println("******************************************************");
  }
  Serial.print("############## Current value of cnt = ");
  Serial.println(cnt);
  _C_Relays = 0;
  if(bestCnt > 0) bitSet(_C_Relays, bestCnt - 1);
  setRelays(C); // Leave capacitors with best capacitor set

  // At this point we have found the capacitor which gives the lowest SWR. Now try Inductors.
  // Inductor relays are all released and we have bestSWR for this state.
  Serial.println("Doing Inductor sequence");
  bestCnt = 0;
  for(cnt = 0; cnt < 9; cnt++){
    if(cnt > 0){
      _L_Relays = 0;
      bitSet(_L_Relays, cnt - 1);
      setRelays(L); // Stepping through the Inductor relays
      currentSWR = getSWR();
    }
        // debug
//    if(cnt == 1) currentSWR = bestSWR - 5; // Make currentSWR worse than bestSWR
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
    Serial.print(currentSWR, 4);
    Serial.print(",  bestSWR = ");
    Serial.println(bestSWR, 4);
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

void shiftOutGJ(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, byte val)
{
      int i;
      
      #ifdef DEBUG_SHIFT
        Serial.print("dataPin = ");
        Serial.print(dataPin);
        Serial.print(", ");
        Serial.print("clockPin = ");
        Serial.print(clockPin);
        Serial.print(", ");
        Serial.print("Entered with val = ");
        Serial.println(val);
      #endif  
      digitalWrite(clockPin, LOW);
      delayMicroseconds(45);
      for (i = 0; i < 8; i++)  {
            if (bitOrder == LSBFIRST) {
                  digitalWrite(dataPin, !!(val & (1 << i)));
                  delayMicroseconds(45);
            }
            else  {    
                  digitalWrite(dataPin, !!(val & (1 << (7 - i))));
                  #ifdef DEBUG_SHIFT
                    Serial.print("i = ");
                    Serial.print(i);
                    Serial.print(", ");
                    Serial.print("Shifted value of val = ");
                    Serial.println(!!(val & (1 << (7 - i))));
                  #endif
                  delayMicroseconds(45);
            }    
            digitalWrite(clockPin, HIGH);
            delayMicroseconds(45);
            digitalWrite(clockPin, LOW);
            delayMicroseconds(45);    
      }
      #ifdef DEBUG_SHIFT      
        Serial.print("Exited with val  = ");
        Serial.println(val);
      #endif
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

  int fwdVolts;
  int revVolts;
  
  digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
  digitalWrite(LEDpin, HIGH);     // Indicate state of amplifier gain
  fwdVolts = analogRead(forward);
  if(fwdVolts > 950) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    digitalWrite(LEDpin, LOW);
    fwdVolts = analogRead(forward); // and re-read the forward power
  }
  revVolts = analogRead(reverse);
  if(fwdVolts > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    if (fwdVolts <= revVolts) revVolts = (fwdVolts - 1); //Avoid division by zero or negative.
    _SWR = float((fwdVolts + revVolts)) / float((fwdVolts - revVolts));
  } else {
    _SWR = 100;
  }
  #ifdef DEBUG_SWR_VALUES
    Serial.print("getSWR: fwd, rev, swr = ");
    Serial.print(fwdVolts);
    Serial.print(", ");
    Serial.print(revVolts);
    Serial.print(", ");
    Serial.println(_SWR, 4);
  #endif
  return _SWR;
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
