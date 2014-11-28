//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Controller for the EB104.ru Auto Antenna Tuner
//Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

//Shift Register for L & C driver Pin assign
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

void setup() { 
  pinMode(Cclock, OUTPUT); // make the Capacitor clock pin an output
  pinMode(Cdata , OUTPUT); // make the Capacitor data pin an output
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output  

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  // Debug The coRelay will be an output in final. Used to detect C or L relays to be stepped
  pinMode(coRelay, INPUT);
  digitalWrite(coRelay, HIGH); // pull-up activated
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600); 
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
    if (digitalRead(coRelay)){
      Serial.print(", doing C relays, digitalRead(coRelay) = ");
      Serial.println(digitalRead(coRelay));
      step_CorL(C); // Do capacitor relays
    } else {
      Serial.print(", doing L relays, digitalRead(coRelay) = ");
      Serial.println(digitalRead(coRelay));
      step_CorL(L);  // Do Inductor relays
    }
  }
  delay(DELAY);
}

// relaySet = true, do Capacitor relay set
void step_CorL(boolean relaySet) {
  // The first time called no relays would have been operated so we want to operate #1
  int count = 0;  // General loop counter etc.
  byte relays;
  
  if(relaySet){
    relays = _C_Relays;
  } else {
    relays = _L_Relays;
  }
  Serial.println("Arrived at step_CorL(boolean relaySet)");
  Serial.println();
  
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
  
  setRelays(relaySet);

  delay(DELAY);
  Serial.println("--------------------------------------------------------------------------------");
}

// Writes a byte either _C_Relays or _L_Relays to a shift register. The shift register chosen
// depends on "relaySet" value where true = C and false = L.
void setRelays(boolean relaySet) {
  byte relays;
  
  Serial.println("Function = setRelays(byte value, boolean relaySet)");
  if(relaySet){
    relays = _C_Relays;
    shiftOut(Cdata, Cclock, LSBFIRST, relays); // send this binary value to the Capacitor shift register
    Serial.print("Writing to _C_Relays using value of ");
    Serial.println(relays);
  } else {
    relays = _L_Relays;
    shiftOut(Ldata, Lclock, LSBFIRST, relays); // send this binary value to the Inductor shift register
    Serial.print("Writing to _L_Relays using value of ");
    Serial.println(relays);
  }
  Serial.println("Relay# 1 2 3 4 5 6 7 8");
  Serial.print("       ");
  for(int x = 0; x < 8; x++){
    Serial.print(bitRead(relays, x));
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
  delay(DELAY);
}
/*
void stepL() {
  static int nextRelay = 1;
  
  Serial.print("Operating Inductor relay number ");
  Serial.println(nextRelay);
  
  int current = OUTL_0 + nextRelay - 1; //OUTC_0..7 mapped from 1 ..8
  nextRelay++; // Increment for next time through
  if (nextRelay == 1) {
    clearRelays(true, false);
  } else {
    if (nextRelay > 8) nextRelay = 0;
    if (current != OUTL_0) {           // Don't release previous relay if doing relay 1
      digitalWrite(current - 1, LOW);  
      delay(DELAY);
    }
    digitalWrite(current, HIGH);      //Turn on next relay in sequence
    delay(DELAY);
  }
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
/*
void clearRelays(boolean clearL, boolean clearC) {
  if (clearC) {
    Serial.print("Relay number cleared = "); // Debug message
    for(int i = OUTC_0; i <= OUTC_7; i++){ 
      digitalWrite(i, LOW);   // Set all Capacitor relays off
      Serial.print(i - 2);  // Debug message
      Serial.print(", ");  // Debug message
    }
    Serial.println("Capacitor Relays cleared");
  }
  if (clearL) {
    Serial.print("Relay number cleared = "); // Debug message
    for(int i = OUTL_0; i <= OUTL_7; i++){ 
      digitalWrite(i, LOW);   // Set all Inductortor relays off
      Serial.print(i - 12);  // Debug message
      Serial.print(", ");  // Debug message
    }
    Serial.println("Inductor Relays cleared");   
  }  
  delay(DELAY);  // Give relays time to release before returning
}
*/
void fineSteps_C(boolean dir) {
  /*
 if (data & (1<<y)) {
                    // turn on the LED at location (x,y)
 } else {
                    // turn off the LED at location (x,y)
 }
 */
}

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

// Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
// (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.
unsigned int getSWR() {
  long fwdPwr;
  long revPwr;
  unsigned int swr;
  
  revPwr = analogRead(reverse);
  fwdPwr = analogRead(forward);
  if (fwdPwr <= revPwr) revPwr = (fwdPwr - 1); //Avoid division by zero or negative.
  swr = ((fwdPwr + revPwr) * 1000) / (fwdPwr - revPwr);// Multiply by 1000 avoids floats
  Serial.print("SWR readings fwd, rev, fwd+rev*1000, swr = "); // & keeps precision.
  Serial.print(fwdPwr);
  Serial.print(", ");
  Serial.print(revPwr);
  Serial.print(", ");
  Serial.print((fwdPwr + revPwr) * 1000);
  Serial.print(", ");
  Serial.println(swr);
  
  return swr;
}

