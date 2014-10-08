//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Controller for the EB104.ru Auto Antenna Tuner
//Course stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

//Digital outputs for L & C Latches
#define OUTC_0         2  // Must be contiguous
#define OUTC_1         3
#define OUTC_2         4
#define OUTC_3         5
#define OUTC_4         6
#define OUTC_5         7
#define OUTC_6         8
#define OUTC_7         9

#define BUTTON_PIN    10   // Push Button
#define coRelay       11   // Capacitor set c/o relay

#define OUTL_0        12  // Must be contiguous
#define OUTL_1        13
#define OUTL_2        14  // A0
#define OUTL_3        15  // A1
#define OUTL_4        16  // A2
#define OUTL_5        17  // A3
#define OUTL_6        18  // A4
#define OUTL_7        19  // A5

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
int  _revSWR   = 0;

void setup() { 
  // initialize the digital pins as outputs.
  for(int i = OUTC_0; i <= OUTC_7; i++){ // Set all C Relay signals to output
    pinMode(i, OUTPUT);
  }

  for(int i = OUTL_0; i <= OUTL_7; i++){ // Set all L Relay signals to output
    pinMode(i, OUTPUT);
  }  

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
  setRelays(OUTC_0, C); // Switch off all the Capacitor relays ( _C_Relays = 0 )
  setRelays(OUTL_0, L); // Switch off all the Inductor relays ( _L_Relays = 0 )
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
  byte lastRelay;   // Relays currently operated. Zero = no relays operated
  byte nextRelay;   // Relays to be operated by this function
  int count = 0;  // General loop counter etc.
  byte relays;
  byte offset;
  
  if(relaySet){
    relays = _C_Relays;
    offset = OUTC_0;
  } else {
    relays = _L_Relays;
    offset = OUTL_0;
  }
  Serial.print("The value of relays = ");
  Serial.println(relays);
  Serial.print("The value of offset = ");
  Serial.println(offset);
  // Loop to find which bit is set. If no bits are set, count will get to 8
  for(count = 0; count < 8; count++){    // Find which bit is set
    Serial.print("The value of bitRead(relays, count) = ");
    Serial.println(bitRead(relays, count));
    if(bitRead(relays, count) == 1) break;
  }
  count++; // Step to next relay
  Serial.print("The value of count = ");
  Serial.println(count);

  if(count == 9) count = 0; // Set to 1st bit as nothing set last time.
  if(relaySet){
    _C_Relays = 0;
    if(count < 8) bitSet(_C_Relays, count); // If 8 we clear relays only
  } else {
    _L_Relays = 0;
    if(count < 8) bitSet(_L_Relays, count);
  }
//  lastRelay = count;
//  nextRelay = lastRelay + 1;
//  relays = count + 1;
  Serial.print("_C_Relays, _L_Relays values =   ");
  Serial.print(_C_Relays);
  Serial.print(",   ");
  Serial.println(_L_Relays);
  
  setRelays(offset, relaySet);

  delay(DELAY);
  Serial.print("The operation was performed on relay ... ");
  Serial.println((count + 1));
  Serial.println("--------------------------------------------------------------------------------");
}

// Sets or clears a set of 8 contiguous data pins starting at offset. The bits are
// read 0 .. 7 from _C_Relays or _L_Relays globals depending on option. True = L.
// The bits are written to digital outputs starting at "offset".
void setRelays(int offset, boolean relaySet) {
  int relays;
  
  if(relaySet){
    relays = _C_Relays;
  } else {
    relays = _L_Relays;
  }
  Serial.print("Wrote pattern (0 .. 7  ");
  for(int x = offset; x < (offset + 8); x++){
    digitalWrite(x, bitRead(relays, (x - offset)));
    Serial.print(bitRead(relays, (x - offset)));
    Serial.print(", ");
  }
  Serial.print(offset);
  Serial.print(" || ");
  Serial.println(relays);
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

// We set each relay (either L or C) in turn and check to see if SWR is better or
// worse. If better do the next relay, if worse go back to previous relay. "C_or_L"
// is used to choose which relay set we are working on. True = C, false = L.

void courseSteps(boolean C_or_L) {
  unsigned int lastSWR;
  unsigned int tempSWR;
  int SWRout = 100;
  int out_0, out_7; //Maps to Capacitor relay bank if C_or_L true, else inductor relay bank.
/*  
  if (C_or_L) { // We are working on "C" relay set here
    clearRelays(false, true); // Reset all capacitor relays
    out_0 = OUTC_0;
    out_7 = OUTC_7;
  } else {      // We are working on "L" relay set here
    clearRelays(true, false); // Reset all inductor relays
    out_0 = OUTL_0;
    out_7 = OUTL_7;    
  }
  if (C_or_L) {
    Serial.print("Initial capacitors no operated relays swr value = "); // Debug Message
  } else {
    Serial.print("Initial inductors no operated relays swr value = "); // Debug Message
  }
  lastSWR = getSWR(); // Get SWR with no relays operated
  Serial.println();

  for(int i = out_0; i <= out_7; i++){ 
    Serial.print("Running through loop at i value = ");
    Serial.println(i);
    // For all but the first relay we need to switch off the previous one
    if (i > out_0) digitalWrite(i-1, LOW);
    digitalWrite(i, HIGH);
    delay(DELAY); // Give relay time to operate & previous relay to release
    
    // read new SWR and compare to lastSWR. If better step again
    // if worse go back to previous relay and exit
    tempSWR = getSWR();    
    if (tempSWR < lastSWR + 10) {
      lastSWR = tempSWR;
    } else {
      digitalWrite(i, LOW); // Turn off current relay and turn on previous one.
      digitalWrite(i-1, HIGH); // lastSWR[2] is holding the previous swr which
                               // now become the current reading
      Serial.print("SWR was greater than previous reading so operated relay = ");
      Serial.println(i-1);
      delay(DELAY);
      break;
    }
  }
  Serial.println();
  */
}

// Worst case would be max analog in voltage of 5 volts fwd and 5 volts rev. The term
// (fwdPwr + revPwr) * 1000 = (1023 + 1023) * 1000 = 2046000 so a long is needed.
unsigned int getSWR() {
  long fwdPwr;
  long revPwr;
  unsigned int swr;
  
  revPwr = analogRead(reverse);
  fwdPwr = analogRead(forward);
  if (fwdPwr <= revPwr) revPwr = fwdPwr - 1; //Avoid division by zero or negative.
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

