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
#define PWMtest       11   // Debug temporary for testing swr

#define OUTL_0        12  // Must be contiguous
#define OUTL_1        13
#define OUTL_2        14
#define OUTL_3        15
#define OUTL_4        16
#define OUTL_5        17
#define OUTL_6        18
#define OUTL_7        19

#define forward       A6  // Measure forward SWR on this pin
#define reverse       A7  // Measure reverse SWR on this pin

#define DELAY         20  // Delay per loop in ms
#define strobeDELAY   10  // Delay time for strobing latches

#define C             true
#define L             false
#define Up            true
#define Dn            false

// Global variables
byte _C_Relays = 0; // Holds map of operated relays with
byte _L_Relays = 0; // 0 = released and 1 = operated
int  _revSWR   = 0;

void setup() { 
 // initialize the digital pins as outputs.
//  pinMode(outCtl, OUTPUT);
//  digitalWrite(outCtl, HIGH); // Latches outputs high ZDisable l so no relays operate
  analogWrite(PWMtest, 0);   // turn analog out to 0 Volts
  for(int i = OUTC_0; i <= OUTC_7; i++){ // Set all C Relay signals to output
    pinMode(i, OUTPUT);
  }

  for(int i = OUTL_0; i <= OUTL_7; i++){ // Set all L Relay signals to output
    pinMode(i, OUTPUT);
  }  

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  clearRelays(true, true); // Set both C & L relay sets to off
} 

void loop(){
  // handle button
  boolean button_pressed = handle_button();
  if (button_pressed) {
    Serial.println("button_pressed");
    courseSteps(C); // Step through each relay in turn
    courseSteps(L); // looking for best SWR
  }
  delay(DELAY);
}

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

void fineSteps_C(boolean dir) {
  /*
 if (data & (1<<y)) {
                    // turn on the LED at location (x,y)
 } else {
                    // turn off the LED at location (x,y)
 }
 */
}

// We set each relay (both L and C) in turn and check to see if SWR is better or
// worse. If better do the next relay, if worse go back to previous relay. C_or_L
// is used to choose which relay set we are working on. True = C, false = L.

void courseSteps(boolean C_or_L) {
  int lastSWR[3] = {0,0,0}; // 0 = fwd pwr, 1 = rev pwr, 2 = VSWR
  int tempSWR;
  int SWRout = 100;
  int out_0, out_7; //Equal to Capacitor relay bank if C_or_L true otherwise inductor relay bank.
  
  analogWrite(PWMtest, SWRout);
  if (C_or_L) { // We are working on "C" relay set here
    clearRelays(false, true); // Reset all capacitor relays
    out_0 = OUTC_0;
    out_7 = OUTC_7;
  } else {      // We are working on "L" relay set here
    clearRelays(true, false); // Reset all inductor relays
    out_0 = OUTL_0;
    out_7 = OUTL_7;    
    SWRout = 60; //Debug testing value
  }
  SWRout = swrWrite(Dn, SWRout); // Set SWR voltage on pin D11 with this call and restore
  SWRout = swrWrite(Up, SWRout); // to original voltage with this call.
  if (C_or_L) {
    Serial.print("Initial capacitors no operated relays swr value = "); // Debug Message
  } else {
    Serial.print("Initial inductors value = "); // Debug Message
  }
  lastSWR[2] = checkSWR(lastSWR[2]); // Get SWR with no relays operated
  Serial.println();
  SWRout = swrWrite(Dn, SWRout); //Set swr for next relay


  for(int i = out_0; i <= out_7; i++){ 
    Serial.print("Running through loop at i value = ");
    Serial.println(i);
    // For all but the first relay we need to switch off the previous one
    if (i > out_0) digitalWrite(i-1, LOW);
    digitalWrite(i, HIGH);
    delay(DELAY); // Give relay time to operate & previous relay to release
    
    // read new SWR and compare to lastSWR. If better step again
    // if worse go back to previous relay and exit
    tempSWR = checkSWR(lastSWR[2]);    
    if (tempSWR < lastSWR[2]+10) {
      lastSWR[2] = tempSWR;
      if (i < 5) { // Debug artificially
      SWRout = swrWrite(Dn, SWRout);  //Debug set swr down for next relay
      } else { //Debug
        SWRout = swrWrite(Up, SWRout);  //Debug set to simulate worse swr for next relay
      } //Debug
    } else {
      digitalWrite(i, LOW); // Turn off current relay and turn on previous one.
      digitalWrite(i-1, HIGH); // lastSWR[2] is holding the previous swr which
                               // now become the current reading
      if (C_or_L) {
        digitalWrite(C_strobe, HIGH); // 10 mSec C strobe pulse
        delay(strobeDELAY);
        digitalWrite(C_strobe, LOW);
      } else {
        digitalWrite(L_strobe, HIGH); // 10 mSec C strobe pulse
        delay(strobeDELAY);
        digitalWrite(L_strobe, LOW);
      }
      Serial.print("SWR was greater than previous reading at position ");
      Serial.println(i);
      delay(DELAY);
      break;
    }
  }

  Serial.println();
}

/* Here I am writing code to send a reducing swr each 6 times called, then increase
    for the 7th time
*/
int checkSWR(int lastSWR) {
  float fwdPwr;
  float revPwr;
  float swr;
  float temp;
  
  revPwr = analogRead(reverse);
//  fwdPwr = analogRead(forward);
  fwdPwr = 512; // Debug Fixed fwdPwr for testing purposes only
  if (fwdPwr <= revPwr) revPwr = fwdPwr - 1; //Avoid division by zero or negative.
  swr = (fwdPwr + revPwr) / (fwdPwr - revPwr) * 1000;
  
  Serial.print("SWR readings fwd, rev, swr, int(swr) = ");
  Serial.print(fwdPwr);
  Serial.print(", ");
  Serial.print(revPwr);
  Serial.print(", ");
  Serial.print(swr);
  Serial.print(", ");
  Serial.println(int(swr));
  
  return int(swr);
}

int swrWrite(boolean up_dn, int swr) {  // Up is true, Dn is false
  if (up_dn) {
    swr = swr + 10;
    analogWrite(PWMtest, swr);
  } else {
    swr = swr - 10;
    analogWrite(PWMtest, swr);
  }
  Serial.print("Priming for next reading so SWR set to ");
  Serial.println(swr);
  
  return swr;
}
