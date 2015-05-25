//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Coarse stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

uint8_t * heapptr, * stackptr;  // I declared these globally for memory checks
// Debug Defines
#define DEBUG_RELAY_FINE_STEPS
//#define DEBUG_RELAY_STATE
#define DEBUG_COARSE_TUNE_STATUS
#define DEBUG_TUNE_SUMMARY
//#define DEBUG_status.rawSWR_VALUES

// We need the printStatus() subroutine for any of these debugs
#if defined  DEBUG_COARSE_TUNE_STATUS
#define PRINT_STATUS
#elif defined DEBUG_TUNE_SUMMARY
#define PRINT_STATUS
#elif defined DEBUG_RELAY_FINE_STEPS
#define PRINT_STATUS
#elif defined DEBUG_RELAY_STATE
#define PRINT_STATUS
#endif

#define DEBUG_BUTTON_ARRAY
//#define DEBUG_BUTTON_INFO
//#define DEBUG_BUTTONS
//#define DEBUG_STEP_BUTTON

#define TX_LEVEL_THRESHOLD 20
#define CAPS_at_INPUT      LOW    //For digitalWrites to Capacitor I/O changeover relay
#define CAPS_at_OUTPUT     HIGH
#define SWR_AVERAGE_COUNT  10     // Number of analog readings to combat jitter

// Shift Register for L & C driver Pin assign
#define Cclock 4           // Pin 11 of 74HC164 U5 to pin 7 of Arduino Nano
#define Clatch 3           // Pin 12 of 74HC164 U5 to pin 6 of Arduino Nano
#define Cdata 2            // Pin 14 of 74HC164 U5 to pin 5 of Arduino Nano

#define Lclock 8           // Pin 11 of 74HC595 U3 to pin 11 of Arduino Nano
#define Llatch 7           // Pin 12 of 74HC595 U4 to pin 10 of Arduino Nano
#define Ldata 6            // Pin 14 of 74HC595 U3 to pin 9 of Arduino Nano

#define coRelay        9   // Capacitor set c/o relay
#define swrGain       12   // Switchable gain for swr amplifiers
#define BUTTON_PIN    A0   // Push Button (Analog pin used as digital)
#define LEDpin        13   // A LED is connected to this pin, use for heartbeat
#define forward       A2   // Measure forward SWR on this pin
#define reverse       A3   // Measure reverse SWR on this pin

#define Button_Debounce_Millis 10   // Delay for pushbutton debounce settle time ms
#define Relay_Settle_Millis    20   // Delay for relay debounce settle time ms

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
#define analog_buttons_pin A6
#define num_of_analog_buttons 4
#define analog_buttons_r1 10  // Resistor value connected to button chain in "K's"
#define analog_buttons_r2 1.2 // Value of each resistor in the chain in "K's"
#define LONG_PRESS_TIME 800 //msec before button considered a long press
#define analog_Button_Debounce_Millis 10

// Setup LCD stuff
#define lcdNumCols 16 // -- number of columns in the LCD
#define lcdNumRows  2 // -- number of rowss in the LCD
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Bar part block characters definitions. Use lcd.write(6) for an empty bar
byte p1[8] = {
  0x00,0x00,0x10,0x10,0x10,0x10,0x00,0x00}; // 1 part of bar block
byte p2[8] = {
  0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00};
byte p3[8] = {
  0x00,0x00,0x1C,0x1C,0x1C,0x1C,0x00,0x00};
byte p4[8] = {
  0x00,0x00,0x1E,0x1E,0x1E,0x1E,0x00,0x00}; // 4 parts of bar block
byte p5[8] = {
  0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x00,0x00}; // full bar block
byte p6[8] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // blank bar block

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
} 
_status;

  //       Inductor definitions     L1   L2   L3   L4    L5    L6    L7    L8   
const unsigned int  _inductors[] = { 6,  17,  35,  73,  136,  275,  568, 1099 };  // inductor values in nH
const unsigned int _strayL = 0;
  //        Capacitor definitions   C1   C2   C3   C4    C5    C6    C7    C8
const unsigned int _capacitors[] = { 6,  11,  22,  44,   88,  168,  300,  660 };  // capacitor values in pF
const unsigned int _strayC = 0;

enum commandMode {
  TUNED, TUNE, TUNING};
byte _cmd = 0;  // Holds the command to be processed

/**********************************************************************************************************/

void setup() {
  // First thing up, set C & L Relays to all off.
  pinMode(Cclock, OUTPUT); // make the Capacitor clock pin an output
  pinMode(Clatch, OUTPUT); // make the Capacitor latch pin an output
  pinMode(Cdata , OUTPUT); // make the Capacitor data pin an output
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Llatch, OUTPUT); // make the Inductor latch pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output  
  pinMode(coRelay, OUTPUT);
  digitalWrite(Cclock, LOW);
  _status.C_relays = 0;
  _status.L_relays = 0;
  _status.outputZ = loZ; // Caps switched to input side of L network
  setRelays(); // Switch off all the relays & set c/o relay to input.

  pinMode(swrGain, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(swrGain, LOW); // Turns off fet shunting swr Start with highest gain for amps.voltages
  _status.ampGain = hi;
  digitalWrite(BUTTON_PIN, HIGH); // pull-up activated
  digitalWrite(analog_buttons_pin, HIGH); // pull-up activated
  digitalWrite(LEDpin, LOW);

  lcd.begin(lcdNumRows, lcdNumCols);
  //  lcd.clear(); //TODO check if this can be removed as splash will write whole screen
  // -- do some delay: some times I've got broken visualization
  delay(100);
  lcd.createChar(1, p1);
  lcd.createChar(2, p2);
  lcd.createChar(3, p3);
  lcd.createChar(4, p4);
  lcd.createChar(5, p5);
  lcd.createChar(6, p6);
  lcdPrintSplash();

  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  initialize_analog_button_array();
  
  Serial.println(F("Arduino antenna tuner ver 1.0.0"));
  Serial.println(F("Copyright (C) 2015, Graeme Jury ZL2APV"));
  Serial.print(F("available RAM = "));
  Serial.println(freeRam());
//  Serial.println ((int) stackptr);
//  Serial.println ((int) heapptr);
  Serial.println();
}

/**********************************************************************************************************/

void loop(){
  byte buttonNumber;
  static unsigned long heartbeat = millis();

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
      Serial.print(F("Loop:  A long press leading edge detected on button "));
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
  byte button_pressed = handle_button(); 
  if (button_pressed) {
    if(_cmd != TUNED) {
      _cmd = TUNED; // Any press halts a pending tune with no RF applied
    }
    else {
      switch (button_pressed) {
      case 1: 
        { // Short press, Bypass tuner
          Serial.println(F("Short press, Initiate Autotune when RF present"));
          _cmd = TUNE;
          break;
        }
      case 2:
        { // Medium press, Initiate Autotune when RF present
          Serial.println(F("Medium press, Initiate Autotune when RF present"));
          _cmd = TUNE;
          break;
        } 
      case 3: 
        { // Long press, Not allocated yet
          Serial.println(F("Long press, Bypass tuner"));
          _status.C_relays = 0;
          _status.L_relays = 0;
          _status.outputZ = loZ; // Caps switched to input side of L network
          setRelays(); // Switch off all the relays & set c/o relay to input.
          lcdPrintSplash();
        }
      }
    }
  }
  // Do heartbeat of 1 sec on and 1 sec off as non blocking routine
  if(millis() > heartbeat) {
    if(digitalRead(LEDpin)) {
      digitalWrite(LEDpin, LOW);   // turn the LED off
    }
    else digitalWrite(LEDpin, HIGH);    // turn the LED on by making the voltage HIGH
    heartbeat = (millis() + 1000);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* The analog input ranges from 0 to 1023 (1024 values) and on a 16 column display will have a
 value 0f 1024/16 = 64 per column. A number like 400 would be represented by 6 full columns i.e.
 (400/64 = 6.25) which is 6 full columns of dots with a remainder of .25 or 64 * .25 = 16. As 
 each column is broken into 5 dots per row so we can represent the partial block value as int(.25*5) = 1.
 */
void displayAnalog(byte col, byte row, int value)
{
  int cnt;
  byte blocks = value / 64;
  byte partBlock = value % 64;
  byte remSpaces = 0; // Number of spaces to write to overwrite old data in the line

  lcd.setCursor(col,row);
  // Calculate how many blank blocks to write to overwrite old data when bargraph is less than full row.
  if(blocks < 15) remSpaces = 16 - (blocks + 1); // We will tack a part block on end of 15 or less

  // Print out the full blocks in bargraph
  for(cnt = 1; cnt < (blocks + 1); cnt++) {
    lcd.write(5);
  }

  // If < 16 full blocks print out the part block of barcode
  if(blocks < 16) {
    if(partBlock < 7)     lcd.write(6); // value too small to show as part block so print blank.
    else if(partBlock < 20) lcd.write(1);
    else if(partBlock < 33) lcd.write(2);
    else if(partBlock < 45) lcd.write(3);
    else if(partBlock < 58) lcd.write(4);
    else lcd.write(5); // Value so close to full block we will show it as so.
  }

  // Now blank rest of blocks in row so barcode not corrupted by old data.
  for(cnt = 0; cnt < remSpaces; cnt++) {
    lcd.write(6);
  }
  /*  
   // Debug stuff
   lcd.setCursor(0,0);
   for(cnt = 0; cnt < 16; cnt++) { // Clear row 0
   lcd.print(" ");
   }
   lcd.setCursor(0,0);
   lcd.print(blocks);
   lcd.print(", ");
   lcd.print(partBlock);
   */
}

/**********************************************************************************************************/

void lcdPrintStatus()
{
  //  static unsigned long displayUpdate = 0;
  float freq = 14.02345;
  char charVal[10];          //buffer, temporarily holds data from values 


  //  if(displayUpdate < millis()) {
  //    displayUpdate = (millis() + 10000);
  lcd.home();   
  dtostrf(float(_status.rawSWR) / 100000, 7, 3, charVal);  //7 is mininum width, 3 is precision;
  //  float value is copied onto buffer
  for(int i=0;i<7;i++)
  {
    lcd.write(charVal[i]);
  }

  // Temporary replace frequency with reverse voltage
  sprintf(charVal, " %4d  ", _status.rev);
  lcd.print(charVal);

  /*    
   lcd.print(" ");
   dtostrf(freq, 6, 3, charVal);  // 6 is mininum width, 3 is precision;
   for(int i=0;i<6;i++)
   {
   lcd.write(charVal[i]);
   }
   */
  if(_status.outputZ == hiZ) {
    lcd.print(" H");
  } 
  else lcd.print(" L");
  lcd.setCursor(0,1);

  sprintf(charVal, "%4dp ", _status.totC);
  lcd.print(charVal);

  sprintf(charVal, "%4du ", _status.totL);
  lcd.print(charVal);

  sprintf(charVal, "%4d", _status.fwd);
  lcd.print(charVal);
  //  } // endif ((millis() - displayUpdate) > 50)
}

/**********************************************************************************************************/

void lcdPrintSplash()
{
  lcd.home();                   // go home
  lcd.print(F("ARDUINO TUNER by"));  
  lcd.setCursor (0, 1);        // go to the next line
  lcd.print(F("ZL2APV (c) 2015 "));
  //  lcd.backlight(); // finish with backlight on
  //  delay ( 5000 );
}

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
      //      Serial.print(F("Got to TUNED, cmd = "));
      //      Serial.println(cmd);
      break;
    }
  case TUNE:
    { // Wait for sufficient RF fwd pwr then start tuning
      //      Serial.print(F("Got to TUNE, cmd = "));
      //      Serial.println(cmd);
      if(_status.fwd > TX_LEVEL_THRESHOLD) {
        cmd = TUNING;
        break;
      } 
      else {
        cmd = TUNING; // TODO remove this line in final code (debug only)
        break;
      }
    }
  case TUNING: 
    { // Tuning is under way so process until finished
      tryPresets();
//      if(_status.rawSWR > 160000) { // Debug change to force coarse tuning
      if(_status.rawSWR > 1) {
        _status.outputZ = hiZ;
        doRelayCoarseSteps();
        //Save SWR and relay states and see if better with C/O relay on output
        C_RelaysTmp = _status.C_relays;
        L_RelaysTmp = _status.L_relays;
        bestZ = _status.outputZ;
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.println(F("HiZ coarse tune results"));
  printStatus(printHeader);
  printStatus(printBody);
#endif        
//        getSWR();
        SWRtmp = _status.rawSWR;

        if(_status.rawSWR > 120000) { // Only try again if swr needs improving
          _status.outputZ = loZ;
          doRelayCoarseSteps(); //Run it again and see if better with C/O relay operated
          //If not better restore relays to input state
//          getSWR();
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.println(F("LoZ coarse tune results"));
  printStatus(printHeader);
  printStatus(printBody);
#endif
          if(SWRtmp <= _status.rawSWR) {             //Capacitors on Input side gave best result so
            _status.C_relays = C_RelaysTmp;       // set relays back to where they were on input.
            _status.L_relays = L_RelaysTmp;
            _status.outputZ = bestZ;
            setRelays();
            getSWR();            
          }        
        }
      }
#ifdef DEBUG_COARSE_TUNE_STATUS
  Serial.println(F("Final coarse tune results"));
  printStatus(printHeader);
  printStatus(printBody);
#endif      
      doRelayFineSteps();
      cmd = TUNED;
      lcdPrintStatus();      
    }
  default: 
    cmd = TUNED; 
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
  _status.C_relays = B11100000; // Debug settings for C and L relays
  _status.L_relays = B00000011;
  _status.outputZ  = loZ;
  setRelays();
//  delay(50);
  getSWR();
  Serial.print(F("20M rawSWR = "));
  Serial.println(_status.rawSWR);
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
  Serial.println(F("Relays set at zero"));
  if(_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  _status = statusTemp;
  setRelays();
  getSWR();    
}

/**********************************************************************************************************/
/*
void doRelayCoarseSteps()
{
  // For each L relay set in turn from 0 relays to the 8th relay we set the capacitor
  // relays one by one from 0 relays operated (cnt = 0) through 1st to 8th relay
  // (cnt = 1 to cnt = 8), checking to see which relays produce the lowest SWR.
  
  // Entry: The caller sets the C/O relay to HiZ or LoZ as required
  // Exit with relay settings which give best SWR for the C/O relay setting on entry.
  
  unsigned long bestSWR = 99900000; // Dummy value to force bestSWR to be written from
  byte bestC;
  byte bestL;
  byte cnt = 0;
  byte cnt_L = 0;

  // Initialise with no relays operated, C/O relay was set by the caller.
  _status.C_relays = 0;
  _status.L_relays = 0;
  setRelays(); // Switch off all the relays

#ifdef DEBUG_COARSE_TUNE_STATUS
    Serial.print(F("doRelayCoarseSteps(): Caps are connected to "));
  if(_status.outputZ == hiZ) Serial.println(F("Output")); 
  else Serial.println(F("Input"));
  Serial.print(F("cnt"));
  Serial.print(F(" "));
  Serial.print(F("bestSWR"));
  Serial.print(F("\t"));
  printStatus(printHeader);
#endif

  getSWR();  //Get SWR with no relays operated at this point.

  // currentSWR first time through for loop

  
    for(cnt_L = 0; cnt_L < 9; cnt_L++){
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
//        bestZ = _status.outputZ;
      }
      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(cnt);
      Serial.print(F("   "));
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
//  Serial.println(freeRam());

    } // end of inner for loop
  } // end of outer for loop
  
  // Now set the relays to the state which gave best SWR
  _status.C_relays = bestC;
  _status.L_relays = bestL;
//  _status.outputZ = bestZ;
  setRelays();
  getSWR();  
}
*/
/**********************************************************************************************************/
void doRelayFineSteps()
{
  unsigned long bestSWR;  
  unsigned long swrTemp;

  getSWR(); // Get swr and relay status up to date.
  swrTemp = _status.rawSWR;

#ifdef DEBUG_RELAY_FINE_STEPS
  int cnt = 1;
  Serial.println(F("doRelayFineSteps: Values on entry")); 
  printStatus(printHeader);
  printStatus(printBody);
#endif

  do {
    bestSWR = swrTemp;
    swrTemp = fineStep_C(); // Starts with best from stepping L and returns best C swr.
    swrTemp = fineStep_L(); // Returns best SWR obtained from stepping L both up and down
#ifdef DEBUG_RELAY_FINE_STEPS    
    Serial.print(F("doRelayFineSteps():  Been through loop ")); 
    Serial.print(cnt); 
    Serial.println(F(" times."));
    cnt++;
    Serial.println(F("-----------------------------------------------------------------------------------"));
#endif    
  } 
  while(swrTemp < bestSWR); // If swr was improved, go again

#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println(F("Exiting doRelayFineSteps(): values on exit ..."));
  printStatus(printHeader);
  printStatus(printBody);
#endif  
}

/**********************************************************************************************************/
unsigned long fineStep_C() // Enter with swr and relay status up to date
{
  unsigned long bestSWR;
  byte C_RelaysTmp = _status.C_relays; //Later will compare the value of _status.C_relays to see if we changed.

#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println(F("fineStep_C():  Stepping capacitors up.")); 
  Serial.print(F("bestSWR\t")); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the C relays. We will increase capacitance as first try.
  if(_status.C_relays != B11111111) { // Step to next capacitor value only if it won't step beyond maximum C.
    do {
      bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values

      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print(F("\t"));
      printStatus(printBody);
#endif      
      _status.C_relays++;
      setRelays();
      getSWR();
    } 
    while(_status.rawSWR <= bestSWR);

    displayAnalog(0, 0, _status.fwd);
    displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS
    // We have not printed the values which caused the loop to exit so do it now
    Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit. 
    Serial.print(F("\t"));
    printStatus(printBody);
#endif

    _status.C_relays--; // On exit, we have stepped one capacitor step too far, so back up one to best value.
    setRelays();
    getSWR();

    displayAnalog(0, 0, _status.fwd);
    displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println(F("Values on exit from capacitor fine steps up. The extra step has been corrected."));
    Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print(F("\t"));
    printStatus(printBody);
    //  Serial.print("C_RelaysTmp, _status.C_relays = "); Serial.print(C_RelaysTmp); Serial.print("' ");Serial.println(_status.C_relays);
#endif
  } // end if(_status.C_relays != B11111111)
  else {
    // Relays were at b'1111_1111' so stepping them up would have rolled over to b'0000_0000' therefore
    // we do nothing and leave the C_Relays at entry state.

    displayAnalog(0, 0, _status.fwd);
    displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println(F("_status.C_Relays = b1111_1111 so are not going to step C_Relays up one"));
    Serial.print(float(bestSWR)/100000, 4); 
    Serial.print(F("\t")); 
    printStatus(printBody);
#endif
  }
  //------------------------------------------------------------------
  if(C_RelaysTmp == _status.C_relays) { // We didn't improve by trying to increase C so try reducing it.
    bestSWR = _status.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println(F("fineStep_C():  Stepping capacitors down.")); 
    Serial.print(F("bestSWR\t")); 
    printStatus(printHeader);
#endif
    if(_status.C_relays != B00000000) { // Step next capacitor down only if it won't roll up to maximum C.
      do {
        bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        Serial.print(float(bestSWR)/100000, 4); 
        Serial.print(F("\t"));
        printStatus(printBody);
#endif        
        _status.C_relays--;
        setRelays();
        getSWR();
      } 
      while(_status.rawSWR <= bestSWR);

      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS
      // We have not printed the values which caused the loop to exit so do it now
      Serial.print(float(_status.rawSWR)/100000, 4); // Print the actual swr on exit.
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
      _status.C_relays++; // On exit, we have stepped one capacitor step too far, so back up one to best value.
      setRelays();
      getSWR();

      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println(F("Values on exit from capacitor fine steps down. The extra step has been corrected."));
      Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
    } // end if(_status.C_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the C_Relays at entry state.

      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);

#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println(F("_status.C_Relays = b'0000_0000' so are not going to step C_Relays down one"));
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print(F("\t")); 
      printStatus(printBody);
#endif
    }
  } // end if(C_RelaysTmp == _status.C_relays)

  return _status.rawSWR;
}

/**********************************************************************************************************/
unsigned long fineStep_L() // Enter with swr and relay status up to date
{
  unsigned long bestSWR;
  byte L_RelaysTmp = _status.L_relays; //Later will compare the value of _status.L_relays to see if we changed.

#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println(F("fineStep_L():  Stepping inductors up.")); 
  Serial.print(F("bestSWR\t")); 
  printStatus(printHeader);
#endif
  //Start off by tweaking the L relays. We will increase inductance as first try.
  if(_status.L_relays != B11111111) { // Step to next inductor value only if it won't step beyond maximum L.
    do {      
      bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
      // We print the swr & status values at entry then each time after relays are stepped.
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print(F("\t"));
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
    Serial.print(F("\t"));
    printStatus(printBody);
#endif
    _status.L_relays--; // On exit, we have stepped one inductor step too far, so back up one to best value.
    setRelays();
    getSWR();
#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
    Serial.println(F("Values on exit from inductor fine steps up. The extra step has been corrected."));
    Serial.print(float(_status.rawSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point.
    Serial.print(F("\t"));
    printStatus(printBody);
#endif
  } // end if(_status.L_relays != B11111111)
  else {
    // Relays were at b'1111_1111' so stepping them up would have rolled over to b'0000_0000' therefore
    // we do nothing and leave the L_Relays at entry state; swrTemp has already been set to best SWR
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println(F("_status.L_Relays = b1111_1111 so are not going to step L_Relays up one"));
    Serial.print(float(bestSWR)/100000, 4); 
    Serial.print(F("\t")); 
    printStatus(printBody);
#endif
  }
  //------------------------------------------------------------------
  if(L_RelaysTmp == _status.L_relays) { // We didn't improve by trying to increase L so try reducing it.
    bestSWR = _status.rawSWR;
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.println(F("fineStep_L():  Stepping inductors down.")); 
    Serial.print(F("bestSWR\t")); 
    printStatus(printHeader);
#endif
    if(_status.L_relays != B00000000) { // Step next inductor down only if it won't roll up to maximum L.
      do {
        bestSWR = _status.rawSWR; // 1st time through, bestSWR equals entry values
#ifdef DEBUG_RELAY_FINE_STEPS
        // We print the swr & status values at entry then each time after relays are stepped.
        //        Serial.print(float(bestSWR)/100000, 4); 
        //        Serial.print("\t");
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
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
      _status.L_relays++; // On exit, we have stepped one inductor step too far, so back up one to best value.
      setRelays();
      getSWR();

#ifdef DEBUG_RELAY_FINE_STEPS // Print values after extra step backed up 1
      Serial.println(F("Values on exit from inductor fine steps down. The extra step has been corrected."));
      Serial.print(float(bestSWR)/100000, 4); // rawSWR should be equal to bestSWR at this point. 
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
    } // end if(_status.L_relays != B00000000)
    else {
      // Relays were at b'0000_0000' so stepping them down would have rolled up to b'1111_1111' therefore
      // we do nothing and leave the L_Relays at entry state.
#ifdef DEBUG_RELAY_FINE_STEPS
      Serial.println(F("_status.L_Relays = b'0000_0000' so are not going to step L_Relays down one"));
      Serial.print(float(bestSWR)/100000, 4); 
      Serial.print(F("\t")); 
      printStatus(printBody);
#endif
    }
  } // end if(L_RelaysTmp == _status.L_relays)

  return _status.rawSWR;
}

/**********************************************************************************************************/
#ifdef PRINT_STATUS
void printStatus(boolean doHeader)
{
  if(doHeader) {
    Serial.println(F("C_relays   L_relays   totC  totL  fwdVolt  revVolt  Gain  outZ    rawSWR  SWR"));
  } 
  else {
    char buffer[16];

    print_binary(_status.C_relays, 8); 
    Serial.print(F("  ")); 
    print_binary(_status.L_relays, 8);
    sprintf(buffer, "  %4u  ", _status.totC); 
    Serial.print(buffer);
    sprintf(buffer, "%4u  ", _status.totL); 
    Serial.print(buffer);
    sprintf(buffer, "   %4u  ", _status.fwd); 
    Serial.print(buffer);
    sprintf(buffer, "   %4u  ", _status.rev); 
    Serial.print(buffer); 
    if(_status.ampGain == hi) Serial.print(F("High")); 
    else Serial.print(F(" Low")); 
    Serial.print(F("  "));
    if(_status.outputZ == hiZ) Serial.print(F(" HiZ  ")); 
    else Serial.print(F(" LoZ  "));  
    sprintf(buffer, "%8lu  ", _status.rawSWR); 
    Serial.print(buffer);
    // NOTE: sprintf doesn't support floats
    Serial.print(float(_status.rawSWR) / 100000, 4);
    Serial.println(F("  "));
  }
}
#endif

/**********************************************************************************************************/
void printFineSteps(float bestSWR)
{
  Serial.print(bestSWR, 4); 
  Serial.print(F("\t")); 
  Serial.print(_status.fwd);
  Serial.print(F("\t"));
  Serial.print(_status.rev);
  Serial.print(F("\t"));
  Serial.print(calcXvalue(C));
  Serial.print(F("\t"));
  Serial.print(calcXvalue(L));
  Serial.print(F("\t"));
  print_binary(_status.C_relays, 8);
  Serial.print(F("\t"));
  print_binary(_status.L_relays, 8);
  Serial.println();
}

/**********************************************************************************************************/
// We calculate the total values of L or C. CorL is a flag to determine which reactance to sum up.
unsigned int calcXvalue(bool CorL)
{
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

void setRelays()
{
  // Writes the bytes from _status.C_relays and _status.L_relays to the associated shift register. Following
  // this the HiZ/LoZ changeover is set to match _status.outputZ (hiZ = true; loZ = false). The capacitor
  // changeover relay switches caps to input (not operated) for hiZ loads and output (operated) for loZ.
  // The total value of C and L is calculated and placed in _status

    // Due to a screw up in my circuit it is necessary to map the relays to the right pins on the relays

    byte Cmap = _status.C_relays;
  byte Lmap = _status.L_relays;
  boolean temp;

  // Set the C Relays from _status.C_relays;
  digitalWrite(Clatch, LOW);
  temp = bitRead(Cmap, 7);
  Cmap = Cmap << 1;
  bitWrite(Cmap, 0, temp); 
  shiftOut(Cdata, Cclock, MSBFIRST, Cmap); // send this binary value to the Capacitor shift register
  digitalWrite(Clatch, HIGH);

  // Set the L Relays from _status.L_relays;
  digitalWrite(Llatch, LOW);
  temp = bitRead(Lmap, 7);
  Lmap = Lmap << 1;
  bitWrite(Lmap, 0, temp);
  shiftOut(Ldata, Lclock, MSBFIRST, Lmap); // send this binary value to the Inductor shift register
  digitalWrite(Llatch, HIGH);

  // Write the total capacitor and inductor values into the "_status" struct.
  _status.totC = calcXvalue(true);
  _status.totL = calcXvalue(false);

  // Set the HiZ/LoZ Changeover Relay from the value of _status.outputZ
  if(_status.outputZ) {
    digitalWrite(coRelay, CAPS_at_OUTPUT); // HiZ loads, relay operated
  } 
  else {
    digitalWrite(coRelay, CAPS_at_INPUT); // LoZ loads, relay not operated
  }
#ifdef DEBUG_RELAY_STATE
  //  dbugRelayState();
  printStatus(printHeader);
  printStatus(printBody);
#endif
  delay(Relay_Settle_Millis); // Let the relays do their contact bounce settling
}

/**********************************************************************************************************/

void getSWR()
{
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
  Serial.print(F("getSWR: fwd, rev, floatSWR, rawSWR = "));
  Serial.print(_status.fwd);
  Serial.print(F(", "));
  Serial.print(_status.rev);
  Serial.print(F(", "));
  Serial.print(float(_status.rawSWR) / 100000, 6);
  Serial.print(F(", "));
  Serial.println(_status.rawSWR);
#endif
}

/**********************************************************************************************************/

// PRINT_BINARY - Arduino
//
// Prints a positive integer in binary format with a fixed withdth
//
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07

void print_binary(int v, int num_places)
{
  int mask=0, n;

  for (n=1; n<=num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while(num_places) {
    if (v & (0x0001 << num_places-1)) {
      Serial.print(F("1"));
    }
    else {
      Serial.print(F("0"));
    }
    --num_places;
    if(((num_places%4) == 0) && (num_places != 0)) {
      Serial.print(F("_"));
    }
  }
}

/**********************************************************************************************************/

byte handle_button()
{
  static boolean button_was_pressed = true; // True = no press i.e. pullup voltage
  static unsigned long timer = 0;
  boolean event;
  byte retval = 0;

  int button_now_pressed = digitalRead(BUTTON_PIN); // pin low -> pressed
  event = (button_now_pressed != button_was_pressed);
  if (event) { // Check if button changed
    delay(Button_Debounce_Millis);
    int button_now_pressed = digitalRead(BUTTON_PIN); // pin low -> pressed
    event = (button_now_pressed != button_was_pressed);
  }
  if(event) { // The re-read says it is a valid change of button state
    if(!button_now_pressed) { // The button has changed from released to pressed
      timer = millis();   // so start the button press length timer
    }
    else { // The button has changed from pressed to released so calc button press type.
      timer = millis() - timer;
#ifdef DEBUG_BUTTONS     
      Serial.print(F("Button released, Timer value = "));
      Serial.println(timer);
#endif      
      if(timer <= 200) { // Short press
        retval = 1;
      } 
      else if(timer < 1000) {
        retval = 2;
      } 
      else retval =3;
    }
    button_was_pressed = button_now_pressed;
  }  
  return retval; // Will be 0 unless button went from pressed to released
}

/**********************************************************************************************************/

void initialize_analog_button_array()
{
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
    Serial.print(F("initialize_analog_button_array: "));
    Serial.print(x);
    Serial.print(F(":  "));
    Serial.print(_Button_array_min_value[x]);
    Serial.print(F(" - "));
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
#ifdef DEBUG_BUTTONS
  if(analogButtonValue < 1020) {
    Serial.print(F("The raw button press value is "));
    Serial.println(analogButtonValue);
  }
#endif  
  // Now determine which button was pressed if any
  if (analogButtonValue <= _Button_array_max_value[num_of_analog_buttons-1]) {
    for (cnt = 0; cnt < num_of_analog_buttons; cnt++) {
      if  ((analogButtonValue > _Button_array_min_value[cnt]) &&
        (analogButtonValue <=  _Button_array_max_value[cnt])) {
        thisButton = cnt + 1;
      }
    }  
  } 
  else thisButton = 0;
  // See if we got 2 identical samples in a row
  if(thisButton != lastButtonValue) {
    lastButtonValue = thisButton; // No but setting up now for next sample match.
  } 
  else { // We have a valid button press or a valid button release
    if(thisButton != 0) { // It is a press so save the button and check for a long press
      if(currentButton != thisButton) {
        currentButton = thisButton;
        longPressTimer = millis();
      }
      if((millis() - longPressTimer) > LONG_PRESS_TIME) {
        retVal = currentButton + num_of_analog_buttons;
        longPress = true;
      } 
      else retVal = 0;
    } 
    else { // We are releasing the button so check if it is from a short or long press
      if(longPress) {
        //        Serial.println(F("At ... if((millis() - longPressTimer) > LONG_PRESS_TIME)"));
        retVal = currentButton + num_of_analog_buttons + num_of_analog_buttons;
        currentButton = 0;
        longPress = false;
      } 
      else {
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
    getSWR();
    lcdPrintStatus();
  }
#ifdef DEBUG_BUTTON_INFO      
  Serial.print(F("Loop:  A short press trailing edge detected on button "));
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
#ifdef DEBUG_TUNE_SUMMARY
  printStatus(printHeader);
  printStatus(printBody);
#endif  
}
/**********************************************************************************************************/
void processLongPressTE(byte button)
{
  static unsigned long repeatValue = 0;

  if((millis() - repeatValue) > 60) {

  }
#ifdef DEBUG_BUTTON_INFO      
  Serial.print(F("Loop:  A long press trailing edge detected on button "));      
  Serial.println(button);
#endif
}

/**********************************************************************************************************/

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/**********************************************************************************************************/

void check_mem() {
  //uint8_t * heapptr, * stackptr;  // I declared these globally
  stackptr = (uint8_t *)malloc(4);  // use stackptr temporarily
  heapptr = stackptr;                  // save value of heap pointer
  free(stackptr);                        // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);       // save value of stack pointer
  }
  
/**********************************************************************************************************/    

void doRelayCoarseSteps()
{
  // For each L relay set in turn from 0 relays to the 8th relay we set the capacitor
  // relays one by one from 0 relays operated (cnt = 0) through 1st to 8th relay
  // (cnt = 1 to cnt = 8), checking to see which relays produce the lowest SWR.
  
  // Entry: The caller sets the C/O relay to HiZ or LoZ as required
  // Exit with relay settings which give best SWR for the C/O relay setting on entry.
  
//  unsigned long initialSWR = 0;
  unsigned long bestSWR = 0;
  int C_cnt = 9;
  byte L_cnt = 0;
  byte bestC;
  byte bestL;
//  boolean matched = false;
  boolean improved = false;

  // Initialise with no L relays operated, all C relays operated and C/O relay set by the caller.
  _status.C_relays = B11111111;
  _status.L_relays = 0;
  setRelays();
  getSWR();  //Get SWR with relays at initial state.
//  initialSWR = _status.rawSWR;
  bestSWR = _status.rawSWR;
  
#ifdef DEBUG_COARSE_TUNE_STATUS // Print the DEBUG header
  Serial.print(F("doRelayCoarseSteps(): Caps are connected to "));
  if(_status.outputZ == hiZ) Serial.println(F("Output")); 
  else Serial.println(F("Input"));
  Serial.print(F("cnt"));
  Serial.print(F(" "));
  Serial.print(F("bestSWR"));
  Serial.print(F("\t"));
  printStatus(printHeader);
#endif

// Start of main outer loop ....................................

// We are looking for the region where altering both C and L are producing an effect on SWR. At this point
// the fine tune can take over and give us the best tune reactances possible.
  
  while(C_cnt >= 0) {
    

    // Increment the inductors looking for the best SWR. On exit from this ...
    // If improvement found best_C, best_L and bestSWR will hold values for best spot.
    // If no improvement best_C, best_L and bestSWR will be set to Capacitor entry values.
    
// TODO Maybe a store of the _status.L_relays so we can get back to the initial value
// would be useful. See TODO further down.
   // L_relaysTmp = _status.L_relays; // TODO possible solution to step_up/step_dn problem

// start of inductor step up inner loop ---------------------------------------

    while(L_cnt < 4) {

      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);
      if(_status.rawSWR <= bestSWR){
        bestSWR = _status.rawSWR;
        bestC = _status.C_relays;
        bestL = _status.L_relays;
        improved = true;
        L_cnt = 0;        
      }
      else L_cnt++;
      
#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(L_cnt); // While in inductor loop we print the L_cnt
      Serial.print(F("---"));
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
      if(_status.L_relays < B11111111) _status.L_relays++;
      else break;
      setRelays(); // Stepping through the Inductor relays
      getSWR();
    } // End of stepping inductor up loop

// End of inductor step up inner loop ---------------------------------------

    L_cnt = 0; //Reset counter
    // _status.L_relays = L_relaysTmp; // TODO possible solution to step up/step dn problem
    
    // TODO If the step up loop immediately started getting worse we need to set the
    // step down loop to start from where the beginning of the step up loop was but
    // only if it was not from zero as first time through will be because we can't
    // step down from there.

// Start of inductor step down inner loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // If we made no improvement from stepping inductors up, try stepping down.
    // _status.L_relays will have been stepped up by 4 to meet loop exit requirements.
// TODO trap trying to overflow L relay settings
    while(!improved && L_cnt < 4)
    {
      displayAnalog(0, 0, _status.fwd);
      displayAnalog(0, 1, _status.rev);
      if(_status.rawSWR <= bestSWR){
        bestSWR = _status.rawSWR;
        bestC = _status.C_relays;
        bestL = _status.L_relays;
//        improved = true;
        L_cnt = 0;        
      }
      else L_cnt++;
      
#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(L_cnt); // While in inductor loop we print the L_cnt
      Serial.print(F("~~~"));
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print(F("\t"));
      printStatus(printBody);
#endif
      if(_status.L_relays > 0) _status.L_relays--;
      else break;
      setRelays(); // Stepping down through the Inductor relays
      getSWR();
    } // End of stepping inductor up loop

// End of inductor step down inner loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    L_cnt = 0; //Reset counter
    Serial.print(F("bestSWR = ")); Serial.println(bestSWR);
    if(bestSWR < 120000) break; // Finish search if SWR low enough for fine tune to capture
    improved = false;
//    _status.L_relays = _status.L_relays / 2;
    _status.L_relays = 0;
    
    //Step to the next C relay
    if(C_cnt > 1) {
      _status.C_relays = 0;
      bitSet(_status.C_relays, (C_cnt - 2));
    }
    else _status.C_relays = 0;
    C_cnt--;
    setRelays(); // Stepping through the capacitor relays
    getSWR();
  } // End of capacitor stepping loop

// End of main outer loop ....................................
  
#ifdef DEBUG_COARSE_TUNE_STATUS
      Serial.print(C_cnt);  // After exiting capacitor loop we print the C_cnt
      Serial.print(F("..."));
      Serial.print(float(bestSWR)/100000, 4);
      Serial.print(F("\t"));
      printStatus(printBody);
#endif  
  
  // Now set the relays to the state which gave best SWR
  _status.C_relays = bestC;
  _status.L_relays = bestL;
//  _status.outputZ = bestZ;
  setRelays();
  getSWR();  
}

/**********************************************************************************************************/  
