//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Coarse stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////
/*
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
Heartbeat    B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
                  | [ ]3.3V           MOSI/D11[ ]~|   B3
                  | [ ]V.ref     ___    SS/D10[ ]~|   B2
             C0   | [ ]A0       / N \       D9[ ]~|   B1
             C1   | [ ]A1      /  A  \      D8[ ] |   B0 Clock
             C2   | [ ]A2      \  N  /      D7[ ] |   D7 Latch
             C3   | [ ]A3       \_0_/       D6[ ]~|   D6 Data
             C4   | [ ]A4/SDA               D5[ ]~|   D5 Freq Counter Input
             C5   | [ ]A5/SCL               D4[ ] |   D4 Output Enable
                  | [ ]A6              INT1/D3[ ]~|   D3 N/C
                  | [ ]A7              INT0/D2[ ] |   D2 N/C
                  | [ ]5V                  GND[ ] |     
             C6   | [ ]RST                 RST[ ] |   C6
                  | [ ]GND   5V MOSI GND   TX1[ ] |   D0
                  | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
                  |          [ ] [ ] [ ]          |
                  |          MISO SCK RST         |
                  | NANO-V3                       |
                  +-------------------------------+
         
                  http://busyducks.com/ascii-art-arduinos
*/
#include <stdlib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "defines.h"

uint8_t * heapptr, * stackptr;  // I declared these globally for memory checks


// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Bar part block characters definitions. Use lcd.write(6) for an empty bar
byte p1[8] = {
  0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00
}; // 1 part of bar block
byte p2[8] = {
  0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00
};
byte p3[8] = {
  0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00
};
byte p4[8] = {
  0x00, 0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00
}; // 4 parts of bar block
byte p5[8] = {
  0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00
}; // full bar block
byte p6[8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
}; // blank bar block

// Global variables always start with an underscore
int _Button_array_max_value[num_of_analog_buttons];
int _Button_array_min_value[num_of_analog_buttons];

struct status {
  unsigned int fwd;
  unsigned int rev;
  unsigned long rawSWR;
  unsigned long currentSWR;
  boolean ampGain;
  byte C_relays;
  byte L_relays;
  unsigned int totC;
  unsigned int totL;
  boolean outputZ;
}
_status;

//         Inductor definitions     L1   L2   L3   L4    L5    L6    L7    L8
const unsigned int  _inductors[] = { 6,  17,  35,  73,  136,  275,  568, 1099 };  // inductor values in nH
const unsigned int _strayL = 0;
//          Capacitor definitions   C1   C2   C3   C4    C5    C6    C7    C8
const unsigned int _capacitors[] = { 6,  11,  22,  44,   88,  168,  300,  660 };  // capacitor values in pF
const unsigned int _strayC = 0;

enum {INDUCTANCE, CAPACITANCE};
enum {POWERUP, TUNE, TUNING, TUNED};
byte _cmd = 0;  // Holds the command to be processed

/**********************************************************************************************************/

void setup() {
  // First thing up, set C & L Relays to all off.
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Llatch, OUTPUT); // make the Inductor latch pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output
  pinMode(coRelay, OUTPUT);
//  digitalWrite(Lclock, LOW);
  _status.currentSWR = 99999000;
  _status.C_relays = 0;
  _status.L_relays = 0;
  _status.outputZ = loZ; // Caps switched to input side of L network
  setRelays(); // Switch off all the relays & set c/o relay to input.
// Pin 13 of SR (output enable) is held high with a 10k pullup so random outputs won't operate the relays
// during power up. It needs to be held low after Arduino is booted and relay values initialised.
  pinMode(outputEnable, OUTPUT); // Switch off HiZ mode for shift register
  digitalWrite(outputEnable, LOW);

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
  Serial.begin(baudRate);
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

void loop() {
  byte buttonNumber;
  static unsigned long heartbeat = millis();

  getSWR();
  _cmd = processCommand(_cmd);
  //  buttonNumber = check_step_buttons();
  buttonNumber = getAnalogButton();
  if (buttonNumber != 0) { // 0x00 is returned with no button press
    _cmd = TUNED; // Stop any automatic tuning process with any button press
    if (buttonNumber <= num_of_analog_buttons) {
      // A short press trailing edge detected
      processShortPressTE(buttonNumber);

    }
    else if (buttonNumber <= (num_of_analog_buttons + num_of_analog_buttons)) {
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
    if (_cmd == TUNING) {
      _cmd = POWERUP; // Any press halts a tune in process
    }
    else {
      switch (button_pressed) {
        case 1:
        { // Leading edge of a button press which we ignore as we process trailing edges only.
          break;
        }
        case 2:
          { // Short press, Initiate Autotune when RF present
#ifdef DEBUG_BUTTONS
            Serial.println(F("Short press, Initiate Autotune when RF present"));
#endif            
            _cmd = TUNE;
            break;
          }
        case 3:
          { // Medium press, Initiate Autotune when RF present
#ifdef DEBUG_BUTTONS
            Serial.println(F("Medium press, Initiate Autotune when RF present"));
#endif
            _cmd = TUNE;
            break;
          }
        case 4:
          { // Long press, Not allocated yet
#ifdef DEBUG_BUTTONS
            Serial.println(F("Long press, Bypass tuner"));
#endif                      
            _status.C_relays = 0;
            _status.L_relays = 0;
            _status.outputZ = loZ; // Caps switched to input side of L network
            setRelays(); // Switch off all the relays & set c/o relay to input.
            _cmd = POWERUP;
            lcdPrintSplash();
          }
      }
    }
  }
  
  // Do heartbeat of 1 sec on and 1 sec off as non blocking routine
  if (millis() > heartbeat) {
    if (digitalRead(LEDpin)) {
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

  lcd.setCursor(col, row);
  // Calculate how many blank blocks to write to overwrite old data when bargraph is less than full row.
  if (blocks < 15) remSpaces = 16 - (blocks + 1); // We will tack a part block on end of 15 or less

  // Print out the full blocks in bargraph
  for (cnt = 1; cnt < (blocks + 1); cnt++) {
    lcd.write(5);
  }

  // If < 16 full blocks print out the part block of barcode
  if (blocks < 16) {
    if (partBlock < 7)     lcd.write(6); // value too small to show as part block so print blank.
    else if (partBlock < 20) lcd.write(1);
    else if (partBlock < 33) lcd.write(2);
    else if (partBlock < 45) lcd.write(3);
    else if (partBlock < 58) lcd.write(4);
    else lcd.write(5); // Value so close to full block we will show it as so.
  }

  // Now blank rest of blocks in row so barcode not corrupted by old data.
  for (cnt = 0; cnt < remSpaces; cnt++) {
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
  dtostrf(float(_status.currentSWR) / 100000, 7, 3, charVal);  //7 is mininum width, 3 is precision;
  //  float value is copied onto buffer
  for (int i = 0; i < 7; i++)
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
  if (_status.outputZ == hiZ) {
    lcd.print(" H");
  }
  else lcd.print(" L");
  lcd.setCursor(0, 1);

  sprintf(charVal, "%4du ", _status.totL);
  lcd.print(charVal);

  sprintf(charVal, "%4dp ", _status.totC);
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
void lcdPrintBargraph(boolean range)
{
  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);
  if(range) {
    lcd.setCursor (lcdNumCols-2, lcdNumRows-1);        // go to 2nd to last chr of last line.
    if(_status.ampGain == hi) {   // If amp gain is high, we are in low power mode
      lcd.print(F("Lo"));
    } else {
      lcd.print(F("Hi"));
    }
  }
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
    case POWERUP:
      { // No tuning has been performed yet
        if(_status.fwd > TX_LEVEL_THRESHOLD) { // We are transmitting so display bargraph
          lcdPrintBargraph(true);
        } else {          // We are receiving so display splash screen
          lcdPrintSplash();
        }
        break;
      }
    case TUNE:
      { // Wait for sufficient RF fwd pwr then start tuning
        //      Serial.print(F("Got to TUNE, cmd = "));
        //      Serial.println(cmd);
        if (_status.fwd > TX_LEVEL_THRESHOLD) {
          lcdPrintBargraph(false);
          cmd = TUNING;
        } else {
          lcd.home();
          lcd.print("  Tune Pending  ");          
        }
        break;
      }
    case TUNING:
      { // Tuning is under way so process until finished
        tryPresets();
        if (_status.rawSWR > 130000) { // Debug change to force coarse tuning
          //      if(_status.rawSWR > 1) {
          _status.outputZ = loZ;
          // doRelayCoarseSteps() returns true if tune aborted with cmd button press
          if(doRelayCoarseSteps()) { 
            _status.C_relays = 0;    // set relays back to power on settings.
            _status.L_relays = 0;
            _status.outputZ = loZ;
            setRelays();
            cmd = POWERUP;
            break;
          }
          //Save SWR and relay states and see if better with C/O relay on output
          C_RelaysTmp = _status.C_relays;
          L_RelaysTmp = _status.L_relays;
          bestZ = _status.outputZ;
#ifdef DEBUG_COARSE_TUNE_STATUS1
          Serial.println(F("LoZ coarse tune results"));
          printStatus(printHeader);
          printStatus(printBody);
#endif
          //        getSWR();
          SWRtmp = _status.rawSWR;

          if (_status.rawSWR > 120000) { // Only try again if swr needs improving
            _status.outputZ = hiZ;
            doRelayCoarseSteps(); //Run it again and see if better with C/O relay operated
            //If not better restore relays to input state
            //          getSWR();
#ifdef DEBUG_COARSE_TUNE_STATUS1
            Serial.println(F("HiZ coarse tune results"));
            printStatus(printHeader);
            printStatus(printBody);
#endif
            if (SWRtmp <= _status.rawSWR) {            //Capacitors on Input side gave best result so
              _status.C_relays = C_RelaysTmp;       // set relays back to where they were on input.
              _status.L_relays = L_RelaysTmp;
              _status.outputZ = bestZ;
              setRelays();
              getSWR();
            }
          }
        }
#ifdef DEBUG_COARSE_TUNE_STATUS1
        Serial.println(F("Final coarse tune results"));
        printStatus(printHeader);
        printStatus(printBody);
#endif
        doRelayFineSteps();
        cmd = TUNED;
        _status.currentSWR = _status.rawSWR;
        lcdPrintStatus();
      }
   case TUNED:
      { // Update LCD display
        if (_status.fwd > TX_LEVEL_THRESHOLD) {
          _status.currentSWR = _status.rawSWR;
          lcdPrintBargraph(true);
        } else {
          lcdPrintStatus();          
        }
        break;;
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
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 80 M wire antenna centred on 3.8 mHz
  _status.C_relays = B11000011; // Debug settings for C and L relays
  _status.L_relays = B00001011;
  _status.outputZ  = loZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 40 M wire antenna centred on 7.05 mHz
  _status.C_relays = B00101100; // Debug settings for C and L relays
  _status.L_relays = B00010000;
  _status.outputZ  = hiZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 30 M wire antenna centred on 10.125 mHz
  _status.C_relays = B01001111; // Debug settings for C and L relays
  _status.L_relays = B00001100;
  _status.outputZ  = hiZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
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
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 17 M wire antenna centred on 18.09 mHz
  _status.C_relays = B00010000; // Debug settings for C and L relays
  _status.L_relays = B00001000;
  _status.outputZ  = loZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 15 M wire antenna centred on 21.025 mHz
  _status.C_relays = B00010110; // Debug settings for C and L relays
  _status.L_relays = B00000001;
  _status.outputZ  = loZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Try 12 M wire antenna centred on 21.025 mHz
  _status.C_relays = B00011010; // Debug settings for C and L relays
  _status.L_relays = B00000011;
  _status.outputZ  = loZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Presets for beam antenna

  // Try 20 M beam antenna centred on 14.025 mHz
  _status.C_relays = B00011110; // Debug settings for C and L relays
  _status.L_relays = B00000010;
  _status.outputZ  = loZ;
  setRelays();
  getSWR();
  Serial.println(_status.rawSWR);
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  // Fallback of no relays operated

  _status.C_relays = B00000000; // Debug settings for C and L relays
  _status.L_relays = B00000000;
  _status.outputZ  = hiZ;
  setRelays();
  Serial.println(F("Relays set at zero"));
  if (_status.rawSWR < statusTemp.rawSWR) {
    statusTemp = _status;
  }

  _status = statusTemp;
  setRelays();
  getSWR();
}

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
    swrTemp = fineStep(CAPACITANCE); // Starts with best from stepping L and returns best C swr.
#ifdef DEBUG_RELAY_FINE_STEPS
    printStatus(printBody);
#endif   
    swrTemp = fineStep(INDUCTANCE); // Returns best SWR obtained from stepping L both up and down
#ifdef DEBUG_RELAY_FINE_STEPS
    printStatus(printBody);
    cnt++;
#endif    
  }
  while (swrTemp < bestSWR); // If swr was improved, go again
  
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.print(F("doRelayFineSteps():  Been through loop "));
  Serial.print(cnt);
  Serial.println(F(" times."));
#endif
#ifdef DEBUG_RELAY_FINE_STEPS
  Serial.println(F("-----------------------------------------------------------------------------------"));
  Serial.println(F("Exiting doRelayFineSteps(): values on exit ..."));
  printStatus(printHeader);
  printStatus(printBody);
#endif
}

/**********************************************************************************************************/

#ifdef PRINT_STATUS
void printStatus(boolean doHeader)
{
  if (doHeader) {
    Serial.println(F("C_relays   L_relays   totC  totL  fwdVolt  revVolt  Gain  outZ    rawSWR  SWR"));
  } else {
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
    if (_status.ampGain == hi) Serial.print(F("High"));
    else Serial.print(F(" Low"));
    Serial.print(F("  "));
    if (_status.outputZ == hiZ) Serial.print(F(" HiZ  "));
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
/*
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
*/
/**********************************************************************************************************/

unsigned int calcXvalue(bool reactance)
{
  // We calculate the total values of L or C. reactance is a flag for sum capacitors or sum inductors.
  
  unsigned int val = 0;

  for (byte cnt = 0; cnt < 8; cnt++) {
    if (reactance) {   // True do capacitors, false do inductors
      if (bitRead(_status.C_relays, cnt)) val = val + _capacitors[cnt]; // add reactance assigned to set bits only.
    }
    else {
      if (bitRead(_status.L_relays, cnt)) val = val + _inductors[cnt];
    }
  }
  if (reactance) {
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
  digitalWrite(Llatch, LOW);
  temp = bitRead(Cmap, 7);
  Cmap = Cmap << 1;
  bitWrite(Cmap, 0, temp);
  shiftOut(Ldata, Lclock, MSBFIRST, Cmap); // send this binary value to the Capacitor shift register

  // Set the L Relays from _status.L_relays;
  temp = bitRead(Lmap, 7);
  Lmap = Lmap << 1;
  bitWrite(Lmap, 0, temp);
  shiftOut(Ldata, Lclock, MSBFIRST, Lmap); // send this binary value to the Inductor shift register
  digitalWrite(Llatch, HIGH);

  // Write the total capacitor and inductor values into the "_status" struct.
  _status.totC = calcXvalue(CAPACITANCE);
  _status.totL = calcXvalue(INDUCTANCE);

  // Set the HiZ/LoZ Changeover Relay from the value of _status.outputZ
  if (_status.outputZ) {
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

  unsigned int fwd = 0;
  unsigned int rev = 0;
  boolean attenuatorFlag = false;

 // When looking at an SSB or AM signal we are wanting a voice peak to calculate the SWR. By taking multiple
 // reads of fwd and rev voltages we stand a good chance of striking a peak.
  _status.fwd = 0;
  _status.rev = 0;
  for (byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
    fwd = analogRead(forward);
    rev = analogRead(reverse);
    if (fwd > _status.fwd) _status.fwd = fwd;
    if (rev > _status.rev) _status.rev = rev;
  }

 // Now check to see if we need to change the SWR attenuator and flag if so
  if ((_status.fwd < 300) && (_status.ampGain == lo)) {
    digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
    _status.ampGain = hi;
    attenuatorFlag = true;
  }
  else if (fwd == 1023) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    _status.ampGain = lo;
    attenuatorFlag = true;
  }

  if(attenuatorFlag) { // We altered the attenuator so have to read the fwd and rev again.
    _status.fwd = 0;
    _status.rev = 0;
    for (byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
      fwd = analogRead(forward);
      rev = analogRead(reverse);
      if (fwd > _status.fwd) _status.fwd = fwd;
      if (rev > _status.rev) _status.rev = rev;
    }
  }
  if (_status.fwd > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    if (_status.rev == 0) {
      _status.rev = 1;
      _status.fwd++;
    }
    if (_status.fwd <= _status.rev) _status.fwd = (_status.rev + 1); //Avoid division by zero or negative.
    _status.rawSWR = ((_status.fwd + _status.rev) * 100000) / (_status.fwd - _status.rev);
  }
  else {
    _status.fwd = 0;
    _status.rev = 0;
    _status.rawSWR = 99900000;
  }
#ifdef DEBUG_status
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
  int mask = 0, n;

  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while (num_places) {
    if (v & (0x0001 << num_places - 1)) {
      Serial.print(F("1"));
    }
    else {
      Serial.print(F("0"));
    }
    --num_places;
    if (((num_places % 4) == 0) && (num_places != 0)) {
      Serial.print(F("_"));
    }
  }
}

/**********************************************************************************************************/

byte handle_button()
// Returns  0 = No button pressed or no change of button state occurred
//          1 = Leading edge of button press
//          2 = Trailing edge of short button press (< 200 mSec)
//          3 = Trailing edge of medium button press (200 to 1000 mSec)
//          4 = Trailing edge of long button press (> 1000 mSec)
{
  static boolean button_was_pressed = true; // True = no press i.e. pullup voltage
  static unsigned long timer = 0;
  boolean event;
  byte retval = 0;

// Check for a button change. Boolean event is true if a released button is pressed or
// a pressed button is released
  int button_now_pressed = digitalRead(BUTTON_PIN); // pin low = pressed
  event = (button_now_pressed != button_was_pressed); // Check if button has changed
  if (event) {
    delay(Button_Debounce_Millis); // Delay and re-read the button
    int button_now_pressed = digitalRead(BUTTON_PIN); // pin low -> pressed
    event = (button_now_pressed != button_was_pressed);
  }
  if (event) { // The re-read says it is a valid change of button state
    if (!button_now_pressed) { // The button has changed from released to pressed
      timer = millis();   // so start the button press length timer
#ifdef DEBUG_BUTTONS
      Serial.print(F("Leading edge of button press detected"));
#endif      
      retval = 1; // Return indicating leading edge of a button press
    }
    else { // The button has changed from pressed to released so calc button press type.
      timer = millis() - timer;
#ifdef DEBUG_BUTTONS
      Serial.print(F("Button released, Timer value = "));
      Serial.println(timer);
#endif
      if (timer <= 200) { // Short press
        retval = 2;
      }
      else if (timer < 1000) {
        retval = 3;
      }
      else retval = 4;
    }
    button_was_pressed = button_now_pressed;
  }
  return retval;
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

  for (int x = 0; x < num_of_analog_buttons; x++) {
    button_value = int(1023 * (float(x * analog_buttons_r2) / float((x * analog_buttons_r2) + analog_buttons_r1)));
    lower_button_value = int(1023 * (float((x - 1) * analog_buttons_r2) / float(((x - 1) * analog_buttons_r2) + analog_buttons_r1)));
    higher_button_value = int(1023 * (float((x + 1) * analog_buttons_r2) / float(((x + 1) * analog_buttons_r2) + analog_buttons_r1)));

    _Button_array_min_value[x] = (button_value - ((button_value - lower_button_value) / 2));
    _Button_array_max_value[x] = (button_value + ((higher_button_value - button_value) / 2));

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
  if ((millis() - lastButtonTime) < analog_Button_Debounce_Millis) {
    return 0;
  }

  // OK we are over 'analog_Button_Debounce_Millis' since the last button read so process button.
  lastButtonTime = millis(); // Set timer for next sample period
  //See if a button was pressed
  //  if (analogRead(analog_buttons_pin) <= button_array_high_limit[num_of_analog_buttons-1]) {

  // 32 reads of button effectively averages it
  for (cnt = 0; cnt < 32; cnt++) {
    analogButtonValue = analogButtonValue + analogRead(analog_buttons_pin);
  }
  analogButtonValue = analogButtonValue / cnt;
#ifdef DEBUG_BUTTONS
  if (analogButtonValue < 1020) {
    Serial.print(F("The raw button press value is "));
    Serial.println(analogButtonValue);
  }
#endif
// Now determine which button was pressed if any, else assign button value of 0
  if (analogButtonValue <= _Button_array_max_value[num_of_analog_buttons - 1]) {
    for (cnt = 0; cnt < num_of_analog_buttons; cnt++) {
      if  ((analogButtonValue > _Button_array_min_value[cnt]) &&
        (analogButtonValue <=  _Button_array_max_value[cnt])) {
        thisButton = cnt + 1;
      }
    }
  } 
  else thisButton = 0; // End of "Now determine which button was pressed if any ..."

      // See if we got 2 identical samples in a row
  if (thisButton != lastButtonValue) {
    lastButtonValue = thisButton; // No but setting up now for next sample match.
  }
  else { // We have a valid button press or a valid button release
    if (thisButton != 0) { // It is a press so save the button and check for a long press
      if (currentButton != thisButton) {
        currentButton = thisButton;
        longPressTimer = millis();
      }
      if ((millis() - longPressTimer) > LONG_PRESS_TIME) {
        retVal = currentButton + num_of_analog_buttons;
        longPress = true;
      }
      else retVal = 0;
    }
    else { // We are releasing the button so check if it is from a short or long press
      if (longPress) {
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
  } // End of "See if we got 2 identical samples in a row"

  return retVal;
}

/**********************************************************************************************************/

void processShortPressTE(byte button)
{
  static unsigned long repeatValue = 0;

  if ((millis() - repeatValue) > 60) {
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

  if ((millis() - repeatValue) > 60) {
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
  
  if ((millis() - repeatValue) > 60) {

  }
#ifdef DEBUG_BUTTON_INFO
  Serial.print(F("Loop:  A long press trailing edge detected on button "));
  Serial.println(button);
#endif
}

/**********************************************************************************************************/

int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
/**********************************************************************************************************/

void check_mem() {
  //uint8_t * heapptr, * stackptr;  // I declared these globally
  stackptr = (uint8_t *)malloc(4);  // use stackptr temporarily
  heapptr = stackptr;               // save value of heap pointer
  free(stackptr);                   // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);      // save value of stack pointer
}

/**********************************************************************************************************/

boolean doRelayCoarseSteps()
{
  // This subroutine steps through the capacitor and inductor relays looking for the combination which gives
  // the lowest SWR. Only individual relays are stepped with no multiple C or L combinations so a fine tune
  // subroutine is later called to set exact values for L and C.

  // This procedure is carried out by initially setting capacitor relays to zero C and stepping the inductor
  // relays one by one, incrementing the capacitor and repeating the procedure.
  // The SWR is read at each step into an 2 dimensional array which is later parsed for the lowest SWR and
  // the C and L combination to give this is set along with rawSWR in the _status array.

  // A check of the command button is made and tune aborted with a return of true if a press detected. The
  // caller should perform the abort process.
  
  // Entry: The caller sets the C/O relay to HiZ or LoZ as required
  // Exit with relay settings giving best SWR for the C/O relay setting on entry. Result held in _status.

  uint32_t values[9][9];
  unsigned long bestSWR = 99900000;
  byte bestC = 0;  
  byte bestL = 0;
  char pntBuffer[16];  
/*
  // Initialise with no L or C relays operated and C/O relay set by the caller.

  _status.C_relays = 0;
  _status.L_relays = 0;
  setRelays();
//  delay(20); // Extra settling time for an ultra stable initial reading
  getSWR();  //Get SWR with relays at initial state.
  bestSWR = _status.rawSWR;
*/
// Step through states of no relays operated to all relays operated

  for(byte c =0; c < 9; c++) {
    _status.C_relays = 0;
    if(c != 0) {    
      bitSet(_status.C_relays, c - 1);
    }
    for(byte x = 0; x < 9; x++) {
      _status.L_relays = 0;
      if(x != 0) {        
        bitSet(_status.L_relays, x - 1);
      }
      setRelays();
      // Check at this point for a command button press and abandon tune if so.
      if(handle_button()) { // Any button change triggers abort tune.
        return true;
      }
      getSWR();
      values[c][x] = _status.rawSWR;
    } // end inner for loop
  } //end outer for loop

   // We parse the array looking for the combination of L and C relays which give lowest SWR
  for(byte c =0; c < 9; c++) {
    for(byte x = 0; x < 9; x++) {
      if(values[c][x] < bestSWR) {
        bestSWR = values[c][x];
        bestC = c;
        bestL = x;
      }
    }
  }

  // Now set the relays to give best coarse tune based on bestSWR
  _status.C_relays = 0; // No bits set for no relays operated
  _status.L_relays = 0;
  if(bestC > 0) bitSet(_status.C_relays, bestC - 1); // Set bits 0 .. 7 here (Relays 1 to 8)
  if(bestL > 0) bitSet(_status.L_relays, bestL - 1);
  setRelays();
  delay(20); // Extra settling time for an ultra stable final reading
  getSWR();  //Get SWR with relays at final state.

#ifdef DEBUG_COARSE_TUNE_STATUS // Print the DEBUG header
  Serial.print(F("doRelayCoarseSteps(): Caps are connected to "));
  if (_status.outputZ == hiZ) Serial.print(F("Output (HiZ)"));
  else Serial.print(F("Input (LoZ)"));
  Serial.println();
  Serial.print(F("Cap\t  "));
  for(int x = 0; x < 9; x++) {
    Serial.print(F("  L")); 
    Serial.print(x);
    Serial.print(F("       "));
  } 
  Serial.println();
  
  // Now print the DEBUG body which is the debug data captured in the array
  for(byte c =0; c < 9; c++) {
    Serial.print(F("C"));
    Serial.print(c);
    Serial.print("\t");
    for(byte x = 0; x < 9; x++) {
      dtostrf(float(values[c][x]) / 100000, 11, 5, pntBuffer);  // 11 is mininum width, 5 is decimal places;
      Serial.print(pntBuffer);
/*      sprintf(buffer, "%3lu",values[c][x] / 100000);
      Serial.print(buffer);
      Serial.print(F("."));
      sprintf(buffer, "%-05lu ",values[c][x] % 100000);
      Serial.print(buffer);
      Serial.print(F(" "));
*/
    }
    Serial.println("");
  }
  Serial.print(F("bestC = "));
  Serial.print(bestC);
  Serial.print(F("; bestL = "));
  Serial.print(bestL);
  Serial.print(F("; SWR = "));
  dtostrf(float(_status.rawSWR) / 100000, 11, 5, pntBuffer);  // 11 is mininum width, 5 is decimal places;
  Serial.println(pntBuffer);
  
//  sprintf(buffer, "%3lu",_status.rawSWR / 100000);
//  Serial.print(buffer);
//  Serial.print(F("."));
//  sprintf(buffer, "%-05lu ",_status.rawSWR % 100000);
//  Serial.println(buffer);

#endif //DEBUG_COARSE_TUNE_STATUS
 return false;
} //end subroutine

/**********************************************************************************************************/

uint32_t fineStep(bool reactance) // Enter with swr and relay status up to date
{
  // On entry, we look at the current SWR and the SWR from the 4 relay steps each side to see if there is a trend and
  // in which direction. The SWR from the 9 relays is read into an array which is examined to see if we need to step up
  // or step down. A check is made to ensure that stepping the relays won't overflow 255 or underflow 0 then the array
  // is traversed up or down consecutive relays until the best SWR is centred at values[4]. The associated relays are
  // set in _status array. Parameter reactance determines whether we are stepping _status.C_relays or _status.L_relays.

  uint32_t values[9]; // An array of SWR values centred around the relays set in "_status" at entry
  uint8_t lowRelay;   // The relay combination which gives the SWR held in Values[0] (0 to 255)
  uint8_t cnt;  // Mostly used to point to a position in the array
  uint8_t *pReactance; // A pointer to either _status.C_relays or _status.L_relays
  boolean header = true; // Used to decide whether to print the data table header on debug

  if(reactance == INDUCTANCE) {  // set to operate on either _status.C_relays or _status.L_relays
    pReactance = &_status.L_relays;
  } else {
    pReactance = &_status.C_relays;
  }

  // Load the array with the SWR values obtained from the current "_status_X_Relays" and the relays 4 above & below.
  // Check to see that "uint8_t lowRelay" stay in bounds, i.e does not become less than 0 or greater than 255.
  if((*pReactance  >= 4) && (*pReactance <= 251)) {  // Do only if lowRelay won't over or underflow
    lowRelay = *pReactance - 4;
  } else {
    if(*pReactance < 4) { // lowRelay could go out of bounds here so limit its value
      lowRelay = 0;
    } else lowRelay = 247;
  }
  // Loading the array with SWR's from 9 relays centred around current relay
  cnt = 0;
  for(int x = lowRelay; x < (lowRelay + 9); x++) {
    *pReactance = x; // Select the relay starting from lowRelay and stepping up over a total of 9 relays.
    setRelays();     // and operate it
    getSWR();
    values[cnt] = _status.rawSWR;
    cnt++;
  }        // On exit, _status.X_relays = lowRelays + 8; cnt = 9
  
  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);

  cnt = findBestValue(values, 9); // Get the array position holding the lowest SWR

// Print the contents of the initialised array and if it is Inductor or Capacitor relays being stepped.
#ifdef DEBUG_FINE_STEP  
    Serial.print(F("fineStep: Values on entry using "));
    if(reactance == INDUCTANCE) {
      Serial.println(F("INDUCTORS"));
    } else {
      Serial.println(F("CAPACITORS"));
    }
    printFineValues(printHeader, values, cnt, lowRelay);
    printFineValues(printBody, values, cnt, lowRelay);
#endif  

  // Assume if cnt < 4, we need to search down but not if lowRelay at 0 or we will underflow
  // If cnt = 4 we have found the SWR dip
  // If cnt > 4, we need to search up but not if lowRelay at 247 or more else we will overflow

  while(cnt != 4) {
    if(((lowRelay == 0) && (cnt < 5)) || ((lowRelay == 247) && (cnt > 3))) {
#ifdef DEBUG_FINE_STEP
      Serial.println(F("We will over/underflow so choosing best value"));
#endif      
      break;
    } // ----------------------------------------------------
    else if(cnt < 4) { // We won't over/underflow and need to search down
#ifdef DEBUG_FINE_STEP
  if(header) Serial.println(F("cnt < 4 so searching down"));
#endif
      lowRelay--;
      for(uint8_t i = 8; i > 0; --i){ // Shift the array to the right 8 steps
        values[i]=values[i-1];   
      }
      *pReactance = lowRelay;
      setRelays();
      getSWR();
      delay(5);
      getSWR();
      values[0] = _status.rawSWR;     
    } // ----------------------------------------------------
    else { // We won't over/underflow and need to search up
#ifdef DEBUG_FINE_STEP
  if(header) Serial.println(F("cnt > 4 so searching up"));
#endif
      lowRelay++;
      for(uint8_t i=0; i < 8; i++){ // Shift the array to the left 8 steps
        values[i]=values[i+1];
      }
      *pReactance = lowRelay + 8;
      setRelays();
      getSWR();
      delay(5);
      getSWR();
      values[8] = _status.rawSWR;
    } // ----------------------------------------------------
    cnt = findBestValue(values, 9);
#ifdef DEBUG_FINE_STEP // Print a table of SWR values in the array at this point
  if(header) {         // Do a header if first time through the "while" loop
    printFineValues(printHeader, values, cnt, lowRelay);
    header = false;
  }
  printFineValues(printBody, values, cnt, lowRelay);
#endif     
    displayAnalog(0, 0, _status.fwd);
    displayAnalog(0, 1, _status.rev);
  } // End while =============================================

  // Set up the _status struct with the relay pointed to by lowRelay plus the cnt offset which
  // will correspond to the lowest SWR for this pass of fine tune.
  *pReactance = lowRelay + cnt;
  setRelays();
  setRelays();     // Extra relay switching for an accurate final reading
  delay(20);   // Extra relay settling time for an accurate final reading
  getSWR();

  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);
  
#ifdef DEBUG_FINE_STEP
    Serial.print(F("fineStep: Values on exit using "));
    if(reactance == INDUCTANCE) {
      Serial.println(F("INDUCTORS"));
    } else {
      Serial.println(F("CAPACITORS"));
    }
    printStatus(printHeader);
    printStatus(printBody);
#endif  
  return _status.rawSWR;
}

/**********************************************************************************************************/

void printFineValues(boolean doHeader, uint32_t values[], uint8_t cnt, uint8_t lowRelay)
{
  int x;
  char pntBuffer[16];
  
  if(doHeader) {
    for(x = 0; x < 9; x++) {
      Serial.print(F("  Values["));
      Serial.print(x);
      Serial.print(F("]"));
    }
    Serial.print(F("   lowRelay "));
    Serial.println(F("cnt"));
  } else {
    for(x = 0; x < 9; x++) {
//      Serial.print(float(values[x]) / 100000, 4);
//      Serial.print(F("     "));
      dtostrf(float(values[x]) / 100000, 11, 4, pntBuffer);  // 8 is mininum width, 4 is decimal places;
      Serial.print(pntBuffer);
    }
    Serial.print("   ");
    Serial.print(lowRelay);
    if(lowRelay < 10) Serial.print("\t");
    Serial.print("\t");
    Serial.println(cnt);
  }
}

/**********************************************************************************************************/

uint8_t findBestValue(uint32_t values[], uint8_t cnt)
// Parses an array of uint32_t values, whose length is equal to cnt. The position in the array (0 to cnt-1)
// containing the smallest value is returned. If all positions are of equal value then position 0 is returned.
// If more than one position contains the lowest value then the first position containing it is returned.

{
  uint32_t bestValue = 0;
  uint8_t bestPosition = 0;

  bestValue--;  // Initialize by rolling back to set to maximum value

  for (uint8_t x = 0; x < cnt; x++) {
    if(values[x] <= bestValue) {
      bestValue = values[x];
      bestPosition = x;
    }
  }
  return bestPosition;
}

/**********************************************************************************************************/
