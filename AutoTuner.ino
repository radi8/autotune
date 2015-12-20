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
             C1   | [ ]A1      /  A  \      D8[ ] |   B0
             C2   | [ ]A2      \  N  /      D7[ ] |   D7
             C3   | [ ]A3       \_0_/       D6[ ]~|   D6
             C4   | [ ]A4/SDA               D5[ ]~|   D5
             C5   | [ ]A5/SCL               D4[ ] |   D4
                  | [ ]A6              INT1/D3[ ]~|   D3
                  | [ ]A7              INT0/D2[ ] |   D2
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
enum commandMode {TUNED, TUNE, TUNING};
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
    if (_cmd != TUNED) {
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
  dtostrf(float(_status.rawSWR) / 100000, 7, 3, charVal);  //7 is mininum width, 3 is precision;
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
        if (_status.fwd > TX_LEVEL_THRESHOLD) {
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
        if (_status.rawSWR > 130000) { // Debug change to force coarse tuning
          //      if(_status.rawSWR > 1) {
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

          if (_status.rawSWR > 120000) { // Only try again if swr needs improving
            _status.outputZ = loZ;
            doRelayCoarseSteps(); //Run it again and see if better with C/O relay operated
            //If not better restore relays to input state
            //          getSWR();
#ifdef DEBUG_COARSE_TUNE_STATUS
            Serial.println(F("LoZ coarse tune results"));
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
    swrTemp = fineStep(INDUCTANCE); // Returns best SWR obtained from stepping L both up and down
#ifdef DEBUG_RELAY_FINE_STEPS
    Serial.print(F("doRelayFineSteps():  Been through loop "));
    Serial.print(cnt);
    Serial.println(F(" times."));
    cnt++;
    Serial.println(F("-----------------------------------------------------------------------------------"));
#endif
  }
  while (swrTemp < bestSWR); // If swr was improved, go again

#ifdef DEBUG_RELAY_FINE_STEPS
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

unsigned int calcXvalue(bool CorL)
{
  // We calculate the total values of L or C. CorL is a flag to determine which reactance to sum up.
  
  unsigned int val = 0;

  for (byte cnt = 0; cnt < 8; cnt++) {
    if (CorL) {   // True do capacitors, false do inductors
      if (bitRead(_status.C_relays, cnt)) val = val + _capacitors[cnt]; // add reactance assigned to set bits only.
    }
    else {
      if (bitRead(_status.L_relays, cnt)) val = val + _inductors[cnt];
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

  unsigned long fwd = 0;
  unsigned long rev = 0;

  for (byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
    fwd = fwd + analogRead(forward);
    rev = rev + analogRead(reverse);
  }
  fwd = fwd / SWR_AVERAGE_COUNT;
  rev = rev / SWR_AVERAGE_COUNT;
  delay(1);

  if ((fwd < 300) && (_status.ampGain == lo)) {
    digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
    _status.ampGain = hi;
    delay(1);
    fwd = analogRead(forward);
  }
  else if (fwd == 1023) {
    digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
    _status.ampGain = lo;
    delay(1);
    fwd = analogRead(forward);
  }
  if (fwd > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    fwd = 0;
    rev = 0;
    for (byte x = 1; x < (SWR_AVERAGE_COUNT + 1); x++) {
      fwd = fwd + analogRead(forward);
      rev = rev + analogRead(reverse);
    }
    fwd = fwd / SWR_AVERAGE_COUNT;
    rev = rev / SWR_AVERAGE_COUNT;
    if (rev == 0) rev = 1;
    if (fwd <= rev) fwd = (rev + 1); //Avoid division by zero or negative.
    _status.fwd = fwd;
    _status.rev = rev;
    _status.rawSWR = ((fwd + rev) * 100000) / (fwd - rev);
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
  if (event) { // The re-read says it is a valid change of button state
    if (!button_now_pressed) { // The button has changed from released to pressed
      timer = millis();   // so start the button press length timer
    }
    else { // The button has changed from pressed to released so calc button press type.
      timer = millis() - timer;
#ifdef DEBUG_BUTTONS
      Serial.print(F("Button released, Timer value = "));
      Serial.println(timer);
#endif
      if (timer <= 200) { // Short press
        retval = 1;
      }
      else if (timer < 1000) {
        retval = 2;
      }
      else retval = 3;
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

void doRelayCoarseSteps()
{
  // This subroutine steps through the capacitor and inductor relays looking for the combination which gives
  // the lowest SWR. Only individual relays are stepped with no multiple C or L combinations so a fine tune
  // subroutine is later called to set exact values for L and C.

  // This procedure is carried out by initially setting capacitor relays to zero C and stepping the inductor
  // relays one by one, incrementing the capacitor and repeating the procedure.
  // The SWR is read at each step into an 2 dimensional array which is later parsed for the lowest SWR and
  // the C and L combination to give this is set along with rawSWR in the _status array.
  
  // Entry: The caller sets the C/O relay to HiZ or LoZ as required
  // Exit with relay settings which give best SWR for the C/O relay setting on entry.

  unsigned long values[9][9];
  unsigned long bestSWR = 99900000;
  byte bestC = 0;  
  byte bestL = 0;
  char buffer[16];  

  // Initialise with no L or C relays operated and C/O relay set by the caller.
  _status.C_relays = 0;
  _status.L_relays = 0;
  setRelays();
//  delay(20); // Extra settling time for an ultra stable initial reading
  getSWR();  //Get SWR with relays at initial state.
  bestSWR = _status.rawSWR;

  for(byte c =0; c < 9; c++) {
    if(c != 0) {
      _status.C_relays = 0;
      bitSet(_status.C_relays, c - 1);
    }
    for(byte x = 0; x < 9; x++) {
      if(x != 0) {
        _status.L_relays = 0;
        bitSet(_status.L_relays, x - 1);
      }
      setRelays();
      getSWR();
      values[c][x] = _status.rawSWR;
    }
  } //endfor    

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
  else Serial.print(F("Input (LoZ"));
  Serial.println();
  Serial.print("Cap\t");
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
      sprintf(buffer, "%3lu",values[c][x] / 100000);
      Serial.print(buffer);
      Serial.print(F("."));
      sprintf(buffer, "%-05lu ",values[c][x] % 100000);
      Serial.print(buffer);
      Serial.print(F(" "));
    }
    Serial.println("");
  }
  Serial.print(F("bestC = "));
  Serial.print(bestC);
  Serial.print(F("; bestL = "));
  Serial.print(bestL);
  Serial.print(F("; SWR = "));
  sprintf(buffer, "%3lu",_status.rawSWR / 100000);
  Serial.print(buffer);
  Serial.print(F("."));
  sprintf(buffer, "%-05lu ",_status.rawSWR % 100000);
  Serial.println(buffer);
#endif //DEBUG_COARSE_TUNE_STATUS
 
} //end subroutine

/**********************************************************************************************************/

uint32_t fineStep(bool reactance) // Enter with swr and relay status up to date
{
  // On entry, the relays we are exploring (_status.C_relays or _status.L_relays) can be anywhere from 0 to
  // 255. Initially we exploreint(lowRelay) the relays each side to see if we need to step up or step down. The SWR from
  // the 8 reactances surrounding the "_status.X_relays" value are read into_status.C_relays an array and examined with the
  // current value centred at values[4]. A check is made to ensure that less or more relays won't overflow
  // or underflow 0 to 255 wheint(lowRelay)n attempting to set the current position data into values[4]

  uint32_t values[9]; // An array of SWR values centred around the relays set in "_status"
  uint8_t lowRelay;   // The relay combination which gives the SWR held in Values[0] (0 to 255)
  uint8_t cnt;
  uint8_t *pReactance;

  if(reactance == INDUCTANCE) {
    pReactance = &_status.L_relays;
  } else {
    pReactance = &_status.C_relays;
  }

  // Load the array with the SWR values obtained from the current "_status_X_Relays" and the relays 4 above
  // & below. Check to see thaint(lowRelay)t relays stay in bounds, i.e not less than 0 or not greater than 255.
  if((*pReactance  >= 4) && (*pReactance <= 251)) {  // Don't let lowRelay over or underflow
    lowRelay = *pReactance - 4;
//    cout << "Relays in bounds, lowRelay = " << int(lowRelay) << endl;
  } else {
    if(*pReactance < 4) {
      lowRelay = 0;
    } else lowRelay = 247;
//    cout << "Relays out of bounds, lowRelay = " << int(lowRelay) << endl;
  }
  // Loading the array
  cnt = 0;
  for(int x = lowRelay; x < (lowRelay + 9); x++) {
    *pReactance = x;
    // setRelays();
    getSWR();
    values[cnt] = _status.rawSWR;
    cnt++;
  }
  // On exit, _status.X_relays = lowRelays + 8; cnt = 9
  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);
//  cout << "_status.X_relays = " << int(*pReactance) << endl; // DEBUG
//  displayArray(values, 9);  // DEBUG

  cnt = findBestValue(values, 9);
//  cout << "cnt = " << int(cnt) << ";  value = " << values[cnt] << "; lowRelay = " << int(lowRelay) << endl; // DEBUG

  // Assume if cnt < 4, we need to search down but not if lowRelay at 0 or we will underflow
  // If cnt = 4 we have found the SWR dip
  // If cnt > 4, we need to search up but not if lowRelay at 247 or we will overflow


  while(cnt != 4) {
    if(((lowRelay == 0) && (cnt < 5)) || ((lowRelay == 247) && (cnt > 3))) {
      break;
    } else if(cnt < 4) { // We need to search down
//      cout << "cnt = " << int(cnt) << " so searching down" << endl;
      lowRelay--;
      shiftRight(values, 8);
      *pReactance = lowRelay;
      // setRelays();
      getSWR();
      values[0] = _status.rawSWR;
    } else { // We need to search up
//      cout << "cnt = " << int(cnt) << " so searching up" << endl;
      lowRelay++;
      shiftLeft(values, 8);
      *pReactance = lowRelay + 8;
      // setRelays();
      getSWR();
      values[8] = _status.rawSWR;
    }
    cnt = findBestValue(values, 9);
    displayAnalog(0, 0, _status.fwd);
    displayAnalog(0, 1, _status.rev);
  }
  *pReactance = lowRelay + cnt;
  _status.rawSWR = values[cnt];
//  cout << "cnt = " << int(cnt) << ";  _status.C_relays = " << int(*pReactance) << "; _status.rawSWR = " << _status.rawSWR << endl;
  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);
  return _status.rawSWR;
}

/**********************************************************************************************************/

void shiftLeft(uint32_t values[], uint8_t cnt)
{
    // Shift left "cnt" places. The leftmost value drops off, rightmost is left vacant (unchanged).
    // cnt is equal to 1 less than the number of elements in the array to shift all elements

  for(uint8_t i=0; i < cnt; i++){
    values[i]=values[i+1];
  }
}

/**********************************************************************************************************/

void shiftRight(uint32_t values[], uint8_t cnt)
{
    // Shift right "cnt" places. The rightmost value drops off, leftmost is left vacant (unchanged).
    // cnt must be equal to the number of elements in the array

  for(uint8_t i = cnt; i > 0; --i){
    values[i]=values[i-1];
  }
}

/**********************************************************************************************************/
/*
void displayArray(uint32_t values[], uint8_t cnt)
{

  for(int x = 0; x < cnt; x++){
    cout << "values[" << x <<"] = " << values[x] <<endl;
  }
}

/**********************************************************************************************************/

uint8_t findBestValue(uint32_t values[], uint8_t cnt)
// Parses an array of uint32_t values, whose length is equal to cnt. The position in the array (0 to cnt-1)
// containing the smallest value is returned. If all positions are of equal value then position 0 is returned.

{
  uint32_t bestValue = 0;
  uint8_t bestPosition = 0;

  bestValue--;  // Initialize by rolling back to set to maximum value

  for (uint8_t x = 0; x < cnt; x++) {
    if(values[x] < bestValue) {
      bestValue = values[x];
      bestPosition = x;
    }
  }
  return bestPosition;
}

/**********************************************************************************************************/
