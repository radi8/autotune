//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
/////////////////////////////////////////////////////////////////
/*
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
  Heartbeat  B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
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
#include <EEPROM.h>
#include "defines.h"


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
String rx_str = "";
boolean not_number = false;
unsigned int eepromTermAddr = 0;

struct status {
  float fwd;
  float rev;
  float retLoss;
  boolean tx_on;
  boolean ampGain;
  unsigned int freq;
  byte C_relays;
  byte L_relays;
  boolean outputZ;
  unsigned int totC;
  unsigned int totL;
} _status;

struct MyValues {
  unsigned int freq;
  byte L;
  byte C;
  byte Z;
} val;

//         Inductor definitions     L1   L2   L3   L4    L5    L6    L7    L8
const unsigned int  _inductors[] = { 6,  17,  35,  73,  136,  275,  568, 1099 };  // inductor values in nH
const unsigned int _strayL = 0;
//          Capacitor definitions   C1   C2   C3   C4    C5    C6    C7    C8
const unsigned int _capacitors[] = { 6,  11,  22,  44,   88,  168,  300,  660 };  // capacitor values in pF
const unsigned int _strayC = 0;

enum {INDUCTANCE, CAPACITANCE};
enum {POWERUP, TUNE, TUNING, TUNED};
byte _cmd = POWERUP;  // Holds the command to be processed
const float tuneSWR = 18; //If retLoss < this value, forces a coarse tune. (Small value forces a tune)

boolean preset;  // TODO remove this debug info on final version
//----------------------------------------------------------------------------------------------------------

void setup()
{
  // First thing up, set C & L Relays to all off.
  pinMode(Lclock, OUTPUT); // make the Inductor clock pin an output
  pinMode(Llatch, OUTPUT); // make the Inductor latch pin an output
  pinMode(Ldata , OUTPUT); // make the Inductor data pin an output
  pinMode(coRelay, OUTPUT);
  digitalWrite(Lclock, LOW);
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
  //  digitalWrite(LEDpin, LOW);

  lcd.begin(lcdNumRows, lcdNumCols);
  // -- do some delay: some times I've got broken visualization
  delay(100); // TODO Is this really necessary?
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
//  EEPROM[EEPROM.length()-1] = 0;  //Uncomment this to force a reload of EEPROM values
//  eeprom_initialise();            // and this line too

  Serial.println(F("Arduino antenna tuner ver 2.0.0"));
  Serial.println(F("Copyright (C) 2015, Graeme Jury ZL2APV"));
  Serial.println();
  EEPROM.get(eepromTermAddr, val.freq); // Initialise the terminator address for the eeprom
  while (val.freq) {
    eepromTermAddr += sizeof(MyValues);
    EEPROM.get(eepromTermAddr, val.freq);
  }
  eeprom_Print();
}

//----------------------------------------------------------------------------------------------------------
void loop()
{

  byte buttonNumber;
  static unsigned long heartbeat = millis();
  
  pollSerial();
  getSWR();
  //  Serial.print(F("_cmd = ")); Serial.println(_cmd);
  _cmd = processCommand(_cmd);
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
      _cmd = POWERUP; // Any press halts a tune in progress
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

byte processCommand(byte cmd)
{

  float SWRtmp;
  byte C_RelaysTmp; // Holds map of operated relays with C/O on input
  byte L_RelaysTmp; //  0 = released and 1 = operated
  boolean bestZ;

  switch (cmd) {
    case POWERUP:
      { // No tuning has been performed yet
        if (_status.tx_on) { // We have enough Tx power so display the bargraph
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
        if (_status.tx_on) {
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
        preset = true;  // TODO remove this debug info on final version
        tryPresets();
        preset = false;  // TODO remove this debug info on final version
#ifdef DEBUG_COARSE_TUNE_STATUS
        Serial.print(F("@processCmd() .. _status.retLoss = "));
        Serial.println(_status.retLoss, 5);
#endif
        if (_status.retLoss < tuneSWR) {
#ifdef DEBUG_COARSE_TUNE_STATUS
          Serial.println(F("Got here as no suitable preset found"));
#endif
          _status.outputZ = loZ;
          // function doRelayCoarseSteps() returns true if tune aborted with cmd button press
          if (doRelayCoarseSteps()) {
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
          SWRtmp = _status.retLoss;

#ifdef DEBUG_COARSE_TUNE_STATUS1
          Serial.println(F("LoZ coarse tune results"));
          printStatus(printHeader);
          printStatus(printBody);
#endif

          if (_status.retLoss < 20) { // Only try again if swr needs improving (VSWR = 1.22:1)
            _status.outputZ = hiZ;
            doRelayCoarseSteps(); //Run it again and see if better with C/O relay operated

#ifdef DEBUG_COARSE_TUNE_STATUS1
            Serial.println(F("HiZ coarse tune results"));
            printStatus(printHeader);
            printStatus(printBody);
#endif
            if (SWRtmp > _status.retLoss) {         // Capacitors on Input side gave best result so
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
        lcdPrintStatus();
      }
    case TUNED:
      { // Update LCD display
        if (_status.tx_on) {
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


//----------------------------------------------------------------------------------------------------------
void doRelayFineSteps()
{
  byte Crelays;
  byte Lrelays;

  getSWR(); // Get swr and relay status up to date.

#ifdef DEBUG_RELAY_FINE_STEPS
  int cnt = 1;
  Serial.println(F("@doRelayFineSteps: Values on entry"));
  printStatus(printHeader);
  printStatus(printBody);
#endif

  do {
    Lrelays = _status.L_relays;
    fineStep(INDUCTANCE); // Starts with best from stepping L relays.

#ifdef DEBUG_RELAY_FINE_STEPS
    //    printStatus(printBody);
#endif
    Crelays = _status.C_relays;
    fineStep(CAPACITANCE); // Try for an improvement from stepping C relays
#ifdef DEBUG_RELAY_FINE_STEPS
    printStatus(printBody);
    cnt++;
#endif
  }
  while ((_status.L_relays != Lrelays) && (_status.C_relays != Crelays)); // If any relays alter, go again

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

//--------------------------------------------------------------------------------------------------------/

void fineStep(bool LC) // Enter with swr and relay status up to date
{
  // On entry, we look at the return Loss from the current relay settings and the returnLoss from the
  // 4 relay steps each side to see if there is a trend and in which direction. The return loss from the
  // 9 relays is read into an array which is examined to see if we need to step up or step down. A check
  // is made to ensure that stepping the relays won't overflow 255 or underflow 0 then the array is
  // traversed up or down consecutive relays until the best SWR is centred at values[4]. The associated
  // relays are set in _status array. Parameter reactance determines whether we are stepping
  // _status.C_relays or _status.L_relays.

  float values[ARRAY_SIZE]; // Array to hold SWR values as L and C relays are traversed
  uint8_t *pReactance; // A pointer to either _status.C_relays or _status.L_relays

  float startRetLoss;
  float bestRetLoss = 0;

  uint8_t bestRelays = 0; // Record the relays holding the best rawSWR to date
  uint8_t arrayStartRelay = 0;
  //  int startRelay = 0; // Declared as int as we may over or underflow 255 during testing relay positions.

  int x;
  /*
    // TEST DATA HERE, ERASE ON FINAL VERSION !!!!!
    _status.L_relays = 251;
    _status.C_relays = 10;
    for (x = 0; x < ARRAY_SIZE; x++) {
      values[x] = x + 10;
    }
    values[4] = 50; // Set to correspond with _status.L_relays after array loaded
    values[4] = 55; // Optional simulated better return loss, otherwise = to above
    // END OF TEST DATA !!!!!
  */
  // 1. Housekeeping
  // ---------------
  if (LC == INDUCTANCE) { // Choose whether we are working on the inductance or capacitance relays
    pReactance = &_status.L_relays;
  } else {
    pReactance = &_status.C_relays;
  }

  // 2. Test that we won't overflow the relay boundries and adjust
  // -------------------------------------------------------------

  // There are 256 steps (0 - 255) for the L and C relays. We check here that we won't go above 255 or
  // below 0 as we check the return loss for the L or C relay set grouped around our starting point.
  // Set the arrayStartRelay value which will point to the first value in the array which may result
  // in its being set over or under 255 for this initial setting.
  // EXIT:  *pReactance points to the relays of best return loss as per function entry value.
  //        arrayStartRelay points to the relays corresponding to the first element of the array.
  // NOTE: an unsigned int will overflow from 255 to 0 or from 0 to 255.

  arrayStartRelay = *pReactance - ARRAY_SIZE / 2;
#ifdef DEBUG_FINE_STEP
  Serial.println();
  Serial.print("arrayStartRelay value 1 = "); Serial.println(arrayStartRelay);
#endif
  if (*pReactance <= ARRAY_SIZE / 2) {
    arrayStartRelay = 0;
#ifdef DEBUG_FINE_STEP
    Serial.print("arrayStartRelay value 2 = "); Serial.println(arrayStartRelay);
#endif
  }
  if (*pReactance >= 255 - ARRAY_SIZE / 2) {
    arrayStartRelay = 255 - ARRAY_SIZE + 1;
#ifdef DEBUG_FINE_STEP
    Serial.print("arrayStartRelay value 3 = "); Serial.println(arrayStartRelay);
#endif
  }
#ifdef DEBUG_FINE_STEP
  Serial.print("After step 2, *pReactance = "); Serial.println(*pReactance);
#endif

  // 3. Load the array with return loss values from the relays surrounding the entry relays
  // --------------------------------------------------------------------------------------

  // Start loading the array with SWR values knowing that we won't overflow either the L or C relays
  // Note that if we are at a relay boundry we may not have the entry relay at the array centre.

  *pReactance = arrayStartRelay;  // Set the L or C Relays values to the left side of the array
#ifdef DEBUG_FINE_STEP
  Serial.print("After step 3 setting to LHS of array, *pReactance = "); Serial.println(*pReactance);
#endif

  // Now read the set of return loss into the array.
  for (x = 0; x < ARRAY_SIZE; x++) {
    setRelays();
    getSWR();
    values[x] = _status.retLoss;
    *pReactance = (*pReactance + 1);
  }
  *pReactance = (*pReactance - 1); // Remove the extra step created before the loop terminated.
  // we finish with the *pReactance aligned with the RHS value of the array.
#ifdef DEBUG_FINE_STEP
  Serial.print("After step 3 array loaded, *pReactance = "); Serial.println(*pReactance);
#endif

  //4.  Shift the array left or right to set the best return loss value to the centre of the array
  // --------------------------------------------------------------------------------------------

  startRetLoss = _status.retLoss;
  bestRelays = *pReactance; // Save our best relay settings to date which is our entry value so far.

#ifdef DEBUG_FINE_STEP
  Serial.println(); // Print the header for the table of relay stepping
  if (LC == INDUCTANCE) {
    Serial.println(F("\t\t\t   Inductance"));
  } else {
    Serial.println(F("\t\t\t   Capacitance"));
  }
  Serial.print("\t");
  for (x = arrayStartRelay; x < arrayStartRelay + ARRAY_SIZE; x++) {
    Serial.print(x); Serial.print("\t");
  }
  Serial.println();
#endif

#ifdef DEBUG_FINE_STEP
  for (x = 0; x < ARRAY_SIZE; x++) {
    Serial.print("\t");
    Serial.print(values[x], 3);
  }
#endif

  findbestRetLoss(arrayStartRelay, pReactance, values);
  Serial.print(F("\t*pReactance = ")); Serial.print(*pReactance);
  Serial.print(F(", arrayStartRelay = ")); Serial.println(arrayStartRelay);
  bestRelays = *pReactance;

  // Testing for the location of best return loss relative to the centre of the array to see if we need
  // to shift right or left. if cnt < valuesCentre, we need to search down but not if lowRelay at 0 or we will underflow
  // If cnt = valuesCentre we have found the SWR dip
  // If cnt > valuesCentre, we need to search up but not if lowRelay at 255-valuesSize or more else
  // we will overflow.

  if (bestRelays == (arrayStartRelay + ARRAY_SIZE / 2)) { // We are already at the array centre
    Serial.println(F("At array centre"));
  } else {
    if (bestRelays < (arrayStartRelay + ARRAY_SIZE / 2)) { // We want to traverse right
      Serial.println(F("Moving array window left"));

      while ((arrayStartRelay > 0) && (bestRelays - arrayStartRelay < (ARRAY_SIZE / 2))) {
        //        Serial.print(F("bestRelays - arrayStartRelay = ")); Serial.println(bestRelays - arrayStartRelay);
        for (x = ARRAY_SIZE - 1; x > 0; x--) {
          values[x] = values[x - 1];
        }
        arrayStartRelay -= 1;
        *pReactance = arrayStartRelay;
        setRelays();
        getSWR();
        values[0] = _status.retLoss;
        findbestRetLoss(arrayStartRelay, pReactance, values);
        bestRelays = *pReactance;

#ifdef DEBUG_FINE_STEP
        for (x = 0; x < ARRAY_SIZE; x++) {
          Serial.print("\t");
          Serial.print(values[x], 3);
        }
        Serial.print(F("\tarrayStartRelay-1+ARRAY_SIZE = ")); Serial.println(arrayStartRelay - 1 + ARRAY_SIZE);
#endif
      }
    } else {
      Serial.println(F("Moving array window right"));
      Serial.print(F("arrayStartRelay = ")); Serial.println(arrayStartRelay);
      Serial.print(F("255 - ARRAY_SIZE = ")); Serial.println(255 - ARRAY_SIZE);
      Serial.print(F("bestRelays - arrayStartRelay = ")); Serial.println(bestRelays - arrayStartRelay);
      while ((arrayStartRelay <= 255 - ARRAY_SIZE) && (bestRelays - arrayStartRelay > (ARRAY_SIZE / 2))) {

        for (x = 0; x < ARRAY_SIZE; x++) {
          values[x] = values[x + 1];
        }
        arrayStartRelay += 1;
        *pReactance = arrayStartRelay + ARRAY_SIZE;
        setRelays();
        getSWR();
        values[ARRAY_SIZE - 1] = _status.retLoss;
        findbestRetLoss(arrayStartRelay, pReactance, values);
        bestRelays = *pReactance;

#ifdef DEBUG_FINE_STEP
        for (x = 0; x < ARRAY_SIZE; x++) {
          Serial.print("\t");
          Serial.print(values[x], 3);
        }
        Serial.print(F("\tarray end = ")); Serial.println(arrayStartRelay - 1 + ARRAY_SIZE);
#endif
      }
    }
  }
  *pReactance = bestRelays;
  setRelays();     // now operate them.
  getSWR();        // and get final rawSWR value

#ifdef DEBUG_FINE_STEP
  Serial.print(F("Exiting fineStep(), *pReactance = ")); Serial.println(*pReactance);
  Serial.print(F("values[ARRAY_SIZE/2] = ")); Serial.println(values[ARRAY_SIZE / 2]);
  Serial.print(F("bestRelays = ")); Serial.println(bestRelays);
  //  Serial.print(F("startRelay = ")); Serial.println(startRelay);
  Serial.print(F("arrayStartRelay = ")); Serial.println(arrayStartRelay);
#endif

}

//--------------------------------------------------------------------------------------------------------/

void findbestRetLoss(uint8_t arrayStartRelay, uint8_t* pReactance, float values[])
{
  // We need to scan the array to find the highest return loss and load the _status struct with
  // the rawSWR value and the C and L relay values which produced it.

  float bestRetLoss = 0;
  uint8_t bestRelays = 0;

  for (uint8_t x = 0; x < ARRAY_SIZE; x++) {
    if (values[x] > bestRetLoss) {
      bestRetLoss = values[x];
      bestRelays = arrayStartRelay + x;
    }
  }
  *pReactance = bestRelays; // ToDo see if we can get rid of bestRelays
}

//--------------------------------------------------------------------------------------------------------/

void eeprom_Load(unsigned int freq)
{
  // Loads the passed frequency into the eeprom table of presets
  // If freq = 0 then the table is cleared and reset to address 0 as start/end of table
  // If frequency is higher than any other in the table it will be appended
  // If frequency is lower than any table entry it will be pre-pended
  // Otherwise the frequency will be inserted so as to keep the table in ascending frequency.

  int eeAddress = 0;
  int eeAddrHi = 0;
  boolean endFlag = false;

  if (freq == 0) {
    eepromTermAddr = 0;
    EEPROM.put(eepromTermAddr, 0);
    Serial.println(F("Relay table has been zeroed"));
  } else {
    EEPROM.get(eeAddress, val.freq);
    if (val.freq == 0) {
      //    endFlag = true; // If 1st addr = 0 (terminator) then we have a non loaded eeprom
      EEPROM.put(eeAddress, freq);
      eepromTermAddr += sizeof(MyValues);
      EEPROM.put(eepromTermAddr, 0);
      Serial.print(F("eeAddress = ")); Serial.print(eeAddress);
      Serial.print(F(", eepromTermAddr = ")); Serial.println(eeAddress);
    }
    while ((val.freq) || (endFlag)) {
      // We only get here if val.freq is a non zero number or we want to append the frequency
      if (val.freq == freq) {
        // The frequency we are entering matches a frequency already in the presets
        // so simply overwrite it.
        val.L = _status.L_relays;
        val.C = _status.C_relays;
        val.Z = _status.outputZ;
        EEPROM.put(eeAddress, val);
        Serial.print(F("freq ")); Serial.print(val.freq); Serial.print(F(" written to presets at address "));
        Serial.println(eeAddress);
        break;
      } else {
        Serial.print(F("val.freq = ")); Serial.print(val.freq);
        Serial.print(F(";  freq = ")); Serial.print(freq);
        Serial.print(F(";  eeAddress = ")); Serial.println(eeAddress);
        // -------------------------------------------------------
        if ((val.freq > freq) || (endFlag)) { // Don't process for frequencies less than the entry frequency
          // val.freq must be greater than freq or we have reached the end to get here.

          // We are going to insert the tune values in the address before this one
          // (which is held in eeAddrHi). All the values above will need to be
          // shifted up one to allow the tune data to be inserted.
          Serial.println();

          eeAddrHi = eeAddress;
          eeAddress = eepromTermAddr;;
          eepromTermAddr += sizeof(MyValues);//Move address to one struct item beyond
          EEPROM.put(eepromTermAddr, 0);
          Serial.print(F("Entering while loop, eeAddrHi = ")); Serial.print(eeAddrHi);
          Serial.print(F(";  eeAddress = ")); Serial.print(eeAddress);
          Serial.print(F(";  eepromTermAddr = ")); Serial.println(eepromTermAddr);

          while ((eeAddress >= eeAddrHi) || (eeAddress == 0)) {
            EEPROM.get(eeAddress, val.freq);
            Serial.print(F("Start address = ")); Serial.print(eeAddress); Serial.print(F("  val.freq = ")); Serial.print(val.freq);
            eeAddress += sizeof(unsigned int);  // Step off freq to L
            EEPROM.get(eeAddress, val.L);
            eeAddress += sizeof(byte);          // Step off L to C
            EEPROM.get(eeAddress, val.C);
            eeAddress += sizeof(byte);          // Step off C to Z
            EEPROM.get(eeAddress, val.Z);
            eeAddress += sizeof(byte);
            EEPROM.put(eeAddress, val);
            Serial.print(F("  Finish address = ")); Serial.println(eeAddress);
            eeAddress -= (sizeof(MyValues) * 2);
          }

          val.freq = freq;
          val.L = _status.L_relays;
          val.C = _status.C_relays;
          val.Z = _status.outputZ;
          eeAddress = eeAddrHi;
          EEPROM.put(eeAddress, val);
          Serial.print(F("insert freq ")); Serial.print(val.freq); Serial.print(F(" written to presets at address "));
          Serial.println(eeAddress);
          eeAddress = (eepromTermAddr - sizeof(MyValues));
          break;
        }
        // -------------------------------------------------------

        eeAddress += sizeof(MyValues);//Move address to the next struct item
        //      Serial.print(F("eeAddress = ")); Serial.println(eeAddress);
        EEPROM.get(eeAddress, val.freq); //Get next frequency or 0000 if at end
        if (val.freq == 0) endFlag = true;
        //      Serial.print(F("Exiting else, val.freq = ")); Serial.println(val.freq);
      }
    }
    EEPROM.put(eepromTermAddr, 0);
    Serial.print(F("eeprom_Load() exit freq ")); Serial.print(val.freq);
    Serial.print(F(", eepromTermAddr = ")); Serial.println(eepromTermAddr);
    eeprom_Print();
  }
}

//--------------------------------------------------------------------------------------------------------/

void eeprom_Print()
{
  int eeAddress = 0;

  Serial.println(F("-------------------"));
  Serial.println(F("Freq, L_relays, C_Relays, outputZ, Address"));
  EEPROM.get(eeAddress, val.freq);
  while (val.freq) {
    eeAddress += sizeof(unsigned int);  // Step off freq to L
    EEPROM.get(eeAddress, val.L);
    eeAddress += sizeof(byte);          // Step off L to C
    EEPROM.get(eeAddress, val.C);
    eeAddress += sizeof(byte);          // Step off C to Z
    EEPROM.get(eeAddress, val.Z);
    eeAddress += sizeof(byte);

    Serial.print(val.freq);
    Serial.print(F("\t")); Serial.print(val.L);
    Serial.print(F("\t  ")); Serial.print(val.C);
    Serial.print(F("\t   ")); Serial.print(val.Z);
    Serial.print(F("\t    ")); Serial.println(eeAddress - sizeof(MyValues));

    EEPROM.get(eeAddress, val.freq); //Get next frequency or 0000 if at end
  }
  Serial.print(val.freq);
  Serial.print(F("  eepromTermAddr = ")); Serial.println(eepromTermAddr);
  Serial.println(F("-------------------")); Serial.println();
}

//--------------------------------------------------------------------------------------------------------/


void pollSerial()
{
  // Uses the following globals ...
  // String rx_str = "";
  // boolean not_number = false;
  
  unsigned int freq;
  char rx_byte = 0;
  
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character

    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      rx_str += rx_byte;
    }
    else if (rx_byte == '\n') {
      // end of string
      if (not_number) {
        Serial.println("Invalid Frequency");
      }
      else {
        freq = rx_str.toInt();
        // print the result
        Serial.print(F("freq = "));
        Serial.print(rx_str);
        Serial.print(F(", L_Relay = "));
        Serial.print(_status.L_relays);
        Serial.print(F(", C_Relay = "));
        Serial.print(_status.C_relays);
        Serial.print(F(", outputZ = "));
        Serial.println(_status.outputZ);
        eeprom_Load(freq);
        Serial.println(F("Enter a frequency to add to eeprom"));
      }
      not_number = false;         // reset flag
      rx_str = "";                // clear the string for reuse
    }
    else {
      // non-number character received
      not_number = true;    // flag a non-number
    }
  } // end: if (Serial.available() > 0)
}

//--------------------------------------------------------------------------------------------------------/

void eeprom_initialise()
{
  // this routine loads some preset values into eeprom for fast tuning. In the final version this will become
  // redundant as the values will be loaded by frequency with the counter installation. A magic number is
  // loaded into the last byte of the eeprom to indicate that it is already loaded with tune values and we
  // don't duplicate the data at each switch on.
/*
  struct MyValues {
    unsigned int freq;
    byte L;
    byte C;
    byte Z;
  } val;
*/
  int eeAddress = 0;

  if (EEPROM[EEPROM.length() - 1] != 120) {
    val.freq = 3535; val.L = B00111001; val.C = B01010100; val.Z = loZ;   // 490u, 410p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);//Move address to the next struct item
    val.freq = 3605; val.L = B00111000; val.C = B10011100; val.Z = loZ;   // 194u, 573p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 3660; val.L = B00001001; val.C = B11010101; val.Z = loZ;   // 194u, 765p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 3770; val.L = B00000011; val.C = B10011101; val.Z = hiZ;   // 108u, 1299p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 3800; val.L = B00111111; val.C = B01001110; val.Z = hiZ;   // 96u, 1092p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 3900; val.L = B01001010; val.C = B00011101; val.Z = hiZ;   // 58u, 1299p
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 7010; val.L = B00000001; val.C = B00000000; val.Z = hiZ;   //Values for 7.020 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 7060; val.L = B00000010; val.C = B00100000; val.Z = loZ;   //Values for 7.060 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 7200; val.L = B00001110; val.C = B00111111; val.Z = hiZ;   //Values for 7.100 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 10120; val.L = B0010100; val.C = B00110001; val.Z = hiZ;   //Values for 7.160 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 14050; val.L = B0001101; val.C = B00010000; val.Z = hiZ;   //Values for 7.200 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 14200; val.L = B0001110; val.C = B00001010; val.Z = hiZ;  //Values for 10.120 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 18100; val.L = B00001010; val.C = B00000011; val.Z = hiZ;  //Values for 10.140 MHz
    EEPROM.put(eeAddress, val);
    eeAddress += sizeof(MyValues);
    val.freq = 0; val.L = 0; val.C = 0; val.Z = loZ;      //Zero values for a terminator
    EEPROM.put(eeAddress, val);
    EEPROM[EEPROM.length() - 1] = 120; //Put a marker to show that data has been loaded into the eeprom
    Serial.println(F("EEPROM initialised"));
  } else {
    Serial.println(F("EEPROM was already initialised"));
  }
}

//----------------------------------------------------------------------------------------------------------
/* Here I pre-load some settings for each band and see if swr is low enough to indicate a
   suitable starting point for a tune.
*/
void tryPresets()
{
  float returnLoss;
  unsigned int frequency;
  byte Crelays;
  byte Lrelays;
  boolean loadZ;
  int eeAddress = 0;

  returnLoss = 0.001; //Force _status to be copied to statusTemp first time through the while loop
  EEPROM.get(eeAddress, _status.freq);
  while (_status.freq) {
    eeAddress += sizeof(unsigned int);  // Step off freq to L
    EEPROM.get(eeAddress, _status.L_relays);
    eeAddress += sizeof(byte);          // Step off L to C
    EEPROM.get(eeAddress, _status.C_relays);
    eeAddress += sizeof(byte);          // Step off C to Z
    EEPROM.get(eeAddress, _status.outputZ);
    eeAddress += sizeof(byte);
    setRelays();
    delay(Relay_Settle_Millis); //Add 20 mSec of settling time before taking the SWR reading
    getSWR();
    Serial.print(F("Freq, L, C, Vf, Vr,  Z, RL, SWR = "));
    Serial.print(_status.freq);
    Serial.print(F(",  ")); Serial.print(_status.L_relays);
    Serial.print(F(",  ")); Serial.print(_status.C_relays);
    Serial.print(F(",  ")); Serial.print(_status.fwd);
    Serial.print(F(",  ")); Serial.print(_status.rev);
    Serial.print(F(",  ")); Serial.print(_status.outputZ);
    Serial.print(F(",  ")); Serial.print(_status.retLoss, 5);
    Serial.print(F(",  ")); Serial.println((pow(10, (_status.retLoss / 20)) + 1) / (pow(10, (_status.retLoss / 20)) - 1));
    if (returnLoss < _status.retLoss) {
      returnLoss = _status.retLoss;
      frequency = _status.freq;
      Crelays = _status.C_relays;
      Lrelays = _status.L_relays;
      loadZ = _status.outputZ;
    }
    EEPROM.get(eeAddress, _status.freq); //Get next frequency or 0000 if at end
  }
  Serial.print(F("@void tryPresets() .. Best returnLoss = "));
  Serial.println(returnLoss, 5);
  Serial.println(F("--------------------"));  // TODO remove this debug info on final version
  _status.freq = frequency;
  _status.C_relays = Crelays;
  _status.L_relays = Lrelays;
  _status.outputZ = loadZ;
  setRelays();
  delay(Relay_Settle_Millis); //Add 20 mSec of settling time before taking the SWR reading
  getSWR();
  Serial.println(F("--------------------"));  // TODO remove this debug info on final version
  Serial.print(F("Freq, L, C, Vf, Vr, Z, RL, SWR = "));
  Serial.print(_status.freq);
  Serial.print(F(",  ")); Serial.print(_status.L_relays);
  Serial.print(F(",  ")); Serial.print(_status.C_relays);
  Serial.print(F(",  ")); Serial.print(_status.fwd);
  Serial.print(F(",  ")); Serial.print(_status.rev);
  Serial.print(F(",  ")); Serial.print(_status.outputZ);
  Serial.print(F(",  ")); Serial.print(_status.retLoss, 5);
  Serial.print(F(",  ")); Serial.println((pow(10, (_status.retLoss / 20)) + 1) / (pow(10, (_status.retLoss / 20)) - 1));
  Serial.println();
}

//----------------------------------------------------------------------------------------------------------
boolean doRelayCoarseSteps()
{
  // This subroutine steps through the capacitor and inductor relays looking for the combination which gives
  // the best return loss. Only individual relays are stepped with no multiple C or L combinations so a fine
  // tune subroutine is later called to set exact values for L and C.

  // This procedure is carried out by initially setting capacitor relays to zero C and stepping the inductor
  // relays one by one from no relays to all 8. The capacitor relay is incremented and procedure repeated.
  // The SWR is read at each step into an 2 dimensional array which is later parsed for the best Return Loss
  // and the C and L combination to give this is set along with retLoss in the _status array.

  // A check of the command button is made and tune aborted with a return of true if a press detected. The
  // caller should perform the abort process.

  // Entry: The caller sets the C/O relay to HiZ or LoZ as required
  // Exit with relay settings giving best return Loss for the C/O relay setting on entry.
  // Result held in _status struct.

  float values[numLrelays][numCrelays];
  float bestRetLoss = 0;
  uint8_t bestC = 0;
  uint8_t bestL = 0;
  char pntBuffer[16];


  // Step through states of no relays operated to all relays operated
  getSWR();
  for (byte c = 0; c < numCrelays; c++) {
    _status.C_relays = 0;
    if (c != 0) {
      bitSet(_status.C_relays, c - 1);
    }
    for (byte x = 0; x < numLrelays; x++) {
      _status.L_relays = 0;
      if (x != 0) {
        bitSet(_status.L_relays, x - 1);
      }
      setRelays();
      // Check at this point for a command button press and abandon tune if so.
      if (handle_button()) { // Any button change triggers abort tune.
        return true;
      }
      getSWR();
      values[c][x] = _status.retLoss;
    } // end inner for loop
  } //end outer for loop

  // We parse the array looking for the combination of L and C relays which give lowest SWR
  for (byte c = 0; c < numCrelays; c++) {
    for (byte l = 0; l < numLrelays; l++) {
      if (values[c][l] > bestRetLoss) {
        bestRetLoss = values[c][l];
        bestC = c;
        bestL = l;
      }
    }
  }

  // Now set the relays to give best coarse tune based on bestSWR
  _status.C_relays = 0; // No bits set for no relays operated
  _status.L_relays = 0;
  if (bestC > 0) bitSet(_status.C_relays, bestC - 1); // Set bits 0 .. 7 here (Relays 1 to 8)
  if (bestL > 0) bitSet(_status.L_relays, bestL - 1);
  setRelays();
  delay(Relay_Settle_Millis); // Extra settling time for an ultra stable final reading
  getSWR();  //Get SWR with relays at final state.

#ifdef DEBUG_COARSE_TUNE_STATUS // Print the DEBUG header
  Serial.println(F("doRelayCoarseSteps(): Caps are connected to "));
  if (_status.outputZ == hiZ) Serial.print(F("Output (HiZ)"));
  else Serial.println(F("Input (LoZ)"));
  Serial.print(F("Cap\t   "));
  for (int x = 0; x < numLrelays; x++) {
    Serial.print(F("  L"));
    Serial.print(x);
    Serial.print(F("       "));
  }
  Serial.println();
  // Now print the DEBUG body which is the debug data captured in the array
  for (byte c = 0; c < numCrelays; c++) {
    Serial.print(F("C"));
    Serial.print(c);
    Serial.print("\t");
    for (byte x = 0; x < numLrelays; x++) {
      dtostrf(values[c][x], 11, 5, pntBuffer);  // 11 is mininum width, 5 is decimal places;
      Serial.print(pntBuffer);
    }
    Serial.println();
  }
  Serial.print(F("_status.C_relays = ")); Serial.print(_status.C_relays);
  Serial.print(F(", _status.L_relays = ")); Serial.println(_status.L_relays);
  Serial.print(F("; bestC = "));
  Serial.print(bestC);
  Serial.print(F("; bestL = "));
  Serial.print(bestL);
  Serial.print(F("; Return Loss = "));
  dtostrf(_status.retLoss, 11, 5, pntBuffer);  // 11 is mininum width, 5 is decimal places;
  //  Serial.println(pntBuffer);
  Serial.println(bestRetLoss, 8);
  Serial.println();
#endif //DEBUG_COARSE_TUNE_STATUS
  return false;
} //end subroutine

//----------------------------------------------------------------------------------------------------------
void getSWR()
{
  // Entry: The caller will set the L and C relays and the Hi Lo relay. This subroutine will not be called if
  // there is insufficient RF for a valid reading.

  // Exit: _status.Crelays, _status.Lrelays contain the relays position, _status.retLoss contains the
  // return loss reading taken. The 2 step amplifier from the swr bridge will have the gain set appropriately.

  // Reflection coefficient (rho) = Vref / Vfwd
  // Return Loss = -20 * log10(rho)
  // In terms of rho, VSWR = (1 + rho) / (1 - rho)
  // In terms of Return Loss, VSWR = (10^(RL÷20)+1) ÷ (10^(RL÷20)−1)

  //       All globals are prefixed with an underscore e.g. _status.fwd

  boolean attenuatorFlag = false;
  float rho;

  // When looking at an SSB or AM signal we are wanting a voice peak to calculate the SWR. By taking multiple
  // reads of fwd and rev voltages we stand a good chance of striking a peak.
  if (analogRead(forward) > TX_LEVEL_THRESHOLD) { // Only do this if enough TX power
    _status.tx_on = true;
    readSWR();
    // Now check to see if we need to change the SWR attenuator and flag if so
    if ((_status.fwd < 300) && (_status.ampGain == lo)) {
      digitalWrite(swrGain, LOW);     // Set swr amplifiers to highest gain
      _status.ampGain = hi;
      attenuatorFlag = true;
    }
    else if (_status.fwd == 1023) {
      digitalWrite(swrGain, HIGH);  // Set to lowest gain for amps.
      _status.ampGain = lo;
      attenuatorFlag = true;
    }

    if (attenuatorFlag) { // We altered the attenuator so have to read the fwd and rev again.
      readSWR();
    }
    if (_status.rev == 0) {
      _status.rev = 0.5;
      _status.fwd += 0.5;
    }
    if (_status.fwd <= _status.rev) _status.fwd = (_status.rev + 1); //Avoid division by zero or negative.

    rho = _status.rev / _status.fwd;
    _status.retLoss = -20 * log10(rho);
  } else {
    _status.tx_on = false;
  }

#ifdef DEBUG_status
  Serial.print(F("getSWR: fwd, rev, retLoss, SWR = "));
  Serial.print(_status.fwd);
  Serial.print(F(", "));
  Serial.print(_status.rev);
  Serial.print(F(", "));
  Serial.print(_status.retLoss, 5);
  Serial.print(F(", "));
  Serial.println((pow(10, (_status.retLoss / 20)) + 1) / (pow(10, (_status.retLoss / 20)) - 1));
#endif
}

//----------------------------------------------------------------------------------------------------------
void readSWR()
{
  // ToDo: write code to check for a value which does not match the others and drop it and get another.
  float Vf[SWR_AVERAGE_COUNT]; // Currently set to 8
  float Vr[SWR_AVERAGE_COUNT];
  float VfHi = 0;
  float VrHi = 0;
  float VfLo = 1023;
  float VrLo = 1023;

  // Fill the arrays with forward and reverse values
  //------------------------------------------------
  for (byte x = 0; x < SWR_AVERAGE_COUNT; x++) {
    Vf[x] = analogRead(forward);
    Vr[x] = analogRead(reverse);
    delayMicroseconds(50);
  }

  if (preset) { // TODO remove this debug info on final version
    for (byte x = 0; x < SWR_AVERAGE_COUNT; x++) {
      Serial.print(Vf[x]); Serial.print("\t");
    }
    Serial.println();
    for (byte x = 0; x < SWR_AVERAGE_COUNT; x++) {
      Serial.print(Vr[x]); Serial.print("\t");
    }
    Serial.println();
  }

  // Test the array for values that deviate too far
  // ----------------------------------------------
  for (byte x = 0; x < SWR_AVERAGE_COUNT; x++) {
    if (Vf[x] > VfHi) VfHi = Vf[x];
    if (Vf[x] < VfLo) VfLo = Vf[x];
    if (Vr[x] > VrHi) VrHi = Vr[x];
    if (Vr[x] < VrLo) VrLo = Vr[x];
  }
  // At this point we are holding the highest and lowest values of both forward and reverse voltages that we read.
  /*
    if ((VfLo * 1.1 < VfHi) || (VrLo * 1.1 < VrHi)) {
      Serial.print(F("VfHi, VfLo, VrHi, VrLo = "));
      Serial.print(VfHi); Serial.print(F(", "));
      Serial.print(VfLo); Serial.print(F(", "));
      Serial.print(VrHi); Serial.print(F(", "));
      Serial.print(VrLo); Serial.println(F(", "));
    }
  */
  _status.fwd = 0;
  _status.rev = 0;
  // Replace all the array values which are more than 10% deviation with the high value
  // ----------------------------------------------------------------------------------
  for (byte x = 0; x < SWR_AVERAGE_COUNT; x++) {
    if ((VfHi * 0.95) > Vf[x]) Vf[x] = VfHi;
    if ((VrHi * 0.95) > Vr[x]) Vr[x] = VrHi;
    _status.fwd = _status.fwd + Vf[x];
    _status.rev = _status.rev + Vr[x];
  }

  _status.fwd = _status.fwd / SWR_AVERAGE_COUNT;
  _status.rev = _status.rev / SWR_AVERAGE_COUNT;

}

//----------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------

byte handle_button()
{
  // Returns  0 = No button pressed or no change of button state occurred
  //          1 = Leading edge of button press
  //          2 = Trailing edge of short button press (< 200 mSec)
  //          3 = Trailing edge of medium button press (200 to 1000 mSec)
  //          4 = Trailing edge of long button press (> 1000 mSec)

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

//----------------------------------------------------------------------------------------------------------

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
#endif // DEBUG_BUTTON_ARRAY  
  }
}

//----------------------------------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------------------------------

void lcdPrintStatus()
{
  // TODO There is a problem when running this in that if there is no transmit the swr and fwd and
  //       rev voltages are lost. Probably a _statusHistory struct needs to be created to preserve
  //       the last RF reading taken and the lcdPrintStatus reads data from that.


  float freq = 14.02345;
  float swr;
  char charVal[16];          //buffer, temporarily holds data from values

  lcd.home();
  // Convert return loss to swr
  swr = (pow(10, (_status.retLoss / 20)) + 1) / (pow(10, (_status.retLoss / 20)) - 1);
  dtostrf(swr, 7, 3, charVal);  //7 is mininum width, 3 is precision;
  //  float value is copied onto buffer
  for (int i = 0; i < 7; i++)
  {
    lcd.write(charVal[i]);
  }

  // Temporary replace frequency with reverse voltage

  pad_utoa(_status.rev, charVal, 5);
  lcd.print(charVal);
  lcd.print("  ");

  if (_status.outputZ == hiZ) {
    lcd.print(" H");
  }
  else lcd.print(" L");

  lcd.setCursor(0, 1); // Switch to start of line 2

  pad_utoa(_status.totL, charVal, 4);
  strcat(charVal, "u");
  lcd.print(charVal);

  pad_utoa(_status.totC, charVal, 5);
  strcat(charVal, "p");
  lcd.print(charVal);

  pad_utoa(_status.fwd, charVal, 5);
  lcd.print(charVal);
}

//----------------------------------------------------------------------------------------------------------

void pad_utoa(unsigned long value, char *myBuffer, int places)
{
  uint8_t cnt;
  uint8_t bufLen = 16;
  int stringLength;
  int x;

  //  itoa(value, myBuffer, 10); // TODO Write a utoa() to shorten the code see getDecStr() below!!
  //  getDecStr (myBuffer, 16, value);

  for (cnt = 2; cnt <= bufLen; cnt++)
  {
    /*
        Serial.print("cnt = ");
        Serial.print(cnt);
        Serial.print(", bufLen - cnt = ");
        Serial.println(bufLen - cnt);
    */
    myBuffer[bufLen - cnt] = (uint8_t) ((value % 10UL) + '0');
    value /= 10;
  }
  /*
    Serial.print("cnt - 2 = ");
    Serial.println(cnt-2);
  */
  myBuffer[cnt - 2] = '\0';
  /*
    Serial.println("                                 0123456789012345");
    Serial.print("Initial value of myBuffer is ... ");
    Serial.println(myBuffer);
  */
  // Convert leading zeros to spaces
  cnt = 0;
  while (myBuffer[cnt] == '0') {
    myBuffer[cnt] = ' ';
    cnt++;
  }

  // add a zero if we have an all spaces string
  if (cnt == (bufLen - 1)) {
    cnt--;
    myBuffer[cnt] = '0'; // At this stage, cnt points to the first alpha numeric.
  }
  /*
    Serial.print("    The value of myBuffer is ... ");
    Serial.println(myBuffer);
    Serial.println(cnt);
    Serial.println(bufLen-1);
    Serial.println();
  */
  stringLength = bufLen - cnt; //StringLength includes the '\0' terminator
  /*
    Serial.print("stringLength = ");
    Serial.println(stringLength);
    Serial.print("places + 1 - stringLength = ");
    Serial.println(places+1 - stringLength);
  */
  if (places + 1 - stringLength <= 0) {
    x = 0;
    places = 15;
  } else {
    x = places + 1 - stringLength;
  }
  //  Serial.println("0123456789012345");
  for (x; x <= places; x++) {
    myBuffer[x] = myBuffer[cnt];
    /*
        Serial.print(myBuffer);
        Serial.print(",  ");
        Serial.print(x);
        Serial.print(",  ");
        Serial.println(cnt);
    */
    cnt++;
  }
  //  Serial.println(myBuffer);
}

//----------------------------------------------------------------------------------------------------------

#ifdef PRINT_STATUS
void printStatus(boolean doHeader)
{

  char charVal[16];

  if (doHeader) {
    Serial.println(F("C_relays   L_relays   totC  totL  fwdVolt  revVolt  Gain  outZ    rawSWR  SWR"));
  } else {
    print_binary(_status.C_relays, 8);
    Serial.print(F("  "));
    print_binary(_status.L_relays, 8);

    pad_utoa(_status.totC, charVal, 6);
    Serial.print(charVal);

    pad_utoa(_status.totL, charVal, 6);
    Serial.print(charVal);

    pad_utoa(_status.fwd, charVal, 9);
    Serial.print(charVal);

    pad_utoa(_status.rev, charVal, 9);
    Serial.print(charVal);

    if (_status.ampGain == hi) Serial.print(F("  High"));
    else Serial.print(F("   Low"));

    if (_status.outputZ == hiZ) Serial.print(F("   HiZ"));
    else Serial.print(F("   LoZ"));

    pad_utoa(_status.retLoss, charVal, 10);
    Serial.print(charVal);

    // NOTE: sprintf doesn't support floats
    Serial.print(F("  "));
    Serial.println(_status.retLoss, 4);
  }
}
#endif

//----------------------------------------------------------------------------------------------------------

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
//----------------------------------------------------------------------------------------------------------

void lcdPrintSplash()
{
  lcd.home();                   // go home
  lcd.print(F("AutoTune v2.0.0 "));
  lcd.setCursor (0, 1);        // go to the next line
  lcd.print(F("ZL2APV (c) 2015 "));
  //  lcd.backlight(); // finish with backlight on
  //  delay ( 5000 );
}
//----------------------------------------------------------------------------------------------------------

void lcdPrintBargraph(boolean range)
{
  displayAnalog(0, 0, _status.fwd);
  displayAnalog(0, 1, _status.rev);
  if (range) {
    lcd.setCursor (lcdNumCols - 2, lcdNumRows - 1);    // go to 2nd to last chr of last line.
    if (_status.ampGain == hi) {  // If amp gain is high, we are in low power mode
      lcd.print(F("Lo"));
    } else {
      lcd.print(F("Hi"));
    }
  }
}

//----------------------------------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------------------------------
