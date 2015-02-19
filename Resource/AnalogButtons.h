#ifndef ANALOGBUTTONS_H
#define ANALOGBUTTONS_H

/*
  AnalogButtons,
  
  created 02 Jan 2009 V 0.1
 
 Connect more than one button to a single analog pin, 
 register a call-back function.which gets called when a button
 is pressed or held down for the defined number of seconds. Includes
 software key debouncing which may need to be adjusted, the the second 
 argument to AnalogButtons class. Define the ANALOG_PIN in the constructor
 of AnalogButtons.
 
 The circuit:

 * 5 buttons, 1 side of all buttons connected together to +5V. 
   The other side of each button is connected via a different value
   resister (tested with) 1k, 2k5, 5k8, 10k, 18k to one side of a
   100k resister which is in turn connected to GND. At the point
   where all the different resisters are joined you make a connection
   to your analog input. Basicly a different voltage divider is setup 
   depending upon which button is pressed. You have to configure the 
   Buttons Hi/Low values, see the comments in example code below and the
   AnalogButtons::configure(ANALOG_PIN) function.
   
   More or less than 5 buttons could be added, just pick different values
   of the resister sot hat all buttons have different values which arn't too
   close in size.
   
   I'm not sure what happens when Arduino is powered from batteries and Power V
   drops below V5.
 
 by Neil DUdman and everyone who's ever used Arduino
 
 */

#include <WProgram.h>

#define MAXBUTTONS 10

class Button 
{
public:  
  int id;
  int BUTTON_L;
  int BUTTON_H;     

  // for additional functions HOLD
  long duration;
  boolean isHeldDown;  
  
  Button();
  Button(int iid, int analogLowVal, int analogHighVal, int holdDuration = 1);

  // Override these fucntions if you want
  void pressed(){ } 

  // Override these fucntions if you want 
  void held(){}
};

class AnalogButtons 
{
private:  
  // for hold button, duration is specified for each button
  long previousMillis;
  
  // AnalogPin
  int pin;
  
  // Debouncng of normal pressing
  int debounce_count;
  long time;
  int counter;
  
  // Status of last press
  int lastButtonPressed;

  Button buttons[MAXBUTTONS];
  int buttonsIndex;
  
  // registered Callback function
  void (*pt2Function)(int, boolean);    
  
public:
  AnalogButtons(int ppin, int ddebounce_count = 100, void (*pt2Func)(int, boolean) = 0);
  
  int addButton(Button b);
  
  void checkButtons();
  
  // Press each button in turn and note the values returned to 
  // Serial monitor
  static void configure(int pin);
};

#endif