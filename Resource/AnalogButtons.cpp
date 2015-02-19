#include "AnalogButtons.h"

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



Button::Button() {
}

Button::Button(int iid, int analogLowVal, int analogHighVal, int holdDuration)
{
  id = iid;
  BUTTON_L = analogLowVal;
  BUTTON_H = analogHighVal;
  duration  = holdDuration * 1000;
}

AnalogButtons::AnalogButtons(int ppin, int ddebounce_count, void (*pt2Func)(int, boolean))
{
  pin = ppin;
  pt2Function = pt2Func;
  debounce_count = ddebounce_count;
  counter = 0;
}  

int AnalogButtons::addButton(Button b)
{
  if (buttonsIndex < MAXBUTTONS) {
    buttons[buttonsIndex] = b;
    buttonsIndex++;
  } 
  else return -1;
}

void AnalogButtons::checkButtons()
{
  if (millis() + time > 100) // don't sample analog more than 100ms
  {
    int val = analogRead(pin);   

    // So we can do some reseting if no buttons were pressed
    boolean foundOne = false; 

    for (int i = 0; i < buttonsIndex; i++) 
    {        
      if (val <= buttons[i].BUTTON_H + 4 && val >= buttons[i].BUTTON_L - 4 ) 
      {      
        foundOne = true;

        // First checking for button held down  
        if ( buttons[i].isHeldDown != true && lastButtonPressed == buttons[i].id && ((millis() - previousMillis) > buttons[i].duration) )
        {
          buttons[i].isHeldDown = true;
          buttons[i].held();
          (*pt2Function) (buttons[i].id, true);

          time = millis(); 
          return;            
        }   // Now if a different button has been pressed
        else if (lastButtonPressed !=  buttons[i].id ) {
          if ( lastButtonPressed != buttons[i].id )
          {
            counter ++; 
          }
          if (counter >= debounce_count)
          {
            lastButtonPressed = buttons[i].id;
            counter = 0;
            previousMillis = millis();  // for detecting held             
            buttons[i].pressed();
            Serial.println(lastButtonPressed);
            (*pt2Function) (buttons[i].id, false);

            time = millis(); 
            return;
          }             
        }
      } 
      else { // This button was not active so it can't any longer be held down
        buttons[i].isHeldDown = false;
      }
    }
    if ( !foundOne ) lastButtonPressed = -1;      
    // no button was detected they must have been released
    //lastButtonPressed = -1;
    time = millis();     
  }
}

// Press each button in turn and note the values returned to 
// Serial monitor
void AnalogButtons::configure(int pin)
{
  // read the analog input into a variable:
  int analogValue = analogRead(pin
    );
  // print the result:
  Serial.println(analogValue);

  // wait 10 milliseconds for the analog-to-digital converter
  // to settle after the last reading:
  delay(1000);    
}
