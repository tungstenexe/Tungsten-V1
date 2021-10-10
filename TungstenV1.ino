/********************************************************************************************************
* Nerf RapidStrike - Tungsten V1
*
* Description
* program for Nerf Rapidstrike with single shot, burst mode and full auto. 
* Adjustable ROF, ammo capacity, burst per shot. Ammo counter.
*
* created  05 Nov 2016
* modified 12 Jan 2017
* by TungstenEXE
*
* If you find my code useful, do support me by subscribing my YouTube Channel, thanks.
*
* My YouTube Channel Link - Nerf related
* https://www.youtube.com/channel/UCDMDwz4Hf8E0R0khyh40SxQ
* 
* Board used     - Arduino Nano
* Pusher Motor   - MTB Honey Badger Motor 
* FlyWheel Motor - MTB Rhino Motor
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Bounce2.h>

#define PIN_DARTCOUNT         2    // PIN listening to dart count

#define PIN_PWM_ROFSELECTED   3    // PIN issuing signal to control voltage to pusher
#define PIN_ROFSELECT         4    // PIN listening to change of ROF press event

#define PIN_DISPLAY_DATA      5    // PIN for 7 segment LED display
#define PIN_DISPLAY_CLOCK     6    // PIN for 7 segment LED display
#define PIN_DISPLAY_LATCH     7    // PIN for 7 segment LED display

#define PIN_REV               8    // PIN listening to rev press event
#define PIN_DARTTRIGGER       9    // PIN listening to trigger pull event
#define PIN_DARTRESET         10   // PIN listening to reset counter event
#define PIN_MODEFIRE          11   // PIN listening to change in mode of fire event
#define PIN_LIMITSELECT       12   // PIN listening to entering into setting limit event for  
                                   // mag cap and burst count

#define PIN_REV_MOSFET        A0   // PIN issuing signal to on/off mosfet to rev flywheels 
#define PIN_DARTTRIGGER_RELAY A1   // PIN issuing signal to on/off relay to pusher motor 

#define PIN_LIMITSELECT_A     A2   // PIN listening to change of state of rotary encoder
#define PIN_LIMITSELECT_B     A3   // PIN listening to change of state of rotary encoder

#define PIN_VOLTREAD          A4   // PIN to receive voltage reading from battery 

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Of PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AMMO_UPPER_LIMIT      35   // Maxmimum ammo configurable
#define AMMO_LOWER_LIMIT      6    // Minimum ammo configurable

#define BURST_UPPER_LIMIT     9    // Maxmimum burst configurable
#define BURST_LOWER_LIMIT     2    // Minimum burst configurable

#define SETTING_LIMIT_OFF     0    // Integer constant to indicate setting of limit is off
#define SETTING_LIMIT_AMMO    1    // Integer constant to indicate setting of magazine capacity is on
#define SETTING_LIMIT_BURST   2    // Integer constant to indicate setting of burst limit is on

#define MODE_SINGLE           0    // Integer constant to indicate firing single shot
#define MODE_BURST            1    // Integer constant to indicate firing burst
#define MODE_AUTO             2    // Integer constant to indicate firing full auto

#define MODE_ROF_HIGH         0    // Integer constant to indicate highest rate of fire 
#define MODE_ROF_STANDARD     1    // Integer constant to indicate standard rate of fire 
#define MODE_ROF_LOW          2    // Integer constant to indicate lowest rate of fire 

#define PWM_SIGNAL_MAX        240  // Highest PWM signal possible for PIN_PWM_ROFSELECTED
#define VOLTAGE_MAX           10.5 // Maximum voltage for highest rate of fire
#define BATTERY_MIN           11   // Minimum voltage of battery for rev to operate

// For 7 Segment LED display
byte num_digits []        = {0b11000000,0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10010000}; // digit 0 to 9
byte mode_letter[]        = {0b10010010,0b10000011,0b10001000}; // letter: S, b, A
byte rof_letter []        = {0b10001001,0b10010010,0b11000111}; // letter: H, S, L
byte dot                  = 0b10000000;                         // dot
byte limit_letter [][2]   = {{0b11111111, 0b11111111}, {0b10001000, 0b10110110}, {0b10000011, 0b10110110}}; // letters: Blank Blank, A 3dashes, b 3dashes
byte lowBattery_letter [] = {0b11000111,0b11000000,0b10110110,0b10000011}; // letter: LO-b

int voltageLimt         = PWM_SIGNAL_MAX;                
int rofLimitArrSize     = 3;                 
int rofLimitArr []      = {100, 40, 25};     // number are in percentage, 100%, 40%, 25%
int modeROFSetted       = MODE_ROF_STANDARD; // select index position 1, which is 40%
int modeROFSelected     = MODE_ROF_STANDARD; // select index position 1, which is 40%

int settingLimit        = SETTING_LIMIT_OFF; // Turn Off setting of limits
int ammoLimit           = 18;                // default set as 18 dart mag
int burstLimit          = 3;                 // default set as 3 darts per burst

int limitSelect_stateA  = HIGH;              // track the pin A status of rotary encoder
int limitSelect_stateB  = HIGH;              // track the pin B status of rotary encoder

int modeFire            = MODE_SINGLE;       // track the mode of fire, Single, Burst or Auto, Single by default
int dartToBeFire        = 0;                 // track amount of dart(s) to fire when trigger pulled, 0 by default
int dartLeft            = ammoLimit;         // track amount of dart in mag, same default value as of ammoLimit

boolean batteryLow      = false;             // track battery status

// Declare and Instantiate Bounce objects
Bounce btnTrigger       = Bounce(); 
Bounce btnDartCount     = Bounce(); 
Bounce btnModeFire      = Bounce();
Bounce btnDartReset     = Bounce();
Bounce btnRev           = Bounce();
Bounce btnRofSelect     = Bounce();
Bounce btnLimitSelect   = Bounce();

void setup() { // initilze
  Serial.begin(9600);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // INPUT PINs setup
  // Note: Input pins will be using internal pull-up resistor. A fall in signal indicate button pressed.
  //       Except PIN_LIMITSELECT_A, PIN_LIMITSELECT_B, PIN_VOLTREAD not using pull-up resistor.
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  
  pinMode(PIN_DARTCOUNT, INPUT_PULLUP);   // PULLUP
  btnDartCount.attach(PIN_DARTCOUNT);
  btnDartCount.interval(5);

  pinMode(PIN_REV,INPUT_PULLUP);          // PULLUP
  btnRev.attach(PIN_REV);
  btnRev.interval(5);

  pinMode(PIN_DARTTRIGGER,INPUT_PULLUP);  // PULLUP
  btnTrigger.attach(PIN_DARTTRIGGER);
  btnTrigger.interval(5);

  pinMode(PIN_MODEFIRE,INPUT_PULLUP);     // PULLUP
  btnModeFire.attach(PIN_MODEFIRE);
  btnModeFire.interval(5);

  pinMode(PIN_DARTRESET, INPUT_PULLUP);   // PULLUP
  btnDartReset.attach(PIN_DARTRESET);
  btnDartReset.interval(5);

  pinMode(PIN_ROFSELECT,INPUT_PULLUP);    // PULLUP
  btnRofSelect.attach(PIN_ROFSELECT);
  btnRofSelect.interval(5);

  pinMode(PIN_LIMITSELECT, INPUT_PULLUP); // PULLUP
  btnLimitSelect.attach(PIN_LIMITSELECT);
  btnLimitSelect.interval(5);  

  pinMode(PIN_LIMITSELECT_A, INPUT);      // Not using PULLUP
  pinMode(PIN_LIMITSELECT_B, INPUT);      // Not using PULLUP

  pinMode(PIN_VOLTREAD, INPUT);           // Not using PULLUP analog read 0 to 1023

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // OUTPUT PINs setup
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(PIN_DISPLAY_DATA , OUTPUT);
  pinMode(PIN_DISPLAY_CLOCK, OUTPUT);
  pinMode(PIN_DISPLAY_LATCH, OUTPUT);

  digitalWrite(PIN_REV_MOSFET,LOW);  
  pinMode(PIN_REV_MOSFET, OUTPUT);

  digitalWrite(PIN_DARTTRIGGER_RELAY,LOW);
  pinMode(PIN_DARTTRIGGER_RELAY, OUTPUT);

  pinMode(PIN_PWM_ROFSELECTED, OUTPUT);
  setVoltageLimit();
  analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[modeROFSetted], voltageLimt));

  updateSettingDisplay();
}

void loop() { // Main Loop
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update all buttons
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  btnLimitSelect.update();
  btnDartCount.update();
  btnRev.update();
  btnTrigger.update();
  btnModeFire.update();
  btnDartReset.update();
  btnRofSelect.update();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Ammo/Burst limit setting button
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  if (btnLimitSelect.fell() && !isFiring() && !isRev()) {
    settingLimit = ++settingLimit % 3;
    if (settingLimit != SETTING_LIMIT_OFF) {
      updateLimitDisplay();
    }    
  }

  if (settingLimit != SETTING_LIMIT_OFF) { // setting either ammo or burst
    settingLimitHandle();
  } else {                                 // check for other events        
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Trigger Pull/Release
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnTrigger.fell() && isRev()) {    // pull
      triggerPressedHandle();
    } else if (btnTrigger.rose()) {        // released
      triggerReleasedHandle();
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Dart Count switch
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnDartCount.rose()) {        // counter switch released, pusher is extending towards dart 
      shotFiredHandle();              // for firing
    } else if (btnDartCount.fell()) { // pusher is returning from firing dart
      shotFiredReturnHandle();
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Rev switch Press/Release
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnRev.fell()) {              // pressed
      setVoltageLimit();              // update voltage limit
      if (!batteryLow) {
        analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[modeROFSetted], voltageLimt));
        if (digitalRead(PIN_REV_MOSFET) == LOW){
          digitalWrite(PIN_REV_MOSFET,HIGH);
        }
      }
    } else if (btnRev.rose()) {       // released
      if (!isFiring()) {
        // Even when the rev button was released,
        // only stop revving if no dart remains to be fired
        digitalWrite(PIN_REV_MOSFET,LOW);
      }
    } else if (isRev() && !isFiring() && (btnRev.read() == HIGH) ) {
      // Rev button was released earlier before firing stopped, hence
      // motors are still revving now when firing had stopped with the
      // rev button already released.
      // Stop revving 
      digitalWrite(PIN_REV_MOSFET,LOW);
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Firing Mode change: Single Shot, Burst, Full Auto
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnModeFire.fell()) { // pressed
      modeFire = ++modeFire % 3; // next mode
      modeROFSetted = (modeFire == MODE_SINGLE) ? MODE_ROF_STANDARD : modeROFSelected;
      // For Single Shot mode, the ROF will be setted to Standard only
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Rate of Firing change: Low, Standard or High
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnRofSelect.fell()) { // pressed
      if (modeFire != MODE_SINGLE) {
        // ROF can only be updated for Non Single Shot mode
        modeROFSelected = ++modeROFSelected % rofLimitArrSize;
        modeROFSetted   = modeROFSelected;
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Reset Ammo Count
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnDartReset.fell()) { // pressed
      dartLeft = ammoLimit;      
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // 7-segment LED display
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!batteryLow) { 
      updateSettingDisplay();    // Update setting display for 7-segment LED
    } else {
      updateBatteryLowDisplay(); // Update battery low display for 7-segment LED
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredHandle() {
  if (dartLeft > 0) {
    dartLeft--;     // decrease dart count
    dartToBeFire--;

    //-------------------------------------------------------------------------------------------------------
    if ((dartToBeFire == 1) && (modeROFSetted != MODE_ROF_LOW)) { // reduce ROF for trigger return
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_LOW], voltageLimt));
    }
    //-------------------------------------------------------------------------------------------------------
    if (dartLeft <= 0 || dartToBeFire <= 0) {   // time to stop the pusher motor
      digitalWrite(PIN_DARTTRIGGER_RELAY,LOW);  // release relay switch of pusher motor
      dartToBeFire = 0;
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[modeROFSetted], voltageLimt)); 
      // reset ROF selected
    }
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredReturnHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredReturnHandle() {
    if ((dartToBeFire == 3) && (modeROFSetted == MODE_ROF_HIGH)) {       
      // start reducing ROF for HIGH ROF firing when pusher is returning from firing the 3rd last shot
      // pusher about to fire off the 2nd last shot
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_STANDARD], voltageLimt));
    } else if ((dartToBeFire == 2) && (modeROFSetted != MODE_ROF_LOW)) { 
      // needs to start reducing ROF for trigger returning from firing the 2nd last shot
      // pusher about to fire off the last shot
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_LOW], voltageLimt));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerPressedHandle() {
  if (dartLeft > 0){
    switch(modeFire) {
      case MODE_SINGLE: dartToBeFire++; break;
      case MODE_BURST : dartToBeFire += burstLimit; 
          if (dartToBeFire > dartLeft) {
            dartToBeFire = dartLeft;
          }
        break;
      case MODE_AUTO  : dartToBeFire = dartLeft;
    }

    if (modeROFSetted == MODE_ROF_LOW && dartToBeFire == 1) {
      // This is a situation when you are in either BURST/AUTO mode and only left One dart in the
      // mag to be fired. If the current ROF mode is LOW, it will be better to fire off this last
      // dart with STANDARD ROF mode for better trigger return
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_STANDARD], voltageLimt));  
    }
  
    if (digitalRead(PIN_DARTTRIGGER_RELAY) == LOW){
      digitalWrite(PIN_DARTTRIGGER_RELAY,HIGH);
    }    
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerReleasedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerReleasedHandle() {  
  if (modeFire == MODE_AUTO && isFiring()) {
    if (modeROFSetted == MODE_ROF_HIGH) { // reduce ROF for trigger return for HIGH ROF
      // If ROF is HIGH and there are 3 or less dart left in Mag, it is better to empty the 
      // mag for smoother trigger return
      if (dartToBeFire > 3) {
        analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_STANDARD], voltageLimt));
        dartToBeFire = 3; // fire off 2 or 3 depending of where the pursher is
                          // while starting to reduce ROF for smoother trigger return 
      }      
    } else {  
      analogWrite(PIN_PWM_ROFSELECTED, percentToValue(rofLimitArr[MODE_ROF_LOW], voltageLimt));
      dartToBeFire = 1;    // fire off 1 last shot
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: isFiring
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean isFiring() {
  return (digitalRead(PIN_DARTTRIGGER_RELAY)==HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: isRev
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean isRev() {
  return (digitalRead(PIN_REV_MOSFET)==HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: settingLimitHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void settingLimitHandle() {
  int updateValue = 0;
  switch (settingLimit) {
    case SETTING_LIMIT_AMMO:
      updateValue = getLimitUpdateValue(ammoLimit, AMMO_UPPER_LIMIT, AMMO_LOWER_LIMIT);
      if (updateValue != 0) {
        ammoLimit += updateValue;
        dartLeft = ammoLimit;
      }
    break;
    case SETTING_LIMIT_BURST:
      updateValue = getLimitUpdateValue(burstLimit, BURST_UPPER_LIMIT, BURST_LOWER_LIMIT);
      if (updateValue != 0) {
        burstLimit += updateValue;
      }
    break;  
  }
  updateLimitDisplay();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: getLimitUpdateValue
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int getLimitUpdateValue(int currentLimit, int upperLimit, int lowerLimit) {
  int updateValue  = 0;

  int newLimitSelect_stateA = digitalRead(PIN_LIMITSELECT_A);
  int newLimitSelect_stateB = digitalRead(PIN_LIMITSELECT_B);
  
  if ((newLimitSelect_stateA != limitSelect_stateA) || (newLimitSelect_stateB != limitSelect_stateB)) {
    if (newLimitSelect_stateA == LOW && limitSelect_stateA == HIGH) { // state going from High to Low
      updateValue = (limitSelect_stateB * 2) - 1;                     // if B is high, direction is CW otherwise CCW
      if (((currentLimit == upperLimit) && (updateValue == 1)) || 
          ((currentLimit == lowerLimit) && (updateValue == -1))) {
        updateValue = 0;
      }
    }
  }
  limitSelect_stateA = newLimitSelect_stateA;
  limitSelect_stateB = newLimitSelect_stateB;
  
  return updateValue;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: percentToValue
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int percentToValue(int percent, int limit) {
  return (int)((percent / 100.0) * limit);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: setVoltageLimit()
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setVoltageLimit() {
  float currentVoltage = readVoltage();

  batteryLow = (currentVoltage < BATTERY_MIN);
  
  voltageLimt = (int) ((PWM_SIGNAL_MAX / currentVoltage) * VOLTAGE_MAX);
  voltageLimt = min(voltageLimt, PWM_SIGNAL_MAX); 
  // assigns voltageLimt to the smaller of voltageLimt or PWM_SIGNAL_MAX
  // ensuring that it never gets above PWM_SIGNAL_MAX (just in case)
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: readVoltage
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
float readVoltage() {
  int voltagePinValue = (int) (analogRead(PIN_VOLTREAD) / 0.4282);       
  return (voltagePinValue / 100.0); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateSettingDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSettingDisplay() {
    digitalWrite(PIN_DISPLAY_LATCH, LOW);
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, num_digits[dartLeft % 10]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, num_digits[dartLeft / 10]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, rof_letter[modeROFSetted]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, mode_letter[modeFire]); 
    digitalWrite(PIN_DISPLAY_LATCH, HIGH);    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateLimitDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateLimitDisplay() {
    digitalWrite(PIN_DISPLAY_LATCH, LOW);
    int numDisplay = 0;
    switch (settingLimit) {
      case SETTING_LIMIT_AMMO : numDisplay = ammoLimit;  break;
      case SETTING_LIMIT_BURST: numDisplay = burstLimit; break;
    }        
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, num_digits[numDisplay % 10]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, num_digits[numDisplay / 10]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, limit_letter[settingLimit][1]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, limit_letter[settingLimit][0]);     
    digitalWrite(PIN_DISPLAY_LATCH, HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateBatteryLowDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateBatteryLowDisplay() {
    digitalWrite(PIN_DISPLAY_LATCH, LOW);
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, lowBattery_letter[3]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, lowBattery_letter[2]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, lowBattery_letter[1]); 
    shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, MSBFIRST, lowBattery_letter[0]); 
    digitalWrite(PIN_DISPLAY_LATCH, HIGH);    
}
