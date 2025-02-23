
/*===================================================================================================*/
/*PROGRAM INTRODUCTION
Motor speed control by PID controller. //Both channel A and B of the encoder are connected to
interrupt 0(pin3) and interrupt 1 (pin3) to calculate the pulses. The formula is as belowing:
processVariable = currentPosition * FEED_CONSTANT/NUMBER_PULSE_REVERLUTION. FEED_CONSTANT is 2 mmm.
The number of pulses for a revolution is 600x4 = 2400 (taking both rising and falling edges of both channels). 
Constants and parameters are denoted by capital letters. E.g.: ENCODER_PINA = 3. 
Nabeel Ahmad Khan - Mechatronics*/
/*===================================================================================================*/
/*CONSTANTS AND GLOBAL VARIABLE DECLARATION*/
#include <LiquidCrystal.h> // include the library code.

// initialize the library with the numbers of the interface pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_DB4 = 49;
const int LCD_DB5 = 47;
const int LCD_DB6 = 45;
const int LCD_DB7 = 43;


const int ENCODER_PINA = 3;//pin for encoder's channel A.
const int ENCODER_PINB = 2;//pin for encoder's channel B.
const int ENA = 7;//PWM to driver.
const int DIR_RIGHT = 36;//DIR_RIGHT=1,DIR_LEFT=0, CW rotation.
const int DIR_LEFT =38;//DIR_RIGHT=0,DIR_LEFT=1, CCW rotation.
const int LMS_LEFT = 40;//left limit switch.
const int LMS_RIGHT = 42;//right limit switch.

const int BUTTON_LEFT = 22;
const int BUTTON_RIGHT = 24;
const int BUTTON_CONFIRM = 26;//confirm mode change.
const int BUTTON_MODE = 28;//mode: FALSE = MAN, TRUE = AUTO.
const int SWITCH_HOME = 30;//home limit switch.
const int BUTTON_SET = 32;//press to confirm set point.
const int BUTTON_RES = 34;//reserve button.

const int LED_MAN = 25;//on at manual mode
const int LED_AUTO = 27;//on at auto mode.
const int LED_LIMIT = 29;//on at limits.
const int LED_HOME = 31;//on if homed.
const int LED_AUTOHOME = 33;//flashing if homing.

const double HIGH_LIMIT = 255.0;//high limit.
const double LOW_LIMIT = -255.0;//low limit.
const double KV = 1.0;

double floatKp = 0;
double floatKi = 0;
double floatKd = 0;
int knobSetpoint = 0;
int knobSpeed = 0;

/*variables used for PID*/
double setPoint = 50;//setting speed at manual mode.
double controlVariable = 0;
double processVariable = 0.0;//current angle
double currentError = 0;
double lmnIntegral = 0;
int manualValue = 50;
double Kp;//proportional gain.
double Ki;//intergral gain.
double Kd;//derivative gain.
double CYCLE = 0.05;//PID processing interval.
boolean manMode = false;
boolean autoMode = false;

//vars in Interrupt Subroutine need to be volatile.  
volatile long currentPosition = 0;//number of pulses at time of calculate
int buttonMode, buttonConfirm, buttonRight, buttonLeft, buttonSet, flashBit;
int lmsLeft, lmsRight, switchHome, homeSet, autoHome;
double readKp, readKi, readKd, readSetpoint, readSpeed;
double EMA_a = 0.6;
int EMA_S = 0;
unsigned long lastLCDTime = 0;
unsigned long lastFlashingTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastPIDTime = 0;//to store the real-time when previous execution is done.
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

/*===================================================================================================*/
void setup() {
  TCCR4B = (TCCR4B & 0xF8) | 0x01; //change PWM frequency to 31.374Kz (no noise)
  Serial.begin(9600);//serial speed 9600kb/s.   
  pinMode(BUTTON_MODE, INPUT);
  pinMode(BUTTON_RIGHT, INPUT);
  pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_CONFIRM, INPUT);
  pinMode(BUTTON_SET, INPUT);//to confirm setting Kp, Ti, Kd, Setpoint.
  pinMode(LMS_LEFT, INPUT);
  pinMode(LMS_RIGHT, INPUT);
  pinMode(BUTTON_RES, INPUT);
  pinMode(SWITCH_HOME, INPUT);
  pinMode(ENCODER_PINA, INPUT);//set pin ENCODER_PINA as input.
  pinMode(ENCODER_PINB, INPUT);//set pin ENCODER_PINB as input.
  
  pinMode(LED_AUTO, OUTPUT);//set pin LED_PIN as output.
  pinMode(LED_MAN, OUTPUT);
  pinMode(LED_HOME, OUTPUT);
  pinMode(LED_LIMIT, OUTPUT);
  pinMode(LED_AUTOHOME, OUTPUT);//On when set button is pressed.
  pinMode(DIR_RIGHT, OUTPUT);//set pin DIR1 as output.
  pinMode(DIR_LEFT, OUTPUT);//set pin DIR2 as output.
  pinMode(ENA, OUTPUT);//set pint ENA as output
  digitalWrite(DIR_RIGHT, LOW);
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(LED_HOME, LOW);
  //FilterOnePole lowpassFilter(LOWPASS, 5.0);
  attachInterrupt(0, doEncoderB, CHANGE);//set up ISR for interrupt 0 - pin 2.
  attachInterrupt(1, doEncoderA, CHANGE);//set up ISR for interrupt 1 - pin 3.
  lcd.clear();
  lcd.begin(16, 2);// set up the LCD's number of columns and rows.
  lcd.print("Linear position control");// Print a message to the LCD.
 } 
/*===================================================================================================*/ 
void loop() {

/*Reading inputs and conversion*/
readKp = analogRead(A1);
readKi = analogRead(A2);
readKd = analogRead(A3);
readSetpoint = analogRead(A4);
readSpeed = analogRead(A5);
// Conversion from analog read.  
floatKp = fmap(readKp, 0, 1023.0, 0.0, 2.0);
floatKi = fmap(readKi, 0, 1023.0, 0.0, 2.0);
floatKd = fmap(readKd, 0, 1023.0, 0.0, 2.0);
knobSetpoint = map(readSetpoint, 0, 1023, 0, 160);
knobSpeed = map(readSpeed, 0, 1023, 0, 255); 
Kp = floatKp; Ki = floatKi; Kd = floatKd;

buttonMode = digitalRead(BUTTON_MODE);
buttonConfirm = digitalRead(BUTTON_CONFIRM);
buttonRight = digitalRead(BUTTON_RIGHT);
buttonLeft = digitalRead(BUTTON_LEFT);
buttonSet = digitalRead(BUTTON_SET);
lmsLeft = digitalRead(LMS_LEFT);
lmsRight = digitalRead(LMS_RIGHT);
switchHome = digitalRead(SWITCH_HOME);
 


/*LCD displays resutls from knobs*/ 
if (millis()-lastLCDTime > 500){
lcd.setCursor(0, 0); lcd.print("P"); lcd.print(floatKp, 4); 
lcd.print("-I"); 
if (floatKi < 10) {lcd.print("0"); lcd.print(floatKi,4);}
else {lcd.print(floatKi, 4);}
lcd.setCursor(0, 1); lcd.print("D"); lcd.print(floatKd, 4); lcd.print("-SP");
if (knobSetpoint < 10) {lcd.print("000"); lcd.print(knobSetpoint); lcd.print("mm");}
else if ((knobSetpoint > 9) && (knobSetpoint < 100)) {lcd.print("00"); lcd.print(knobSetpoint); lcd.print("mm");}
else if ((knobSetpoint > 99) && (knobSetpoint < 1000)) {lcd.print("0"); lcd.print(knobSetpoint); lcd.print("mm");}
else {lcd.print(knobSetpoint); lcd.print("mm");}
lastLCDTime = millis();}

if (millis() - lastFlashingTime > 500){flashBit = !flashBit; lastFlashingTime = millis();}//for flashing.

/*Mode change*/     

if ((buttonMode == LOW) && (buttonConfirm == HIGH)) {autoMode = false; manMode = true;}
if ((buttonMode == HIGH) && (buttonConfirm == HIGH)){manMode = false; autoMode = true;}  
digitalWrite(LED_MAN, manMode); digitalWrite(LED_AUTO, autoMode);
processVariable = ((double)currentPosition * 2)/2400 ; //2400 pulses/rev = 2mm  
  
/*Manual rotation*/  
if (manMode == true) {
      controlVariable = knobSpeed; setPoint = processVariable;
      if (buttonRight == HIGH) {digitalWrite(DIR_LEFT, LOW); digitalWrite(DIR_RIGHT, HIGH);
      analogWrite(ENA, controlVariable);} 
    else if (buttonLeft == HIGH) {digitalWrite(DIR_RIGHT, LOW); digitalWrite(DIR_LEFT, HIGH);
      analogWrite(ENA, controlVariable);} 
    else {analogWrite(ENA,0);}     
      
 if (buttonSet == HIGH && buttonConfirm == HIGH) {autoHome = HIGH;}
 if (autoHome == HIGH && homeSet == LOW){digitalWrite(LED_AUTOHOME, flashBit);
  digitalWrite(DIR_LEFT, LOW);digitalWrite(DIR_RIGHT, HIGH);analogWrite(ENA, controlVariable);}
  
 if ( autoHome == HIGH && switchHome == HIGH) {homeSet = HIGH;
  digitalWrite(LED_HOME, homeSet);digitalWrite(LED_AUTOHOME, LOW);digitalWrite(DIR_RIGHT, LOW);
  digitalWrite(DIR_LEFT, LOW); analogWrite(ENA,0); 
  delay(500);
  autoHome = LOW;
  currentPosition = 0;}}
 
       
/*Automatic mode (PID)
Automatic mode is execuated only when the system is at home and the mode is at AUTO  
PID is processed at interval in ms (1000*CYCLE)*/ 
if ((homeSet == true) && (autoMode == true)) {
  if (buttonSet == HIGH){setPoint = (double)knobSetpoint;}
  if (millis()- lastPIDTime > 1000 * CYCLE) {PID(); lastPIDTime = millis();}
int intControlVariable = int(controlVariable); //Convert to integer.    
/*If the error is > 0 the motor moves CW, other moves CCW.*/ 
if (currentError > 0) {digitalWrite(DIR_LEFT,HIGH); digitalWrite(DIR_RIGHT,LOW);
Serial.println(currentError);}
if (currentError < 0) { digitalWrite(DIR_LEFT,LOW); digitalWrite(DIR_RIGHT,HIGH);}
if (currentError == 0){intControlVariable = 0; digitalWrite(DIR_RIGHT,LOW); digitalWrite(DIR_LEFT,LOW);}
analogWrite(ENA, abs(intControlVariable));}//Write pulse train to PWM output pin 

/*This part is for plotting: setPoint(BLUE), processVariable(RED), controlVariable(GREEN)*/

if (millis() - lastSerialTime > 100) {
 Serial.print(setPoint);
 Serial.print(" ");
 Serial.print(processVariable);
 Serial.print(" ");
 Serial.print(controlVariable);
 Serial.print(" ");
 Serial.println(lmnIntegral);
 lastSerialTime = millis();}
}//END OF LOOP
/*===================================================================================================*/
/*PID ALGORITHM 
lmn is output of the PID controller and consists of 03 portions: proportional, integral, & derivative. 
lmn (called loop-manipulated variable): lmn = lmnP + lmnI + lmnD.
lmn = Kp*ER + Ki*integral(ER*dt) + Kd*d(ER)/dt) (dt = CYCLE: processing interval).
lmnP = Kp*ER = Kp * (setPoint - processVariable). 
lmnI = Ki*integral(ER*dt) <=> d(lmnI) = Ki*ER*dt <=> lmnI-lastLmnI = ER* Ki* CYCLE 
<=> lmnI = lastLmnI + ER*CYCLE*Ki.
lmnD = Kd*d(ER)/dt = Kd* d(ER)/dt = Kd * (ER - lastER)/CYCLE.
lmnD = (Kd/CYCLE)*(ER-lastER)
Bumpless manual-to-auto switch: in manual mode, setPoint = processVariable; then back-calculate lmnI.
Preventing integral-windup: when lmn is out of limit we set lmn to limit and back calculate lmnI.*/
 void PID()
 { 
  static double lastError;//last value of error. 
  double lmn;//loop manipulated variable
  double lmnP;//proportional portion.
  double lmnD;//derivative portion.
  double integralDiff;//Integral difference
  static double lmnI;//integral portion. Static vars for remembering last value.
  
  currentError = setPoint - processVariable;
  //Proportional portion calculation
  lmnP = Kp * currentError;

  //Integral portion calculation.
  if (((integralDiff > 0) && (lmn > HIGH_LIMIT)) || ((integralDiff < 0) && (lmn < LOW_LIMIT))){
        integralDiff = 0.0;}
  else { integralDiff = currentError * CYCLE * Ki;}
  if (currentError == 0.0) {lmnI = 0;} 
  else {lmnI += integralDiff;}
  lmnIntegral = lmnI;    
  //Derivative portion calculation.
  lmnD     = (Kd/CYCLE) * (currentError - lastError);
  lastError = currentError;//update error.
  lmn      = lmnP + lmnI + lmnD;//sum of portions
      
  //Safety limit
  if ((lmn > HIGH_LIMIT) && (lmnI > HIGH_LIMIT)){lmnI = lmnI - (lmn - HIGH_LIMIT);}
  if ((lmn < LOW_LIMIT) && (lmnI < LOW_LIMIT)){lmnI =  lmnI + (LOW_LIMIT - lmn);}
  if (lmn > HIGH_LIMIT) {lmn = HIGH_LIMIT;}
  if (lmn < LOW_LIMIT) {lmn = LOW_LIMIT;}
  if (currentError == 0) {controlVariable = 0;} else {controlVariable = lmn;}
  }//END PID  
 
/*===================================================================================================*/
/*INTERRUPT SUBROUTINE CHANNEL A
When an interrupt on channel A occurs then: CW rotation: A != B, CCW rotation: A = B.8*/
void doEncoderA() {

  if (digitalRead(ENCODER_PINA) == digitalRead(ENCODER_PINB)){
    currentPosition++;}
  else {currentPosition--;}}
/*===================================================================================================*/
/*INTERRUPT SUBROUTINE CHANNEL B
When an interrupt on channel B occurs then: CW rotation: A = B, CCW rotation: A != B.*/
void doEncoderB() {
  if (digitalRead(ENCODER_PINA) != digitalRead(ENCODER_PINB)){
    currentPosition++;}
  else {currentPosition--;}}
/*===================================================================================================*/
double fmap(double x, double in_min, double in_max, double out_min, double out_max)
{ if (x < 3) {x = 0;}
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;}
/*END OF PROGRAM
====================================================================================================*/

