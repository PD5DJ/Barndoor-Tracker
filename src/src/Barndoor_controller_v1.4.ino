/*
    Project     : Barndoor tracker controller based on metric M8 Threaded Rod
    Designer    : Björn Pasteuning
    Website     : https://www.thingiverse.com/PD5DJ/about
    Design Date : February 2019
    version     : 1.4
*/


/*
   Description of use:

   On power up, the tracker will home first to its starting point. the led will solidly lid up. led wil go off when home position has reached.
   After pressing the button briefly the tracker starts to track, and the led will start blink fast.
   Pressing the button again briefly the tracker stops tracking and remember its position. and the led also stops blinking.
   Pressing again it will continues to track.
   Pressing the button for longer then 2 seconds the tracker automaticly starts homing back to its start position.

*/


/*
   Example:

   1.8degree stepper has to make 200 steps to make a 360 rotation.
   In full step mode the Rotation_Step_Interval = 60(seconds) / 200(steps) = 0.3sec = 300mS
   In 16x Microstepping the Rotation_Step_Interval = 0.3 / 16 = 0.01875sec = 18.750mS = 18750uS

   0.9degree stepper has to make 400 steps to make a 360 rotation.
   In full step mode the Rotation_Step_Interval = 60(seconds) / 400(steps) = 0.15sec = 150mS
   In 16x Microstepping the Rotation_Step_Interval = 0.15 / 16 = 0.009375sec = 9.375mS = 9.375uS

   Day has 24 hours.
   Rotation of galaxy is 360 degrees in 24 hours.
   360 / 24 = 15 degree rotation per hour.
   Hour has 60 minutes.
   15 / 60 = 0.25 degree per minute.
   So 1 RPM is 0.25 degree
   Now you can check if your tracker stepper has made 1 rotation, it must inclined 0.25degree on a inclinometer

*/

#include <TimerOne.h>
#include <Arduino.h>
#include <A4988.h>

//============================================================================
//------------------------------ Configuration -------------------------------
//============================================================================
const int Stepper_Enable  = 8;               // Stepper Enable pin of stepperboard.
const int Stepper_Speed = 14;               // MSx Pins of stepperboard, combine MS1 MS2 MS3 togheter to this pin.

const int HomeStart_Pin = 9;                // Microswitch in Normal Open state connected to ground.
const int Status_Led_Pin = 17;              // Connected with a 330ohm resistor to ground.
const int Button_Pin = 12;                  // Momentary push button connected to ground.

/*
 * Thread - Pitch per mm
 * M4 = 0.70
 * M5 = 0.80
 * M6 = 1.00
 * M8 = 1.25
 * M10 = 1.50
 * M12 = 1.75
 * 8mm Leadscrew = 2.00
 */
const double Thread_Pitch_mm = 1.25;        // Enter Thread Pitch in mm's, see table above.
const int Hinge_Distance = 260;          // This is the distance between de center of the hinge joint and the center of the treaded rod. in mm's
const int Threaded_UpperRod_Length = 100;   // Length of Threaded rod in mm's, measured from the NUT up to the end of the rod.

// Configuring Stepper Driver
#define MOTOR_STEPS 400                     // 200 for 1.8 degree stepper, 400 for 0.9 degree stepper.
#define DIR 2                               // Direction pin of stepperboard.
#define STEP 5                              // Step pin of stepperboard.
#define MS1 0                               // not used
#define MS2 0                               // not used
#define MS3 0                               // not used
//============================================================================

A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

bool Run_Tracker;                           // Flag to determine to activate Tracking
bool Home_Tracker;                          // Flag to determine to activate Homeing
bool Button_Pressed;                        // Flag to detect if button has pressed
int Time;                                   // Seconds time variable
int Button_Timer = 0;                       // Seconds time variable counts when button is keeped pressed
long Steps;                                 // Counts overall position
long Steps_mm;                              // Steps needed to move the NUT 1mm
long Step_Limit;                            // the maximum amount of steps to be made (soft endstop) depends on rod length.
int Rotation_Step_Interval;                 // Timing variable used between steps for speed control
int Delay_1;                                // Timing variable
int Delay_2;                                // Timing variable



//============================================================================
//------------------------------- Initialize ---------------------------------
//============================================================================
void setup() {

  Serial.begin(115200);                     // open the serial port at 115200 bps:

  Serial.println("Barndoor tracker v1.4 by Björn Pasteuning");

  Timer1.initialize(100000);                // Configuring 100mS timer
  Timer1.attachInterrupt(Timer_1);          // Jump to "Timer_1" when 100mS interrupts are reached

  pinMode(HomeStart_Pin, INPUT_PULLUP);
  pinMode(Status_Led_Pin, OUTPUT);
  pinMode(Button_Pin, INPUT_PULLUP);
  digitalWrite(Stepper_Enable, HIGH);       //LOW = ON, High = OFF
  pinMode(Stepper_Speed, OUTPUT);
  digitalWrite(Stepper_Speed, LOW);         //LOW = Fast speed (Full step), HIGH = Slow speed (16 or 32x microstepping depending on stepper driver)
  stepper.begin(1, 16);                     // Dummy init on max microstepping function further not used in this scope)


  if (MOTOR_STEPS == 200)
  {
    Steps_mm = 2560;
    
    Rotation_Step_Interval = ( (0.3/16) / (Hinge_Distance / ( (Thread_Pitch_mm * 1436) / (2*PI) ) ) ) * 1000000;
  }
  else if (MOTOR_STEPS == 400)
  {
    Steps_mm = 5120;
    
    Rotation_Step_Interval = ( (0.15/16) / (Hinge_Distance / ( ( Thread_Pitch_mm * 1436) / (2*PI) ) ) ) * 1000000;
  }

  // Prevents the DelayMicroseconds to overflow
  if (Rotation_Step_Interval > 16000)
  {
    Delay_1 = 16000;
    Delay_2 = Rotation_Step_Interval - 16000;
  }
  else
  {
    Delay_1 = Rotation_Step_Interval;
    Delay_2 = 0;
  }
  


  Step_Limit = Steps_mm * (Threaded_UpperRod_Length - 10);

  Home_Tracker = true;                      // First time homeing after power loss
  Run_Tracker = false;
  Time = 0;
  Steps = 0;
  Button_Pressed = false;

  if (digitalRead(Button_Pin) == LOW)
  {
    while (digitalRead(Button_Pin) == LOW) {
    }
    digitalWrite(Stepper_Speed, LOW); // Full step on
    digitalWrite(Stepper_Enable, LOW);
    while (digitalRead(HomeStart_Pin) == HIGH)
    {
      stepper.move(-1);
      delay(1);
    }
    digitalWrite(Stepper_Enable, HIGH);
  }


  // Debug
  //====================================
  Serial.print("Step Limit = ");
  Serial.println(Step_Limit);

  Serial.print("Rotation speed = ");
  Serial.print(Rotation_Step_Interval);
  Serial.println("uS");
  Serial.print(Delay_1);
  Serial.print("uS + ");
  Serial.print(Delay_2);
  Serial.println("uS");
  //=====================================
}


//============================================================================
// ------------------------------ Main Loop ----------------------------------
//============================================================================
void loop()
{
  Button_Scan();
  Homeing();
  Tracking();
}


//============================================================================
// ------------------------- Button Handling loop ----------------------------
//============================================================================
void Button_Scan()
{

  if (digitalRead(Button_Pin) == LOW)
  {
    if (Button_Pressed == false)
    {
      Button_Pressed = true;
      Button_Timer = 0;

      delay(100);
    }
  }

  if (Button_Pressed == true && digitalRead(Button_Pin) == HIGH)
  {
    if (Button_Timer < 1 && Run_Tracker == false)
    {
      delay(100);
      digitalWrite(Stepper_Enable, LOW);
      Serial.println("Tracking Started");
      Run_Tracker = true;
      Time = 0;
      Button_Pressed = false;
    }
    else if (Button_Timer < 1 && Run_Tracker == true)
    {
      delay(100);
      Run_Tracker = false;
      digitalWrite(Stepper_Enable, HIGH);
      Serial.println("Tracking Stopped");
      Serial.print("Actual step position: ");
      Serial.println(Steps);
      Button_Pressed = false;
      digitalWrite(Status_Led_Pin, LOW);
    }
    else
    {
      Button_Pressed = false;
    }
  }
  if (Button_Timer > 2)
  {
    delay(100);
    Run_Tracker = false;
    Home_Tracker = true;
    Button_Pressed = false;
    Button_Timer = 0;
  }
}

//============================================================================
// ----------------------------- Timer Interrupt -----------------------------
//============================================================================
void Timer_1()
{
  static int Button_Timer_temp;
  static int Time_temp;
  static boolean Toggle_Flag;                 // Led toggle variable

  if (Button_Pressed == true)
  {
    if (Button_Timer_temp > 9)
    {
      Button_Timer_temp = 0;
      Button_Timer++;
    }
    Button_Timer_temp++;
  }

  if (Run_Tracker == true)
  {
    if (Time_temp > 9)
    {
      Time_temp = 0;
      Time++;
    }
    Time_temp++;
  }

  // Toggles Status led when tracking is active
  if (Run_Tracker == true)
  {
    if (Toggle_Flag)
    {
      digitalWrite(Status_Led_Pin, HIGH);   // set the LED on
      Toggle_Flag = !Toggle_Flag;
    }
    else
    {
      digitalWrite(Status_Led_Pin, LOW);    // set the LED off
      Toggle_Flag = !Toggle_Flag;
    }
  }
}

//============================================================================
// ----------------------------- Homeing Loop --------------------------------
//============================================================================
void Homeing()
{
  if (Home_Tracker == true)
  {
    digitalWrite(Stepper_Speed, LOW); // Full step on
    Serial.println("Homing!");
    Run_Tracker = false;
    digitalWrite(Status_Led_Pin, HIGH);

    digitalWrite(Stepper_Enable, LOW);

    //This loop can never be interrupted while activated for safety reasons.
    while (digitalRead(HomeStart_Pin) == HIGH)
    {
      stepper.move(1);
      delay(1);
    }

    Serial.println("Starting point reached");
    Steps = 0;
    Home_Tracker = false;
    digitalWrite(Stepper_Enable, HIGH);
    digitalWrite(Status_Led_Pin, LOW);
  }
}

//============================================================================
// ----------------------------- Tracking Loop -------------------------------
//============================================================================
void Tracking()
{
  if (Run_Tracker == true)
  {
    digitalWrite(Stepper_Speed, HIGH);  // Max Microstepping on

    stepper.move(-1);
    Steps++;

    delayMicroseconds(Delay_1);
    delayMicroseconds(Delay_2);

    // Checks if the maxium steps vs rod length are met then stops.
    if (Steps > Step_Limit)
    {
      Run_Tracker = false;
      digitalWrite(Stepper_Enable, HIGH);
      Serial.println("Tracking Stopped");
      Serial.print("Actual step position: ");
      Serial.println(Steps);
      digitalWrite(Status_Led_Pin, LOW);
      //Home_Tracker = true;
    }
  }

}
