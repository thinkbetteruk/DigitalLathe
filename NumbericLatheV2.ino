 /*
    MIT License

 /*
 *   Nano/Atmega328 PINS: connect LCD to A4/A5 (i2c)
 *   ESP8266: GPIO4(SDA) / GPIO5( SCL )
 *   STM32: B7(SDA), B6(SCL)
 */

#include <assert.h>
#include <math.h>
#include "lcdgfx.h"
// #include "owl.h"
#include <Stepper.h>


#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false
#include <ISR_Timer.h>
#include <TimerInterrupt.h>



  
#define DEBUG

#define DISPLAY 1
#define RUN 2

#define LEFT    1
#define RIGHT (-1)

#define TIMER1_INTERVAL_MS        5


#define STEPS_PER_REVOLUTION 200
#define UMM_PER_REVOLUTION 1000
#define MAX_STEPS_PER_CYCLE 50 // Steps per loop cycle 
#define UMM_PER_STEP (UMM_PER_REVOLUTION / STEPS_PER_REVOLUTION)

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(STEPS_PER_REVOLUTION, 8, 9, 10, 11);

/* Initialise the display */
DisplaySSD1306_128x64_I2C display(-1); // This line is suitable for most platforms by default

/* For managing the rotary encoder */
byte CLK = 2;    //CLK->D2
byte DT = 3;     //DT->D3
byte SW = 4;     //SW->D4
int count =     0;//Define the count
int lastCLK =   0;//CLK initial value
char szBuf[128]="";
signed char direction;

#define INTERRUPT0 0

long oldX;       // Position at last loop in mm/10
long currentX;   // Current position in mm/10
long targetX;    // Target  position in mm/10
long num_steps;  // How many stepper motor steps to go
int currentSpeed;
byte currentMode;
int rotaryDelta = 0;
unsigned long ulRunStartTime;

SAppMenu menu;

const char *menuItems[] =
{
    "Menu",
    "Set 0 reference",
    "Set Speed",
    "Goto X",
    "Run"
};


class pushButton
{
  private:
    unsigned int _status;
    byte _pin;
    bool _hasChanged;
    const unsigned int TIMEOUT = 20;
    const unsigned int PRESSED_MASK = 0x00FF;

  public:
    pushButton (byte pin);
    void init();
    bool hasChanged() { return _hasChanged; }
    bool status() { return _status; }
    void update();
    bool isPressed();
};

pushButton::pushButton(byte pin)
{
  _pin = pin;
  _hasChanged = false;
}

void pushButton::init()
{
  pinMode(_pin, INPUT_PULLUP);  
}


void pushButton::update()
{
  _status = _status << 1;
  _status |= digitalRead(_pin);

  #ifdef DEBUG_BUTTON_UPDATE
  Serial.print ("_status = "); Serial.print (_status); Serial.println("");
  #endif
}

bool pushButton::isPressed() 
{ 
  return ((_status & PRESSED_MASK) == 0x0F);
}

pushButton myButton(5);


//The interrupt handlers
void ClockChanged()
{
  byte clkValue = digitalRead(CLK);//Read the CLK pin level
  byte dtValue = digitalRead(DT);//Read the DT pin level

  if (lastCLK != clkValue)
  {
    lastCLK = clkValue;
    rotaryDelta = (clkValue != dtValue ? -1 : +1);
  }
  else
  {
    // rotaryDelta = 0;
  }
}

void stopMotor()
{
  myStepper.step(0);
}

int readSpeed()
{
  display.clear();
  display.printFixed(0,  8, "Enter speed:", STYLE_NORMAL);
  sprintf (szBuf, "%i     ", currentSpeed);
  display.printFixed(0,  16, szBuf, STYLE_NORMAL);

  do
  {
    currentSpeed += rotaryDelta;
    rotaryDelta = 0;

    if (currentSpeed <= 0)
      currentSpeed = 1;
      
    sprintf (szBuf, "%i     ", currentSpeed);
    display.printFixed(0,  16, szBuf, STYLE_NORMAL);
    
#ifdef DEBUG_READ_SPEED
    Serial.print("Speed=");
    Serial.println(currentSpeed);
#endif
  
  } while (!myButton.isPressed());
}

int readTargetX()
{
  display.clear();
  display.printFixed(0,  8, "Final X:", STYLE_NORMAL);
  sprintf (szBuf, "%li     ", targetX);
  display.printFixed(0,  16, szBuf, STYLE_NORMAL);

  do
  {
    targetX += rotaryDelta*100;
    rotaryDelta = 0;
    
    sprintf (szBuf, "%li     ", targetX);
    display.printFixed(0,  16, szBuf, STYLE_NORMAL);

#ifdef DEBUG_READ_TARGET
    Serial.print("X=");
    Serial.println(targetX);
#endif

  } while (!myButton.isPressed());
}

void TimerHandler()
{
  myButton.update();
}


void setup()
{
  currentX = 0;
  currentSpeed = 20;
  targetX = 0;
  
  /* Init for the rotary encoder */
  myButton.init();
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  attachInterrupt(INTERRUPT0, ClockChanged, CHANGE);//Set the interrupt 0 handler, trigger level change

  #ifdef DEBUG
  Serial.begin(9600);
  #endif


  /* Init the clock interrupt */
  // Using ATmega328 used in UNO => 16MHz CPU clock , 
  ITimer1.init();

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer1 OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer1. Select another freq., duration or timer");
  

  /* Select the font to use with menu and all font functions */
  display.setFixedFont( ssd1306xled_font6x8 );
  display.begin();
  display.createMenu( &menu, menuItems, sizeof(menuItems) / sizeof(char *) );
  display.clear();

  /* And start in menu mode */
  currentMode = DISPLAY;
  display.showMenu( &menu );

  // Button for Enter/Stop
  pinMode(5, INPUT_PULLUP);

  #ifdef DEBUG
    Serial.println("Setup complete");
  #endif
}



void loop()
{
  if(currentMode == DISPLAY)
  {
    do
    {
      if (rotaryDelta > 0)
      {
#ifdef DEBUG_MENU
        Serial.println ("Menu down");
#endif
        display.menuDown (&menu);
        display.updateMenu (&menu);
      }
      else if (rotaryDelta < 0)
      {
#ifdef DEBUG_MENU
        Serial.println ("Menu up");
#endif
        display.menuUp (&menu);
        display.updateMenu (&menu);
      }
      rotaryDelta = 0;

      myButton.update();
    } while (!myButton.isPressed());

    
    switch (display.menuSelection(&menu))
    {
      case 0:
        break;

      case 1:
        /* Set X reference point */
#ifdef DEBUG
        Serial.println ("Set zero");
#endif
        currentX = 0;
        display.clear();
        display.printFixed(0,  8, "Current position set to 0", STYLE_NORMAL);
        lcd_delay (2000);
        display.clear();
        display.showMenu( &menu );
        break;

      case 2:
        /* Set Speed */
        #ifdef DEBUG
          Serial.println ("Set Speed");
        #endif
        
        display.clear();
        readSpeed();
        myStepper.setSpeed(currentSpeed);

        #ifdef DEBUG
          sprintf (szBuf, "Speed= %i",  currentSpeed); Serial.println(szBuf);
        #endif

        display.clear();
        display.showMenu( &menu );
        break;

      case 3:
        /* Goto X */
        #ifdef DEBUG
          Serial.println ("Set target X");
        #endif
        readTargetX();
        display.clear();
        display.showMenu( &menu );
        break;

      case 4:
        /* Go */
        direction = (targetX > currentX) ? LEFT : RIGHT;
        #ifdef DEBUG
          Serial.println ("Run!"); Serial.flush();
        #endif
        num_steps = (targetX - currentX) / UMM_PER_STEP;
        if (num_steps < 0)
          num_steps = -num_steps;
          
        ulRunStartTime = millis();

        #ifdef DEBUG
          Serial.print ("Direction = "); Serial.println (direction);
          Serial.print ("Number of steps: "); Serial.println (num_steps); 
          Serial.flush();
        #endif
 

        currentMode = RUN;
           

        delay (500);
        display.clear();
        sprintf (szBuf, "Target=%li", targetX);      display.printFixed(0,  8, szBuf, STYLE_NORMAL);
        sprintf (szBuf, "Speed=%i",  currentSpeed); display.printFixed(0, 16, szBuf, STYLE_NORMAL);
        break;

      default:
        break;
    }
  }
  else /* Current mode is RUN */
  {   
    #ifdef DEBUG_2
      sprintf (szBuf, "X= %li",      currentX);     Serial.println(szBuf);
      sprintf (szBuf, "Target= %li", targetX);      Serial.println(szBuf);
      sprintf (szBuf, "Speed= %i",  currentSpeed); Serial.println(szBuf);
      delay (1);
    #endif

    assert (currentMode == RUN);

    sprintf (szBuf, "X=%li",  currentX); display.printFixed(0, 24, szBuf, STYLE_NORMAL);

    
    if (myButton.isPressed())
    {
      #ifdef DEBUG
        Serial.println ("Stop Run!"); delay (100);
      #endif
      
      stopMotor();
      display.clear();   
      display.showMenu( &menu );
      currentMode = DISPLAY;
      return;
    }

    long iStepsInThisLoop = num_steps;
    
    if (iStepsInThisLoop > MAX_STEPS_PER_CYCLE)
      iStepsInThisLoop = MAX_STEPS_PER_CYCLE;
   
    #ifdef DEBUG_2
      sprintf (szBuf, "Step in this loop = %li", iStepsInThisLoop); Serial.println(szBuf);
    #endif
      
    myStepper.setSpeed(currentSpeed);
    myStepper.step(direction == LEFT ? -iStepsInThisLoop : iStepsInThisLoop);

    oldX = currentX;
    currentX += (direction == LEFT ? iStepsInThisLoop : -iStepsInThisLoop) * UMM_PER_STEP;
    num_steps -= iStepsInThisLoop;
    
    /* Check if arrived */
//    if ((currentX == oldX) || (direction == LEFT && currentX >= targetX) || (direction == RIGHT && currentX <= targetX))
    if (num_steps <= 0)
    {
      unsigned long ulCurrentTime = millis();
      stopMotor();
      display.clear();   
      display.showMenu( &menu );
      currentMode = DISPLAY;

      #ifdef DEBUG
        sprintf (szBuf,"Job completed in %lu milliseconds", ulCurrentTime - ulRunStartTime); 
        Serial.println (szBuf);
      #endif
    }
  }
}
