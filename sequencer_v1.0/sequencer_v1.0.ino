#define DEBUG (false)
#if DEBUG
#define DEBUG_RATE (false)
#define DEBUG_STEPS (false)
#define DEBUG_EXT_CLOCK (false)
#endif



// digital outputs
#define CLK_LED           0 //D0

#define STEP1             1 //D1
#define STEP2             2 //D2
#define STEP3             3 //D3
#define STEP4             4 //D4
#define STEP5             5 //D5
#define STEP6             6 //D6
#define STEP7             7 //D7
#define STEP8             8 //D8

#define CLK_OUT            9 //D9

// digital inputs
#define CLK_EXT_SW        10 //D10
#define TOGGLE_UP         11 //D11
#define TOGGLE_DOWN       12 //D12
#define DIR_SWITCH        13 //D13

// Analog pins
#define CLK_EXT_IN        A0 // A0
#define PROG_BTN          A1 // A1
#define STEPS_DIAL        A2 // A2
#define PWM_POT           A3 // A3
#define FINE_POT          A4 // A4
#define RATE_POT        A5 // A4
#define TOUCH_SENSOR    A6 


//--------------------------------------------------------------------
// globals

unsigned long half_period_ms;
unsigned long avg_half_period_ms;

float min_step_ms = 5.0;
float max_step_ms = 1500.0;

int MAX_VOLT_READ = 1024; //1000; //875;

float STEP_8 = 1020; //MAX_VOLT_READ -10; //7*MAX_VOLT_READ/8;  
float STEP_7 = 890; //75 - 1; //7*MAX_VOLT_READ/8;  
float STEP_6 = 700; //38 - 1; //6*MAX_VOLT_READ/8;  
float STEP_5 = 590; //26 - 1; //5*MAX_VOLT_READ/8;  
float STEP_4 = 400; //19 - 1; //4*MAX_VOLT_READ/8;*  
float STEP_3 = 250; //15 - 1; //3*MAX_VOLT_READ/8;  
float STEP_2 = 100; //12 - 1; //2*MAX_VOLT_READ/8;  


float bpm = 100;

enum DIRECTION
{
  FWD_,
  BACK_
};

enum STATE
{
  IDLE_,
  PLAY_,
  PROG_

};

enum CLOCK_STATE
{
  INT_,
  EXT_
};

enum TIME_SAMPLER_STATE
{
  WAIT_,
  SAMPLE_
};

// state variables
int state;
int dir;

int clock_state;

int curr_chan;
int curr_state;

unsigned long prev_timer = 0;

int stop_chan;
int start_chan;
int prev_stop_chan;

bool cycle_up;
bool change_trigger;
bool toggle_down;
bool toggle_up;
bool steps_pot_state;

bool reset_pressed;

bool prev_ext_cycle;


float maxtime = 10.0;
int max_samplings = 32;
bool lastRst = false;
unsigned long rst_lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long rst_debounceDelay = 10;    // the debounce time; increase if the output 

unsigned long lastClick = 0;

unsigned long avg_time = 0.0;
unsigned long currtime;

bool firstOn = true;
int timeSamplerState = TIME_SAMPLER_STATE::WAIT_;





//---------------------------------------------------------------------------------------
void initPins()
{
  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(STEP4, OUTPUT);
  pinMode(STEP5, OUTPUT);
  pinMode(STEP6, OUTPUT);
  pinMode(STEP7, OUTPUT);
  pinMode(STEP8, OUTPUT);
  pinMode(CLK_OUT, OUTPUT);
  pinMode(CLK_LED, OUTPUT);

  pinMode(CLK_EXT_SW, INPUT);
  pinMode(CLK_EXT_IN, INPUT);
  pinMode(TOGGLE_UP, INPUT);
  pinMode(TOGGLE_DOWN, INPUT);
  pinMode(DIR_SWITCH, INPUT);


  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, LOW);
  digitalWrite(STEP4, LOW);
  digitalWrite(STEP5, LOW);
  digitalWrite(STEP6, LOW);
  digitalWrite(STEP7, LOW);
  digitalWrite(STEP8, LOW);
  digitalWrite(CLK_OUT, LOW);
  digitalWrite(CLK_LED, LOW);

}

void initState()
{
  state = STATE::IDLE_;
  timeSamplerState = TIME_SAMPLER_STATE::WAIT_;
  
  half_period_ms =  (unsigned long) ( 1000.0*1.0/(2*bpm/60.0) );//
  avg_half_period_ms =  (unsigned long) ( 1000.0*1.0/(2*bpm/60.0) );//

  lastRst = false;
  
  lastClick = 0;



  start_chan = 1;
  stop_chan = 8;
  prev_stop_chan = stop_chan;

  cycle_up = false;  // start with clock down
  change_trigger = false;

  toggle_up = false;
  toggle_down = false;
  reset_pressed = false;


  steps_pot_state = false;

  for (int i = 1; i < 9; i++)
    digitalWrite(i, LOW);

  digitalWrite(CLK_LED, cycle_up);
  digitalWrite(CLK_OUT, cycle_up);

  if (digitalRead(DIR_SWITCH) == LOW)
    dir =  DIRECTION::FWD_;
  else
    dir = DIRECTION::BACK_;

  if (dir == DIRECTION::FWD_)
    curr_chan = stop_chan;
  else
    curr_chan = start_chan;

  
  if(!digitalRead(CLK_EXT_SW))
  {
    clock_state = CLOCK_STATE::EXT_;
        //cycle_up = analogRead(CLK_EXT_IN);
  }
  else
  {
    clock_state = CLOCK_STATE::INT_;
  }

  prev_timer = millis(); // initialise clock timer
  digitalWrite(curr_chan, HIGH);
}

bool lastTup = false;
bool lastTdw = false;
unsigned long tup_lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long tdw_lastDebounceTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output 

void readTransportToggle()
{
  bool tup = digitalRead(TOGGLE_UP);

  if (tup != lastTup)
  {
    tup_lastDebounceTime = millis();
  }

  if ((millis() - tup_lastDebounceTime) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (tup)
    {
      
      if (!toggle_up)
      {
        if (state == STATE::IDLE_)
        {
          state = STATE::PLAY_;        
        }

        else if (state == STATE::PROG_)
        {
          state = STATE::IDLE_;
          //Serial.println("idle");
        }

        toggle_up = true;

      }
    }
    else
      toggle_up = false;

  }
  lastTup = tup;

  bool tdw = digitalRead(TOGGLE_DOWN);
  if (tdw != lastTdw)
  {
    tdw_lastDebounceTime = millis();
  }

  if ((millis() - tdw_lastDebounceTime) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (tdw)
    {
      if (!toggle_down)
      {
        if (state == STATE::PLAY_)
        {
          state = STATE::IDLE_;
          if (timeSamplerState == TIME_SAMPLER_STATE::SAMPLE_)
          {
            timeSamplerState = TIME_SAMPLER_STATE::WAIT_;         
          }        
        }

        else if (state == STATE::IDLE_)
        {
          state = STATE::PROG_;        
        }

        else if (state == STATE::PROG_)
        {
          digitalWrite(curr_chan, LOW);// switch off the current channel
    
          // update curr_chan to the new channel
          if (dir == DIRECTION::FWD_)
          {
            if (curr_chan >= stop_chan)
            {
              curr_chan = start_chan;
            }
            else
            {
              curr_chan++;
            }
          }
          else if (dir == DIRECTION::BACK_)
          {
            if (curr_chan <= start_chan)
            {
              curr_chan = stop_chan;
            }
            else
            {
              curr_chan--;
            }
          }
          digitalWrite(curr_chan, HIGH); // switch on the new channel
        }

        toggle_down = true;
      }
    }
    else
      toggle_down = false;
  }
  lastTdw = tdw;
}

void readRatePot()
{
  int rate_read = analogRead(RATE_POT);
  int fine_read = analogRead(FINE_POT);

  rate_read = rate_read > MAX_VOLT_READ ? MAX_VOLT_READ : rate_read;
  fine_read = fine_read > MAX_VOLT_READ ? MAX_VOLT_READ : fine_read;
  
  //float rate_val = ( (MAX_VOLT_READ - rate_read) * max_step_ms + rate_read * min_step_ms ) / MAX_VOLT_READ;
  //float fine_val = ( (MAX_VOLT_READ - fine_read) * max_step_ms + fine_read * min_step_ms ) / MAX_VOLT_READ;

  float frac_read = (float)rate_read + ((float)fine_read)/32.0;
  
  float comb_val = ( (MAX_VOLT_READ - frac_read) * max_step_ms + frac_read * min_step_ms ) / MAX_VOLT_READ;

  comb_val = (comb_val > max_step_ms) ? max_step_ms : ( (comb_val < min_step_ms) ? min_step_ms : comb_val );

  //half_period_ms = rate_val + fine_val/( (MAX_VOLT_READ + 1)/100.0 ); //10.240;
  half_period_ms = comb_val;

#if DEBUG
#if DEBUG_RATE
  Serial.println(rate_read);
#endif
#endif
  
}


void readStepsPot()
{
  
  int step_voltage = analogRead(STEPS_DIAL);

  #if DEBUG
  #if DEBUG_STEPS
    Serial.println(step_voltage);  
  #endif
  #endif

  if (step_voltage >= STEP_8)  
  {
    stop_chan = 8;
  }
  else if (step_voltage > STEP_7)
  {
    stop_chan = 7;
  }
  else if (step_voltage > STEP_6)
  {
    stop_chan = 6;
  }
  else if (step_voltage > STEP_5)
  {
    stop_chan = 5;
  }
  else if (step_voltage > STEP_4)
  {
    stop_chan = 4;
  }
  else if (step_voltage > STEP_3) 
  {
    stop_chan = 3;
  }
  else if (step_voltage > STEP_2) 
  {
    stop_chan = 2;
  }
  else // 1
  {
    stop_chan = 1;
  }
}

unsigned long pressStart = 0.0;
unsigned long releaseStart = 0.0;

unsigned int clickCounter = 0;
unsigned long cumClickDistance = 0.0;
unsigned int countBufferLength = 3;

void readResetButton()
{
  bool rst = digitalRead(PROG_BTN);

  if (rst != lastRst)
  {
    rst_lastDebounceTime = millis();
  }

  if ((millis() - rst_lastDebounceTime) > rst_debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (rst) // the button is definitely pressed down
    { 
      if ( state == STATE::PLAY_)
      {
        for (int i = 1; i < 9; i++)
          digitalWrite(i, LOW);

        if (dir == DIRECTION::FWD_)
          curr_chan = start_chan;
        else
          curr_chan = stop_chan;

        prev_timer = millis(); // initialise clock timer
        digitalWrite(curr_chan, HIGH);
      }
      else if(state == STATE::IDLE_)
      {
        if (clock_state == CLOCK_STATE::EXT_)
        {
          if (timeSamplerState == TIME_SAMPLER_STATE::WAIT_)
          {
              for (int i = 1; i < 9; i++)
              {
                digitalWrite(i, LOW);
              }

              if (dir == DIRECTION::FWD_)
              {
                curr_chan = start_chan;
              }
              else
              {
                curr_chan = stop_chan;
              }

              cycle_up = false; // set cycle_down
              change_trigger = false;
              
              prev_timer = millis(); // initialise clock timer
              digitalWrite(curr_chan, HIGH);

              timeSamplerState = TIME_SAMPLER_STATE::SAMPLE_;
              clickCounter = 0;
              lastClick = millis();                            
          }
          else // if the sampler is sampling (not in wait) the each click on the sensor should reset the playhead at start_chan
          {
            if (dir == DIRECTION::FWD_)
                curr_chan = start_chan;
              else
                curr_chan = stop_chan;
          }

          if (!reset_pressed)
          {            
            currtime = millis();
            pressStart = currtime;                          
          }      
          reset_pressed = true;
        }
        else // if clock is internal and we are in idle mode (stopped) a reset input will set the state to zero
        {
          initState();
        }
      }            
    }
    else // the button is not pressed
    { 
      if (clock_state == CLOCK_STATE::EXT_)
      {     
        if(state == STATE::IDLE_)
        {         
          if (reset_pressed) // the button was pressed, and now is not
          {
            releaseStart = millis();
            unsigned long presd = releaseStart-pressStart;  

            if(presd > 2000) // more than 2 seconds set the sampler back to wait
            {              
              clickCounter = 0;
              cumClickDistance = 0.0;              
              avg_half_period_ms =  (unsigned long) ( 1000.0*1.0/(2*bpm/60.0) );

              if (dir == DIRECTION::FWD_)
                curr_chan = start_chan;
              else
                curr_chan = stop_chan;

              cycle_up = false; // set cycle_down
              change_trigger = false;

              timeSamplerState == TIME_SAMPLER_STATE::WAIT_;
            }
            
            if (timeSamplerState = TIME_SAMPLER_STATE::SAMPLE_)
            {
              if (clickCounter > 1)
              {
                unsigned long timediff = currtime - lastClick;         
              
                cumClickDistance = ((clickCounter - 1)*cumClickDistance + timediff)/clickCounter;
                avg_half_period_ms = cumClickDistance/(2*stop_chan);
                lastClick = currtime;                   
              }
              clickCounter <  countBufferLength? clickCounter++ : countBufferLength;                           
            }                  
                                      
            reset_pressed = false; 
          }
          else
          {              
              unsigned long releaseLength = millis()-releaseStart;
              if (releaseLength > 6000) // 6 seconds without click reset to wait
              {
                clickCounter = 0;
                cumClickDistance = 0.0;
                //avg_half_period_ms =  (unsigned long) ( 1000.0*1.0/(2*bpm/60.0) );
                timeSamplerState == TIME_SAMPLER_STATE::WAIT_;
              }
          } 
        } 
      }
    }    
  }
  lastRst = rst; 
}


//---------------------------------------------------------------------------------------

void setup()
{
  initPins();
  initState();

  //Serial.begin(9600); // open the serial port at 9600 bps:


}

void loop()
{

  bool ext_switch_val = !digitalRead(CLK_EXT_SW);

  if(ext_switch_val)
  {
    clock_state = CLOCK_STATE::EXT_;
        //cycle_up = analogRead(CLK_EXT_IN);
  }
  else
  {
    clock_state = CLOCK_STATE::INT_;
  }
  
  if (digitalRead(DIR_SWITCH) == LOW)
    dir =  DIRECTION::FWD_;
  else
    dir = DIRECTION::BACK_;


  readStepsPot();
  readTransportToggle();
  readRatePot();
  readResetButton();

  if(state != STATE::PROG_)
  {        
    if (change_trigger)
    {
      change_trigger = false;    
    }
    if (clock_state == CLOCK_STATE::INT_)
    {    
      if ( (millis() - prev_timer) >= half_period_ms)
      {
        if (cycle_up)
        {
          cycle_up = false; // set cycle_down
          prev_timer = millis(); // reset timer
        }
        else // one full cycle has finished.
        {
          change_trigger = true;
          cycle_up = true; // set cycle_up and reset timer
          prev_timer = millis();
        }
      }
    }
    else // EXT
    {
      if ( (millis() - prev_timer) >= avg_half_period_ms)
      {
        if (cycle_up)
        {
          cycle_up = false; // set cycle_down
          prev_timer = millis(); // reset timer
        }
        else // one full cycle has finished.
        {
          change_trigger = true;
          cycle_up = true; // set cycle_up and reset timer
          prev_timer = millis();
        }
      }      
    }      
    
    if (state == STATE::PLAY_)
    {      
      if (change_trigger)
      {        
        digitalWrite(curr_chan, LOW);// switch off the current channel
  
        // update curr_chan to the new channel
        if (dir == DIRECTION::FWD_)
        {
          if (curr_chan >= stop_chan)
          {            
            curr_chan = start_chan;
          }
          else
          {
            curr_chan++;
          }          
        }
        else if (dir == DIRECTION::BACK_)
        {
          if (curr_chan <= start_chan)
          {
            curr_chan = stop_chan;
          }
          else
          {
            curr_chan--;
          }
        }
        digitalWrite(curr_chan, HIGH); // switch on the new channel
        change_trigger = false;
      }
    }
    digitalWrite(CLK_LED, cycle_up);
    digitalWrite(CLK_OUT, cycle_up);
  }
  else // PROG
  {
    digitalWrite(CLK_LED, HIGH);
    
    if(reset_pressed){
      digitalWrite(CLK_LED, HIGH);
      digitalWrite(CLK_OUT, HIGH);
    }
    else
    {
      digitalWrite(CLK_LED, LOW);
      digitalWrite(CLK_OUT, LOW);
    }
      
  }

  delay(5);
}
