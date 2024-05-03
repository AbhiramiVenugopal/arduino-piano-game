#include "pitches.h" //this file contauns public constants that indiciate musical notes and their associated frequencies

int val=0; 

unsigned long on_time=0;
unsigned long off_time=0;
unsigned long button_ontime[20];
unsigned long button_offtime[20];

int button_seq[20];
int buzzer = 10; //piezo buzzer 
int button1=3; //all 7 buttons of the piano has been defined
int button2=4;
int button3=5;
int button4=6;
int button5=7;
int button6=8;
int button7=9;

//int melody[] = {NOTE_C5 , NOTE_A1 , NOTE_A2, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5}; //an array of notes that correspond to each button of the piano, for e.g. the first button of piano play C4 note
int melody[] = {NOTE_A4, NOTE_B4, NOTE_C5, NOTE_DS5, NOTE_E5, NOTE_G5, NOTE_F6};

int buttonPin = 2; //REMOVE
int previousState = HIGH;
unsigned int previousPress;
volatile int buttonFlag;
int buttonDebounce = 20; //this variable is used to help debounce the push buttons used

int noteDurations[] = {10, 10, 10, 10, 10, 10, 10, 10}; //the durations that each note is played for 


int path=1;
int i=0;
int led=12; //REMOVE 
void playback (void); //the method signature for the playback feature of the project 

void setup() 
{
Serial.begin(9600);
pinMode(buzzer,OUTPUT); //setting the LED and Buzzer as the outputs
pinMode(led,OUTPUT);

///////////////

pinMode(button1,INPUT_PULLUP); //setting the 7 piano keys(push-buttons) as inputs 
pinMode(button2,INPUT_PULLUP);
pinMode(button3,INPUT_PULLUP);
pinMode(button4,INPUT_PULLUP);
pinMode(button5,INPUT_PULLUP);
pinMode(button6,INPUT_PULLUP);
pinMode(button7,INPUT_PULLUP);

pinMode(buttonPin,INPUT_PULLUP); //REMOVE
attachInterrupt(digitalPinToInterrupt(2), button_ISR, CHANGE);
analogWrite(buzzer,0); //making sure that the buzzer is not making any noise initially 
digitalWrite(led,HIGH); //REMOVE
}

void loop() 
{
if(path==0)
{
  Serial.println("playback");
  playback(); //calling the playback mode 
}
if((millis() - previousPress) > buttonDebounce && buttonFlag)
  {
    previousPress = millis();
    if(digitalRead(buttonPin) == LOW && previousState == HIGH)
    {
      path =! path;
      previousState = LOW;
    }
    
    else if(digitalRead(buttonPin) == HIGH && previousState == LOW)
    {
      previousState = HIGH;
    }
    buttonFlag = 0;
  }
 for (int thisNote = 0; thisNote < 8; thisNote++) { //we have 7 keys on the piano, so we are looping through them all

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];

    /*Storing all the keys that are pressed while in recording mode*/
    if(digitalRead(button1)==LOW) 
    { 
      
      //analogWrite(buzzer,frequency[0]);
      tone(10, melody[0], noteDuration);
      on_time=millis();
      if(i>0)
      {
        button_offtime[i-1]=on_time-off_time;
      }
      while(digitalRead(button1)==LOW);
      if(path==1)
      {
        off_time=millis();
        button_ontime[i]=(off_time-on_time);
        button_seq[i]=0; //button 1 has been stored
        i++;
        Serial.println("button 1 stored");
      }
    }

  else if(digitalRead(button2)==LOW)
  {
    tone(10, melody[1], noteDuration);
    //analogWrite(buzzer,frequency[1]);
    on_time=millis();
    if(i!=0)
    button_offtime[i-1]=on_time-off_time;
    while(digitalRead(button2)==LOW);
    if(path==1)
    {
      off_time=millis();
      button_ontime[i]=(off_time-on_time);
      button_seq[i]=1; //button 2 has been stored
       i++;
       Serial.println("button 2 stored");
    }
  }

  else if(digitalRead(button3)==LOW)
  {
  
    tone(10, melody[2], noteDuration);
    //analogWrite(buzzer,frequency[2]);
    on_time=millis();
    if(i!=0)
    button_offtime[i-1]=on_time-off_time;
    while(digitalRead(button3)==LOW);
    if(path==1)
      {
        off_time=millis();
        button_ontime[i]=(off_time-on_time);
        button_seq[i]=2; //button 3 has been stored
         i++;
         Serial.println("button 3 stored");
      }
  }

  else if(digitalRead(button4)==LOW)
  {
    //analogWrite(buzzer,frequency[3]);
    tone(10, melody[3], noteDuration);
    on_time=millis();
    if(i!=0)
    button_offtime[i-1]=on_time-off_time;
    while(digitalRead(button4)==LOW);
    if(path==1)
      {
        off_time=millis();
        button_ontime[i]=(off_time-on_time);
        button_seq[i]=3; //button 4 has been stored
         i++;
         Serial.println("button 4 stored");
      }
  }

  else if(digitalRead(button5)==LOW)
  {
    //analogWrite(buzzer,frequency[4]);
    tone(10, melody[4], noteDuration);
    on_time=millis();
    if(i!=0)
    button_offtime[i-1]=on_time-off_time;
    while(digitalRead(button5)==LOW);
    if(path==1)
      {
        off_time=millis();
 
        button_ontime[i]=(off_time-on_time);
        button_seq[i]=4; //button 5 has veen stored
         i++;
         Serial.println("button 5 stored");
      }
  }

  else if(digitalRead(button6)==LOW)
  {
  //analogWrite(buzzer,frequency[5]);
  tone(10, melody[5], noteDuration);
  on_time=millis();
  if(i!=0)
  button_offtime[i-1]=on_time-off_time;
  while(digitalRead(button6)==LOW);
  if(path==1)
    {
      off_time=millis();
      button_ontime[i]=(off_time-on_time);
      button_seq[i]=5; //button 6 has been stored
       i++;
       Serial.println("button 6 stored");
    }
  }

  else if(digitalRead(button7)==LOW)
  {
    //analogWrite(buzzer,frequency[6]);
    tone(10, melody[6], noteDuration);
    on_time=millis();
    if(i!=0)
    button_offtime[i-1]=on_time-off_time;
    while(digitalRead(button7)==LOW);
    if(path==1)
      {
        off_time=millis();
        button_ontime[i]=(off_time-on_time);
        button_seq[i]=6; //button 7 has been stored
         i++;
         Serial.println("button 7 stored");
      }
    }
 }
//analogWrite(buzzer,0);
noTone(10); //stopping the buzzer from making any more noise 
}

/**
 * This method allows us to playback the tune that the user recorded 
 * while in recording mode. We are reading through the array called button_seq[]
 * that contains all the buttons that were pressed in the recording mode in the 
 * correct order 
 */
void playback (void)
{
 digitalWrite(led,LOW); //REMOVE
 for(int j=0; j<i; j++) //iterating through the array that stores the notes in order 
 {
  Serial.print("button in sequence: ");Serial.print(melody[button_seq[j]]);
  //analogWrite(buzzer,melody[button_seq[j]]);
  tone(10, melody[button_seq[j]], 100); //playing the note 
  delay(400);
  //noTone(buzzer);
  //delay(400);
  if(j == i-1) break;
 }
 noTone(buzzer); //ensuring that the buzzer stops making noise 
 
 i=0;
 off_time=0;
 on_time=0;
 path=1;
 digitalWrite(led,HIGH);
}




void button_ISR()
{
  buttonFlag = 1;
  
}

/*
Tone generator
v1  use timer, and toggle any digital pin in ISR
   funky duration from arduino version
   TODO use FindMckDivisor?
   timer selected will preclude using associated pins for PWM etc.
    could also do timer/pwm hardware toggle where caller controls duration
*/


// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;

volatile static int32_t toggle_count;
static uint32_t tone_pin;

// frequency (in hertz) and duration (in milliseconds).

void tone(uint32_t ulPin, uint32_t frequency, int32_t duration)
{
    const uint32_t rc = VARIANT_MCK / 256 / frequency;
    tone_pin = ulPin;
    toggle_count = 0;  // strange  wipe out previous duration
    if (duration > 0 ) toggle_count = 2 * frequency * duration / 1000;
     else toggle_count = -1;

    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)TONE_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
    }
    if (!pinEnabled[ulPin]) {
      pinMode(ulPin, OUTPUT);
      pinEnabled[ulPin] = 1;
    }
    TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
  TC_Stop(chTC, chNo);  // stop timer
  digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);
  if (toggle_count != 0){
    // toggle pin  TODO  better
    digitalWrite(tone_pin,pin_state= !pin_state);
    if (toggle_count > 0) toggle_count--;
  } else {
    noTone(tone_pin);
  }
}
