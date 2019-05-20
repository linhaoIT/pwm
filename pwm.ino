#include <avr/io.h>
#include <avr/interrupt.h>

#define MAX 255
#define WASH 0
#define CLEAN 1
#define SPIN 2
// ADC set up
#define ANALOGUE_PIN A3
// # target voltage (5V ~ 1024, 0V~0)
#define TARGET 500

/********************************/
/*          pin config          */
/*          pin D10: PWM        */
/*          pin D11: inv PWM    */
/*          pin A3: ADC Input   */
/*          pin D4  start       */
/*          pin D5  palse       */
/*          pin D6  end         */
/*          pin D7  wash        */
/*          pin D8  clean       */
/*          pin D9  spin        */
/********************************/

/********************************/
/*         Timer config         */
/* Timer 1: finishing interrupt */
/*        Timer 2: PWM          */
/********************************/

const int timeSpan = 8;
int counter = 0;
const int washSpeed = 255;
const int cleanSpeed = 200;
const int spinSpeed = 200;
boolean isForward;
boolean interrupt;
int mode = -1;
boolean isRunning = false;
boolean forceStop = false;

int speedA;
int speedB;
int currentTime;
int firstTime = true;


const byte ledPin = 12;
const byte startInterruptPin = 2;
const byte washInterruptPin = 4;
const byte cleanInterruptPin = 5;
const byte spinInterruptPin = 6;
const byte stopInterruptPin = 7;

// integral error  
int error_I = 0;

int get_new_period(int reference, int current_speed);


void setup() {
  // put your setup code here, to run once:
  cli();
  
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  //test
  Serial.begin(9600);
  // pin change interrupt (example for D4)
  PCMSK2 |= bit (PCINT20);
  PCMSK2 |= bit (PCINT21);  // want pin 4
  PCMSK2 |= bit (PCINT22);  // want pin 4
  PCMSK2 |= bit (PCINT23);  // want pin 7
  
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7
  
  pinMode(startInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(startInterruptPin), start, FALLING);
  
  pinMode (washInterruptPin, INPUT_PULLUP);
  pinMode (cleanInterruptPin, INPUT_PULLUP);
  pinMode (spinInterruptPin, INPUT_PULLUP);
  pinMode (stopInterruptPin, INPUT_PULLUP);
  pinMode (12, OUTPUT);
    
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  //TIMSK1 |= (1 << OCIE1A);


 
  TCCR2A = _BV(COM2A1)  // non-inverting PWM on OC2A
       | _BV(COM2B0)  // PWM on OC2B:
       | _BV(COM2B1)  //    inverting mode
       | _BV(WGM20)   // fast PWM
       | _BV(WGM21);  // ditto
  TCCR2B = _BV(CS10);   // clock at F_CPU / 1
  OCR2A  = 127;          // duty cycle ~ 1/2
  OCR2B  = 127;          // same signal, complemented
  sei();
}




void loop() {
   
  while(!isRunning){
    if(mode == -1){
      isRunning = false;
    }
    delay(50);
  }
  select(mode);
  mode = -1;
  isRunning = false;
  forceStop = false;
  firstTime = true;
}

void select(int mode){
  switch(mode){
    case WASH:
      wash(washSpeed);
      if(forceStop){
        return;
      }
    case CLEAN:
      clean(cleanSpeed);
      if(forceStop){
        return;
      }
    case SPIN: 
      spin(spinSpeed);
      if(forceStop){
        return;
      }
      break;
    default:
      break;
  }
}

void wash(int speed){
  counter = 0;
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  interrupt = false;
  isForward = true;
  while(!interrupt){
    if(forceStop){
        return;
     }

     while(!isRunning){
        delay(20);
     }

     
    if(isForward){
      OCR2A  = speed;          
      OCR2B  = speed; 
      Serial.println("Washing Forward");
    }else{
      OCR2A  = MAX - speed;          
      OCR2B  = MAX - speed;
      Serial.println("Washing Backward");
    }
    
  }
    
}

void clean(int speed){
  counter = 0;
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  interrupt = false;
  isForward = true;
  while(!interrupt){
    if(forceStop){
        return;
     }

     while(!isRunning){
        delay(20);
     }
    if(isForward){
      OCR2A  = speed;          
      OCR2B  = speed; 
      Serial.println("Cleaning Forward");
    }else{
      OCR2A  = MAX - speed;          
      OCR2B  = MAX - speed; 
      Serial.println("Cleaning Backward");
    }
  }
}

void spin(int speed){
  int period;
  int current_speed;
  counter = 0;
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  interrupt = false;
  while(!interrupt){
    while(!isRunning){
        delay(20);
     }
     
    if(forceStop){
        return;
    }
    current_speed = analogRead(ANALOGUE_PIN);
    speed = get_new_period(speed, current_speed);
    OCR2A  = speed;         
    OCR2B  = speed;  
    Serial.println("Spining");
    //Serial.println(speed);
  }
}

ISR(TIMER1_COMPA_vect){
    if(counter < timeSpan/2){
      counter++;
    }else if(counter < timeSpan){
      isForward = false;
      counter++;
    }else{
      interrupt = true;
    }
}

/****************/
/* pin config       */
/* pin D4  start*/
/* pin D5  palse*/
/* pin D6  end*/
/* pin D7  wash*/
/* pin D8  clean*/
/* pin D9  spin*/
/****************/

ISR (PCINT2_vect)
{
  // handle pin change interrupt for D0 to D7 here
  if (PIND & bit (4)) {
   mode = WASH;
  }
  else if (PIND & bit (5)) {
   mode = CLEAN;
  }
  else if (PIND & bit (6)) {
   mode = SPIN;
  }
  else if (PIND & bit(7)) {
    Serial.println("Kill");
    mode = -1;
    isRunning = false;
    OCR2A  = 127;          // duty cycle ~ 1/2
    OCR2B  = 127;          // same signal, complemented
    forceStop = true;
  }

  
}    // end of PCINT2_vect




void start() {
  cli();
  delay(1000);
  if(!isRunning){
    Serial.println("Start...");
    isRunning = true;
    if(!firstTime){
      OCR2A = speedA;
      OCR2B = speedB;
      TCNT1 = currentTime;
      //turn on the timer
      TCCR1B |= (1 << CS12) | (1 << CS10);  
      TIMSK1 |= (1 << OCIE1A);
    }
  }else{
    //todo
    //stop the watch
    Serial.println("Palse...");
    isRunning = false;
    speedA = OCR2A;          // duty cycle ~ 1/2
    speedB = OCR2B;
    currentTime = TCNT1;
    firstTime = false;
    OCR2A = 127;
    OCR2B = 127;
    TCCR1B &= B11111010; 
    TIMSK1 &= B11111101; 
    //turn off the timer
  }
  sei();
  
}

void end() {
//  digitalWrite(ledPin, HIGH);
  Serial.println("Kill");
  mode = -1;
  isRunning = false;
  OCR2A  = 127;          // duty cycle ~ 1/2
  OCR2B  = 127;          // same signal, complemented
  forceStop = true;
}

// the closed loop control only works in one direction
// Pin3 -> Q2&3       Pin 11 -> Q1&4
int get_new_period(int reference, int current_speed){
  reference = TARGET;
  double k_P;     // proportional gain
  double k_I;     // integral gain
  int offset;     // off set used in proportional gain
  int error_P;    // proportional error
  int period;     // new period returned to PWM module

// P control
  k_P = 0.1;
  offset = 0;
  error_P = reference - current_speed;

  // I control
  k_I = 0.05;
  // manual overflow if error has gone too big
  if(error_I >= 10000 || error_I <= -10000){
    error_I = 0;
  }
  else{
    // calcuculate integral error
    error_I += (error_P/10);
  }
  
  // calculate correction factor
  period = 127 + error_P * k_P + error_I * k_I;

  // manual limits the range of dutu cycle
  if(period>255){
    period = 255; 
  }
  else if(period < 0){
    period = 0;
  }
  return period;
}
