#include <avr/io.h>
#include <avr/interrupt.h>

#define MAX 255
#define WASH 0
#define CLEAN 1
#define SPIN 2

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
const int cleanSpeed = 50;
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
const byte stopInterruptPin2 = 3;
const byte washInterruptPin = 4;
const byte cleanInterruptPin = 5;
const byte spinInterruptPin = 6;

int get_new_period(int reference, int current_speed);


void setup() {
  // put your setup code here, to run once:
  cli();
  
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  //test
  Serial.begin(9600);
  // pin change interrupt (example for D4)
  PCMSK2 |= bit (PCINT20);
  PCMSK2 |= bit (PCINT21);  // want pin 4
  PCMSK2 |= bit (PCINT22);  // want pin 4
  
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7
  
  pinMode(startInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(startInterruptPin), start, FALLING);
  pinMode(stopInterruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stopInterruptPin2), end, FALLING);

  pinMode (washInterruptPin, INPUT_PULLUP);
  pinMode (cleanInterruptPin, INPUT_PULLUP);
  pinMode (spinInterruptPin, INPUT_PULLUP);
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
  current_speed = analogRead(A3);
  while(!interrupt){
    while(!isRunning){
        period = get_new_period(speed, current_speed);
     }
     
    if(forceStop){
        return;
    }
    OCR2A  = speed;         
    OCR2B  = speed;  
    Serial.println("Spining");
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

  
}    // end of PCINT2_vect




void start() {
  cli();
  delay(50);
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

int get_new_period(int reference, int current_speed){
  int k_P;
  int offset;
  int error;
  int period;
  
  k_P = 10;
  offset = 0;

  error = current_speed-reference;

  period = 127 + error * k_P;

  if(period>255){
    period = 255; 
  }
  else if(period < 0){
    period = 0;
  }
  return period;
}
