#include <avr/io.h>
#include <avr/interrupt.h>

#define MAX 255


/********************************/
/*          pin config          */
/*          pin D3: PWM         */
/*          pin D11: inv PWM    */
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


const int timeSpan = 4;
int counter = 0;
const int washSpeed = 20;
const int cleanSpeed = 70;
const int spinSpeed = 120;
boolean isForward;
boolean interrupt;
int mode = 0;
boolean start = false;

const byte startButtonInterruptPin = 4;
const byte palseWashButtonInterruptPin = 5;
const byte endWashButtonInterruptPin = 6;
const byte washButtonInterruptPin = 7;
const byte cleanWashButtonInterruptPin = 8;
const byte spinButtonInterruptPin = 9;
volatile byte stop = LOW;


void setup() {
  // put your setup code here, to run once:
  cli();
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);

  EICRA &= ~(bit(ISC00) | bit (ISC01));  // clear existing flags
  EICRA |=  bit (ISC01);    // set wanted flags (falling edge interrupt)
  EIFR   =  bit (INTF0);    // clear flag for interrupt 0
  EIMSK |=  bit (INT0);     // enable it

  pinMode (startButtonInterruptPin, INPUT_PULLUP);
  pinMode (palseWashButtonInterruptPin, INPUT_PULLUP);
  pinMode (endWashButtonInterruptPin, INPUT_PULLUP);
  pinMode (washButtonInterruptPin, INPUT_PULLUP);
  pinMode (cleanWashButtonInterruptPin, INPUT_PULLUP);
  pinMode (spinButtonInterruptPin, INPUT_PULLUP);
  
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
  OCR2A  = 127;          // duty cycle ~ 1/3
  OCR2B  = 127;          // same signal, complemented
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(start && mode != -1){
    start = false;
  }
  select(mode);
  mode = -1;
}

void select(int mode){
  switch(mode){
    case WASH:
      wash(washSpeed);
    case CLEAN:
      clean(cleanSpeed);
    case SPIN: 
      spin(spinSpeed);
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
    if(isForward){
      OCR2A  = speed;          
      OCR2B  = speed; 
    }else{
      OCR2A  = MAX - speed;          
      OCR2B  = MAX - speed; 
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
    if(isForward){
      OCR2A  = speed;          
      OCR2B  = speed; 
    }else{
      OCR2A  = MAX - speed;          
      OCR2B  = MAX - speed; 
    }
  }
}

void spin(int speed){
  counter = 0;
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  interrupt = false;
  while(!interrupt){
    OCR2A  = speed;         
    OCR2B  = speed;  
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
ISR (INT0_vect){
  if (PIND & bit (4)){
  start = true;
 }else if (PIND & bit (5)){
  start = false;
  
   while(!start){
    
   }
 }else if(PIND & bit (6)){
   
 }else if(PIND & bit (7)){
  mode = 0;
 }else if(PIND & bit (8)){
  mode = 1;
 }else if(PIND & bit (9)){
  mode = 2;
 }
}
