#define MAX 255
#define WASH 0
#define CLEAN 1
#define SPIN 2

int timeSpan = 4;
int counter = 0;
int washSpeed = 100;
int cleanSpeed = 100;
int spinSpeed = 100;
boolean isForward = true;
boolean interrupt = false;

void setup() {
  // put your setup code here, to run once:
  cli();
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  
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
  select(WASH);
  
}

void select(int mode){
  switch(mode){
    case WASH:
      wash(washSpeed, isForward);
    case CLEAN:
      interrupt = false;
      wash(cleanSpeed, isForward);
    case SPIN:
    interrupt = false;
      wash(spinSpeed, isForward);
      break;
    default:
      break;
  }
}

void wash(int speed, int isForward){
  TIMSK1 |= (1 << OCIE1A);
  while(interrupt){
    if(isForward){
      OCR2A  = speed;          
      OCR2B  = speed; 
    }else{
      OCR2A  = MAX - speed;          
      OCR2B  = MAX - speed; 
    }
    
  }
    
}

void clean(int speed, int isForward){
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  while(interrupt){
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
  TCNT1  = 0;//initialize counter value to 0
  TIMSK1 |= (1 << OCIE1A);
  while(interrupt){
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

//const byte ledPin = 13;
//const byte interruptPin = 2;
//volatile byte stop = LOW;

//void setup() {
//  pinMode(interruptPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin), stopISR, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(interruptPin), stopISR, CHANGE);
//}
//
//void stopISR() {
//  stop = !stop;
//  while(stop){
//    delay(200);
//  }
//  
//}
