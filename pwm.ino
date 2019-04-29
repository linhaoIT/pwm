void setup() {
  // put your setup code here, to run once:
pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A1)  // non-inverting PWM on OC2A
       | _BV(COM2B0)  // PWM on OC2B:
       | _BV(COM2B1)  //    inverting mode
       | _BV(WGM20)   // fast PWM
       | _BV(WGM21);  // ditto
  TCCR2B = _BV(CS10);   // clock at F_CPU / 1
  OCR2A  = 127;          // duty cycle ~ 1/3
  OCR2B  = 127;          // same signal, complemented
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
   OCR2A  = 84;          // duty cycle ~ 1/3
  OCR2B  = 84;  
  
}
