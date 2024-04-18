int OCRA;

void setupTimer3(){
  cli();
  //reset timer3 to blank condition
  //TCR = Timer/Counter control Register
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B = TCCR3B |(1 << WGM32);
  TCCR3B = TCCR3B |(1 << CS32);
  OCRA = 626;//100hz
  TIMSK3 = TIMSK3| (1<<OCIE3A);
  }
