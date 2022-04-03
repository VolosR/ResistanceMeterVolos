#include <HT1621.h> // include our library
HT1621 lcd;


int raw = 0;
float Vin = 0;
float Vout = 0;
float R1 = 1174;
float R2 = 0;
float buffer = 0;


void setup() {
  lcd.begin(12, 11, 10); 
  lcd.clear(); 

  pinMode(3,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);

}

int d1=0;
int bat=0;

void loop() {

    int v=analogRead(A7);
    analogWrite(9,v/4);
  Vin=readVcc()/1000.00;
  raw = analogRead(A6);
  if(raw){
    buffer = raw * Vin;
    Vout = (buffer)/1024.0;
    buffer = (Vin/Vout) - 1;
    R2= R1 * buffer;
   
    delay(100);
  }

   if(digitalRead(2)==0)
   {
    if(d1==0)
     {
       d1=1;
       bat++;
       if(bat>3)
       bat=0;
       lcd.setBatteryLevel(bat);
       
      }
   }else d1=0;
  
  if(digitalRead(3)==0)
  lcd.print(Vin, 2);
  else
  lcd.print(R2, 1);

}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
