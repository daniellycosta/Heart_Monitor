#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define START_ADC _BV(ADSC) //Use OR
#define STOP_ADC !(_BV(ADSC)) //Use AND

#define START_CONT _BV(TOIE2)
#define STOP_CONT !(_BV(TOIE2))

#define CONT_MAX 0x1

#define BAUDRATE 9600
#define BAUD_PRESCALLER 16000000/16/BAUDRATE - 1

void intUSART();
void txByte(uint8_t info);
uint8_t  rxByte();
void USART_Flush(void);

volatile uint16_t ADCReading = 0;
volatile uint16_t PWM_Value = 1023;
volatile bool sendData = false;
volatile bool circuitoEnable = false;
volatile uint16_t counter = 0;
 
ISR(ADC_vect){
  if(circuitoEnable){
  ADCReading = ADC;
  PWM_Value = ADC;
  PORTB = ~(0xFF<<(ADC/127));
  ADCSRA |= START_ADC;
  }
}

ISR(TIMER2_OVF_vect){
  if(circuitoEnable){
  counter++;
  if(counter>CONT_MAX){   //Send data every 2ms
    sendData = true;
    counter = 0;
    ADCSRA |= START_ADC; 
  }
}
}

ISR(TIMER1_OVF_vect)
{
  if(circuitoEnable){
    OCR1A = PWM_Value;  
}
}

ISR(INT1_vect){
  circuitoEnable = !circuitoEnable;
  if(circuitoEnable){
    
    ADCSRA |= _BV(ADEN)| _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1)| _BV(ADPS0);
    ADCSRA |= START_ADC;
   
   //starts ADC reading
    TCNT2 &= 0x00;             //Reset counter
    TIMSK2 |= START_CONT;     //Starts counting (enables overflow interruption)
    counter = 0;
    
  }else{
    ADCSRA &= 0x00;
    TIMSK2 &= STOP_CONT;    //disable overflow interruption
  }
}

int main(){
  unsigned char *chptr;
  intUSART();
//ADC settings
  ADMUX &= 0x00;
  ADMUX |= _BV(MUX0);      //Using ADC1

//counter settings
  TCCR2A &= 0x00;
  
  TCCR2B &=0x00;
  TCCR2B |= _BV(CS22);   //prescale = 64

  TIMSK2 &= 0x00;

//Led - Out(on/off)
  DDRD &= 0x00;
  DDRD |=_BV(DDD7); 

//Led - Out (PWM)
  DDRB &= 0x00;
  DDRB |= _BV(DDB1); 

  TCCR1A = (1<<COM1A1)|(1<<WGM11)|(1<<WGM10);
  TCCR1B = (1<<WGM12)|(1<<CS10);
  OCR1A = 0;  // LED initially off
  TIMSK1 = 1<<TOIE1;
  
  PORTB &= 0x00;
  PORTD &= 0x00;

//external interrupt
  EICRA &= 0x00;
  EICRA|= _BV(ISC10)|_BV(ISC11); //rising edge
  
  EIMSK &=0x00;
  EIMSK|= _BV(INT1); //activates external interrupt pin
  
  PCICR&=0x00;
  PCICR |= _BV(PCIE2); //activates int2 interruption

  PCMSK1&=0x00;
  PCMSK1|=_BV(PCINT18); //using pin 3 for activating the interruption

  while(1){
    sei();
    while(circuitoEnable){
       PORTD |= _BV(PORTD7);
       if(sendData){        
        chptr = (unsigned char *) &ADCReading;
        //Sending 16bits
            txByte(*chptr++);
            txByte(*chptr);    
      }  
    }
    if(!circuitoEnable){
       PORTD &= !(_BV(PORTD7));
    }
  }
}

void intUSART(){
  // 38.4kbps
  UBRR0L = BAUD_PRESCALLER;
  UBRR0H = (BAUD_PRESCALLER << 8);
  // UCSZ2 = 0 - 8 bits
  UCSR0B |= _BV(RXEN0)|_BV(TXEN0);
  // UCSZ1 = 1 e UCSZ0 = 1 -> 8 bits
  // USBS0 = 0 -> 1 - stop bits
  // UPM0 = 0 e UMP0 = 0 -> without parity bit
  UCSR0C |= _BV(UCSZ01)|_BV(UCSZ00);
}

void txByte(uint8_t info){
  // Data transmission
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = info;
}

uint8_t  rxByte(){
  while(!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

void USART_Flush( void ){
 unsigned char dummy;
 while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}