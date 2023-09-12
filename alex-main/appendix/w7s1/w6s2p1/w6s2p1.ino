#include <avr/io.h>
#include <avr/interrupt.h>


// Masks for pins 12 and 11
#define PIN12MASK   0b00010000
#define PIN11MASK   0b00001000

static volatile char flashWhich=1;
static volatile char buttonVal=1;


void sendData(const char data)
{
  // Poll UDRE bit in UCSR0A until it is 1
  // Send data
  while((UCSR0A & (1 << 5)) == 0);
  UDR0 = data;
}

char recvData()
{
  // If RXC0 bit is 0, return 0
  // Otherwise return contents of UDR0 - '0'
  if((UCSR0A & (1 << 7)) == 0) return 0;
  return UDR0 - '0';
}

void setupSerial()
{
  // Set up for 115200 8N1 for testing with Serial Monitor
  // Change to 115200 7E1 when communicating with another
  // Arduino
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0A = 0;
  //UCSR0C = 0b00000110;
  UCSR0C = 0b00100100;
}

void startSerial()
{
  // Start the transmitter and receiver, but disable
  // all interrupts.
  UCSR0B = 0b00011000;
  cli();
}

// Enable external interrupts 0 and 1
void setupEINT()
{
  // Configure INT0 and INT1 for rising edge triggered.
  // Remember to enable INT0 and INT1 interrupts.
  EICRA |= 0b1111;
  EIMSK |= 0b11;
  sei();
}

// ISRs for external interrupts
ISR(INT0_vect)
{
  buttonVal=1;  
  sendData('0'+buttonVal);
}

ISR(INT1_vect)
{
  buttonVal=2;
  sendData('0'+buttonVal);
}

// Red is on pin 12
void flashRed()
{
    PORTB |= PIN12MASK;
    delay(100);
    PORTB &= ~PIN12MASK;
    delay(500);
}

// Green is on pin 11
void flashGreen()
{
    PORTB |= PIN11MASK;
    delay(100);
    PORTB &= ~PIN11MASK;
    delay(500);
}

void setup() {

  cli();
  // put your setup code here, to run once:

  DDRB |= (PIN11MASK | PIN12MASK);
  setupEINT();
  setupSerial();
  startSerial();
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:

  char data = recvData();

  if(data != 0)
    flashWhich = data;
    
  if(flashWhich == 2)
    flashGreen();
    //flashRed();
  else
    flashRed();
    // flashGreen();
}
