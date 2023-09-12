#include <serialize.h>
#include <stdarg.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4,
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV_LEFT       191
#define COUNTS_PER_REV_RIGHT      181
//define COUNTS_PER_REV_AVG        (COUNTS_PER_REV_LEFT + COUNTS_PER_REV_RIGHT)/2
//#define FACTOR_LEFT               COUNTS_PER_REV_LEFT/COUNTS_PER_REV_AVG
//#define FACTOR_RIGHT              COUNTS_PER_REV_RIGHT/COUNTS_PER_REV_AVG
#define COUNTS_PER_REV_AVG	186
#define FACTOR_LEFT 		1.02688172043
#define FACTOR_RIGHT		0.97311827957
#define TURNS_OFFSET 0.5

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.106

//Alex dimension

#define LENGTH 20
#define BREADTH 16

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  OCR0A //6   // Left forward pin
#define LR                  OCR0B //5   // Left reverse pin
#define RF                  OCR1A //9  // Right forward pin
#define RR                  OCR1B //10  // Right reverse pin

//Color Sensor Things
#define S0 7
#define S1 4
#define S2 12
#define S3 11
#define LED 8
#define LOUT 13

typedef enum
{
  RED,
  GREEN
} TColor;

int colorMin[2] = {70, 100};
int colorMax[2] = {120, 199};



#define MAX_BUF_SIZE 200

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

//Bearings

float diagonal = 0.0;
float circ = 0.0;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...){
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendColor(TColor color)
{
  TPacket colorPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_COLOR;
  statusPacket.params[0] = color == RED ? 0 : 1;
  sendResponse(&statusPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch(dir){
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV_LEFT * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV_LEFT * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break; 
  }
}

void rightISR()
{
  switch(dir){
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break; 
  } 
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
//  UCSR0A = 0;
//  UCSR0C = 0b00000110;
//  UBRR0L = 103;
//  UBRR0H = 0;
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
//  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.
int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
   Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  DDRD = (1 << 5) | (1 << 6);
  DDRB = (1 << 1) | (1 << 2);
  
  TCCR0A = 0b10100001;
  TCCR0B = 0b00000001;
  TCNT0 = 0;
  
  TCCR1A = 0b10100001;
  TCCR1B = 0b00000001;
  TCNT1 = 0;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  deltaDist = (dist == 0) ? 9999999 : dist;
  newDist = forwardDist + deltaDist;
  dir = FORWARD;

  LF = pwmVal(speed * FACTOR_LEFT * 0.9);
  RF = pwmVal(speed * FACTOR_RIGHT);
  LR = 0;
  RR = 0;
}


// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  deltaDist = (dist == 0) ? 9999999 : dist;
  newDist = reverseDist + deltaDist;
  dir = BACKWARD;

  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  LR = pwmVal(speed * FACTOR_LEFT);
  RR = pwmVal(speed * FACTOR_LEFT);
  //LR = val;
  //RR = val;
  LF = 0;
  RF = 0;
}

unsigned long computeDeltaTicks(float ang){
 return (unsigned long) (TURNS_OFFSET * (ang * circ * COUNTS_PER_REV_AVG) / (360.0 * WHEEL_CIRC)); 
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  deltaTicks = (ang == 0) ? 99999999 : computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  dir = LEFT;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  LR = pwmVal(speed * FACTOR_LEFT);
  RF = pwmVal(speed * FACTOR_RIGHT);
  //LR = val;
  //RF = val;
  LF = 0;
  RR = 0;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  deltaTicks = (ang == 0) ? 99999999 : computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  dir = RIGHT;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  LF = pwmVal(speed * FACTOR_LEFT);
  RR = pwmVal(speed * FACTOR_RIGHT);
  //RR = val;
  //LF = val;
  LR = 0;
  RF = 0;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  LF = 0;
  LR = 0;
  RF = 0;
  RR = 0;
}


// =============== COLOR Sensing ================

/**
 * Minor documentation regarding Colour Sensor
   * Red Colour Sensing: S2 is low, S3 is low
   * Blue Colour Sensing: S2 is low, S3 is high
   * Green Colour Sensing: S2 is high, S3 is high
   * To switch off, set S0 and S1 to low
   * To switch on (20% freq scaling), set S0 is H and S1 is L
*/


int readColor(int idx, int s2val, int s3val) {
  digitalWrite(S2, s2val);
  digitalWrite(S3, s3val);
  int freq = pulseIn(sensorOut, LOW);
  return map(freq, colorMin[idx], colorMax[idx], 255, 0);
}

TColor detectColor() {
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  int redReading = readColor(i, LOW, LOW);
  int greenReading = readColor(i, HIGH, HIGH);
  
  if(redReading >= greenReading) {
    // Red
    return RED;
  } else {
    // Green
    return GREEN;
  }
  
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
}












/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch(which){
    case 0: clearCounters(); break;
    case 1: leftForwardTicks=0; break;
    case 2: rightForwardTicks=0; break;
    case 3: leftReverseTicks=0; break;
    case 4: rightReverseTicks=0; break;
    case 5: leftForwardTicksTurns=0; break;
    case 6: rightForwardTicksTurns=0; break;
    case 7: leftReverseTicksTurns=0; break;
    case 8: rightReverseTicksTurns=0; break;
    case 9: forwardDist=0; break;
    case 10: reverseDist=0; break;
    default: clearCounters();
  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_GET_STATS:
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;
    case COMMAND_GET_COLOR:
      TColor color = detectColor();

    
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setupColor() {
  DDRB |= 1;
  PORTB &= ~1;
}

void setup() {
  // Setup bearings
  diagonal = sqrt(LENGTH * LENGTH + BREADTH * BREADTH);
  circ = PI * diagonal;
  
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  setupColor();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 50);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  if (deltaDist > 0){
    switch(dir){
      case FORWARD:
        if (forwardDist >= newDist){
          deltaDist = 0;
          newDist = 0;
          stop();
        }
        break;
      case BACKWARD:
        if (reverseDist >= newDist){
          deltaDist = 0;
          newDist = 0;
          stop();  
        }
        break;
    }
  }
  if (deltaTicks > 0){
    switch(dir){
      case LEFT:
        if (leftReverseTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
        break;
      case RIGHT:
        if (rightReverseTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();  
        }
    }
  }
  if (dir == STOP){
    deltaTicks = 0;
    targetTicks = 0;
    stop();
  }
}
