#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"
#include "isr.h"
#include "defines.h"
#include "buffer.h"

/** Boolean for emergancy stop */
volatile bool stop = false;

/** Current position of all servos */
volatile word_t servoAngles[18] = {
  0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF
  , 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF
  , 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x01FF};
/** Lookup table for servo ids */
volatile byte_t servoID[18] = {
  1, 3, 5, 13, 15, 17, 7, 9, 11,
  8, 10, 12, 14, 16, 18, 2, 4, 6};
/** Reverse lookup tablefor servo ids */
volatile byte_t servoIDReverse[18] = {
  0, 15, 1, 16,  2, 17, 6, 9, 7,
  10, 8, 11, 3, 12, 4, 13, 5, 14};



/**
 * Initalizes the uart which communicates with the robot servos.
 * @Param baud - UART baud rate
 */
void initServoUART( unsigned int baud ){
  // Set baud rate
  UBRR0H = (unsigned char)(baud>>8);
  UBRR0L = (unsigned char)baud;

  // Set frame format: 8data, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  // Enable receiver, transmitter and interrupts
  UCSR0B = (1 << TXEN0) | (1  << TXCIE0) | (1<<RXCIE0) |  (1<<RXEN0);
  // Double the uart baud rate
  UCSR0A = (1 << U2X0);
  // Init uart
  initUart(&servoUart);
}

/**
 * Initalizes the uart which communicates with the Beagle board
 * @Param baud - UART baud rate
 */

void initBeagleUart( unsigned int baud ){
  // Set baud rate
  UBRR1H = (unsigned char)(baud>>8);
  UBRR1L = (unsigned char)baud;

  // Set frame format: 8data, 1 stop bit
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  // Enable receiver, transmitter and interrupts
  UCSR1B = (1 << TXEN1) | (1  << TXCIE1) | (1<<RXCIE1) |  (1<<RXEN1);
  // Double the uart baud rate
  UCSR1A = (1 << U2X1);
  // Set uart
  initUart(&beagleUart);
}

/**
 * Initiates a timer that sets flag which tells the program to send a data
 * packet to the beagleboard.
 */
void initTimer3() {
  // Enable Interrupt TimerCounter3 Compare Match A (TIMER3_COMPA_vect)
  TIMSK3 = (1<<OCIE3A);
  // Clock/1024, 0.000064 seconds per tick, mode=CTC
  TCCR3B = (1<<CS32) | (1<<CS30) | (1<<WGM32);
  //Define the top value for the counter, interrupt every 192ms
  OCR3A=3000;
}

/**
 * Initiaties a timer that set a flag which tells the program to read data from
 * a servo.
 */
void initTimer1() {
  // Enable Interrupt TimerCounter3 Compare Match A (TIMER3_COMPA_vect)
  TIMSK1 = (1<<OCIE1A);
  // Clock/1024, 0.000064 seconds per tick, mode=CTC
  TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);
  // Define the top value for the counter, interrupt every 32 ms
  OCR1A=500;
}

/**
 * Initiates a timer that is used as a timeout for beagleboard uart
 * communication.
 */
void initTimer0() {
  // Enable Interrupt TimerCounter0 Compare Match A (TIMER0_COMPA_vect)
  TIMSK0 = (1<<OCIE0A);
  // Mode = CTC
  TCCR0A = (1<<WGM01);
  //Define the top value for the counter, interrupt every 12ms~
  OCR0A=200;
}

/**
 * Initiates a timer that is used as a timeout for servo uart communication.
 */
void initTimer2() {
  // Enable Interrupt TimerCounter0 Compare Match A (TIMER0_COMPA_vect)
  TIMSK2 = (1<<OCIE2A);
  // Mode = CTC
  TCCR2A = (1<<WGM21);
  //Define the top value for the counter, interrupt every 4~ms
  OCR0A=60;
}

/**
 * Initalizes the avr by configuring ports, interrupts, and UARTs.
 */
void init(){

  //Disable interrupts
  cli();

  //Set packet sizes
  instr_size[SET_SERVO]      = 38;
  instr_size[SET_MAX_TORQUE] = 4;
  instr_size[SET_SPEED]      = 4;
  instr_size[SET_ENABLE]     = 3;
  instr_size[SET_ID]         = 3;
  instr_size[SET_LED]        = 20;
  instr_size[SET_DISP]       = 34;

  //Set port directions
  DDRA  = 0;
  DDRB  = 0;
  DDRC  = (1 << PC0);
  DDRD  = (1<<servo_dir) | (1<<CTS);
  //Set port values
  PORTA = 0xFF;
  PORTB = 0xFF;
  PORTC = 0xFF;
  PORTD = 0xFF;
  PORTD &= ~(1 << servo_dir);

  //Enable button interrupt
  externalInterruptEnable();

  //Init timers
  initTimer0();
  initTimer2();
  initTimer1();
  initTimer3();

  //Init uarts 1 = 1M bps
  initServoUART(1);
  initBeagleUart(25);
  //Enable interrupts
  sei();
}

/**
 * Setups the header for a servo packet.
 * @Param size - The total size for the packet.
 * @Param id - The servo id.
 * @Param instr - The instruction id.
 * @Return Returns a pointer to a newly allocated packet.
 */
byte_t* setupServoPack(byte_t size, byte_t id, byte_t instr){
  //Allocate memory
  cli();
  byte_t* pack = (byte_t*)malloc(sizeof(byte_t)*size);
  sei();
  //If malloc passed setup package
  if(pack != NULL){
    pack[0] = 0xFF;
    pack[1] = 0xFF;
    pack[2] = id;
    pack[3] = size- 4;
    pack[4] = instr;
    return pack;
  }
  return NULL;
}

/**
 * Update all servo angles by sending a syncwrite pacakge.
 * @Param angles - An array of 18 integers.
 */
void updateServos(int angles[18]){
  byte_t* pack = setupServoPack(62, BROADCASTING_ID, INST_SYNC_WRITE);
  if(pack == NULL){
    return;
  }
  //Add parameters

    //Dont move servos to far
    if (n%3 == 0){
      angles[n] = fmax(fmin(840, angles[n]), 230);
    }else{
      angles[n] = fmax(fmin(740, angles[n]), 180);
    }
    //Store id and position
    pack[7 + i] = servoID[n];
    pack[8 + i] = (byte_t)(angles[n]);
    pack[9 + i] = (byte_t)(angles[n] >> 8);
  }

  //Checksum
  pack[61] = calcServoCheckSum(pack, 61);
  
  byte_t* txStruct = (byte_t*)newTxPackage(62, pack, false);
  if(txStruct == NULL){
    PORTB |= (1 << PB0);
    cli();
    free(pack);
    sei();
  }else{
    if (bufferInsert(&servoUart.txBuffer, txStruct)){
      UCSR0B |= (1 << UDRIE0);
    }else{
      cli();
      free(pack);
      free(txStruct);
      sei();
    }
  }
}

/**
 * Sends a broadcast to all servos.
 * @Param noParameters - Number of parameters.
 * @Param instr - Instruction id.
 * @Param parameters - A pointer to the parameters.
 */
void sendServoBroadcast(byte_t noParameters, byte_t instr, byte_t* parameters){
  byte_t* pack = setupServoPack(6+noParameters, BROADCASTING_ID, instr);
  if(pack == NULL){
    return;
  }
  //Add parameters
  for(int i = 0; i < noParameters; i++){
    pack[i + 5] = parameters[i];
  }
  //Checksum
  pack[noParameters + 5] = calcServoCheckSum(pack, noParameters + 5);

  byte_t* txStruct = (byte_t*)newTxPackage(noParameters + 6, pack, false);
  if (txStruct == NULL){
    PORTB |= (1 << PB0);
    cli();
    free(pack);
    sei();
  }else{
    if(bufferInsert(&servoUart.txBuffer, txStruct)){
      UCSR0B |= (1 << UDRIE0);
    }else{
      cli();
      free(pack);
      free(txStruct);
      sei();
    }
  }
}


/**
 * External button interrupts, disables interrupts
 */
ISR (PCINT3_vect){
  byte_t parameters[2] = {P_TORQUE_ENABLE ,0};
  sendServoBroadcast(2, INST_WRITE, parameters);
  stop = true;
/**
 * External button interrupts, disables interrupts
 */
ISR (PCINT3_vect){
  byte_t parameters[2] = {P_TORQUE_ENABLE ,0};
  sendServoBroadcast(2, INST_WRITE, parameters);
  stop = true;

ISR (PCINT3_vect){
  byte_t parameters[2] = {P_TORQUE_ENABLE ,0};
  sendServoBroadcast(2, INST_WRITE, parameters);
  stop = true;
}

int main (void){

  init();

  PORTD &= ~(1 << CTS);

  byte_t idCounter = 0;
  byte_t* data;
  byte_t beaglePacket[38];
  struct txPackage beagleTx;

  while(1){

    /***********************************/
    /* Handle data sent by beagleboard */
    /***********************************/
    if(!bufferEmpty(&beagleUart.rxBuffer) && !stop){
      //Get data
      data = (byte_t*)bufferHead(&beagleUart.rxBuffer);
      byte_t checkSum = data[1];
      switch(data[0]){
      case SET_SERVO:
        if (checkSum == (((calcCheckSum(data+2, 36) + data[0])) & 0xFF)){
          updateServos((int*)(data+2));
        }
        cli();
        free(data);
        break;
      case SET_MAX_TORQUE:
        if (checkSum == ((data[0] + data[2] + data[3]) & 0xFF)){
          data[1] = P_TORQUE_LIMIT_L;
          sendServoBroadcast(3, INST_WRITE, data+1);
        }
        cli();
        free(data);
        break;
      case SET_SPEED:
        if (checkSum == ((data[0] + data[2] + data[3]) & 0xFF)){
          data[1] = P_MOVING_SPEED_L;
          sendServoBroadcast(3, INST_WRITE, data+1);
        }
        cli();
        free(data);
        break;
      case SET_ENABLE:
        if (checkSum == ((data[0] + data[2]) & 0xFF)){
          data[1] = P_TORQUE_ENABLE;
          sendServoBroadcast(2, INST_WRITE, data+1);
        }
        cli();
        free(data);
        break;
      case SET_ID:
        cli();
        free(data);
        break;
      case SET_LED:
        cli();
        free(data);
        break;
      case SET_DISP:
        cli();
        free(data);
        break;
      default:
        cli();
        free(data);
      }
      sei();
    }


    /****************************/
    /* Send data to beagleboard */
    /****************************/
    if (sendData){
      //Setup tx packet
      beagleTx.data = beaglePacket;
      beagleTx.index = 0;
      beagleTx.response = false;
      beagleTx.size = 38;
      //Magic constant
      beaglePacket[0] = 0xBB;

      //Servo angles
      for(int i = 0; i < 35; i+=2){
        beaglePacket[i+2] = (byte_t)servoAngles[i/2];
        beaglePacket[i+3] = (byte_t)(servoAngles[i/2] >> 8);
      }
      //Checksum
      beaglePacket[1] = 0;
      beaglePacket[1] = calcCheckSum(beaglePacket, 38);

      bufferInsert(&beagleUart.txBuffer, (byte_t*)&beagleTx);
      UCSR1B |= (1 << UDRIE1);
      sendData = false;
    }


    /*******************************/
    /* Handle data sent by a servo */
    /*******************************/
    if(!bufferEmpty(&servoUart.rxBuffer)){
      struct statusPackage* package = (struct statusPackage*)bufferHead(&servoUart.rxBuffer);
      //Update servo position
      servoAngles[servoIDReverse[package->id - 1]] = (*(package->parameters)) +
        ((int)(*(package->parameters + 1)) << 8);

      //Deallocate memory
      cli();
      free(package->parameters);
      free(package);
      sei();
    }


    /****************************/
    /* Request data from servos */
    /****************************/
    if (readData){
      //Setup packet
      data = setupServoPack(8, servoID[idCounter], INST_READ);
      if(data != NULL){
        data[5] = P_PRESENT_POSITION_L;
        data[6] = 0x02;
        //Checksum
        data[7] = calcServoCheckSum(data, 7);
        //Send
        byte_t* txStruct = (byte_t*)newTxPackage(8, data, true);
        if(txStruct == NULL){
          cli();
          free(data);
          sei();
        }else{
          if(bufferInsert(&servoUart.txBuffer, txStruct)){
            UCSR0B |= (1 << UDRIE0);
            idCounter = (idCounter +1)%18;
          }else{
            cli();
            free(data);
            free(txStruct);
            sei();
          }
        }
      }
      //Update variables
      readData = false;
    }
  }
  return 1;
}
