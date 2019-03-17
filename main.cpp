#include "mbed.h"
#include "Crypto.h"
#include "SHA256.h"
#include "PwmOut.h"
#include "stdlib.h"
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0
    
//Declare and initialise the input sequence and hash 
uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48); 
uint64_t* nonce = (uint64_t*)((int)sequence + 56); 
uint8_t hash[32];
int counter = 0;

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
//const int8_t lead = 2;  //2 for forwards, -2 for backwards
int8_t lead = 2;
// string output for user friendly representation -> can remove
string movementDirection;

//Mailbox structure for key
typedef struct {
    uint64_t* data;    
} mail_t;

//Mailbox structure for motor control
typedef struct {
    string* data;
} motor_t;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Thread initialisations
Thread thread; // declare stack size?
Thread inputThread; // declare stack size?
Thread motorCtrlT(osPriorityNormal,1024);

// use mutex to prevent simultaneous access
Mutex newKey_mutex;
Mutex dutyCycle_mutex;

// add volatile so that value can change between threads
uint64_t newKey;
char inputKey[18];
string receivedDutyCycle;
string dutyCycle;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

// create a global instance of Queue<void, 8> to buffer incoming characters or strings
Queue<void, 8> inCharQ;

Mail<mail_t, 16> mail_box;
Mail<motor_t, 16> mail_box_motor;

//MBED timer class
Timer t; 

// PWMOut class
PwmOut motor(D9);

//Rotor offset at motor state 0
int8_t orState = 0;    
int8_t intState = 0;
int8_t intStateOld = 0;
int8_t stateChange = 0;
int8_t velocity = 0;
int iterCount = 0;

//Create an instance of the class SHA256 
SHA256 sha256;

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
    inline int8_t readRotorState(){
        return stateMap[I1 + 2*I2 + 4*I3];
    }

    //Basic synchronisation routine    
    int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

// new photointerrupt ISR
void motorPosition() {
    intStateOld = intState;
    intState = readRotorState();
    if (intState != intStateOld) {
        stateChange = intState - intStateOld;
        if(intState > intStateOld){
            // state has increased -> positive direction
            movementDirection = "positive"; 
            lead = 2;
        }
        else{
            // state has decreased -> negative(?) direction
            movementDirection = "negative";
            lead = -2;
        }
        intStateOld = intState;
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
         //pc.printf("%d\n\r",intState);
    }
}

void motorCtrlTick(){ 
    iterCount++; 
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(){
    // run every 100ms
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick,100000); 
    while(1){
        motorCtrlT.signal_wait(0x1);
        // temporarily disable interrupts
        core_util_critical_section_enter();
        if(iterCount == 10){
            if(motor.read() == 0.0){
                velocity = 0;
            }
            else{
                velocity = stateChange * 10;
            }
            pc.printf("Position: %d\n\r", intState);
            pc.printf("Velocity: %d\n\r", velocity);
            motorPosition();
            iterCount = 0;
        }
        core_util_critical_section_exit();
    }
}

// create an ISR to receive each incoming byte and place it into a queue
void serialISR(){
    uint8_t newChar = pc.getc();
    pc.putc(newChar);
    inCharQ.put((void*)newChar);
}

void send_thread(uint64_t* data) {
    mail_t *mail = mail_box.alloc();
    mail->data = data;
    mail_box.put(mail);
    wait(1);
}

void send_motor_control(string* data) {
    motor_t *mail = mail_box_motor.alloc();
    mail->data = data;
    mail_box_motor.put(mail);
    wait(1);
}

void print_thread(void) {
    while(true){
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            
            mail_t *mail = (mail_t*)evt.value.p;

            pc.printf("\n\r");
            pc.printf("nonce: ");
            uint64_t* receivedNonce = mail->data;
            for(int i = 0; i < 8; ++i){
                pc.printf("%02x ", (((uint8_t*)receivedNonce)[i]));
            }
            
        mail_box.free(mail);
        }
    }
}

void decode_instruction(char* input){
    if(input[0] == 'K'){
        newKey_mutex.lock();
        sscanf(input,"K%10llx", &newKey);
        newKey_mutex.unlock();
        send_thread(&newKey);
    }
    else if(input[0] == 'M'){
            dutyCycle_mutex.lock();
            sscanf(input,"M%s", dutyCycle.c_str());
            pc.printf("M%s", input);
            dutyCycle_mutex.unlock();
            send_motor_control(&dutyCycle);
        }
}

void decode_thread(void){
    pc.printf("Enter the key:\n\r");
    pc.attach(&serialISR);
    uint8_t ptr = 0;
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if(newChar != '\r' && newChar != '\n'){
            inputKey[ptr] = newChar;
            ptr++;
        }
        else{
            inputKey[ptr] = '\0';
            ptr = 0;
            decode_instruction(inputKey);
        }
    }
}

void motorControl() {
    dutyCycle_mutex.lock();
    receivedDutyCycle = dutyCycle;
    dutyCycle_mutex.unlock();

    string s = receivedDutyCycle;
    float newDutyCycle = atof(s.c_str());
    motor.write(newDutyCycle);
}

void computationRate(float startTime, float elapsedTime, int counter){
    
    if(elapsedTime >= 1){
        int computationRate = counter / elapsedTime;
        pc.printf("\n\r");
        pc.printf("Computation Rate = ");
        pc.printf("%d", computationRate);
        pc.printf(" Hashes per sec");
        t.reset();
        startTime = t.read();
        counter = 0;
        } 
}
//Main
int main() {
    
    pc.printf("\n");
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    //Attach the ISR to rising and falling edge events on each photointerrupter input 
    I1.rise(&motorPosition);
    I2.rise(&motorPosition);
    I3.rise(&motorPosition);
    I1.fall(&motorPosition);
    I2.fall(&motorPosition);
    I3.fall(&motorPosition);
    
    thread.start(callback(print_thread));
    inputThread.start(callback(decode_thread));
    motorCtrlT.start(callback(motorCtrlFn));
    
    motor.period(0.002f);     // 2ms period
    motor.write(0.50f);       // 50% duty cycle, relative to period
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        
        motorControl();
        
        newKey_mutex.lock(); 
        *key = newKey; 
        newKey_mutex.unlock();
        //sequence now contains the new key
        sha256.computeHash(hash, sequence, 64);
        counter++;
        
        //Test for both hash[0] and hash[1] equal to zero to indicate a successful ‘nonce’
        t.start();
        float startTime = t.read();
        if(hash[0] == 0 &&  hash[1] == 0) {
            send_thread(nonce);
        }
        
        //There is no efficient method for searching for the ‘nonce’, so just start at zero and increment by one on each attempt
        (*nonce)++;

        //Every second, report the current computation rate
        float elapsedTime = t.read() - startTime;
        computationRate(startTime, elapsedTime, counter);   
    }
}
