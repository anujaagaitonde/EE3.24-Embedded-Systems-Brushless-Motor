#include "mbed.h"
#include "rtos.h"
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
//2 for forwards, -2 for backwards
int8_t lead = 2;

// code options for input to mail_t structure
enum dataCode {
    ERROR1 = 0,
    NONCE = 1,
    KEY = 2,
    VELOCITY = 3,
    ROTATIONS = 4
};

//Mailbox structure
typedef struct {
    uint8_t code;
    int64_t data;    
} mail_t;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

SHA256 sha256; //Create an instance of the class SHA256 

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs - Declare low pins as Pwm to take torque input
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Thread initialisations
Thread thread; // declare stack size?
Thread inputThread; // declare stack size?
Thread motorCtrlT(osPriorityNormal,1024);

Queue<void, 8> inCharQ;
Mail<mail_t, 16> mail_box;

Timer t; //MBED timer class

//PwmOut motor(PWMpin); // PWMOut class
int64_t pwm_period = 2000; // PWM period is us

Mutex input_mutex; // use mutex to prevent simultaneous access

//Rotor offset at motor state 0
int8_t orState = 0;    
int8_t intState = 0;
int8_t intStateOld = 0;
int8_t stateChange = 0;
volatile int64_t velocity = 0;           // revolutions / sec
int iterCount = 0;
int interruptCount = 0;             // count number of interrupts
float revCount = 0;             // count number of revolutions = interrupts / 6
volatile int64_t target_speed = 0;             // target speed
volatile int64_t target_position = 0;

uint64_t newKey;                    // key
char inputKey[18];                   // setting rotation speed
int64_t T = 0;                      // speed torque
int64_t torque_speed = 0;                 // speed torque (abs)
int64_t Tp = 0;                     // position torque
int64_t torque_position = 0;        // position torque (abs)
int old_rotation_count;
float er_prev;    // previous rotation error

//Set a given drive state
void motorOut(int8_t driveState, uint64_t torque){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(torque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(torque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(torque);
    if (driveOut & 0x20) L3H = 0;
}
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0,pwm_period);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

// new photointerrupt ISR
void motorPosition() {
    //intStateOld = intState;
    intState = readRotorState();
    stateChange = intState - intStateOld;
    if (intState != intStateOld) {
        motorOut((intState-orState+lead+6)%6,torque_position); //+6 to make sure the remainder is positive
        if(intState - intStateOld == 5) interruptCount++;
        else if (intState - intStateOld == -5) interruptCount--;
        else interruptCount += (intState - intStateOld);
        intStateOld = intState;
    }
    
}

// create an ISR to receive each incoming byte and place it into a queue
void serialISR(){
    uint8_t newChar = pc.getc();
    pc.putc(newChar);
    inCharQ.put((void*)newChar);
}

void send_thread(uint8_t code, uint64_t data) {
    mail_t *mail = mail_box.alloc();
    mail->code = code;
    mail->data = data;
    mail_box.put(mail);
    wait(1);
}

void print_thread(void) {
    while(true){  

        osEvent evt = mail_box.get();

        if (evt.status == osEventMail) {
            
            mail_t *mail = (mail_t*)evt.value.p;

            switch(mail->code){
                case ERROR:
                    break;
                case KEY:
                    //???
                    break;
                case NONCE: // KEY
                    pc.printf("\n\r");
                    pc.printf("nonce: ");
                    uint64_t receivedNonce = mail->data;
                    for(int i = 0; i < 8; ++i){
                        pc.printf("%02x ", (((uint8_t*)receivedNonce)[i]));
                    }
                    break;
                case VELOCITY: // proportional motor speed control
                    pc.printf("Vel thread accessed\n\r");
                    pc.printf("target vel = %d\n\r", target_speed);
                    break;
                case ROTATIONS: // PD position control
                    pc.printf("Position thread accessed\n\r");
                    pc.printf("target pos = %d\n\r", target_position);
            }
            
        mail_box.free(mail);
        }
    }
}

void decode_instruction(char* input){
    if(input[0] == 'K'){
        input_mutex.lock();
        sscanf(input,"K%10llx", &newKey);
        input_mutex.unlock();
        send_thread(KEY, newKey);
    }
    else if(input[0] == 'V'){
        input_mutex.lock();
        // expected input of format V\d{1,3}(\.\d{1,3})?
        sscanf(input,"V%d", &target_speed);
        input_mutex.lock();
        send_thread(VELOCITY, target_speed);
    }
    else if(input[0] == 'R'){
        input_mutex.lock();
        // expected input of format V\d{1,3}(\.\d{1,3})?
        sscanf(input,"R%d", &target_position);
        input_mutex.lock();
        send_thread(ROTATIONS, target_position);
    }
}

void decode_thread(void){
    pc.printf("Enter the command:\n\r");
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

void motorCtrlTick(){ 
    iterCount++; 
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(){
    //interruptCount = 0;  // move but check first
    // run every 100ms
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);  
    uint32_t iterTime = 0;


    int64_t Kps = 25;   // proportional constant for speed controller - maximum to avoid oscillation
    int64_t Kpr = 25;   // proportional constant for rotation controller
    int64_t Kdr = 25;   // derivative constant for rotation controller
    int rotation_count = 0;
    float current_position;
    int64_t currentS = 0;     // absolute value of current speed
    int64_t es = 0;            // speed error  
    float er;         // rotation error
    float er_derivative = 0;
    // previous rotation error

    while(true){
        motorCtrlT.signal_wait(0x1);
        core_util_critical_section_enter(); // temporarily disable interrupts
        
        rotation_count = interruptCount;
        current_position = rotation_count / 6;
        velocity = (rotation_count - old_rotation_count) * 10; // determine state changes per second
        old_rotation_count = rotation_count;

        currentS = abs(velocity); // absolute value of speed
        es = target_speed - currentS; // speed error

        T = Kps * es; // torque

        if(T < 0){
            pc.printf("hi\n\r");
            torque_speed = -T; // negative values need to be made positive
            lead = -2; // backwards rotation
        } 
        else{
            torque_speed = T;
            lead = 2; // forwards rotation
        }
    
        if(torque_speed > (pwm_period)) torque_speed = pwm_period;

        if(iterCount == 10){
            //revCount = interruptCount / 6;
            pc.printf("v: %d\n\r", velocity);
            pc.printf("T: %d\n\r", torque_speed);
            pc.printf("TARGET v: %d\n\r", target_speed);
           // interruptCount = 0;

            iterCount = 0;
        }

        motorOut((intState-orState+lead+6)%6, torque_speed); // write to motor output with new torque


        er = target_position - current_position; // position error
        er_derivative = er - er_prev; // time derivative of position error
        er_prev = er;
        Tp = Kpr * er + Kdr * er_derivative; // position torque

        if(Tp < 0){
            pc.printf("hi\n\r");
            torque_position = -Tp; // negative values need to be made positive
            lead = -2; // backwards rotation
        } 
        else{
            torque_position = Tp;
            lead = 2; // forwards rotation
        }

        if(torque_position > pwm_period) torque_position = pwm_period;

        if(iterCount == 10){
            //revCount = interruptCount / 6;
            pc.printf("p: %d\n\r", current_position);
            pc.printf("T: %d\n\r", torque_position);
            pc.printf("TARGET p: %d\n\r", target_position);
            pc.printf("Remaining: %f\n\r", er);
           // interruptCount = 0;

            iterCount = 0;
        }

        if(current_position == target_position) current_position = 0;

        motorOut((intState-orState+lead+6)%6, torque_position);

        
        core_util_critical_section_exit();
        motorPosition();
    }
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

    // 2ms period
   // motor.period_us(pwm_period);     
    L1L.period_us(pwm_period);
    L2L.period_us(pwm_period);
    L3L.period_us(pwm_period);
    L1L.write(100.0);
    L2L.write(100.0);
    L3L.write(100.0);
    //motor.write(0.50f);       // 50% duty cycle, relative to period
   // motor.write(100.0);       // 100% duty cycle, relative to period
    
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
    
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        
        input_mutex.lock(); 
        *key = newKey; 
        input_mutex.unlock();
        //sequence now contains the new key
        sha256.computeHash(hash, sequence, 64);
        counter++;
        
        //Test for both hash[0] and hash[1] equal to zero to indicate a successful ‘nonce’
        t.start();
        float startTime = t.read();
        if(hash[0] == 0 &&  hash[1] == 0) {
            //send_thread(NONCE, nonce);
        }
        
        //There is no efficient method for searching for the ‘nonce’, so just start at zero and increment by one on each attempt
        (*nonce)++;

        //Every second, report the current computation rate
        float elapsedTime = t.read() - startTime;
        computationRate(startTime, elapsedTime, counter);   
    }
}

