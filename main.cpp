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
int computation_counter = 0;

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
    ERRORCODE = 0,
    NONCE = 1,
    KEY = 2,
    TARGETVELOCITY = 3,
    ROTATIONS = 4,
    VELOCITY = 5,
    TORQUE = 6,
    POSITION = 7,
    HASHRATE = 8
};

//Mailbox structure
typedef struct {
    uint8_t code;
    uint64_t data;    
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

// parameters modified by threads
volatile float s_max = 0;              // user defined maximum velocity in revolutions / sec
volatile float current_velocity;           // current velocity in interrupts / sec
volatile float current_position;        // current position in states
volatile float p_max = 0;    // user defined number of rotations
volatile float current_er;

int iterCount = 0;
float interruptCount = 0;             // count number of interrupts
float revCount = 0;             // count number of revolutions = interrupts / 6       // max speed in rev/s

volatile uint64_t newKey;                    // key
char inputKey[18];                   // setting rotation speed
int64_t torque_speed = 1000;                      // speed torque
int64_t output_torque = 1000;                 // output torque             
int64_t torque_position = 1000;        // position torque
// int old_state_change_count;
float er_prev;    // previous rotation error

// function prototypes
int sgn(float x);
float min(float x, float y);
float max(float x, float y);
void motorOut(int8_t driveState, uint64_t torque);
inline int8_t readRotorState();
int8_t motorHome();
void motorPosition();
void serialISR();
void send_thread(uint8_t code, uint64_t data);
void print_thread(void);
void decode_instruction(char* input);
void decode_thread(void);
void motorCtrlTick();
void motorCtrlFn();
void computationRate(float elapsedTime, int computation_counter);



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
        motorOut((intState-orState+lead+6)%6,output_torque); //+6 to make sure the remainder is positive
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
                case ERRORCODE:
                    pc.printf("Error. Try again\n\r");
                    break;
                case KEY:
                    pc.printf("Decoded key = 0x%016x\n\r", mail -> data);
                    break;
                case NONCE:
                    pc.printf("Found a nonce = 0x%016x\n\r", mail -> data);
                    // uint64_t receivedNonce = mail->data;
                    // for(int i = 0; i < 8; ++i){
                    //     pc.printf("%02x ", (((uint8_t*)receivedNonce)[i]));
                    // }
                    // pc.printf("\n\r");
                    break;
                case TARGETVELOCITY: // proportional motor speed control
                    pc.printf("Target Velocity = %.3f\n\r", s_max);
                    break;
                case ROTATIONS: // PD position control
                    pc.printf("Target Rotations = %.3f\n\r", p_max);
                    break;
                case VELOCITY:
                    pc.printf("Current Velocity = %.3f\n\r", (current_velocity/6));
                    break;
                case TORQUE:
                    pc.printf("Current Torque = %d\n\r", output_torque);
                    break;
                case POSITION:
                    pc.printf("Current Rotations = %.3f\n\r", current_position);
                    break;
                case HASHRATE:
                    pc.printf("Computation Rate = %d\n\r", mail -> data);
                    break;
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
        sscanf(input,"V%f", &s_max);
        input_mutex.unlock();
        send_thread(VELOCITY, s_max);
    }
    else if(input[0] == 'R'){
        input_mutex.lock();
        // expected input of format V\d{1,3}(\.\d{1,3})?
        sscanf(input,"R%f", &p_max);
        interruptCount = 0;
        input_mutex.unlock();
        send_thread(ROTATIONS, p_max);
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


    int64_t Kps = 25;   // proportional constant for speed controller - maximum to avoid oscillation
    int64_t Kpr = 25;   // proportional constant for rotation controller
    int64_t Kdr = 50;   // derivative constant for rotation controller
    float state_change_count = 0;
    float currentS = 0;     // absolute value of current speed
    float es = 0;            // speed error  
    float er = 0;         // rotation error
    float er_derivative = 0;
    static float old_state_change_count = 0;
    static float er_prev;
    // previous rotation error

    while(true){
        motorCtrlT.signal_wait(0x1);
        core_util_critical_section_enter(); // temporarily disable interrupts
        
        // calculate velocity and position change
        state_change_count = interruptCount;
        float position = state_change_count / 6; // determine current rotation position
        float velocity = (state_change_count - old_state_change_count) * 10; // determine number of interrupts (number of state changes) per second
        old_state_change_count = state_change_count;
        current_velocity = velocity; // write current velocity to global copy

        // PD position torque controller: Tp = Kpr * er + Kdr * d/dt(er)
        float target_position = p_max; 
        er = target_position - position; // position error
        er_derivative = er - er_prev; // time derivative of position error
        er_prev = er;

        float target_speed = s_max * 6; // target speed in interrupts / sec - overwritten each time
        currentS = abs(velocity); // absolute value of speed
        es = target_speed - currentS; // speed error

        current_position = position;
        current_er = er;

        // P velocity torque controller: Ts = (Kps * es) * sgn (er_derivative)
        // PD position torque controller: Ts = Kpr * er + Kdr * er_derivative
        if (target_position == 0) // R0
        {
            if (target_speed == 0) // V0
            {
                torque_speed = pwm_period;
            }
            else
            {
                torque_speed = (Kps * es);
            } 
            output_torque = torque_speed;        
        }
        else
        {
            if(target_speed == 0) // V0
            {
                torque_speed = pwm_period;
            }
            else
            {
                torque_speed = (Kps * es);
            }
            torque_position = Kpr * er + Kdr * er_derivative;

            if(er < 0) torque_speed = -torque_speed; // only change when moving backwards
            
            if((velocity < 0))
            {
                output_torque = max(torque_speed, torque_position);
            }
            else
            {
                output_torque = min(torque_speed, torque_position);
            }
        }


        if(output_torque < 0){
            output_torque = -output_torque; // negative values need to be made positive
            lead = -2; // backwards rotation
        } 
        else{
            lead = 2; // forwards rotation
        }
    
        if(output_torque > (pwm_period)) output_torque = pwm_period;

        motorOut((intState-orState+lead+6)%6, output_torque); // write to motor output with new torque // MOVE LATER
       
        core_util_critical_section_exit(); // re-enable interrupts
        if(iterCount == 10){ // display current velocity and position data every 1s
            send_thread(VELOCITY, (velocity/6));
            send_thread(POSITION, position);
            iterCount = 0;
        }
        motorPosition();
    }
}

void computationRate(float elapsedTime, int computation_counter){ // calculate hash computation rate
    
    if(elapsedTime >= 1){
        send_thread(HASHRATE, computation_count)
        t.reset();
        computation_counter = 0;
    } 
}

int sgn(float x) // return sign
{
    if(x < 0) return -1;
    else return 1;
}

float min(float x, float y) // return minimum value
{
    if(x < y) return x;
    else return y;
}

float max(float x, float y) // return maximum value
{
    if(x > y) return x;
    else return y;
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
    
    t.start();
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        
        input_mutex.lock(); 
        *key = newKey; 
        input_mutex.unlock();
        //sequence now contains the new key
        sha256.computeHash(hash, sequence, 64);
        computation_count++;
        
        //Test for both hash[0] and hash[1] equal to zero to indicate a successful ‘nonce’
        if(hash[0] == 0 &&  hash[1] == 0) 
        {
            send_thread(NONCE, *nonce);
        }
        
        //There is no efficient method for searching for the ‘nonce’, so just start at zero and increment by one on each attempt
        (*nonce)++;

        //Every second, report the current computation rate
        float elapsedTime = t.read();
        computationRate(elapsedTime, computation_count);   
    }
}
