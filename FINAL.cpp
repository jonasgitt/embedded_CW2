/************************************TODO BY TONIGHT********************
TUNE K VALUES
INCLUDE RICHARDS CORRECTED HASH CALC
CODE CLEAN UP - REMOVE UNUSED PARAMS ETC (CHECK MBED COMPILER, IT TELLS U WHAT IS UNUSED
CAN WE GET IT DOWN TO WORK WITH SPEED 5 FOR SPECIFICATION 
DOES R0 WORK CORRECTLY - IVE EMAILED STOTT TO ASK HIM WHAT IT EXACTLY MEANS (SEE SLIDES)
ACTUAL ROTATIONS = MOTORPOS/6 -- WHERE SHOULD WE INCLUDE THIS CORRECTION
MELODY
MEASUREMENTS
REPORT

*/


#include "mbed.h"
#include "Crypto.h"
#include <math.h>
#include <string.h>

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

//--------FUNCTION PROTOTYPES---------//
void putMessage2Q(uint8_t type, int64_t value);
float speedCtrl();
float positionCtrl(float &der);


//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;
float motorPos = 0;

int hcount = 0; //hash rate
//----------------------------------------//



PwmOut pwmCtrl(PWMpin);

//Status LED
DigitalOut led1(LED1);
RawSerial pc(SERIAL_TX, SERIAL_RX);
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

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if(pwmCtrl){
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
    else{
    L1L = 0;
    L1H = 0;
    L2L = 0;
    L2H = 0;
    L3L = 0;
    L3H = 0;
    }
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
    
//Poll the rotor state and set the motor outputs accordingly to spin the motor
volatile int32_t revCount = 0;
void photoInterrupterISR(){
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
   
    //determine direction
    int8_t direction =((intState - intStateOld)+6)%6;  
    if (direction == 5){ //negative direction
        revCount--; 
    }else{
         revCount++;   
    }
    
    if(intState - intStateOld == 5) motorPos--; //negative direction
    else if (intState - intStateOld == -5) motorPos++;
    else motorPos += intState - intStateOld;
    intStateOld = intState;
}


//--------------------------------------------------------------//
//-------------------------Lab3 Motor Stuff---------------------//
//--------------------------------------------------------------//
#define VELOCITY 5
#define TARGETSPEED 6
#define TARGETPOSITION 7
#define ROTATIONS 8

#define kps 30
#define kis 0.105
#define kpr 0.074
#define kdr 0.44


#define iesmax 350
#define maxpulsewidth 2000
#define rotation_limit 100

#define debug_msg 0 
#define hashrate 1
#define NONCE 2
#define NEW_KEY 3
#define TORQUE 4


Thread motorCtrlThread(osPriorityNormal,1024);
void motorCtrlTick(){motorCtrlThread.signal_set(0x1);}//ISR trigger by ticker.
volatile float corrected_pwm;
volatile float  pos_corr_pwm;
volatile float  speed_corr_pwm;
volatile float prevPositionError=0;
volatile float PositionError = 0;
volatile float pwmTorque = 1;
volatile float target_speed = 1;
volatile int64_t target_pos = 0;
volatile int64_t oldPos = 1;
volatile float oldMotorPos;
volatile float velocity;
volatile float rotations;
volatile float ts = 0;
volatile float es = 0; 
volatile float ies = 0;

bool speed_set=false;
bool position_set = false; //turns true if user sets speed


Mutex vel_mutex;

void motorCtrlFunction(){
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    
    uint8_t velCounter = 0; //because we want to print velocity once per second. 
    
    
    Timer motorTimer; //for more accurate velocity calculation
    motorTimer.start();
    float der = 0; //to store derivative of error position

    while(1){
        
        
        motorTimer.reset();
        //calculate velocity


        oldMotorPos = motorPos;  
        motorCtrlThread.signal_wait(0x1); //block until motorCtrlTick is triggered (which is triggered by the ticker)   
        
        rotations = motorPos/6;
        velocity  = 10*(motorPos - oldMotorPos);
        
        
        //print velocity once per second
        if (velCounter++ >= 10){
            velCounter = 0;
        }
        
        
        if(speed_set && target_speed==0){ //V0 
            pwmCtrl.write(1);
            if(velCounter==9){
                putMessage2Q(VELOCITY, (int64_t)velocity);
            }
        }
        else if (speed_set && !position_set){ //only speed requested
            corrected_pwm= speedCtrl();
            pwmCtrl.write(corrected_pwm);
            if(velCounter==9){
                putMessage2Q(VELOCITY, (int64_t)velocity);
                putMessage2Q(TARGETSPEED, (int64_t)target_speed);
            }
        }
        else if (!speed_set && position_set){ //only position requested
            corrected_pwm= positionCtrl(der);
            
            if(velCounter==9){
                putMessage2Q(ROTATIONS, (int64_t)rotations);
                putMessage2Q(TARGETPOSITION, (int64_t)target_pos);
            }
            if((long)PositionError>0)lead = 2; //positive direction
            else lead = -2; //negative direction
            corrected_pwm = (corrected_pwm>rotation_limit) ? rotation_limit : abs(corrected_pwm); //take magnitude of positionCtrl
            pwmCtrl.write(corrected_pwm);
       }
  
        else if (speed_set && position_set){ //both rotation and speed requested
            pos_corr_pwm = positionCtrl(der);
            speed_corr_pwm = speedCtrl();
            
             if (der>=0) { //overshoot has occured, position must return back
                 speed_corr_pwm=-speed_corr_pwm; //change direction of speed
                 corrected_pwm = max(pos_corr_pwm,speed_corr_pwm);
                } 
            else {
                corrected_pwm= min(pos_corr_pwm,speed_corr_pwm); //still seeking to reach desired position
                }
                
            if (corrected_pwm<=0){
            corrected_pwm= abs(corrected_pwm);
            lead = -2; //negative direction
            }else lead = 2; // positive direction
            pwmCtrl.write(corrected_pwm);
            if(velCounter==9){
                putMessage2Q(VELOCITY, (int64_t)velocity);
                putMessage2Q(ROTATIONS, (int64_t)rotations);
            }   
        }
       
    }
}


//SPEED: Return corrected motor pulse-width
float speedCtrl(){

    
    es = abs(target_speed)-abs(velocity); //speed error
    
    ies = ies + es; //integral correction
    if(abs(ies) > iesmax) ies = (abs(ies)/ies)*iesmax; //signum
    
    ts = (kps*es+kis*ies); //speed controller
       
    return ts;
}


//POSITION: Return corrected motor pulse-width

float positionCtrl(float &der){
    float tr = 0;
    
    
    PositionError = (long)target_pos - motorPos; //target_pos signed, poserror is float
    der = (PositionError - prevPositionError); //derivative by time, but it is 1 second
    prevPositionError=PositionError; //update position for next funciton call
    
    tr = 0.973*(kpr*(PositionError) + kdr*der); //added heuristics constant; position controller
        
    tr =( tr>rotation_limit) ? rotation_limit : tr;

   return tr;
}


//--------------------------------------------------------------//
//----------------Lab2 Communications Stuff---------------------//
//--------------------------------------------------------------//



Mutex newKey_mutex;

typedef struct {
  uint8_t type;
  int32_t value;
} mail_msg;

Mail<mail_msg, 16> mail_box;
Queue<void, 8> inCharQ; //Buffers incoming characters

Thread comms_out_thread; 
Thread input_decode_thread;

void printFromMail(){ //printing to serial from mail queue
    
    while (true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            
            mail_msg *mail = (mail_msg*)evt.value.p;
            
            switch(mail->type){
                case(NEW_KEY):
                    pc.printf("NewKey: %d\r\n", mail->value);
                    break;
                case(hashrate):
                    pc.printf("Hashrate: %d\r\n", mail->value);
                    break;
                case(debug_msg):
                    if(mail->value == 404) {pc.printf("Invalid user input.\r\n");}
                    else{
                        pc.printf("This is a debug Message, code: %i\r\n", mail->value);
                    }
                    break;
                case(NONCE):
                    pc.printf("Nonce: %d\r\n", mail->value); 
                    break;
               /* case(TORQUE):
                    pc.printf("Torque: %d\r\n", mail->value);
                    break; */
                case(VELOCITY):
                    pc.printf("Velocity: %.1f\r\n", velocity/6);
                    break;
                case(TARGETSPEED):
                    pc.printf("TARGETSPEED: %.1f\r\n", target_speed/6);
                    break;
                case(TARGETPOSITION):
                    pc.printf("TARGETPOSITION: %i\r\n", target_pos/6);
                    break;
                case(ROTATIONS):
                    pc.printf("POSITION: %.2f\r\n", rotations);
                    break;
                default:
                    pc.printf("Unknown data: %d\r\n", mail->value);
                    break;
            }
            
            mail_box.free(mail); //return the memory unit
        } 
    }
}

//Adds message to output queue
void putMessage2Q(uint8_t type, int64_t value){
        mail_msg *mail = mail_box.alloc();
        mail->type = type; 
        mail->value = value;
        mail_box.put(mail);
    }

//**********INCOMING**********//
//get new character from the user and place it in the input queue
void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
    }

//Decodes the users input to the serial monitor
volatile int64_t userInput;
volatile uint64_t newkey;  
char input_chars[17];

void decoder(){
    pc.attach(&serialISR); //checks if anything is being entered in serial
    
    uint8_t input_chars_index =0;
    int8_t error = 404; //error message
    
    
    while(1){
        //get new character from serial port
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)(newEvent.value.p); 
 
        
        //If possible, store character in array
        if(input_chars_index > 18){
            input_chars_index = 0;  
        }else{
            input_chars[input_chars_index] = newChar;         
        }
        
        //Start decoding once return is received
        if (newChar == '\r'){
            input_chars[input_chars_index]= '\0'; //required to terminate char array
            input_chars_index = 0;
            
            //decode the user input
            switch (input_chars[0]){
                //Input is a Key
                case 'K':       
                    sscanf(input_chars, "K%x", &userInput);
                    putMessage2Q(NEW_KEY, userInput); //add the newkey to the message queue
                    newKey_mutex.lock();
                    newkey=userInput;
                    newKey_mutex.unlock();
                    break;
                /*case 'T':
                    sscanf(input_chars, "T%x", &userInput); FOR MELODY IMPLEMENTATION
                    putMessage2Q(TORQUE, userInput);
                    pwmTorque = userInput/10;
                    break;*/
                case 'V':
                    sscanf(input_chars, "V%d", &userInput);
                    putMessage2Q(TARGETSPEED, userInput);
                    target_speed = 6*userInput;
                    speed_set = true;
                    break;
                case 'R':
                    sscanf(input_chars, "R%i", &userInput);
                    putMessage2Q(TARGETPOSITION, userInput);
                    target_pos = 6*userInput;
                    position_set = true;
                    motorPos = 0;
                    break;
                default: 
                    putMessage2Q(debug_msg, error);
                    break;
                }
        }
        else{
            input_chars_index++;     
        }
    }
    

}


//-----------------------------------------------------------------//
//-------------------BITCOIN STUFF------------------------//
//-----------------------------------------------------------------//
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

   
    SHA256 h;
    Timer t;


void hasher(){
    newKey_mutex.lock();
        *key = newkey;
        newKey_mutex.unlock();
        
        h.computeHash(hash, sequence, 64);
        hcount++;
             
         if(hash[0]==0 && hash[1]==0){
             putMessage2Q(NONCE, *nonce);
         }
        *nonce+=1;
}
int main() {
    
    //initialise communication threads
    comms_out_thread.start(printFromMail); // callback allows us pass arguments to spawned threads.
    input_decode_thread.start(decoder); //WAS serialISR
    
    //initialise motor thread
    motorCtrlThread.start(motorCtrlFunction);
    
    pwmCtrl.period(0.002f);      // 2ms second period
    pwmCtrl.write(0.5f);       // 50%% duty cycle
   
    //Run the motor synchronisation
   orState = motorHome();


    I1.rise(&photoInterrupterISR);
    I2.rise(&photoInterrupterISR);
    I3.rise(&photoInterrupterISR);
    I1.fall(&photoInterrupterISR);
    I2.fall(&photoInterrupterISR);
    I3.fall(&photoInterrupterISR);  
    
    
        
    t.start();    
    while(true){
        hasher();
        if(t.read()>1){
            putMessage2Q(hashrate, hcount);
            hcount=0;
            t.reset();
        }
    }
}