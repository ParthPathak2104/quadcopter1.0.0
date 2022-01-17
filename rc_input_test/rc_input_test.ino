#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// Refresh Rate
#define REFRESH_RATE 4000   // 4 Micro Seconds

// ================================================================
// ===               MPU BASIC SETTINGS                         ===
// ================================================================

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               PID AND MOTOR OUTPUT SETTINGS              ===
// ================================================================

// PID weights

float PID_ROLL_P =        0.;
float PID_ROLL_I =        0.;
float PID_ROLL_D =        0.;

float PID_PITCH_P =       0.;
float PID_PITCH_I =       0.;
float PID_PITCH_D =       0.;

float PID_YAW_P =         0.;
float PID_YAW_I =         0.;
float PID_YAW_D =         0.;

// PID ERRORS

float PID_ROLL_ERROR_CURR =   0;
float PID_PITCH_ERROR_CURR =  0;
float PID_YAW_ERROR_CURR =    0;

float PID_ROLL_ERROR_PREV =   0;
float PID_PITCH_ERROR_PREV =  0;
float PID_YAW_ERROR_PREV =    0;

// PID OUTPUTS

float PID_ROLL_OUTPUT =       0;
float PID_PITCH_OUTPUT =      0;
float PID_YAW_OUTPUT =        0;

// Motor OUTPUTS

float ROLL_OUTPUT =         0;
float PITCH_OUTPUT =        0;
float YAW_OUTPUT =          0;

// MAX MOTOR OUTPUT

float PID_MAX_OUTPUT =     1900;

// ================================================================
// ===               RECIEVER SETTINGS                          ===
// ================================================================

// ISR Currtime Variable
unsigned long ISR_CURR_TIME, zero_timer, ESC_LOOP_TIMER;
unsigned long timer_BR, timer_BL, timer_FR, timer_FL;
unsigned long timer1, timer2, timer3, timer4; 

uint8_t flag_channel_1, flag_channel_2, flag_channel_3, flag_channel_4;
int reciever_channel_1, reciever_channel_2, reciever_channel_3, reciever_channel_4;

MPU6050 mpu;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    mpu.testConnection();

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
    }  
  
  //Setting D(8-11) to output
  DDRB |= B00001111;

  //Giving Intial 1000 Microsecond Pulse
  for(int i =0; i<10; i++){
    PORTB |= B00001111;
    delayMicroseconds(1000);
    PORTB &= B11110000;
  }
   
  //Enabling External Interrupt for pin no. 4,5,6 and 7 -> PD4, PD5, PD6, PD6  
  PCICR  |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT23);
  PCMSK2 |= (1<<PCINT22);
  PCMSK2 |= (1<<PCINT21);
  PCMSK2 |= (1<<PCINT20);

}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = map( ypr[0] * 180/M_PI, -180, 180, 1000, 2000 );
    ypr[1] = map( ypr[1] * 180/M_PI, -180, 180, 1000, 2000 );
    ypr[2] = map( ypr[2] * 180/M_PI, -180, 180, 1000, 2000 );
    
    Serial.print("ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.println(ypr[2]);
    
  }

  //  Wait for 4Us to expire
  while(zero_timer + REFRESH_RATE > micros());
  zero_timer = micros();

  //  SET PIN 8, 9, 10, 11 HIGH
  PORTB |= B00001111;
  timer_BR = reciever_channel_1 + zero_timer;
  timer_BL = reciever_channel_1 + zero_timer;
  timer_FR = reciever_channel_1 + zero_timer;
  timer_FL = reciever_channel_1 + zero_timer;

  while(PORTB & B00001111){

    // SET PIN 8,9,10,11 LOW    
    ESC_LOOP_TIMER = micros();
    if(timer_BR <= ESC_LOOP_TIMER)PORTB &= B11111110;
    if(timer_BL <= ESC_LOOP_TIMER)PORTB &= B11111101;
    if(timer_FR <= ESC_LOOP_TIMER)PORTB &= B11111011;
    if(timer_FL <= ESC_LOOP_TIMER)PORTB &= B11110111; 
  }
  
}

// ISR for Reading the Data from the Reciever on the Pins :- A0(PC0), A1(PC1), A2(PC2), A3(PC3) 

ISR(PCINT2_vect){

  ISR_CURR_TIME =  micros();

  //  ISR condition for channel 1 at PIN 7
  if(flag_channel_1 == 0 && PIND & B10000000){
    timer1 = ISR_CURR_TIME;
    flag_channel_1=1;
  }
  else if(flag_channel_1 == 1 && !(PIND & B10000000)){
    reciever_channel_1 = ISR_CURR_TIME - timer1;
    flag_channel_1=0;
  }

  //  ISR condition for channel 2 at PIN 6
  if(flag_channel_2 == 0 && PIND & B01000000){
    timer2 = ISR_CURR_TIME;
    flag_channel_2=1;
  }
  else if(flag_channel_2 == 1 && !(PIND & B01000000)){
    reciever_channel_2 = ISR_CURR_TIME - timer2;
    flag_channel_2=0;
  }

  //  ISR condition for channel 3 at PIN 5
  if(flag_channel_3 == 0 && PIND & B00100000){
    timer3 = ISR_CURR_TIME;
    flag_channel_3=1;
  }
  else if(flag_channel_3 == 1 && !(PIND & B00100000)){
    reciever_channel_3 = ISR_CURR_TIME - timer3;
    flag_channel_3=0;
  }

  //  ISR condition for channel 4 at PIN 4
  if(flag_channel_4 == 0 && PIND & B00010000){
    timer4 = ISR_CURR_TIME;
    flag_channel_4=1;
  }
  else if(flag_channel_4 == 1 && !(PIND & B00010000)){
    reciever_channel_4 = ISR_CURR_TIME - timer4;
    flag_channel_4=0;
  }

  //  Limiting the Output
  if(reciever_channel_1 < 1050) reciever_channel_1 = 1000;
  if(reciever_channel_1 > 1950) reciever_channel_1 = 1900;

  if(reciever_channel_2 < 1050) reciever_channel_1 = 1000;
  if(reciever_channel_2 > 1950) reciever_channel_1 = 1900;

  if(reciever_channel_3 < 1050) reciever_channel_1 = 1000;
  if(reciever_channel_3 > 1950) reciever_channel_1 = 1900;

  if(reciever_channel_4 < 1050) reciever_channel_1 = 1000;
  if(reciever_channel_4 > 1950) reciever_channel_1 = 1900;
  
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
