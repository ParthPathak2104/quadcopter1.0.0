#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// Refresh Rate
#define REFRESH_RATE 4000   // 4000 Micro Seconds

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

float PID_ROLL_P =        2.4;
float PID_ROLL_I =        0.003;
float PID_ROLL_D =        66;

float PID_PITCH_P =       PID_ROLL_P;
float PID_PITCH_I =       PID_ROLL_I;
float PID_PITCH_D =       PID_ROLL_D;

float PID_YAW_P =         4.0;
float PID_YAW_I =         0.001;
float PID_YAW_D =         0.;

// PID ERRORS

float PID_ROLL_P_ERROR   = 0.;
float PID_ROLL_I_ERROR   = 0.;
float PID_ROLL_D_ERROR   = 0.;

float PID_PITCH_P_ERROR   = 0.;
float PID_PITCH_I_ERROR   = 0.;
float PID_PITCH_D_ERROR   = 0.;

float PID_YAW_P_ERROR   = 0.;
float PID_YAW_I_ERROR   = 0.;
float PID_YAW_D_ERROR   = 0.;

// PID ERRORS

float PID_ROLL_ERROR_CURR =   0.;
float PID_PITCH_ERROR_CURR =  0.;
float PID_YAW_ERROR_CURR =    0.;

float PID_ROLL_ERROR_PREV =   0.;
float PID_PITCH_ERROR_PREV =  0.;
float PID_YAW_ERROR_PREV =    0.;


// PID OUTPUTS

float PID_ROLL_OUTPUT =       0.;
float PID_PITCH_OUTPUT =      0.;
float PID_YAW_OUTPUT =        0.;

// Gyro Data for Yaw

float GYRO_OFFSET =         0.06;

// Motor OUTPUTS

float ROLL_OUTPUT =         0.;
float PITCH_OUTPUT =        0.;
float YAW_OUTPUT =          0.;

// MAX PID OUTPUT ( @NOT SURE )

float PID_MAX_OUTPUT =     500;
float PID_MIN_OUTPUT =     500;

float PID_MIN_INTEGRAL  =  -200;
float PID_MAX_INTEGRAL  =   200;

// MAX AND MIN MOTOR OUTPUT

float MAX_MOTOR_OUTPUT   =  2000;
float MIN_MOTOR_OUTPUT   =  1000;

// Timer Variables for PID Derivative Term
float timePrev=0., timeCurr=0., elapsedTime;

// Variable to store battery voltage
int battery_voltage = 0;
float battery_compensation = 0;


// ================================================================
// ===               RECIEVER SETTINGS                          ===
// ================================================================

// ISR Currtime Variable
unsigned long ISR_CURR_TIME, zero_timer, ESC_LOOP_TIMER;
unsigned long timer_BR, timer_BL, timer_FR, timer_FL;
unsigned long timer1, timer2, timer3, timer4; 

byte flag_channel_1, flag_channel_2, flag_channel_3, flag_channel_4;
int reciever_channel_1, reciever_channel_2, reciever_channel_3, reciever_channel_4;

//Variable For Throttle
int throttle = 1000;

MPU6050 mpu;

void setup() {
  
//  PIN 13 with red LED as INDICATOR

  pinMode(13, OUTPUT);
  
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
  
  //Enabling External Interrupt for pin no. 4,5,6 and 7 -> PD4, PD5, PD6, PD6  
  PCICR  |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT23);
  PCMSK2 |= (1<<PCINT22);
  PCMSK2 |= (1<<PCINT21);
  PCMSK2 |= (1<<PCINT20);

  //  Reading the battery voltage
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  //Giving Intial 1000 Microsecond Pulse
  for(int i =0; i<5; i++){
    PORTB |= B00001111;
    delayMicroseconds(1000);
    PORTB &= B11110000;
    delayMicroseconds(1000);
  }

}

void loop() {

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//  ypr[0] = -ypr[0] * 180/M_PI;
    ypr[1] = -ypr[1] * 180/M_PI;
    ypr[2] = -ypr[2] * 180/M_PI;

    // Find A way to get gyro data through DMP in float mpu.dmpGetGyro(gyroXYZ, fifoBuffer);
    // Getting Yaw in deg/sec  (clockwise +ve)
    ypr[0] = - ((mpu.getRotationZ()/ 16.4) + GYRO_OFFSET) ;

  }

  // If channel_3 (that is throttel) > 1010 then only start the process
    
  if( throttle > 1010){

    // 2. CALCULATE PID OUTPUT
    calculatePID();

    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    // Throttle -> channel_3 will always be added  
    timer_BR = reciever_channel_3 - PID_ROLL_OUTPUT + PID_PITCH_OUTPUT + PID_YAW_OUTPUT;  // CW
    timer_BL = reciever_channel_3 - PID_ROLL_OUTPUT - PID_PITCH_OUTPUT - PID_YAW_OUTPUT;  // CCW
    timer_FR = reciever_channel_3 + PID_ROLL_OUTPUT + PID_PITCH_OUTPUT - PID_YAW_OUTPUT;  // CCW
    timer_FL = reciever_channel_3 + PID_ROLL_OUTPUT - PID_PITCH_OUTPUT + PID_YAW_OUTPUT;  // CW

    battery_compensation = ((1240. - battery_voltage)/(float)3500);

    //  Limit the Output
    if(timer_BR > MAX_MOTOR_OUTPUT) timer_BR = MAX_MOTOR_OUTPUT;
    if(timer_BR < 1020) timer_BR = MIN_MOTOR_OUTPUT;

    if(timer_BL > MAX_MOTOR_OUTPUT) timer_BL = MAX_MOTOR_OUTPUT;
    if(timer_BL < 1020) timer_BL = MIN_MOTOR_OUTPUT;

    if(timer_FR > MAX_MOTOR_OUTPUT) timer_FR = MAX_MOTOR_OUTPUT;
    if(timer_FR < 1020) timer_FR = MIN_MOTOR_OUTPUT;

    if(timer_FL > MAX_MOTOR_OUTPUT) timer_FL = MAX_MOTOR_OUTPUT;
    if(timer_FL < 1020) timer_FL = MIN_MOTOR_OUTPUT;

  }
  
  else{
    timer_BR = 1000;
    timer_BL = 1000;
    timer_FR = 1000;
    timer_FL = 1000;
  }
  
//    Serial.print(timer_BR);
//    Serial.print("\t");
//    Serial.print(timer_BL);
//    Serial.print("\t");
//    Serial.print(timer_FR);
//    Serial.print("\t");
//    Serial.println(timer_FL);

    // 3. GIVE THE OUTPUT TO MOTORS  (Wait for 4000Us to expire)
  
    while(zero_timer + REFRESH_RATE > micros());
    zero_timer = micros();

    //  SET PIN 8, 9, 10, 11 HIGH
    PORTB |= B00001111;

    timer_BR += zero_timer;
    timer_BL += zero_timer;
    timer_FR += zero_timer;
    timer_FL += zero_timer;

    while(PORTB & B00001111){

      // SET PIN 8 -> BR, 9 -> BL, 10 -> FR, 11 -> FL LOW    
      ESC_LOOP_TIMER = micros();
      if(timer_BR <= ESC_LOOP_TIMER)PORTB &= B11111110;
      if(timer_BL <= ESC_LOOP_TIMER)PORTB &= B11111101;
      if(timer_FR <= ESC_LOOP_TIMER)PORTB &= B11111011;
      if(timer_FL <= ESC_LOOP_TIMER)PORTB &= B11110111; 
    }
    

}

void calculatePID(){

  // Claculate Error for Roll Pitch and Yaw  
  
  //  FOR ROLL
  PID_ROLL_ERROR_CURR = ypr[2] - reciever_channel_2;                                             
  
  PID_ROLL_P_ERROR = PID_ROLL_ERROR_CURR * PID_ROLL_P;                                              // Propotional Error
  
  PID_ROLL_I_ERROR += PID_ROLL_ERROR_CURR * PID_ROLL_I;                                             // Integral Error and its limit

  PID_ROLL_D_ERROR = (PID_ROLL_ERROR_CURR - PID_ROLL_ERROR_PREV) * PID_ROLL_D;     // Derivative Error  

  PID_ROLL_OUTPUT = PID_ROLL_P_ERROR + PID_ROLL_I_ERROR + PID_ROLL_D_ERROR;                         // Total Roll Error    

  PID_ROLL_ERROR_PREV = PID_ROLL_ERROR_CURR;                                                        // Assign Current Error to Previous Error 


  // For Pitch  
  PID_PITCH_ERROR_CURR = ypr[1] - reciever_channel_1;

  PID_PITCH_P_ERROR = PID_PITCH_ERROR_CURR * PID_PITCH_P;                                               // Propotional Error
  
  PID_PITCH_I_ERROR += PID_PITCH_ERROR_CURR * PID_PITCH_I;                                              // Integral Error and its limit

  PID_PITCH_D_ERROR = (PID_PITCH_ERROR_CURR - PID_PITCH_ERROR_PREV) * PID_PITCH_D;                      // Derivative Error  

  PID_PITCH_OUTPUT = PID_PITCH_P_ERROR + PID_PITCH_I_ERROR + PID_PITCH_D_ERROR;                         // Total Pitch Error   

  PID_PITCH_ERROR_PREV = PID_PITCH_ERROR_CURR;                                                          // Assign Current Error to Previous Error 
  

  // For Yaw
  PID_YAW_ERROR_CURR = ypr[0] - reciever_channel_4;

  PID_YAW_P_ERROR = PID_YAW_ERROR_CURR * PID_YAW_P;                                              // Propotional Error
  
  PID_YAW_I_ERROR += PID_YAW_ERROR_CURR * PID_YAW_I;                                             // Integral Error and its limit

  PID_YAW_D_ERROR = (PID_YAW_ERROR_CURR - PID_YAW_ERROR_PREV) * PID_YAW_D;                       // Derivative Error  

  PID_YAW_OUTPUT = PID_YAW_P_ERROR + PID_YAW_I_ERROR + PID_YAW_D_ERROR;                          // Total Roll Error

  PID_YAW_ERROR_PREV = PID_YAW_ERROR_CURR;                                                       // Assign Current Error to Previous Error 

//  Serial.println(PID_YAW_ERROR_CURR);

}

// ISR for Reading the Data from the Reciever on the Pins :- A0(PC0), A1(PC1), A2(PC2), A3(PC3) 

ISR(PCINT2_vect){

  ISR_CURR_TIME =  micros();

  //  ISR condition for channel 1 at PIN 7
  if(PIND & B10000000){
    if(flag_channel_1 == 0 ){
      flag_channel_1=1;
      timer1 = ISR_CURR_TIME; 
    }
  }
  else if(flag_channel_1 == 1 && !(PIND & B10000000)){
    flag_channel_1=0;
    reciever_channel_1 = ISR_CURR_TIME - timer1;
    if(reciever_channel_1 < MIN_MOTOR_OUTPUT) reciever_channel_1 = MIN_MOTOR_OUTPUT;
    if(reciever_channel_1 > MAX_MOTOR_OUTPUT) reciever_channel_1 = MAX_MOTOR_OUTPUT;
    reciever_channel_1 = map(reciever_channel_1, 1000, 2000, -15, 15);
  }

  // ISR condition for channel 2 at PIN 6
  if(PIND & B01000000){
    if(flag_channel_2 == 0 ){
      flag_channel_2=1; 
      timer2 = ISR_CURR_TIME;
    }
  }
  else if(flag_channel_2 == 1 && !(PIND & B01000000)){
    flag_channel_2=0;
    reciever_channel_2 = ISR_CURR_TIME - timer2;
    if(reciever_channel_2 < MIN_MOTOR_OUTPUT) reciever_channel_2 = MIN_MOTOR_OUTPUT;
    if(reciever_channel_2 > MAX_MOTOR_OUTPUT) reciever_channel_2 = MAX_MOTOR_OUTPUT;
    reciever_channel_2 = map(reciever_channel_2, 1000, 2000, -15, 15);
    
  }

  //  ISR condition for channel 3 at PIN 5
  if(PIND & B00100000){
    if(flag_channel_3 == 0 ){
      flag_channel_3=1; 
      timer3 = ISR_CURR_TIME;
    }
  }
  else if(flag_channel_3 == 1 && !(PIND & B00100000)){
    flag_channel_3=0;
    reciever_channel_3 = ISR_CURR_TIME - timer3;
    if(reciever_channel_3 < MIN_MOTOR_OUTPUT) reciever_channel_3 = MIN_MOTOR_OUTPUT;
    if(reciever_channel_3 > MAX_MOTOR_OUTPUT) reciever_channel_3 = MAX_MOTOR_OUTPUT;
    throttle = reciever_channel_3;
  }

  //  ISR condition for channel 4 at PIN 4
  if(PIND & B00010000){
    if(flag_channel_4 == 0){
      flag_channel_4=1; 
      timer4 = ISR_CURR_TIME;
    }
  }
  else if(flag_channel_4 == 1 && !(PIND & B00010000)){
    flag_channel_4=0;
    reciever_channel_4 = ISR_CURR_TIME - timer4;
    if(reciever_channel_4 < MIN_MOTOR_OUTPUT) reciever_channel_4 = MIN_MOTOR_OUTPUT;
    if(reciever_channel_4 > MAX_MOTOR_OUTPUT) reciever_channel_4 = MAX_MOTOR_OUTPUT;
    reciever_channel_4 = ( reciever_channel_4 - 1500 ) / 3;

  }
  
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
