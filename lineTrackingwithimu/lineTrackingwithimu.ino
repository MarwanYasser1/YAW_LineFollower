// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

float yaw=0;
float error;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// IR Sensors
#define sensor1  A0      // Left most sensor
#define sensor2  A1
#define sensor3  A2
#define sensor4  A3
#define sensor5  6   // Right most sensor

// Initial Values of Sensors
int sensor[5] = {0, 0, 0, 0, 0};

//Initial Speed of Motor
int initial_motor_speed = 140;

// Motor Variables
#define ENA  9
#define motorInput1  11
#define motorInput2  12
#define motorInput3  8
#define motorInput4  7
#define ENB  10

#define SpeedLimit 110     //motors upper speed limit
#define LowerSpeedLimit 90 // lower speed limit

float Motor_speed_R=SpeedLimit;
float Motor_speed_L=SpeedLimit;

/////////////PID Variables/////////////////
float kp= 2.5;
float ki=0;
float kd= 0.4;
//float kir=0.0;
float lineConstant=3.7; // Kp for line tracking that tunes how much the robot adjusts
float accumilation = 0;

float Y_actual_read;
int Y_setpoint=1;
int Y_setpoint_img=0;
float Y_lasterror;
float Y_error=0;
float delta_error;
float Y_area=0;
float Y_slope=0;
unsigned long previous_t=0;
unsigned long new_t;
unsigned long dt;
float pid_val;
unsigned long d_time =0;



float RevTime=0;
float NewRevTime=0 ;
int white_falg=0;
void check(){
  if (white_falg==0){
    RevTime=millis();
    white_falg++;
  }
}
void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
    //Motor_speed_R=90-pid_val;
    //Motor_speed_L=225+pid_val;
    //Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    //Motor_speed_L = abs( constrain(Motor_speed_L, LowerSpeedLimit, SpeedLimit));
    
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, LOW);   
    digitalWrite(motorInput4, HIGH);
    analogWrite(ENA,Motor_speed_R);
    analogWrite(ENB,Motor_speed_L);
    Serial.println("forward");

}
void reverse()
{

}

void Right() {

  delay(90);//delay 3alshan el center line beta3 el wheels yeb2a 3and el line
  
  //////////////////////////Changing Setpoint/////////////////
  Serial.println("Ana Right");

      // Y_setpoint=yaw;
    Y_setpoint_img = Y_setpoint;//ben8ayar el setpoint el hayturn 3ala asasha
    
      if (Y_setpoint_img+90==270){
       Y_setpoint=-83;
       Y_setpoint_img=-83;
       }
       else if (Y_setpoint_img+90==180){
       Y_setpoint=176;
       Y_setpoint_img=180;
       //////////////////////////////////////specail case law el robot haylef towards el 180/////////////////////////////////
       while(!(yaw < -176 || yaw > 176)){

////////////////////////////////updating MPU values//////////////////////////////////
        if (!dmpReady) return;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                 
                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("yaw\t");
                Serial.print(ypr[0] * 180/M_PI);
                yaw=ypr[0] * 180/M_PI;
                Serial.print("\n");
    
            #endif
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////calculating PID//////////////////////////////
        previous_t=new_t;
        new_t=millis();
        dt=new_t-previous_t;
    
        Y_setpoint+=error;
        Y_lasterror=Y_error;
        Y_actual_read=yaw;
        Y_error=Y_setpoint-Y_actual_read;
        delta_error=Y_error-Y_lasterror;  
        Y_slope=delta_error/dt;
        accumilation+= Y_error*dt;
        pid_val=(kp*Y_error)+(kd*Y_slope)+(ki*accumilation);
        Serial.print("pid value: ");
        Serial.println(pid_val);
        }
    /////////////////////////////////////////////////////////////////////

    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);   
    digitalWrite(motorInput4, HIGH);

    Motor_speed_R=pid_val;
    Motor_speed_L=pid_val;
    Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    Motor_speed_L = abs( constrain(Motor_speed_L, LowerSpeedLimit, SpeedLimit));

    analogWrite(ENA,Motor_speed_R);
    analogWrite(ENB,Motor_speed_L);
    

    Serial.println("turning right");
    }
//    continue;
       ///////////////////////////////////////////////////////////////////////////////////
       
      }
      else{
       Y_setpoint+=83;
       Y_setpoint_img+=83;
      }

      while(!(yaw < Y_setpoint+1 && yaw > Y_setpoint-1)){

////////////////////////////////updating MPU values//////////////////////////////////
        if (!dmpReady) return;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                 
                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("yaw\t");
                Serial.print(ypr[0] * 180/M_PI);
                yaw=ypr[0] * 180/M_PI;
                Serial.print("\n");
    
            #endif
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////calculating PID//////////////////////////////
        previous_t=new_t;
        new_t=millis();
        dt=new_t-previous_t;
    
        Y_setpoint+=error;
        Y_lasterror=Y_error;
        Y_actual_read=yaw;
        Y_error=Y_setpoint-Y_actual_read;
        delta_error=Y_error-Y_lasterror;  
        Y_slope=delta_error/dt;
        accumilation+= Y_error*dt;
        pid_val=(kp*Y_error)+(kd*Y_slope)+(ki*accumilation);
        Serial.print("pid value: ");
        Serial.println(pid_val);
        }
    /////////////////////////////////////////////////////////////////////

    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);   
    digitalWrite(motorInput4, HIGH);

    Motor_speed_R=pid_val;
    Motor_speed_L=pid_val;
    Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    Motor_speed_L = abs( constrain(Motor_speed_L, LowerSpeedLimit, SpeedLimit));

    analogWrite(ENA,Motor_speed_R);
    analogWrite(ENB,Motor_speed_L);
    

    Serial.println("turning right");
    }
    Stop();
    delay(500);

   
}
void Left() {

  delay(10);//delay 3alshan el center line beta3 el wheels yeb2a 3and el line

  ////////////////////////////Cahnging setpoint////////////////////////
   Serial.println("ana left");
   Y_setpoint_img = Y_setpoint;
  
  if (Y_setpoint_img-90==-270){
    Y_setpoint=80;
    Y_setpoint_img=80;
  }
  else if (Y_setpoint_img-90==-180){
    Y_setpoint=-176;
    Y_setpoint_img=-180;
    ///////////////////////////////////////specail case law el robot haylef towards el 180//////////////////////////////
    while(!(yaw < -176 || yaw > 176)){

////////////////////////////////updating MPU values//////////////////////////////////
        if (!dmpReady) return;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                 
                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("yaw\t");
                Serial.print(ypr[0] * 180/M_PI);
                yaw=ypr[0] * 180/M_PI;
                Serial.print("\n");
    
            #endif
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////calculating PID//////////////////////////////
        previous_t=new_t;
        new_t=millis();
        dt=new_t-previous_t;
    
        Y_setpoint+=error;
        Y_lasterror=Y_error;
        Y_actual_read=yaw;
        Y_error=Y_setpoint-Y_actual_read;
        delta_error=Y_error-Y_lasterror;  
        Y_slope=delta_error/dt;
        accumilation+= Y_error*dt;
        pid_val=(kp*Y_error)+(kd*Y_slope)+(ki*accumilation);
        Serial.print("pid value: ");
        Serial.println(pid_val);
        }
    /////////////////////////////////////////////////////////////////////

    Motor_speed_R= abs(pid_val);
    Motor_speed_L= abs(pid_val);
    Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    Motor_speed_L = abs( constrain (Motor_speed_L,LowerSpeedLimit, SpeedLimit));

    analogWrite(ENA,Motor_speed_R);
    analogWrite(ENB,Motor_speed_L);
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, HIGH);   
    digitalWrite(motorInput4, LOW);

    Serial.println("turning left");
    }
    //continue;
    //////////////////////////////////////////////////////////////////////////////////////////////////
  }
  else{
    Y_setpoint-=80;
    Y_setpoint_img-=80; 
  }
  Serial.println("waiting to ccomplete 90 degree");

    while(!(yaw < Y_setpoint+1 && yaw > Y_setpoint-1)){

////////////////////////////////updating MPU values//////////////////////////////////
        if (!dmpReady) return;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                 
                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("yaw\t");
                Serial.print(ypr[0] * 180/M_PI);
                yaw=ypr[0] * 180/M_PI;
                Serial.print("\n");
    
            #endif
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////calculating PID//////////////////////////////
        previous_t=new_t;
        new_t=millis();
        dt=new_t-previous_t;
    
        Y_setpoint+=error;
        Y_lasterror=Y_error;
        Y_actual_read=yaw;
        Y_error=Y_setpoint-Y_actual_read;
        delta_error=Y_error-Y_lasterror;  
        Y_slope=delta_error/dt;
        accumilation+= Y_error*dt;
        pid_val=(kp*Y_error)+(kd*Y_slope)+(ki*accumilation);
        Serial.print("pid value: ");
        Serial.println(pid_val);
        }
    /////////////////////////////////////////////////////////////////////

    Motor_speed_R= abs(pid_val);
    Motor_speed_L= abs(pid_val);
    Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    Motor_speed_L = abs( constrain (Motor_speed_L,LowerSpeedLimit, SpeedLimit));

    analogWrite(ENA,Motor_speed_R);
    analogWrite(ENB,Motor_speed_L);
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, HIGH);   
    digitalWrite(motorInput4, LOW);

    Serial.println("turning left");
    }
    Stop();// a stop so that we can see what happened
    delay(500);

}


void Stop()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
  
}


void setup() {

  Serial.begin(115200);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);




  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
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
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


     new_t=millis();
     d_time=millis();
  
}

void loop() {

///////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////a delay while updating the mpu readings to acount for the yaw values that drift at the begining of a power up ///////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    while (millis()-d_time<=5000){
      
      if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
             
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("yaw\t");
            Serial.print(ypr[0] * 180/M_PI);
            yaw=ypr[0] * 180/M_PI;
            Serial.print("\n");

        #endif
    }
    }
    ///////////updating mpu readings
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
             
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("yaw\t");
            Serial.print(ypr[0] * 180/M_PI);
            yaw=ypr[0] * 180/M_PI;
            Serial.print("\n");

        #endif
    }
    ////////////end of delay

  //updating line sensor readings
    sensor[0] = !digitalRead(sensor1);
    sensor[1] = !digitalRead(sensor2);
    sensor[2] = !digitalRead(sensor3);
    sensor[3] = !digitalRead(sensor4);
    sensor[4] = !digitalRead(sensor5);


    /////////////almost every possible condition////////////////////////////////////////
  
    if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 1 ))//0 0 0 0 1adjust by turning right 
    {
        Motor_speed_R-=3*lineConstant;
        Motor_speed_L+=3*lineConstant; 
        forward();
        Serial.println("adjust to the right4");     
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 0 )) // 0 0 0 1 0adjust by turning right
    {
        Motor_speed_R-=2*lineConstant;
        Motor_speed_L+=2*lineConstant; 
        forward();
        Serial.println("adjust to the right2");
      
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 1 )&&(sensor[4]== 0 )) //0 0 1 1 0adjust by turning right
    {
        Motor_speed_R-=1*lineConstant;
        Motor_speed_L+=1*lineConstant; 
        forward();
        Serial.println("adjust to the right1");
      
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) //0 0 1 0 0forward
    {
        Motor_speed_R=SpeedLimit;
        Motor_speed_L=SpeedLimit;
        forward();
        Serial.println("forward");
      
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) //reverse bedahro
    {     
       // check();
       forward();
        
      if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ) && ((millis() - RevTime) == 300)) // 0 0 0 0 0reverse bedahro
    {
      forward();

      /*
    analogWrite(ENA,SpeedLimit);
    analogWrite(ENB,SpeedLimit);
    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, HIGH);   
    digitalWrite(motorInput4, LOW); 
    white_falg=0;
         */
    }
}

    else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) //0 1 1 0 0adjust by turning left
    {
        Motor_speed_R+=1*lineConstant;
        Motor_speed_L-=1*lineConstant;
        forward();
        Serial.println("adjust to the left1");
    
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) //0 1 0 0 0adjust by turning left
    {
        Motor_speed_R+=2*lineConstant;
        Motor_speed_L-=2*lineConstant;
        forward();
        Serial.println("adjust to the left2");
      
    }
     
    else if((sensor[0]== 1 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) //1 0 0 0 0 adjust by turning left
    {
        Motor_speed_R+=3*lineConstant;
        Motor_speed_L-=3*lineConstant;
        Serial.println("adjust to the left4");
        forward();
      
    }
    
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)  && (sensor[4] == 0))//1 1 1 0 0 90 degree left
  {
     /* Motor_speed_R=120;
      Motor_speed_L=0; */
      /*Serial.println("turn left");

      Left();*/
      forward();
      
  }  
   else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0)  && (sensor[4] == 0))//1 1 0 0 0 90 degree left
  {
      Serial.println("turn left");
      Left();
      
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))//0 0 1 1 1 forward
  {
      
      Motor_speed_R=SpeedLimit;
      Motor_speed_L=SpeedLimit; 
      forward();
      Serial.println("forward");   
      
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))//0 1 1 1 0 forward
  {
      
      Motor_speed_R=SpeedLimit;
      Motor_speed_L=SpeedLimit; 
      forward();
      Serial.println("forward");   
      
  }
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 1 )) //0 0 0 1 1 90 right
    {
        Serial.println("turn right");
        Right();
    
    }
    else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 1 )) //0 1 0 1 1 90 right
    {
        Serial.println("turn right");
        forward();
    
    }
    

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)){ //1 1 1 1 1 90 degree right
        
        Serial.println("turn right");
        
        Right();
    }
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)){ //1 1 0 1 1 90 degree right
        
        Serial.println("turn right");
        
        Right();
    }
    else if((sensor[0]== 1 ) && (sensor[1]== 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) //1 1 0 0 0 90degree left
    {
       /* Motor_speed_R=120;
        Motor_speed_L=0;*/
        Serial.println("turn left");
        Left();
    }
    else if((sensor[0]== 1 ) && (sensor[1]== 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) //1 1 1 1 0 forward
    {
       
        Serial.println("forward");
        forward();
    }


    ////////////////////calculating PID//////////////////////////////
    previous_t=new_t;
    new_t=millis();
    dt=new_t-previous_t;

    Y_setpoint+=error;
    Y_lasterror=Y_error;
    Y_actual_read=yaw;
    Y_error=Y_setpoint-Y_actual_read;
    delta_error=Y_error-Y_lasterror;  
    Y_slope=delta_error/dt;
    accumilation+= Y_error*dt;
    pid_val=(kp*Y_error)+(kd*Y_slope)+(ki*accumilation);
    Serial.print("pid value: ");
    Serial.println(pid_val);
    /////////////////////////////////////////////////////////
    

    Motor_speed_R = abs( constrain(Motor_speed_R, LowerSpeedLimit, SpeedLimit));
    Motor_speed_L = abs( constrain (Motor_speed_L,LowerSpeedLimit, SpeedLimit));



    
}
