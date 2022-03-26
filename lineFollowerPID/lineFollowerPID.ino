
// IR Sensors
int sensor1 = A0;      // Left most sensor
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A6;  // Right most sensor

// Initial Values of Sensors
int sensor[5] = {1, 1, 1, 1, 1};

//Initial Speed of Motor
int initial_motor_speed = 140;

// Motor Variables
int ENA = 9;
int motorInput1 = 11;
int motorInput2 = 12;
int motorInput3 = 6;
int motorInput4 = 7;
int ENB = 10;

int Motor_speed=200;
// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;
void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);   
  digitalWrite(motorInput4, HIGH);
  analogWrite(ENA,Motor_speed);
  analogWrite(ENB,Motor_speed);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  /*Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);*/

  analogWrite(ENA, left_motor_speed); //Left Motor Speed
  analogWrite(ENB, right_motor_speed - 30); //Right Motor Speed

//following lines of code are to make the bot move forward
  forward();
}

void read_sensor_values()
{
  sensor[0] = !digitalRead(sensor1);
  sensor[1] = !digitalRead(sensor2);
  sensor[2] = !digitalRead(sensor3);
  sensor[3] = !digitalRead(sensor4);
  sensor[4] = !digitalRead(sensor5);
  /*
    Serial.print(sensor[0]);
    Serial.print("\t");
    Serial.print(sensor[1]);
    Serial.print("\t");
    Serial.print(sensor[2]);
    Serial.print("\t");
    Serial.println(sensor[3]);*/
if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 1 )) 
{error = 4;
  Serial.println(error);}
else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 1 )) 
{error = 3; 
  Serial.println(error);
}
else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 0 )) 
{error = 2;
  Serial.println(error);
}
else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 1 )&&(sensor[4]== 0 )) 
{error = 1;
  Serial.println(error);
}
else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) 
{error = 0;
  Serial.println(error);}
else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) 
{error =- 1;
  Serial.println(error);
}
else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) 
{error = -2;
  Serial.println(error);}
  
else if((sensor[0]== 1 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) 
{
  error = -3;
    Serial.println(error);
}
else if((sensor[0]== 1 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 )) 
{error = -4;
  Serial.println(error);}
    
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)  && (sensor[4] == 0)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) // Turn robot right side
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) // Make U turn
    error = 102;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) // Turn left side or stop
    error = 103;
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

  
void setup()
{
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

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!\n");
  delay(1000);
}
void loop()
{
  forward();
//  read_sensor_values();
//  Serial.println(error);
//  if (error == 100) {               // Make left turn untill it detects straight path
//    //Serial.print("\t");
//    //Serial.println("Left");
//    do {
//      read_sensor_values();
//      analogWrite(ENA, 110); //Left Motor Speed
//      analogWrite(ENB, 90); //Right Motor Speed
//      sharpLeftTurn();
//    } while (error != 0);
//
//  } else if (error == 101) {          // Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
//                                      // untill it detects straight path.
//    //Serial.print("\t");
//    //Serial.println("Right");
//    analogWrite(ENA, 110); //Left Motor Speed
//    analogWrite(ENB, 90); //Right Motor Speed
//    forward();
//    delay(200);
//    stop_bot();
//    read_sensor_values();
//    if (error == 102) {
//      do {
//        analogWrite(ENA, 110); //Left Motor Speed
//        analogWrite(ENB, 90); //Right Motor Speed
//        sharpRightTurn();
//        read_sensor_values();
//      } while (error != 0);
//    }
//  } else if (error == 102) {        // Make left turn untill it detects straight path
//    //Serial.print("\t");
//    //Serial.println("Sharp Left Turn");
//    do {
//      analogWrite(ENA, 110); //Left Motor Speed
//      analogWrite(ENB, 90); //Right Motor Speed
//      sharpLeftTurn();
//      read_sensor_values();
//      if (error == 0) {
//        stop_bot();
//        delay(200);
//      }
//    } while (error != 0);
//  } else if (error == 103) {        // Make left turn untill it detects straight path or stop if dead end reached.
//    if (flag == 0) {
//      analogWrite(ENA, 110); //Left Motor Speed
//      analogWrite(ENB, 90); //Right Motor Speed
//      forward();
//      delay(200);
//      stop_bot();
//      read_sensor_values();
//      if (error == 103) {     /** Dead End Reached, Stop! **/
//        stop_bot();
//        
//        flag = 1;
//      } else {        /** Move Left **/
//        analogWrite(ENA, 110); //Left Motor Speed
//        analogWrite(ENB, 90); //Right Motor Speed
//        sharpLeftTurn();
//        delay(200);
//        do {
//          //Serial.print("\t");
//          //Serial.println("Left Here");
//          read_sensor_values();
//          analogWrite(ENA, 110); //Left Motor Speed
//          analogWrite(ENB, 90); //Right Motor Speed
//          sharpLeftTurn();
//        } while (error != 0);
//      }
//    }
//  } else {
//    calculate_pid();
//    motor_control();
//  }
}
