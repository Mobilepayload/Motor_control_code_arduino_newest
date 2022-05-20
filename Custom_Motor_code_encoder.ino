#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#define MODBUS_DATA_TRANSACTION_PIN_LEFT A0   //DE & DE Pin of MAX485
#define MODBUS_DATA_TRANSACTION_PIN_RIGHT A2
#define VOLTAGE_READING_PIN A3
//Constant Setup Values
//double leftMotorSpeed = 0;
//double rightMotorSpeed = 0;
double leftMotorSpeed = 0;
double rightMotorSpeed = 0;
double spinSpeed = 0;
short batteryLow=30;

double w_r=0, w_l=0;
double w_r_const=0, w_l_const=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.15, wheel_sep = 0.3;

ros::NodeHandle nh;

double speed_ang=0, speed_lin=100;

void messageCb( const geometry_msgs::Twist& msg){
  nh.loginfo("Callback function entered");
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;

 w_r = abs((speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad)));
 w_l = abs((speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad)));
 
 w_r_const = (7.78/wheel_rad) + ((7.78*wheel_sep)/(2.0*wheel_rad));
 w_l_const = (7.78/wheel_rad) - ((7.78*wheel_sep)/(2.0*wheel_rad));
 
 leftMotorSpeed = (w_l<=w_l_const)?((w_l/w_l_const * 890) + 110):1000;
 rightMotorSpeed =  (w_r<=w_r_const)?((w_r/w_r_const * 890) + 110):1000;
 
 spinSpeed = ((leftMotorSpeed+rightMotorSpeed)/2<=400)?((leftMotorSpeed+rightMotorSpeed)/2):400;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); 

char hello[13] = "hello world!";

/*
std_msgs::Float64 lin_vel;
ros::Publisher lin_velPub("linvel", &lin_vel);

std_msgs::Float64 ang_vel;
ros::Publisher ang_velPub("angvel", &ang_vel);
*/
uint8_t LeftMotorSlaveId = 1;   //Slave ID of LEFT Drive
uint8_t RightMotorSlaveId = 1;   //Slave ID of RIGHT Drive
ModbusMaster modbusMasterLeftMotorNode;                     
ModbusMaster modbusMasterRightMotorNode;
//SoftwareSerial Max485LeftMotorSerial(3, 2);   //Serial Port(Rx,Tx) Connected with MAX 485 Module on Left Motor
//SoftwareSerial Max485RightMotorSerial(5, 4);  //Serial Port(Rx,Tx) Connected with MAX 485 Module on Right Motor
SoftwareSerial Max485LeftMotorSerial(3, 2);   //Serial Port(Rx,Tx) Connected with MAX 485 Module on Left Motor
SoftwareSerial Max485RightMotorSerial(5, 4);  //Serial Port(Rx,Tx) Connected with MAX 485 Module on Right Motor

//Working Variables
int runningSpeed = 0;


// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 7 //3
#define ENC_IN_RIGHT_A 6 //2
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 9 //5
#define ENC_IN_RIGHT_B 8 //4
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}

void setup() 
{
  //Serial.begin(9600);
  //while(!Serial.available());
  
  // put your setup code here, to run once:
  pinMode(MODBUS_DATA_TRANSACTION_PIN_LEFT, OUTPUT);
  pinMode(MODBUS_DATA_TRANSACTION_PIN_RIGHT, OUTPUT);
  //Setting BaudRate 
  Max485LeftMotorSerial.begin(9600);   //Modbus Baud rate is 9600 8N1
  Max485RightMotorSerial.begin(9600);   //Modbus Baud rate is 9600 8N1
  
  
  modbusMasterLeftMotorNode.begin(Max485LeftMotorSerial);
  modbusMasterLeftMotorNode.preTransmission(preTransmission);
  modbusMasterLeftMotorNode.postTransmission(postTransmission);
  
  modbusMasterRightMotorNode.begin(Max485RightMotorSerial);
  modbusMasterRightMotorNode.preTransmission(preTransmission);
  modbusMasterRightMotorNode.postTransmission(postTransmission);

  StopTheBot();

  //Encoder Ticks
   // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
  
  nh.getHardware()->setBaud(19200);
  
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(rightPub);
  nh.advertise(leftPub);

  //nh.advertise(chatter);
  /*
  nh.advertise(lin_velPub);
  nh.advertise(ang_velPub);
  */
}


void loop() 
{
double p1= w_l*10;
double p2= w_r*10;
//ForwardTheBot();
 
  if(speed_lin>0 && speed_ang==0)
   ForwardTheBot();

 else if(speed_lin<0 && speed_ang==0)
   ReverseTheBot();

 else if((speed_lin>=0 && speed_ang<0)||(speed_lin<0 && speed_ang>0))
   SpinRightTheBot();

 else if((speed_lin>=0 && speed_ang>0)||(speed_lin<0 && speed_ang<0))
   SpinLeftTheBot();

 else 
    StopTheBot();

    str_msg.data = hello;
    chatter.publish( &str_msg );

  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {

     
    Serial.print("Left Encoder:");
    Serial.println(left_wheel_tick_count.data);
    Serial.print("Right Encoder:");
    Serial.println(right_wheel_tick_count.data);
    previousMillis = currentMillis;
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
  }

 nh.spinOnce();
}

void StopTheBot()
{
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 515); // CCW-521, stop-512, brake-515, CW-513
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 515); // CCW-521, stop-512, brake-515, CW-513
  
}
void ReverseTheBot()
{
  //Serial.println("Forward");
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 513); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);

  
}
void ForwardTheBot()
{
  //Serial.println("Backward");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 513); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
 
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  
}
void TurnRightTheBot()
{
  //Serial.println("TurnRight");
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 512); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
 
  
}
void TurnLeftTheBot()
{
  //Serial.println("TurnLeft");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 512); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  
}
void spinRobot(int m, int n)
{
  //Serial.println("Spin");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, m); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, n); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}
void SpinRightTheBot()
{
   //Serial.println("SpinRight");
   spinRobot(513, 513);
  
}
void SpinLeftTheBot()
{
  //Serial.println("SpinLeft");
  spinRobot(521, 521);
}


void setMotorSpeed(int leftMotorRPM, int rightMotorRPM)
{
  modbusMasterLeftMotorNode.Set_Speed(LeftMotorSlaveId, leftMotorSpeed  ); // Set Speed, speed range is (0-1000 RPM)
  //delay(1);
  modbusMasterRightMotorNode.Set_Speed(RightMotorSlaveId, rightMotorSpeed ); // Set Speed, speed range is (0-1000 RPM)
   //delay(1);
  
}

void preTransmission() {
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_LEFT, 1);
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_RIGHT, 1);
}
void postTransmission() {
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_LEFT, 0);
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_RIGHT, 0);
}

bool GetBatteryVoltage() {
  // read the input on analog pin 0:
  float voltage;
  int voltageDigital = analogRead(VOLTAGE_READING_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5.0V):
  voltage = (voltageDigital * (5.0 / 1024.0) ) / 0.09;
  voltage = voltage - 2; // add on for calibration only for AB2
  float batteryPercentage = (voltage - 19.2) / 0.074; // full charge 26.6V
  if (voltage < 19.2)
  {
    //Serial.print("LOW BATTERY : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
    return false;
    //Serial.print("LOW BATTERY : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
  }
  else 
  {
    //Serial.print("Battery : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
    return true;
    //Serial.print("Battery : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
  }
}
