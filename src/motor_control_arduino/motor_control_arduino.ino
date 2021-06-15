// Libraries 
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// pins 
int steering_pin = 9;                   // steering on/off
int dir_pin = 6;                        // Steering direction
int EL=8;                               // EL - POWER
//int SIGNAL=9;                         // Signal - Hall sensor
int ZF=10;                              // ZF - DIRECTION
int VR=11;                              // VR - SPEED

// Init values
//int pos = 0, steps = 0;               // constant speed - for hub motor

// node handles 
ros::NodeHandle nh;
geometry_msgs::Twist twist;

double z_old = 100;

void callback(const geometry_msgs::Twist& twist)
{
  double x = twist.linear.x;
  double z = twist.angular.z;

  if(z_old - z < 0) Steering(HIGH, 100);
  else if(z_old - z > 0) Steering(LOW, 100);
  z_old = z;
  
  if(x<0) MoveBackward(-1*x);
  else if(x>0) MoveForward(x);
  else if(x==0) Brake();
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(steering_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(EL, OUTPUT);
  //pinMode(SIGNAL, INPUT);
  pinMode(ZF, OUTPUT);
  pinMode(VR, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(1);         // trying this by no delay time
}

void Steering(boolean dir, int duration){
  digitalWrite(steering_pin, HIGH);      // steering_on
  digitalWrite(dir_pin, dir);
  delay(duration);
  digitalWrite(steering_pin, LOW);       // steering_off
}

void Brake(){
  digitalWrite(EL,LOW);
}

void MoveForward(double speed1){
  digitalWrite(ZF, HIGH);
  analogWrite(VR, speed1);
  digitalWrite(EL, HIGH);
}

void MoveBackward(double speed1){
  digitalWrite(ZF, LOW);
  analogWrite(VR, speed1);
  digitalWrite(EL, HIGH);
}
