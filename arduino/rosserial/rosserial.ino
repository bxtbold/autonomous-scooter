// Libraries 
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// pins 
int steering_pin = 8;                   // steering on/off
int dir_pin = 9;                        // Steering direction
int EL=8;                               // EL - POWER
int SIGNAL=9;                           // Signal - Hall sensor
int ZF=10;                              // ZF - DIRECTION
int VR=11;                              // VR - SPEED

// Init values
int pos = 0, steps = 0;                 // constant speed - for hub motor
int steering_degree = 0;                // steering degree


// Constants
int MAX_DEGREE = 45;

// node handles 
ros::NodeHandle nh;
geometry_msgs::Twist twist;



void callback(const geometry_msgs::Twist& twist)
//void callback(const std_msgs::Float64& twist)
{
  double x = twist.linear.x;
  double z = twist.angular.z;

  if(z<0){
    if(steering_degree < MAX_DEGREE){
      Steering(LOW, 100);
      steering_degree++;
      }
    }
  else if (z>0){
      if(steering_degree < -MAX_DEGREE){
        Steering(HIGH, 100);
        steering_degree--;
      }
    }


  if(x<0){
    MoveBackward(x);
  }
  else if(x>0){
    MoveForward(x);
  }
  
}

//ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", callback);
ros::Subscriber <std_msgs::Float64> sub("cmd_vel", callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  pinMode(steering_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(EL, OUTPUT);         
  pinMode(SIGNAL, INPUT);      
  pinMode(ZF, OUTPUT);         
  pinMode(VR, OUTPUT);       
}

void loop()
{
  nh.spinOnce();
  delay(10);
}

void Steering(boolean dir, int duration){
  digitalWrite(steering_pin, HIGH);      // steering_on
  digitalWrite(dir_pin, dir);
  delay(duration);
  digitalWrite(steering_pin, LOW);       // steering_off
}

void plus() {
  pos++; //count steps
  Serial.println(pos);
    if(pos>=steps){
    Brake();
    pos=0;}
}

void Brake(){
  digitalWrite(EL,LOW);
}

void MoveForward(double vel){
  analogWrite(VR, vel);
  digitalWrite(EL, LOW);
  delay(100);
  digitalWrite(ZF,HIGH);
  delay(100);
  digitalWrite(EL,HIGH);
}

void MoveBackward(double vel){
  analogWrite(VR, vel);
  digitalWrite(EL, LOW);
  delay(100);
  digitalWrite(ZF,LOW);
  delay(100);
  digitalWrite(EL,HIGH);
}
