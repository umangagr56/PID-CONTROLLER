#include <Wire.h>
#include "TimerOne.h"
//#include<SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <ros.h>
#include <geometry_msgs/Pose.h>
//SoftwareSerial BTserial();

#define InputAA 8
#define InputAB 7
#define InputBA 4
#define InputBB 9
#define interruptEncoderPinA 3
#define interruptEncoderPinD 2
#define PWMOutputMotorA 6  // right motor
#define PWMOutputMotorB 5  // leftmotor

#define wheel_radius 3.5   //In cm
#define w2w 20    //cm
#define CPR 32
#define gear_ratio 19
#define del_t 0.05 //in seconds

volatile int ext_counterA=0;
volatile int ext_counterB=0;

#define max_rpmA 479.6
volatile float set_rpmA=0;
volatile float curr_rpmA=0;
float errorA=0,preverrorA=0,IA=0;

#define max_rpmB 489.4
volatile float set_rpmB=0;
volatile float curr_rpmB=0;
float errorB=0,preverrorB=0,IB=0;

volatile float curr_pos_x=0.0;
volatile float curr_pos_y=0.0;
volatile float set_pos_x=0.0;
volatile float set_pos_y=0.0;

volatile float theta=0.0; // Current orientation of the bot.
volatile float set_theta=0;
volatile float set_velocity=0;
volatile float omega;

volatile float curr_distance;
volatile float error_theta;

bool motion_directionA=0;
bool motion_directionB=0;

int PWMA,PWMB;

ros::NodeHandle nh;
geometry_msgs::Pose posemsg;

void messageCb(const geometry_msgs::Pose& msg)
{
  curr_pos_x=msg.position.x;
  curr_pos_y=msg.position.y;
  theta=msg.orientation.z;
} 

ros::Subscriber<geometry_msgs::Pose>sub("pose",&messageCb);

void setup() {
nh.initNode();
interrupt_init();
//mag_init();
nh.subscribe(sub)

//nh.initNode();
//nh.advertise(odom_pub);

pinMode(PWMOutputMotorA,OUTPUT);
pinMode(PWMOutputMotorB,OUTPUT);
pinMode(InputAA,OUTPUT);
pinMode(InputAB,OUTPUT);
pinMode(InputBA,OUTPUT);
pinMode(InputBB,OUTPUT);

}

void loop(){
  nh.spinOnce();
  calculateRPM();
  pwm_pid();
  analogWrite(PWMOutputMotorA,PWMA);
  analogWrite(PWMOutputMotorB,PWMB);
  delay(50);
}
