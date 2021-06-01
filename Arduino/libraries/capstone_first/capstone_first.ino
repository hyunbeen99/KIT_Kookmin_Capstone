/*
#include <HttpClient.h>
#include <YunServer.h>
#include <BridgeServer.h>
#include <YunClient.h>
#include <Bridge.h>
#include <Process.h>
#include <BridgeClient.h>
#include <FileIO.h>
#include <Mailbox.h>
#include <BridgeUdp.h>
#include <Console.h>
#include <BridgeSSLClient.h>
*/

#include <ros.h>

#include <Servo.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
Servo steer_servo;
Servo velocity_servo;

int output;


void servo_cb(const geometry_msgs::Twist& cmd_msg) {

  float steering_angle = cmd_msg.angular.z;
  float velocity = cmd_msg.linear.x;

  // PWM
  // velocity_servo.writeMicroseconds(1515);

  if (steering_angle < -20){
    output = 1220;
  }else if (steering_angle < -10){
    output = 1320;
  }else if (steering_angle > 20){
    output = 1680;
  }else if (steering_angle > 10){
    output = 1580;
  }else {
    output = 1450;
  }

  // write 1250 ~ 1750 (middle = 1450)
  steer_servo.writeMicroseconds(output);
  
  // -5 < steer < 5
  if (output != 1450){
    velocity_servo.writeMicroseconds(1520);
  }else if (output == 1450){
    velocity_servo.writeMicroseconds(1570);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub_servo("controller", servo_cb);


void setup() {
  nh.initNode();
  nh.subscribe(sub_servo);
  steer_servo.attach(9);
  velocity_servo.attach(10);
}

void loop() {
  nh.spinOnce();

  delay(1);
}
