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

int output = 1460;
int weight = 1;


void servo_cb(const geometry_msgs::Twist& cmd_msg) {

  float steering_angle = cmd_msg.angular.z;
  float velocity = cmd_msg.linear.x;

  output = 1460 + int(10 * steering_angle);

  // max angle == 23 degree
  if (output < 1200){
    output = 1200;
  }else if (output > 1720){
    output = 1720;
  }

  steer_servo.writeMicroseconds(output);

  // velocity_servo must be more than 1520
  velocity_servo.writeMicroseconds(1550);
}

ros::Subscriber<geometry_msgs::Twist> sub_servo("controller", servo_cb);


void setup() {
  nh.initNode();
  nh.subscribe(sub_servo);
  steer_servo.attach(10);
  velocity_servo.attach(9);
}

void loop() {
  nh.spinOnce();

  delay(5);
}
