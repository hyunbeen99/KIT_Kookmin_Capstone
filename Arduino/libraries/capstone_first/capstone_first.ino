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

int output_steer = 1460;
int output_vel = 1560;
int weight = 1;


void servo_cb(const geometry_msgs::Twist& cmd_msg) {

  float steering_angle = cmd_msg.angular.z;
  float velocity = cmd_msg.linear.x;

  output_steer = 1460 + int(10 * steering_angle);

  // max angle == 23 degree
  if (output_steer < 1200){
    output_steer = 1200;
  }else if (output_steer > 1720){
    output_steer = 1720;
  }

  steer_servo.writeMicroseconds(output_steer);

  // velocity_servo must be more than 1520
  if (int(velocity) == 0){
    output_vel = 1510;
  }else if (int(velocity) == 1){
    output_vel = 1530;
  }else if (int(velocity) == 2){
    output_vel = 1550;
  }
  
  velocity_servo.writeMicroseconds(output_vel);
  
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
