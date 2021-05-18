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

#include <ros.h>

#include <Servo.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

Servo servo;
Servo esc;

int potValue;

void servo_cb(const geometry_msgs::Twist& cmd_msg) {
  servo.write(cmd_msg.linear.x);
  digitalWrite(13, HIGH-digitalRead(13));

  potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 180);
  esc.write(potValue);
}

ros::Subscriber<geometry_msgs::Twist> sub_servo("controller", servo_cb);


void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_servo);
  servo.attach(11);
  esc.attach(9, 1000, 2000);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
