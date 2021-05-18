#include <ros.h>

#include <Servo.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

Servo servo;
Servo esc;

int potValue;

void servo_cb(const std_msgs::UInt16& cmd_msg) {
  servo.write(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));
}

void esc_cb(const std_msgs::UInt16& cmd_msg) {
  potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 180);
  esc.write(potValue);
}

ros::Subscriber<std_msgs::UInt16> sub_servo("servo", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub_esc("esc", esc_cb);

void setup() {
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_servo);
  nh.subscribe(sub_esc);

  servo.attach(11);
  ESC.attach(9, 1000, 2000);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
