#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from ackermann_msgs.msg import AckermannDriveStamped


def cmd_callback(data):
  global max_throttle
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  brake=0
  throttle=(data.drive.speed/max_throttle)*100
  if(throttle<prv_throttle):
      brake=((prv_throttle-throttle)/max_throttle)*100
      throttle=0
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.speed = throttle
  msg.drive.jerk = brake
  prv_throttle=throttle
  pub.publish(msg)





if __name__ == '__main__':
  try:

    rospy.init_node('ackermann_drive_to_arduino_msg')

    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/rbcar_robot_control/command')
    arduino_cmd_topic = rospy.get_param('~arduino_cmd_topic', '/arduino/command')
    max_throttle = rospy.get_param('~max_throttle', 2)
    wheelbase = rospy.get_param('~wheelbase', 2.07)
    frame_id = rospy.get_param('~frame_id', 'odom')

    rospy.Subscriber(ackermann_cmd_topic, AckermannDriveStamped, cmd_callback, queue_size=1)
    pub=rospy.Publisher(arduino_cmd_topic, AckermannDriveStamped, queue_size=1)

    rospy.loginfo("Node 'ackermann_drive_to_arduino_msg' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/rbcar_robot_control/command", ackermann_cmd_topic, frame_id, wheelbase)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass
